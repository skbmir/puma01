#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <string>

class PUMA_Controller
{

	private:

	ros::NodeHandle n; 
	
	ros::Subscriber joint_states_q_sub,
					cmd_sub; 
	
	ros::Publisher	joint_1_pub_, 
					joint_2_pub_, 
					joint_3_pub_, 
					joint_4_pub_,
					joint_5_pub_,
					joint_6_pub_,
					info_pub_; 							

	static const unsigned int n_joints = 6;

	std::array<control_toolbox::Pid,n_joints> pid_controllers;	


	std::array<double, n_joints> 	qd = {0, 0, 0, 0, 0, 0}, 
									dqd = {0, 0, 0, 0, 0, 0}, 
									ddqd = {0, 0, 0, 0, 0, 0}, 
									q = {0, 0, 0, 0, 0, 0}, 
									dq = {0, 0, 0, 0, 0, 0}, 
									q_last = {0, 0, 0, 0, 0, 0},							
									P = {1, 1, 1, 1, 1, 1},
									D = {1, 1, 1, 1, 1, 1},
									lower_limits = {-2.79, -3.93, -0.78, -1.92, -1.74, -4.64},
									upper_limits = {2.79, 0.78, 3.93, 2.97, 1.74, 4.64}; 		

	double max_delta = 0, time_last;
			
	std::array<std_msgs::Float64, n_joints> tau;

	std_msgs::Float64MultiArray info;

	void CommandReceive(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		if(msg->data.size()!=n_joints*3)
		{
			ROS_ERROR_STREAM("Command error...");
			return;
		}
		for(unsigned int i=0; i<n_joints; i++)
		{
			qd[i] = (double)msg->data[i];    
			dqd[i] = (double)msg->data[i+n_joints]; 		
			ddqd[i] = (double)msg->data[i+2*n_joints]; 							
		}
	}

	void JointStatesObserve(const sensor_msgs::JointState::ConstPtr& msg)
	{

	// evaluating time
		double delta = ros::Time::now().toSec() - time_last; // it can't be more than joint_state_controller rate!

		time_last= ros::Time::now().toSec();

		ros::Duration period = ros::Duration(delta);


		std::array<double, n_joints> 	err = {0, 0, 0, 0, 0, 0},
										derr = {0, 0, 0, 0, 0, 0},
										G = {0, 0, 0, 0, 0, 0},
										C = {0, 0, 0, 0, 0, 0},
										PD = {0, 0, 0, 0, 0, 0},
										M_ddq = {0, 0, 0, 0, 0, 0};

	// computing velocities dq and PID terms
		for(unsigned int i=0; i<n_joints; i++)
		{

		// position error
			angles::shortest_angular_distance_with_large_limits(
				msg->position[i],
				qd[i],
				lower_limits[i],
				upper_limits[i],
				err[i]);

			if(delta!=0){
				//dq[i] = (msg->position[i] - q_last[i])/delta;
				dq[i] = msg->velocity[i];
				q_last[i] = msg->position[i];      	
			}

			derr[i] = dqd[i] - dq[i];

			//PD[i] = 50*P[i]*err[i] + 50*D[i]*derr[i];

			PD[i] = pid_controllers[i].computeCommand(err[i], derr[i], period);
		}

		double 	s2 = sin(msg->position[1]), 								//sin(q2)
				s2_2 = sin(2.0*msg->position[1]),							//sin(2*q2)
				c2_2 = cos(2.0*msg->position[1]),							//cos(2*q2)
				c2 = cos(msg->position[1]),									//cos(q2)
				s3 = sin(msg->position[2]),									//sin(q3)
				c3 = cos(msg->position[2]),									//cos(q3)
				s23 = sin(msg->position[1] + msg->position[2]),				//sin(q2 + q3)
				c23 = cos(msg->position[1] + msg->position[2]),				//cos(q2 + q3)
				s23_23 = sin(2.0*(msg->position[1] + msg->position[2])), 	//sin(2*q2 + 2*q3)
				c23_2 = cos(2.0*msg->position[1] + msg->position[2]),		//cos(2*q2 + q3)
				s4 = sin(msg->position[3]),
				c4 = cos(msg->position[3]),
				s5 = sin(msg->position[4]),
				c5 = cos(msg->position[4]); 


// vector C for N = 3
		C[0] = 0.12418*dq[1]*dq[1]*s23 - 0.021135*dq[1]*dq[1]*s2 + 0.12418*dq[2]*dq[2]*s23 + 0.61451*dq[1]*dq[1]*c2 - 0.052884*dq[0]*dq[1]*s23_23 - 0.052884*dq[0]*dq[2]*s23_23 + 0.74714*dq[0]*dq[1]*c23_2 + 0.37357*dq[0]*dq[2]*c23_2 - 0.014198*dq[0]*dq[1]*c2_2 - 0.81386*dq[0]*dq[1]*s2_2 + 0.24837*dq[1]*dq[2]*s23 + 0.37357*dq[0]*dq[2]*c3;
		C[1] = 0.026442*dq[0]*dq[0]*s23_23 - 0.37357*dq[0]*dq[0]*c23_2 + 0.0070992*dq[0]*dq[0]*c2_2 + 0.40693*dq[0]*dq[0]*s2_2 + 0.37357*dq[2]*dq[2]*c3 + 0.74714*dq[1]*dq[2]*c3;
		C[2] = 0.026442*dq[0]*dq[0]*s23_23 - 0.18679*dq[0]*dq[0]*c23_2 - 0.18679*dq[0]*dq[0]*c3 - 0.37357*dq[1]*dq[1]*c3;

// diagonal terms of mass matrix M for N = 6
		double	M11, M22, M33, M44, M55, M66;

		double	c2_sqr = c2*c2,
				c3_sqr = c3*c3,
				c4_sqr = c4*c4,
				c5_sqr = c5*c5,
				c2_c3 = c2*c3,
				c2_c4 = c2*c4,
				c3_c4 = c3*c4,
				s5_s5 = s5*s5,
				s3_s5 = s3*s5,
				s2_s3 = s2*s3,
				s2_s5 = s2*s5;

		M11 = 0.0007462*s4*s5 - 0.007099*s2_2 + 1.013*c2_sqr + 0.1987*c3_sqr + 0.00001784*c5_sqr + 0.002538*c2_sqr*c5 + 0.002538*c3_sqr*c5 + 0.7524*c2_sqr*s3 - 0.3974*c2_sqr*c3_sqr + 0.0001178*c2_sqr*c4_sqr - 0.00001784*c2_sqr*c5_sqr + 0.0001178*c3_sqr*c4_sqr - 0.00001784*c3_sqr*c5_sqr + 0.002487*c2_sqr*c5*s3 - 0.005077*c2_sqr*c3_sqr*c5 + 0.7524*c2_c3*s2 - 0.0002357*c2_sqr*c3_sqr*c4_sqr + 0.00003568*c2_sqr*c3_sqr*c5_sqr - 0.00001784*c2_sqr*c4_sqr*c5_sqr - 0.00001784*c3_sqr*c4_sqr*c5_sqr + 0.002487*c2_sqr*c3_c4*s5 + 0.00003568*c2_sqr*c3_sqr*c4_sqr*c5_sqr + 0.002487*c2_c3*c5*s2 + 0.3974*c2_c3*s2_s3 - 0.002538*c2_c4*s2_s5 - 0.002538*c3_c4*s3_s5 + 0.0002357*c2_c3*c4_sqr*s2_s3 - 0.00003568*c2_c3*c5_sqr*s2_s3 + 0.005077*c2*c3_sqr*c4*s2_s5 + 0.005077*c2_sqr*c3_c4*s3_s5 + 0.005077*c2_c3*c5*s2_s3 + 0.00003568*c2_c4*c5*s2_s5 + 0.00003568*c3_c4*c5*s3_s5 - 0.002487*c2_c4*s2*s3_s5 - 0.00003568*c2_c3*c4_sqr*c5_sqr*s2_s3 - 0.00007136*c2*c3_sqr*c4*c5*s2_s5 - 0.00007136*c2_sqr*c3_c4*c5*s3_s5 + 2.053;
		M22 = 0.002538*c5 + 0.7524*s3 + 0.002487*c5*s3 - 0.0001178*c4_sqr - 0.00001784*c5_sqr + 0.00001784*c4_sqr*c5_sqr + 0.002487*c3_c4*s5 + 2.199;
		M33 = 0.002538*c5 - 0.0001178*c4_sqr - 0.00001784*c5_sqr + 0.00001784*c4_sqr*c5_sqr + 0.3354;
		M44 = 0.0046 - 0.00001784*s5*s5;
		M55 = 0.0008822;
		M66 = 0.0003;

		M_ddq[0] = M11*(PD[0]+ddqd[0]);
		M_ddq[1] = M22*(PD[1]+ddqd[1]);
		M_ddq[2] = M33*(PD[2]+ddqd[2]);
		M_ddq[3] = M44*(PD[3]+ddqd[3]);
		M_ddq[4] = M55*(PD[4]+ddqd[4]);
		M_ddq[5] = M66*(PD[5]+ddqd[5]);

// gravity vector G for N = 6
		// G[1] = 1.02416*s2 - 37.2347*c2 - 8.54702*s23 - 0.0282528*s23*cos(msg->position[4]) - 0.0282528*c23*cos(msg->position[3])*sin(msg->position[4]);
		// G[2] = -8.54702*s23 - 0.0282528*s23*cos(msg->position[4]) - 0.0282528*c23*cos(msg->position[3])*sin(msg->position[4]);
		// G[3] = 0.0282528*c23*sin(msg->position[3])*sin(msg->position[4]);
		// G[4] = -0.0282528*c23*sin(msg->position[4]) - 0.0282528*s23*cos(msg->position[3])*cos(msg->position[4]);

// gravity vector G for N = 3
		G[1] = 1.02416*s2 - 37.2347*c2 - 8.48712*s23;
		G[2] = - 8.48712*s23;



// compute command torques
		for(unsigned int i=0; i<n_joints; i++)
		{

//full
			tau[i].data = M_ddq[i] + G[i] + C[i];   

// no C
			//tau[i].data = M_ddq[i] + G[i];					

//no M		
			//tau[i].data = PD[i] + G[i] + C[i];   

//for  N = 3, no M, no C 
			//tau[i].data = PD[i] + G[i];		

//just PID
			//tau[i].data = PD[i];		

		}

		joint_1_pub_.publish(tau[0]);
		joint_2_pub_.publish(tau[1]);
		joint_3_pub_.publish(tau[2]);
		joint_4_pub_.publish(tau[3]);
		joint_5_pub_.publish(tau[4]);
		joint_6_pub_.publish(tau[5]);

		// double delta2 = ros::Time::now().toNSec() - time_last;

		// if(delta2>max_delta){
		// 	max_delta = delta2;
		// 	if(max_delta>=delta){
		// 		max_delta = 0;
		// 	}
		// } 

		// info.data[0] = (float)delta2;
		// info.data[1] = (float)max_delta;
		// info.data[2] = 0;
		// info_pub_.publish(info);

		//ROS_INFO("delta: %f, max delta: %f", delta2,max_delta);

	}

	public: 	
		
	PUMA_Controller()
	{

		time_last= ros::Time::now().toSec();

		joint_states_q_sub = n.subscribe("/puma01/joint_states",1,&PUMA_Controller::JointStatesObserve,this);
		cmd_sub = n.subscribe("/puma01/cmd_trajectory_point",1,&PUMA_Controller::CommandReceive,this);

		joint_1_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_1_effort_controller/command",1);
		joint_2_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_2_effort_controller/command",1);
		joint_3_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_3_effort_controller/command",1);
		joint_4_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_4_effort_controller/command",1);
		joint_5_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_5_effort_controller/command",1);
		joint_6_pub_ = n.advertise<std_msgs::Float64>("/puma01/joint_6_effort_controller/command",1);

		info.data.resize(3);
		info_pub_ = n.advertise<std_msgs::Float64MultiArray>("/puma01/info",1);

		unsigned int i_p_1 = 0;
		for(unsigned int i=0; i<n_joints; i++)
		{
			i_p_1 = i+1;
			if (!pid_controllers[i].init(ros::NodeHandle(n, std::string("/puma01/joint_") + std::to_string(i_p_1) + std::string("_effort_controller/pid")))) 
			{  
				ROS_ERROR_STREAM("Failed to load PID parameters!");
			}else{
				ROS_INFO("Got PID gains!");
			}
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PUMA_Controller");

	PUMA_Controller puma_controller;   

	ROS_INFO("PUMA controller is running!");

	ros::spin();  
  
	return 0;
 }