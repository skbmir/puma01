#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <hardware_interface/posvelacc_command_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <force_test/ForceControlAction.h>  

namespace puma01_hw_interface_ns
{

typedef actionlib::SimpleActionClient<force_test::ForceControlAction> ForceControllerActionClient;

class puma01HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:

	puma01HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

	virtual void init();

	virtual void read(ros::Duration& elapsed_time);

	virtual void write(ros::Duration& elapsed_time);

	virtual void enforceLimits(ros::Duration& period);

	void initJointInterfaces();

	// callback for '/joint_states' in simulation
	void SimJointStatesCB(const sensor_msgs::JointState::ConstPtr& msg); 
	
	// force controller action client Done callback - for force controller Result receiving
	void force_controller_ac_DoneCB(const actionlib::SimpleClientGoalState &state, const force_test::ForceControlResultConstPtr &result); 

protected:

  // Name of this class
	std::string name_;

	std::vector<double> joint_acceleration_command_; // acceleration commands vector

	hardware_interface::PosVelAccJointInterface posvelacc_joint_interface_; // position-velocity-acceleration joint interface

	ros::Subscriber sim_joint_states_sub_; // subscriber for joint_states in simulation
	ros::Publisher sim_cmd_pub_; // publisher for publishing command to simulation

	unsigned int traj_cmd_full_size_,	// number of command vector elements: pos, vel, acc, tau
				 traj_cmd_acc_num_, 	// offset for acceleration commands in command vector
				 traj_cmd_tau_num_;		// offset for torque commands in command vector

	std_msgs::Float64MultiArray traj_cmd_; // trajectory command array

	geometry_msgs::Wrench wrench_command_; // wrench command for force controller

	ForceControllerActionClient force_controller_ac_; // action client for force_controller
	force_test::ForceControlGoal force_controller_goal_;

};  // class

}  // namespace 

