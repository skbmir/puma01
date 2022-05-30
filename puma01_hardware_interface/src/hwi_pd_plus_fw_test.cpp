
#include <puma01_hardware_interface/puma01_hwi.h>

namespace puma01_hwi_ns
{
puma01HWInterface::puma01HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
 	 : 	ros_control_boilerplate::GenericHWInterface(nh, urdf_model), 
	  	name_("puma01_hardware_interface"), 
		force_controller_ac_("force_controller", true),
		force_control_ac_connected_(false)
{
	// for simulation
	sim_joint_states_sub_ = nh.subscribe("/puma01_sim/joint_states",1,&puma01HWInterface::SimJointStatesCB,this);
	sim_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/puma01_sim/sim_pd_plus_fw_controller/command",1);

	init();
}

void puma01HWInterface::init()
{

	num_joints_ = joint_names_.size();  // std::size_t

	initJointInterfaces(); // call it after num_joints_ has been defined!

}

void puma01HWInterface::wrench_command_CB(const geometry_msgs::Wrench& wrench)
{ 
	ROS_ERROR_NAMED(name_, "This HWI doesn't support force_controller!");
}

void puma01HWInterface::read(ros::Duration& elapsed_time)
{

}

void puma01HWInterface::write(ros::Duration& elapsed_time)
{
	// Safety !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// enforceLimits(elapsed_time);	

	forwardDynamics(); // calculate joint_effort_command_, dynamic compensation torques

	for(std::size_t i=0; i<num_joints_; i++)
	{
		traj_cmd_.data[i] = joint_position_command_[i];
		traj_cmd_.data[i+num_joints_] = joint_velocity_command_[i];
		// traj_cmd_.data[i+traj_cmd_acc_offset_] = joint_acceleration_command_[i];
		traj_cmd_.data[i+traj_cmd_acc_offset_+num_joints_] = joint_effort_command_[i];
	}

	sim_cmd_pub_.publish(traj_cmd_);

}

void puma01HWInterface::enforceLimits(ros::Duration& period)
{
	// Enforces position and velocity
	// pos_jnt_sat_interface_.enforceLimits(period);
}

void puma01HWInterface::initJointInterfaces()
{
	// resize sim cmd vector
	traj_cmd_full_size_ = num_joints_*4;
	traj_cmd_acc_offset_ = num_joints_*2;
	traj_cmd_.data.resize(traj_cmd_full_size_, 0.0);

	// Status
	joint_position_.resize(num_joints_, 0.0);
	joint_velocity_.resize(num_joints_, 0.0);
	joint_effort_.resize(num_joints_, 0.0); 

	// Command
	joint_position_command_.resize(num_joints_, 0.0);
	joint_velocity_command_.resize(num_joints_, 0.0);
	joint_acceleration_command_.resize(num_joints_, 0.0);
	joint_effort_command_.resize(num_joints_, 0.0); 

	// Limits
	joint_position_lower_limits_.resize(num_joints_, 0.0);
	joint_position_upper_limits_.resize(num_joints_, 0.0);
	joint_velocity_limits_.resize(num_joints_, 0.0);
	joint_effort_limits_.resize(num_joints_, 0.0);

	// Initialize interfaces for each joint
	for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
	{
		ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

		// Create joint state interface
		joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
			joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

		// Add command interfaces to joints
		hardware_interface::PosVelAccJointHandle joint_handle_posvelacc = hardware_interface::PosVelAccJointHandle(
			joint_state_interface_.getHandle(joint_names_[joint_id]), 
			&joint_position_command_[joint_id],
			&joint_velocity_command_[joint_id],
			&joint_acceleration_command_[joint_id]);

		posvelacc_joint_interface_.registerHandle(joint_handle_posvelacc);

		// Load the joint limits !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id); 

	}  // end for each joint

	registerInterface(&joint_state_interface_);     // From RobotHW base class.
	registerInterface(&posvelacc_joint_interface_);    // From RobotHW base class.

}

void puma01HWInterface::SimJointStatesCB(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(std::size_t i=0; i<num_joints_; i++)
	{
		joint_position_[i] = msg->position[i];	
		joint_velocity_[i] = msg->velocity[i];				
	}
}

// calculate forward dynamics for desired trajectory ON TIME
void puma01HWInterface::forwardDynamics()
{
	double 	s2 = sin(joint_position_command_[1]), //sin(q2)
			s2_2 = sin(2*joint_position_command_[1]), //sin(2*q2)
			c2_2 = cos(2*joint_position_command_[1]), //cos(2*q2)
			c2 = cos(joint_position_command_[1]), //cos(q2)
			s3 = sin(joint_position_command_[2]), //sin(q3)
			c3 = cos(joint_position_command_[2]), //cos(q3)
			s23 = sin(joint_position_command_[1] + joint_position_command_[2]), //sin(q2 + q3)
			c23 = cos(joint_position_command_[1] + joint_position_command_[2]), //cos(q2 + q3)
			s23_23 = sin(2*(joint_position_command_[1] + joint_position_command_[2])),  //sin(2*q2 + 2*q3)
			c23_2 = cos(2*joint_position_command_[1] + joint_position_command_[2]), //cos(2*q2 + q3)
			s23_2 = sin(2*joint_position_command_[1] + joint_position_command_[2]), //sin(2*q2 + q3)
			c23_23 = cos(2*(joint_position_command_[1] + joint_position_command_[2])),  //cos(2*q2 + 2*q3)
			c2_2c = cos(2*joint_position_command_[1] + 0.017444);

	std::array<double, 3>	G_vector = {0, 0, 0},  // MAGIC number! compute only for 3 first joints!
							C_vector = {0, 0, 0},
							M_ddq_vector = {0, 0, 0};

	std::array<double, 9>  M_matrix = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// mass matrix M (n = 3)
    M_matrix[0] = 0.37357*c3 - 0.37357*c23_2 - 0.40699*c2_2c + 0.026442*c23_23 + 2.6859; //M11
    M_matrix[1] = 0.021135*s2 - 0.61451*c2 - 0.12418*c2*c3 + 0.12418*s2*s3; //M12
    M_matrix[2] = -0.12418*c23; //M13

    M_matrix[4] = 0.74714*c3 + 2.1942; //M22
    M_matrix[5] = 0.37357*c3 + 0.33112; //M23

    M_matrix[8] = 0.33112; //M33

    M_ddq_vector[0] = M_matrix[0]*joint_acceleration_command_[0] + M_matrix[1]*joint_acceleration_command_[1] + M_matrix[2]*joint_acceleration_command_[2];
	M_ddq_vector[1] = M_matrix[1]*joint_acceleration_command_[0] + M_matrix[4]*joint_acceleration_command_[1] + M_matrix[5]*joint_acceleration_command_[2];
	M_ddq_vector[2] = M_matrix[2]*joint_acceleration_command_[0] + M_matrix[5]*joint_acceleration_command_[1] + M_matrix[8]*joint_acceleration_command_[2]; 

// gravity vector G 
    G_vector[1] = - 8.48712*s23 - 1.02416*c2 - 37.2347*s2;
    G_vector[2] = - 8.48712*s23;

// centrifugal and coriolis forces vector C
    C_vector[0] = 0.61451*joint_velocity_command_[1]*joint_velocity_command_[1]*s2 + 0.12418*joint_velocity_command_[1]*joint_velocity_command_[1]*s23 + 0.12418*joint_velocity_command_[2]*joint_velocity_command_[2]*s23 + 0.021135*joint_velocity_command_[1]*joint_velocity_command_[1]*c2 - 0.37357*joint_velocity_command_[0]*joint_velocity_command_[2]*s2 - 0.052884*joint_velocity_command_[0]*joint_velocity_command_[1]*s23_23 - 0.052884*joint_velocity_command_[0]*joint_velocity_command_[2]*s23_23 + 0.74714*joint_velocity_command_[0]*joint_velocity_command_[1]*s23_2 + 0.37357*joint_velocity_command_[0]*joint_velocity_command_[2]*s23_2 + 0.014198*joint_velocity_command_[0]*joint_velocity_command_[1]*c2_2 + 0.81386*joint_velocity_command_[0]*joint_velocity_command_[1]*s2_2 + 0.24837*joint_velocity_command_[1]*joint_velocity_command_[2]*s23;
    C_vector[1] = 0.026442*joint_velocity_command_[0]*joint_velocity_command_[0]*s23_23 - 0.37357*joint_velocity_command_[2]*joint_velocity_command_[2]*s2 - 0.37357*joint_velocity_command_[0]*joint_velocity_command_[0]*s23_2 - 0.0070992*joint_velocity_command_[0]*joint_velocity_command_[0]*c2_2 - 0.40693*joint_velocity_command_[0]*joint_velocity_command_[0]*s2_2 - 0.74714*joint_velocity_command_[1]*joint_velocity_command_[2]*s2;
    C_vector[2] = 0.18679*joint_velocity_command_[0]*joint_velocity_command_[0]*s2 + 0.37357*joint_velocity_command_[1]*joint_velocity_command_[1]*s2 + 0.026442*joint_velocity_command_[0]*joint_velocity_command_[0]*s23_23 - 0.18679*joint_velocity_command_[0]*joint_velocity_command_[0]*s23_2;

	for(size_t i=0; i<3; i++)
	{
		joint_effort_command_[i] = M_ddq_vector[i] + G_vector[i] + C_vector[i];  
	}

}

}  // namespace 
