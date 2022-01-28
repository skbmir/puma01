
#include <puma01_hardware_interface/puma01_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace puma01_hw_interface_ns
{
puma01HWInterface::puma01HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
 	 : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), name_("puma01_hw_interface"), force_controller_ac_("force_controller", true)
{
	// for simulation
	sim_joint_states_sub_ = nh.subscribe("/puma01_sim/joint_states",1,&puma01HWInterface::SimJointStatesCB,this);
	wrench_command_sub_ = nh.subscribe("/puma01/wrench_command",1,&puma01HWInterface::wrench_command_CB,this);
	sim_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/puma01_sim/sim_computed_torque_controller/command",1);

	init();
}

void puma01HWInterface::init()
{
	// Call parent class version of this function
	// ros_control_boilerplate::GenericHWInterface::init();

	// OR do:
	/*
		- num_joints <- joint_names.size()
		- resize telemetry vectors of variables (pos, vel, maybe eff)
		- resize vectors of commands (pos, vel, acc)
		- resize vectors of joint limits
		- for joint id do: ...
			- JointStateInterface: register JointStateHandle
			- joint handle: define PosVelAccJointHandle
			- PosVelAccJointInterface: register PosVelAccJointHandle
			- register joint limits
		- register joint interfaces (JointStateInterface and PosVelAccJointInterface)
	*/

	num_joints_ = joint_names_.size();  // std::size_t

	initJointInterfaces(); // call it after num_joints_ has been defined!

	wrench_command_.force.x = 0.0;
	wrench_command_.force.y = 0.0;
	wrench_command_.force.z = 0.0;
	wrench_command_.torque.x = 0.0;
	wrench_command_.torque.y = 0.0;
	wrench_command_.torque.z = 0.0;

	double as_wait_timeout = 5.0;

	ROS_WARN_NAMED(name_, "Waiting for force controller action server to be started in %d secs...",(int)as_wait_timeout);
	force_controller_ac_.waitForServer(ros::Duration(as_wait_timeout)); // waiting for force controller action server to start
	if(force_controller_ac_.isServerConnected())
	{
		ROS_INFO_NAMED(name_, "Connection with force controller established.");
	}else{
		ROS_ERROR_NAMED(name_, "No force_controller with action interface found! Ignoring.");
	}

}

void puma01HWInterface::wrench_command_CB(const geometry_msgs::Wrench& wrench)
{ 
	// define Goal for force controller
	wrench_command_.force.x = wrench.force.x;
	wrench_command_.force.y = wrench.force.y;
	wrench_command_.force.z = wrench.force.z;
	wrench_command_.torque.x = wrench.torque.x;
	wrench_command_.torque.y = wrench.torque.y;
	wrench_command_.torque.z = wrench.torque.z;

	ROS_INFO_NAMED(name_, "Got new wrench command.");
	
}

void puma01HWInterface::read(ros::Duration& elapsed_time)
{
	force_controller_goal_.current_joint_angles.data = joint_position_; // actual joint positions
	force_controller_goal_.desired_wrench = wrench_command_; // desired wrench

	force_controller_ac_.sendGoal(force_controller_goal_, boost::bind(&puma01_hw_interface_ns::puma01HWInterface::force_controller_ac_DoneCB, this, _1, _2));

}

void puma01HWInterface::write(ros::Duration& elapsed_time)
{
	// Safety !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// enforceLimits(elapsed_time);	

	for(std::size_t i=0; i<num_joints_; i++)
	{
		traj_cmd_.data[i] = joint_position_command_[i];
		traj_cmd_.data[i+num_joints_] = joint_velocity_command_[i];
		traj_cmd_.data[i+traj_cmd_acc_offset_] = joint_acceleration_command_[i];
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
	joint_effort_.resize(num_joints_, 0.0); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// Command
	joint_position_command_.resize(num_joints_, 0.0);
	joint_velocity_command_.resize(num_joints_, 0.0);
	joint_acceleration_command_.resize(num_joints_, 0.0);
	joint_effort_command_.resize(num_joints_, 0.0); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

void puma01HWInterface::force_controller_ac_DoneCB(const actionlib::SimpleClientGoalState &state, const force_test::ForceControlResultConstPtr &result)
{
	for(std::size_t i=0; i<num_joints_; i++)
	{
		joint_effort_command_[i] = result->output_torques.data[i];			
	}		
}

}  // namespace 
