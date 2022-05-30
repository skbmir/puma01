#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <hardware_interface/posvelacc_command_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <puma01_force/ForceControlAction.h>  

#include <control_msgs/JointControllerState.h>

#include <vscan_usbcan_api/usbcan.h>
#include <vscan_usbcan_api/puma_parameters.h>

namespace puma01_hwi_ns
{

typedef actionlib::SimpleActionClient<puma01_force::ForceControlAction> ForceControllerActionClient;

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
	void force_controller_ac_DoneCB(const actionlib::SimpleClientGoalState& state, const puma01_force::ForceControlResultConstPtr& result); 

	void wrench_command_CB(const geometry_msgs::Wrench& wrench);

	void getNewPidCB(const control_msgs::JointControllerStateConstPtr& new_pids_msg);

	void initCAN();

	void forwardDynamics();

protected:

  // Name of this class
	std::string name_;

	std::vector<double> joint_acceleration_command_; // acceleration commands vector

	hardware_interface::PosVelAccJointInterface posvelacc_joint_interface_; // position-velocity-acceleration joint interface

	ros::Subscriber sim_joint_states_sub_,  // subscriber for joint_states in simulation
					wrench_command_sub_,
					new_pid_sub_;

	ros::Publisher sim_cmd_pub_; // publisher for publishing command to simulation

	unsigned int traj_cmd_full_size_,	// number of command vector elements: pos, vel, acc, tau
				 traj_cmd_acc_offset_; 	// offset for acceleration commands in command vector

	std_msgs::Float64MultiArray traj_cmd_; // trajectory command array

	geometry_msgs::Wrench wrench_command_; // wrench command for force controller

	ForceControllerActionClient force_controller_ac_; // action client for force_controller
	puma01_force::ForceControlGoal force_controller_goal_;

	bool force_control_ac_connected_;

	std::array<double,6> force_control_position_corr_;
 
	std::array<float,3> p_gains_, i_gains_, d_gains_; 

	vscan_api::usbcan_handle	usbcan_handle_; 

	std::array<VSCAN_MSG,3>		new_pid_p_vscan_msgs_, new_pid_i_vscan_msgs_, new_pid_d_vscan_msgs_,
								joint_pos_cmd_vscan_msgs_, joint_vel_cmd_vscan_msgs_;

	std::array<uint32_t,3>	drv_code_ids_, drv_feedback_ids_;
	std::array<double,3> 	enc_to_joint_consts_;
	std::array<int16_t,3>	int_pos_cmds_, int_vel_cmds_;
	std::array<int16_t,6> 	int_traj_cmds_;

	VSCAN_MSG 	heartbeat_frame_, pos_cmd_vscan_msg_, vel_cmd_vscan_msg_;

	std::vector<VSCAN_MSG> read_buffer_;
	int read_buff_size_;

};  // class

}  // namespace 

