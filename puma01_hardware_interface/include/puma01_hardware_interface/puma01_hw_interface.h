#pragma once

#include <ros_control_boilerplate/generic_hw_interface.h>

#include <hardware_interface/posvelacc_command_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

namespace puma01_hw_interface_ns
{
class puma01HWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:

  puma01HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  virtual void init();

  virtual void read(ros::Duration& elapsed_time);

  virtual void write(ros::Duration& elapsed_time);

  virtual void enforceLimits(ros::Duration& period);

  void SimJointStatesCB(const sensor_msgs::JointState::ConstPtr& msg);

protected:

  // Name of this class
  std::string name_;

  std::vector<double> joint_acceleration_command_;
  hardware_interface::PosVelAccJointInterface posvelacc_joint_interface_;
  ros::Subscriber sim_joint_states_sub_; 
	ros::Publisher sim_cmd_pub_;
  unsigned int traj_cmd_full_size_, traj_cmd_acc_num_;
  std_msgs::Float64MultiArray traj_cmd_;

};  // class

}  // namespace 

