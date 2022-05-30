#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <std_msgs/Float64.h>


namespace puma01_controllers
{


class PDplusFWController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:
	PDplusFWController();
	~PDplusFWController();

	bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
	void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
	void starting(const ros::Time& /*time*/);

//****************************************************************************************************************************
//              inherits from JointGroupPositionController
//****************************************************************************************************************************
	std::vector< std::string > joint_names_;
	std::vector< hardware_interface::JointHandle > joints_;
	// realtime_tools::RealtimeBuffer<std::array<double,18> > commands_buffer_; //realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
	size_t n_joints_;
//****************************************************************************************************************************


private:

//****************************************************************************************************************************
//              inherits from JointGroupPositionController
//****************************************************************************************************************************
	ros::Subscriber sub_command_;

	std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

	std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

	void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
	void enforcePosLimits(double &pos, size_t index);
	void enforceEffLimits(double &eff, size_t index);
	void enforceVelLimits(double &vel, size_t index);
//****************************************************************************************************************************

	std::array<double,6>	q_desired_, dq_desired_, tau_compensate_;

	double cycle_period_;

	ros::Publisher	info_pub_; 

	std_msgs::Float64MultiArray info_msg_;


}; //class


} //namespace