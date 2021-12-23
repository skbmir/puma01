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

#include <chain.hpp>
#include <tree.hpp>
#include <chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace puma01_controllers
{


class ComputedTorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:
	ComputedTorqueController();
	~ComputedTorqueController();

	bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
	void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
	void starting(const ros::Time& /*time*/);

//****************************************************************************************************************************
//              inherits from JointGroupPositionController
//****************************************************************************************************************************
	std::vector< std::string > joint_names_;
	std::vector< hardware_interface::JointHandle > joints_;
	// realtime_tools::RealtimeBuffer<std::array<double,18> > commands_buffer_; //realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
	unsigned int n_joints_;
//****************************************************************************************************************************


private:

//****************************************************************************************************************************
//              inherits from JointGroupPositionController
//****************************************************************************************************************************
	ros::Subscriber sub_command_;

	std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

	std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

	void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
	void enforceJointLimits(double &command, unsigned int index);
//****************************************************************************************************************************

	std::array<double,6>	q_desired,dq_desired,ddq_desired;

	double cycle_period;

	ros::Publisher	info_pub_; 

	KDL::Chain robot_chain_;
	KDL::JntArray kdl_q_, kdl_dq_, kdl_ddq_, kdl_gravity_, kdl_coriolis_;
	KDL::JntSpaceInertiaMatrix kdl_mass_matrix_;
	KDL::Vector g_vector_;
	// KDL::ChainDynParam matrices_solver_;


}; //class


} //namespace