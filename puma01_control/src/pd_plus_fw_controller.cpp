#include <math.h>
#include <angles/angles.h>
#include <puma01_control/pd_plus_fw_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace puma01_controllers
{

  PDplusFWController::PDplusFWController() {}
  PDplusFWController::~PDplusFWController() {sub_command_.shutdown();}

// init
  bool PDplusFWController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {

//****************************************************************************************************************************
//              inherits from JointGroupPositionController
//****************************************************************************************************************************
	// List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }

    n_joints_ = joint_names_.size(); // mention the passive joint at the end-effector!!

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

	// Get URDF
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", n))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(n_joints_);

    for(size_t i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);

	    // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_name + "/pid")))
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_name + "/pid");
        return false;
      }

    }
    // commands_buffer_.writeFromNonRT(std::array<double,18>({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0})); 
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &PDplusFWController::commandCB, this);

//****************************************************************************************************************************

  // init desired values
    q_desired_ = {0,0,0,0,0,0};
    dq_desired_ = {0,0,0,0,0,0};
    tau_compensate_ = {0,0,0,0,0,0};

  // init cycle period value
    cycle_period_ = 0.001;

  // info topic
    info_pub_ = n.advertise<std_msgs::Float64MultiArray>("/error_info",1);

    info_msg_.data.resize(2);

    return true;
  }

// starting 
  void PDplusFWController::starting(const ros::Time& time)
  {
    //std::array<double,18> initial_commands({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      // initial_commands[i] = joints_[i].getPosition();
      // enforceJointLimits(initial_commands[i], i);
      pid_controllers_[i].reset();
    }
    // commands_buffer_.initRT(initial_commands);
  }

// update
  void PDplusFWController::update(const ros::Time& time, const ros::Duration& period)
  {

    double time_last= time.now().toSec();

		std::array<double, 6> 	err = {0, 0, 0, 0, 0, 0},
                            derr = {0, 0, 0, 0, 0, 0},
                            PD = {0, 0, 0, 0, 0, 0};
        
    double cmd_effort;

	// computing velocities dq and PID terms
		for(size_t i=0; i<n_joints_; i++)
		{

		// position error
			angles::shortest_angular_distance_with_large_limits(
				joints_[i].getPosition(),
				q_desired_[i],
        joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper,
				err[i]);

			derr[i] = dq_desired_[i] - joints_[i].getVelocity();

			PD[i] = pid_controllers_[i].computeCommand(err[i], derr[i], period);

    }

		for(size_t i=0; i<n_joints_; i++)
		{

			cmd_effort = PD[i] + tau_compensate_[i];  

      enforceEffLimits(cmd_effort, i); 

      joints_[i].setCommand(cmd_effort);

		}

    cycle_period_ = time.now().toSec() - time_last;

    // std_msgs::Float64 delta_period;
    // delta_period.data = cycle_period_;
    // info_msg_.data[0] = PD[1]+ddq_desired_[1];
    // info_msg_.data[1] = M_ddq[1];
    // info_pub_.publish(info_msg_);    
    
  }

// commandCB
  void PDplusFWController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
		if(msg->data.size()!=n_joints_*4)
		{
			ROS_ERROR_STREAM("Command error...");
			return;
		}
		for(size_t i=0; i<n_joints_; i++)
		{
			q_desired_[i] = (double)msg->data[i];    
			dq_desired_[i] = (double)msg->data[i+n_joints_]; 				
      tau_compensate_[i] = (double)msg->data[i+3*n_joints_]; 

      enforcePosLimits(q_desired_[i],i);
      enforceVelLimits(dq_desired_[i],i);

		}
    // commands_buffer_.writeFromNonRT(msg->data);
  }

// enforce position command limits
  void PDplusFWController::enforcePosLimits(double &pos, size_t index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( pos > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        pos = joint_urdfs_[index]->limits->upper;
      }
      else if( pos < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        pos = joint_urdfs_[index]->limits->lower;
      }
    }
  }

// enforce velocity command limits
  void PDplusFWController::enforceVelLimits(double &vel, size_t index)
  {
    double vel_limit = joint_urdfs_[index]->limits->velocity - 0.3;
    // check if velocity command transcedent joint limits
    if( vel > vel_limit) // above upper limit
    {
      vel = vel_limit;
    }else if( vel < -vel_limit){
      vel = -vel_limit;
    }
  }

// enforce effort command limits
  void PDplusFWController::enforceEffLimits(double &eff, size_t index)
  {

    double eff_limit = joint_urdfs_[index]->limits->effort - 10;
    // check if effort command transcedent joint limits
    if( eff > eff_limit) // above upper limit
    {
      eff = eff_limit;
    }else if( eff < -eff_limit){
      eff = -eff_limit;
    }
  }

} //namespace

PLUGINLIB_EXPORT_CLASS(puma01_controllers::PDplusFWController, controller_interface::ControllerBase)