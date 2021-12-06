#include <math.h>
#include <angles/angles.h>
#include <puma01_control/computed_torque_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace puma01_controllers
{

  ComputedTorqueController::ComputedTorqueController() {}
  ComputedTorqueController::~ComputedTorqueController() {sub_command_.shutdown();}

// init
  bool ComputedTorqueController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
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

    n_joints_ = joint_names_.size();

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

    for(unsigned int i=0; i<n_joints_; i++)
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
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ComputedTorqueController::commandCB, this);

//****************************************************************************************************************************

  // init desired values
    q_desired = {0,0,0,0,0,0};
    dq_desired = {0,0,0,0,0,0};
    ddq_desired = {0,0,0,0,0,0};

  // init cycle period value
    cycle_period = 0.001;

  // info topic
    info_pub_ = n.advertise<std_msgs::Float64>("/error_info",1);

    // KDL tree from URDF
    KDL::Tree robot_tree;

    if (!kdl_parser::treeFromUrdfModel(urdf, robot_tree)){
      ROS_ERROR("Failed to construct kdl tree");
    //return false;
    }

    // KDL chain from KDL tree
    if(!robot_tree.getChain("world","link_6",robot_chain_)){
      ROS_ERROR("Failed to get kdl chain");
    }

    // defining JntArray for inverse dynamics computing
    kdl_q_ = KDL::JntArray(robot_chain_.getNrOfJoints());
    kdl_dq_ = KDL::JntArray(robot_chain_.getNrOfJoints());
    kdl_ddq_ = KDL::JntArray(robot_chain_.getNrOfJoints());
    kdl_gravity_ = KDL::JntArray(robot_chain_.getNrOfJoints());
    kdl_coriolis_ = KDL::JntArray(robot_chain_.getNrOfJoints());
    kdl_mass_matrix_ = KDL::JntSpaceInertiaMatrix(robot_chain_.getNrOfJoints());

    kdl_mass_matrix_ = KDL::JntSpaceInertiaMatrix(6);
    g_vector_ = KDL::Vector(0, 0, -9.82);

    return true;
  }

// starting 
  void ComputedTorqueController::starting(const ros::Time& time)
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
  void ComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
  {

    double time_last= time.now().toSec();

    // // defining inverse dynamics solver
    KDL::ChainDynParam dynamics_solver(robot_chain_,g_vector_);

		std::array<double, 6> 	err = {0, 0, 0, 0, 0, 0},
                            derr = {0, 0, 0, 0, 0, 0},
                            G = {0, 0, 0, 0, 0, 0},
                            C = {0, 0, 0, 0, 0, 0},
                            PD = {0, 0, 0, 0, 0, 0},
                            M_ddq = {0, 0, 0, 0, 0, 0};

    std::array<double, 9>  M = {0, 0, 0, 0, 0, 0, 0, 0, 0};

		double 	s2 = sin(joints_[1].getPosition()), //sin(q2)
            s2_2 = sin(2.0*joints_[1].getPosition()), //sin(2*q2)
            c2_2 = cos(2.0*joints_[1].getPosition()), //cos(2*q2)
            c2 = cos(joints_[1].getPosition()), //cos(q2)
            s3 = sin(joints_[2].getPosition()), //sin(q3)
            c3 = cos(joints_[2].getPosition()), //cos(q3)
            s23 = sin(joints_[1].getPosition() + joints_[2].getPosition()), //sin(q2 + q3)
            c23 = cos(joints_[1].getPosition() + joints_[2].getPosition()), //cos(q2 + q3)
            s23_23 = sin(2.0*(joints_[1].getPosition() + joints_[2].getPosition())),  //sin(2*q2 + 2*q3)
            c23_2 = cos(2.0*joints_[1].getPosition() + joints_[2].getPosition()); //cos(2*q2 + q3)
    
    // double 	s2 = sin(q_desired[1]), //sin(q2)
    //         s2_2 = sin(2.0*q_desired[1]), //sin(2*q2)
    //         c2_2 = cos(2.0*q_desired[1]), //cos(2*q2)
    //         c2 = cos(q_desired[1]), //cos(q2)
    //         s3 = sin(q_desired[2]), //sin(q3)
    //         c3 = cos(q_desired[2]), //cos(q3)
    //         s23 = sin(q_desired[1] + q_desired[2]), //sin(q2 + q3)
    //         c23 = cos(q_desired[1] + q_desired[2]), //cos(q2 + q3)
    //         s23_23 = sin(2.0*(q_desired[1] + q_desired[2])),  //sin(2*q2 + 2*q3)
    //         c23_2 = cos(2.0*q_desired[1] + q_desired[2]); //cos(2*q2 + q3)

    double cmd_effort;


  // vector C for N = 3
		C[0] = 0.12418*joints_[1].getVelocity()*joints_[1].getVelocity()*s23 - 0.021135*joints_[1].getVelocity()*joints_[1].getVelocity()*s2 + 0.12418*joints_[2].getVelocity()*joints_[2].getVelocity()*s23 + 0.61451*joints_[1].getVelocity()*joints_[1].getVelocity()*c2 - 0.052884*joints_[0].getVelocity()*joints_[1].getVelocity()*s23_23 - 0.052884*joints_[0].getVelocity()*joints_[2].getVelocity()*s23_23 + 0.74714*joints_[0].getVelocity()*joints_[1].getVelocity()*c23_2 + 0.37357*joints_[0].getVelocity()*joints_[2].getVelocity()*c23_2 - 0.014198*joints_[0].getVelocity()*joints_[1].getVelocity()*c2_2 - 0.81386*joints_[0].getVelocity()*joints_[1].getVelocity()*s2_2 + 0.24837*joints_[1].getVelocity()*joints_[2].getVelocity()*s23 + 0.37357*joints_[0].getVelocity()*joints_[2].getVelocity()*c3;
		C[1] = 0.026442*joints_[0].getVelocity()*joints_[0].getVelocity()*s23_23 - 0.37357*joints_[0].getVelocity()*joints_[0].getVelocity()*c23_2 + 0.0070992*joints_[0].getVelocity()*joints_[0].getVelocity()*c2_2 + 0.40693*joints_[0].getVelocity()*joints_[0].getVelocity()*s2_2 + 0.37357*joints_[2].getVelocity()*joints_[2].getVelocity()*c3 + 0.74714*joints_[1].getVelocity()*joints_[2].getVelocity()*c3;
		C[2] = 0.026442*joints_[0].getVelocity()*joints_[0].getVelocity()*s23_23 - 0.18679*joints_[0].getVelocity()*joints_[0].getVelocity()*c23_2 - 0.18679*joints_[0].getVelocity()*joints_[0].getVelocity()*c3 - 0.37357*joints_[1].getVelocity()*joints_[1].getVelocity()*c3;

  // mass matrix M for N = 3
		M[0] = 0.38049*c2_2 - 0.0070992*s2_2 + 0.37357*s3 + 0.052884*c2_2*c3*c2_2*c3 + 0.37357*c2_2*s3 + 0.37357*s2_2*c3 - 0.052884*s2_2*c3*s3 + 2.6859; //M11
		M[1] = 0.021135*c2 + 0.61451*s2 - 0.12418*c2*c3 + 0.12418*s2*s3;  //M12
		M[2] =	-0.12418*c23; //M13

    //M[3] = M[1]; //M21 = M12
		M[4] = 0.74714*s3 + 2.1942; //M22
		M[5] = 0.37357*s3 + 0.33112; //M23

    //M[6] = M[2];  //M31 = M13
    //M[7] = M[5];  //M32 = M23
		M[8] = 0.33112;  //M33

  // gravity vector G for N = 6
		// G[1] = 1.02416*s2 - 37.2347*c2 - 8.54702*s23 - 0.0282528*s23*cos(msg->position[4]) - 0.0282528*c23*cos(msg->position[3])*sin(msg->position[4]);
		// G[2] = -8.54702*s23 - 0.0282528*s23*cos(msg->position[4]) - 0.0282528*c23*cos(msg->position[3])*sin(msg->position[4]);
		// G[3] = 0.0282528*c23*sin(msg->position[3])*sin(msg->position[4]);
		// G[4] = -0.0282528*c23*sin(msg->position[4]) - 0.0282528*s23*cos(msg->position[3])*cos(msg->position[4]);

  // gravity vector G for N = 3
		G[1] = 1.02416*s2 - 37.2347*c2 - 8.48712*s23;
		G[2] = - 8.48712*s23;

	// computing velocities dq and PID terms
		for(unsigned int i=0; i<n_joints_; i++)
		{

		// position error
			angles::shortest_angular_distance_with_large_limits(
				joints_[i].getPosition(),
				q_desired[i],
        joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper,
				err[i]);

			derr[i] = dq_desired[i] - joints_[i].getVelocity();

			PD[i] = pid_controllers_[i].computeCommand(err[i], derr[i], period);

    }

  // computing M*PD
    M_ddq[0] = M[0]*(PD[0]+ddq_desired[0]) + M[1]*(PD[1]+ddq_desired[1]) + M[2]*(PD[2]+ddq_desired[2]);
		M_ddq[1] = M[1]*(PD[0]+ddq_desired[0]) + M[4]*(PD[1]+ddq_desired[1]) + M[5]*(PD[2]+ddq_desired[2]);
		M_ddq[2] = M[2]*(PD[0]+ddq_desired[0]) + M[5]*(PD[1]+ddq_desired[1]) + M[8]*(PD[2]+ddq_desired[2]); 
		M_ddq[3] = PD[3]+ddq_desired[3];
		M_ddq[4] = PD[4]+ddq_desired[4];
		M_ddq[5] = PD[5]+ddq_desired[5];

		for(unsigned int i=0; i<n_joints_; i++)
		{

      //M_ddq[i] = PD[i]+ddq_desired[i];

			cmd_effort = M_ddq[i] + G[i] + C[i];   

      joints_[i].setCommand(cmd_effort);

		}

    cycle_period = time.now().toSec() - time_last;

    std_msgs::Float64 delta_period;
    delta_period.data = cycle_period;
    info_pub_.publish(delta_period);    
    
  }

// commandCB
  void ComputedTorqueController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
		if(msg->data.size()!=n_joints_*3)
		{
			ROS_ERROR_STREAM("Command error...");
			return;
		}
		for(unsigned int i=0; i<n_joints_; i++)
		{
			q_desired[i] = (double)msg->data[i];    
			dq_desired[i] = (double)msg->data[i+n_joints_]; 		
			ddq_desired[i] = (double)msg->data[i+2*n_joints_]; 		

      enforceJointLimits(q_desired[i],i);

		}
    // commands_buffer_.writeFromNonRT(msg->data);
  }

// enforceJointLimits
  void ComputedTorqueController::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }


} //namespace

PLUGINLIB_EXPORT_CLASS(puma01_controllers::ComputedTorqueController, controller_interface::ControllerBase)