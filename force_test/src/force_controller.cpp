#include <force_test/ForceControlAction.h>  
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <control_toolbox/pid.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace puma01_controllers
{

class ForceController 
{

private:

    ros::NodeHandle nh_; // = ros::NodeHandle("force_controller");

    actionlib::SimpleActionServer<force_test::ForceControlAction> action_server_;
    force_test::ForceControlResult as_result_;

    // geometry_msgs::Wrench p_gains_,i_gains_; // using wrench structure to store gains is more intuitive

// structure to store cartesian gains for wrench control
    struct gains
    {
        struct force
        {
            struct x {double p = .0, i = .0;};
            struct y : x {};
            struct z : y {};
        };
        struct torque : force {};
    };

    void getTranspose(KDL::Jacobian &jaco, KDL::Jacobian &jaco_transpose)
    {
        for(unsigned int i=0; i<cartesian_parameters_names_.size(); i++)
        {
            jaco_transpose.setColumn(i, jaco.getColumn(i));
        }
    }

    std::vector<std::string> cartesian_parameters_names_;

// PID
    std::vector<control_toolbox::Pid> pid_controllers_;

    // geometry_msgs::WrenchStamped current_wrench_;
    // geometry_msgs::Wrench desired_wrench_; 
    std::vector<double> current_joint_angles_;
    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_;

    std::string action_name_;

// KDL
    KDL::JntArray kdl_current_joint_angles_, kdl_current_wrench_, kdl_tau_, kdl_last_wrench_, kdl_desired_wrench_, kdl_last_last_wrench_, kdl_wrench_error_;
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    KDL::Jacobian jacobian_;
    // KDL::ChainJntToJacSolver kdl_JacSolver_;

    ros::Publisher info_pub_;
    std_msgs::Float64MultiArray info_msg_;

    ros::Duration cycle_period_;
    double time_last_ = 0.0;


public:
    ForceController(std::string action_name) : action_name_(action_name), action_server_(nh_, action_name, boost::bind(&ForceController::executeCB, this, _1), false)
    {
        action_server_.start();
        ROS_INFO("Started %s Action server.", action_name_.c_str());
        init();
    }

    ~ForceController(){}

    bool init()
    {  

        // cartesian_parameters_names_.resize(6);
        cartesian_parameters_names_ = {"force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"};

    // get URDF from Parameter Server
        urdf::Model urdf;
        if (!urdf.initParamWithNodeHandle("robot_description", nh_))
        {
            ROS_ERROR("Failed to parse URDF file.");
            return false;
        }

        ROS_INFO("Loaded URDF.");

    // resizing current joint angles vector after getting the number of joints
        current_joint_angles_.resize(joints_number_,0.0);

    // init PID
        pid_controllers_.resize(cartesian_parameters_names_.size());

        for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        {
        // Load PID Controller using gains set on parameter server
            if (!pid_controllers_[i].init(ros::NodeHandle(nh_,  action_name_ + "/" + cartesian_parameters_names_[i] + "/pid")))
            {
                ROS_ERROR_STREAM("Failed to load PID parameters from " << action_name_ + "/" + cartesian_parameters_names_[i] + "/pid");
                return false;
            }
        }

        ROS_INFO("PID controllers initialised.");

        sensor_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/puma01_sim/ft_sensor", 1, &ForceController::getCurrentWrenchCB, this);

        as_result_.output_torques.data.resize(6, 0.0);

    // KDL tree from URDF

        if (!kdl_parser::treeFromUrdfModel(urdf, robot_tree_)){
            ROS_ERROR("Failed to construct KDL Tree");
            return false;
        }
        ROS_INFO("KDL Tree constructed from given URDF.");

    // KDL chain from KDL tree
        if(!robot_tree_.getChain("link0","link6",robot_chain_)){
            ROS_ERROR("Failed to get KDL chain");
            return false;
        }
        ROS_INFO("KDL Chain got from given KDL Tree.");
        
        // robot_chain_ = generateChain(); // generate chain from given D-H parameters

    // define KDL structures
        kdl_current_joint_angles_ = KDL::JntArray(6);
        kdl_current_wrench_ = KDL::JntArray(6);
        kdl_desired_wrench_ = KDL::JntArray(6);
        kdl_wrench_error_ = KDL::JntArray(6);
        kdl_tau_ = KDL::JntArray(6);
        kdl_last_wrench_ = KDL::JntArray(6);
        kdl_last_last_wrench_ = KDL::JntArray(6);
        jacobian_ = KDL::Jacobian(6);


        info_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/force_control_info",1);
        info_msg_.data.resize(7);

        ROS_INFO("force_controller initialized.");

        KDL::ChainJntToJacSolver kdl_JacSolver_ = KDL::ChainJntToJacSolver(robot_chain_);
    
        int solver_ret = kdl_JacSolver_.JntToJac(kdl_current_joint_angles_, jacobian_); // computing jacobian
        if(solver_ret!=0)
        {
            ROS_ERROR("Failed to get jacobian. Error code: %i",solver_ret);
        }

        for(unsigned int i=0; i<6; i++)
        {
            for(unsigned int j=0; j<6; j++)
            {
                if(fabs(jacobian_(5-i,j))<0.1)
                {
                    jacobian_(5-i,j) = 0;
                }
            }
            ROS_INFO("%i : %g %g %g %g %g %g",i,jacobian_(5-i,0),jacobian_(5-i,1),jacobian_(5-i,2),jacobian_(5-i,3),jacobian_(5-i,4),jacobian_(5-i,5));
        }

        return true;
    }

// taking goal from action client (hardware_interface in puma01) and performing control cycle action
    void executeCB(const force_test::ForceControlGoalConstPtr &as_goal)
    {
        bool succeed = true; // execution success mark
        std::array<double,6> PI;

    // get robot configuration
        // current_joint_angles_ = as_goal->current_joint_angles.data;
        for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        {
            // kdl_current_joint_angles_(i) = as_goal->current_joint_angles.data[i];
        }

    // get wrench command
        // desired_wrench_ = as_goal->desired_wrench;
        kdl_desired_wrench_(0) = as_goal->desired_wrench.force.x;
        kdl_desired_wrench_(1) = as_goal->desired_wrench.force.y;
        kdl_desired_wrench_(2) = as_goal->desired_wrench.force.z;
        kdl_desired_wrench_(3) = as_goal->desired_wrench.torque.x;
        kdl_desired_wrench_(4) = as_goal->desired_wrench.torque.y;
        kdl_desired_wrench_(5) = as_goal->desired_wrench.torque.z;

    // compute transpose jacobian
        KDL::ChainJntToJacSolver kdl_JacSolver_ = KDL::ChainJntToJacSolver(robot_chain_);
    
        int solver_ret = kdl_JacSolver_.JntToJac(kdl_current_joint_angles_, jacobian_); // computing jacobian
        if(solver_ret!=0)
        {
            succeed = false;
            ROS_ERROR("Failed to get jacobian. Error code: %i",solver_ret);
        }

        // KDL::MultiplyJacobian(jacobian_T_, kdl_current_wrench_, kdl_tau_);  // not usable???

    // computing tau = J^T * F
        double time_now = ros::Time::now().toSec();
        cycle_period_ = ros::Duration(time_now - time_last_);
        time_last_ = time_now; 

        for(unsigned int i=0; i<cartesian_parameters_names_.size(); i++)
        {
            kdl_wrench_error_(i) = kdl_desired_wrench_(i)-kdl_current_wrench_(i); // calculate wrench error
            PI[i] = pid_controllers_[i].computeCommand(kdl_wrench_error_(i), cycle_period_); // compute wrench PI output

            for(unsigned int j=0; j<kdl_current_joint_angles_.rows(); j++)
            {
                kdl_tau_(i)+=jacobian_(j,i)*PI[i];
            }

            // kdl_tau_(i) *= 0.2;

            info_msg_.data[i] = kdl_tau_(i);
            as_result_.output_torques.data[i] = 0; //kdl_tau_(i);
        }

        info_msg_.data[6] = kdl_current_wrench_(2);

        as_result_.header = as_goal->header;

        info_pub_.publish(info_msg_);

        if(succeed)
        {   
            // ROS_INFO("Sent result back.");
            action_server_.setSucceeded(as_result_);
        }else{
            action_server_.setAborted();
        }
    }

// getting current measurements from force sensor
    void getCurrentWrenchCB(const geometry_msgs::WrenchStampedConstPtr &sensor_msg)
    {
        // current_wrench_.header = sensor_msg->header;
        // current_wrench_.wrench = sensor_msg->wrench;

        kdl_current_wrench_(0) = sensor_msg->wrench.force.x;
        kdl_current_wrench_(1) = sensor_msg->wrench.force.y;
        kdl_current_wrench_(2) = sensor_msg->wrench.force.z;
        kdl_current_wrench_(3) = sensor_msg->wrench.torque.x;
        kdl_current_wrench_(4) = sensor_msg->wrench.torque.y;
        kdl_current_wrench_(5) = sensor_msg->wrench.torque.z;

    // filter wrench data
        for(unsigned int i = 0; i<6; i++)
        {
            kdl_current_wrench_(i) = fabs(kdl_current_wrench_(i))<0.003 ? 0 : kdl_current_wrench_(i);
            if(kdl_current_wrench_(i) != 0)
            {
                kdl_current_wrench_(i) = (1*kdl_last_last_wrench_(i)+1*kdl_last_wrench_(i)+1*kdl_current_wrench_(i))/3;
                kdl_last_last_wrench_(i) = kdl_last_wrench_(i);
                kdl_last_wrench_(i) = kdl_current_wrench_(i);     
            }
        }
        
    }

    KDL::Chain generateChain()
    {

        double d2 = 0.21844;
        double a2 = 0.4318;
        double d4 = 0.0889;
        double l3 = 0.440669;

        KDL::Chain local_robot_chain;

        // joint 1
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, 0, 0)));
        // joint 2
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, -M_PI/2, d2, 0)));
        // joint 3
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, 0, -d4, 0)));
        // joint 4
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, l3, 0)));
        // joint 5
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, -M_PI/2, 0, 0)));
        // joint 6
        local_robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0, 0)));

        ROS_INFO("Generated chain of %i joints.", local_robot_chain.getNrOfJoints());

        return local_robot_chain;

    }


}; // class
} // namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_controller");

    puma01_controllers::ForceController force_controller(ros::this_node::getName());

    ros::spin();
    return 0;
} // main