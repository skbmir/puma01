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

    geometry_msgs::WrenchStamped current_wrench_;
    geometry_msgs::Wrench desired_wrench_; 
    std::vector<double> current_joint_angles_;
    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_;

    std::string action_name_;

// KDL
    KDL::JntArray kdl_current_joint_angles_, kdl_current_wrench_;
    KDL::Twist kdl_tau_; // don't mind... it's for MultiplyJacobian() func
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    KDL::Jacobian jacobian_, jacobian_T_;
    // KDL::ChainJntToJacSolver kdl_JacSolver_;


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

    // define KDL structures
        kdl_current_joint_angles_ = KDL::JntArray(6);
        kdl_current_wrench_ = KDL::JntArray(6);
        jacobian_ = KDL::Jacobian(6);
        jacobian_T_ = KDL::Jacobian(6);

        ROS_INFO("force_controller initialized.");

        return true;
    }

// taking goal from action client (hardware_interface in puma01) and performing control cycle action
    void executeCB(const force_test::ForceControlGoalConstPtr &as_goal)
    {
        bool succeed = true;

    // get robot configuration
        // current_joint_angles_ = as_goal->current_joint_angles.data;
        for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        {
            kdl_current_joint_angles_(i) = as_goal->current_joint_angles.data[i];
        }

    // get wrench command
        // desired_wrench_ = as_goal->desired_wrench;
        

    // compute jacobian transpose
        KDL::ChainJntToJacSolver kdl_JacSolver_ = KDL::ChainJntToJacSolver(robot_chain_);
        
        int solver_ret = kdl_JacSolver_.JntToJac(kdl_current_joint_angles_, jacobian_); // computing jacobian
        if(solver_ret!=0)
        {
            succeed = false;
            ROS_ERROR("Failed to get jacobian.");
        }

        getTranspose(jacobian_, jacobian_T_); // transposing jacobian
        
    // getting tau = J^T * F
        KDL::MultiplyJacobian(jacobian_T_, kdl_current_wrench_, kdl_tau_);  

    // force control cycle
        if(!controlCycle())
        {
            succeed = false;
            ROS_ERROR("Control cycle error.");
        }

    // get product of PI-controller output and transposed jacobian
    
    //set a result

        as_result_.header = as_goal->header;

        for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        {
            as_result_.output_torques.data[i] = kdl_tau_[i];
        }

        if(succeed)
        {   
            // ROS_INFO("Sent result back.");
            action_server_.setSucceeded(as_result_);
        }
    }

// getting current measurements from force sensor
    void getCurrentWrenchCB(const geometry_msgs::WrenchStampedConstPtr &sensor_msg)
    {
        // current_wrench_.header = sensor_msg->header;
        // current_wrench_.wrench = sensor_msg->wrench;

        // kdl_current_wrench_.data[0] = sensor_msg->wrench.force.x;
        // kdl_current_wrench_.data[1] = sensor_msg->wrench.force.y;
        // kdl_current_wrench_.data[2] = sensor_msg->wrench.force.z;
        // kdl_current_wrench_.data[3] = sensor_msg->wrench.torque.x;
        // kdl_current_wrench_.data[4] = sensor_msg->wrench.torque.y;
        // kdl_current_wrench_.data[5] = sensor_msg->wrench.torque.z;

        kdl_current_wrench_.data[2] = 20; // force z

    }

// controller cycle
    bool controlCycle()
    {

        // compute wrench error
        // compute PI output
        // return PI output as wrench

        // for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        // {

        // }

        return true;
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