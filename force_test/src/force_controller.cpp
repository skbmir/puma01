#include <force_test/ForceControlAction.h>  
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

namespace puma01_controllers
{

class ForceController 
{

public:
    ForceController(ros::NodeHandle &nh) : action_server_(nh, "force_controller_action_server", boost::bind(&ForceController::as_ExecuteCB, this, _1), false)
    {
        action_server_.start();
        init(nh);
    }

    ~ForceController(){}

    bool init(ros::NodeHandle &nh)
    {  

    // get parameters from Parameter Server
        // if(!nh.param<double>("p_gains", p_gains_, 0.0))
        // {
        //     ROS_ERROR_STREAM("Failed to getParam: p_gains.");
        //     return false;
        // }

        // if(!nh.param<double>("i_gains", i_gains_, 0.0))
        // {
        //     ROS_ERROR_STREAM("Failed to getParam: i_gains.");
        //     return false;
        // }

    // define gains 
        p_gains_.force.x = 0.0;
        p_gains_.force.y = 0.0;
        p_gains_.force.z = 0.0;
        p_gains_.torque.x = 0.0;
        p_gains_.torque.y = 0.0;
        p_gains_.torque.z = 0.0;

        i_gains_.force.x = 0.0;
        i_gains_.force.y = 0.0;
        i_gains_.force.z = 0.0;
        i_gains_.torque.x = 0.0;
        i_gains_.torque.y = 0.0;
        i_gains_.torque.z = 0.0;

    // get URDF from Parameter Server
        urdf::Model urdf;
        if (!urdf.initParamWithNodeHandle("robot_description", nh))
        {
            ROS_ERROR("Failed to parse URDF file.");
            return false;
        }

    // resizing current joint angles vector after getting the number of joints
        current_joint_angles_.resize(joints_number_);

        sensor_sub_ = nh.subscribe<geometry_msgs::WrenchStamped>("/puma01_sim/ft_sensor", 1, &ForceController::getCurrentWrenchCB, this)

        return true;
    }

// taking goal from action client (hardware_interface in puma01) and performing control cycle action
    void as_ExecuteCB(const force_test::ForceControlGoalConstPtr &as_goal)
    {
        // get robot configuration
        current_joint_angles_.data = as_goal->current_joint_angles->data;

        // compute jacobian transpose??

        // force control cycle
        // controlCycle(as_goal.desired_wrench)

        // get product of PI-controller output and transposed jacobian, and set it as a result

        // as_result_.header = ;
        as_result_->output_torques = current_joint_angles_.data;

        action_server_->setSucceeded(as_result_);
    }

// getting current measurements from force sensor
    void getCurrentWrenchCB(const geometry_msgs::WrenchStampedConstPtr &sensor_msg)
    {
        current_wrench_.header = sensor_msg.header;
        current_wrench_.wrench = sensor_msg.wrench;
    }

// controller cycle
    void controlCycle(geometry_msgs::Wrench desired_wrench)
    {

        // compute wrench error
        // compute PI output
        // return PI output as wrench
    }

private:
    actionlib::SimpleActionServer<force_test::ForceControlAction> action_server_;
    force_test::ForceControlResult as_result_;

    geometry_msgs::Wrench p_gains_,i_gains_; // using wrench structure for use

    geometry_msgs::WrenchStamped current_wrench_;
    std::vector<double> current_joint_angles_;
    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_;

}; // class
} // namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_controller");

    ros::NodeHandle nh;

    ForceController force_controller(nh);

    ros::spin();
    return 0;
} // main