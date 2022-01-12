#include <force_test/ForceControlAction.h>  
#include <actionlib/server/simple_action_server.h>

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
        if(!nh.param<double>("p_gain", p_gain_, 0.0))
        {
            ROS_ERROR_STREAM("Failed to getParam '" << p_gain << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }

        if(!nh.param<double>("i_gain", i_gain_, 0.0))
        {
            ROS_ERROR_STREAM("Failed to getParam '" << i_gain << "' (namespace: " << nh.getNamespace() << ").");
            return false;
        }

    // get URDF from Parameter Server
        urdf::Model urdf;
        if (!urdf.initParamWithNodeHandle("robot_description", nh))
        {
            ROS_ERROR("Failed to parse URDF file.");
            return false;
        }

        return true;
    }

    void as_ExecuteCB(const force_test::ForceControlGoalConstPtr &as_goal)
    {
        // get robot configuration
        // compute jacobian transpose
        // force control cycle
        // get product of PI-controller output and transposed jacobian, and set it as a result

        action_server_->setSucceeded(as_result_);
    }

    void controlCycle(/* desired wrench */)
    {
        // compute wrench error
        // compute PI output
        // return PI output as wrench
    }

private:
    actionlib::SimpleActionServer<force_test::ForceControlAction> action_server_;
    force_test::ForceControlResult as_result_;

    double p_gain_;
    double i_gain_;

}; // class
} // namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_controller_node");

    ros::NodeHandle nh;

    ForceController force_controller(nh);

    ros::spin();
    return 0;
} // main