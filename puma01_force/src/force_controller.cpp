#include <puma01_force/ForceControlAction.h>  
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_toolbox/pid.h>

#include <urdf/model.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

namespace puma01_controllers
{

class ForceController 
{

private:

    ros::NodeHandle nh_; // = ros::NodeHandle("force_controller");

    actionlib::SimpleActionServer<puma01_force::ForceControlAction> action_server_; 
    puma01_force::ForceControlResult as_result_;

    std::vector<std::string> cartesian_parameters_names_;

    std::vector<control_toolbox::Pid> pid_controllers_; // PID

    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_, tf_sub_;

    std::string action_name_;

    ros::Publisher info_pub_;
    std_msgs::Float64MultiArray info_msg_;

    double time_last_ = 0.0; 

    tf2::Vector3    force_, projected_force_;

    std::array<tf2::Transform,6>    local_transforms_, transforms_;

    std::array<tf2::Vector3,6>  jacobian_w_, jacobian_v_;  // components of jacobian 6x6

    boost::shared_ptr<ros::AsyncSpinner> ft_sensor_spinner_, tf_spinner_; // AsyncSpinners for /tf and "/wrench" subscribers
    ros::CallbackQueue tf_queue_, ft_sensor_queue_;

    const uint8_t filter_order_ = 50;
    std::vector<std::array<double,3>> filter_conv_; // filter input signal convolution




public:
ForceController(std::string action_name) : action_name_(action_name), action_server_(nh_, action_name, boost::bind(&ForceController::executeCB, this, _1), false)
{
    action_server_.start();
    ROS_INFO("Started %s Action server.", action_name_.c_str());
    init();
}

~ForceController()
{
    tf_spinner_.reset();
    ft_sensor_spinner_.reset();
}

bool init()
{  

// names of cartesian PIDs
    cartesian_parameters_names_ = {"torque_x", "torque_y", "torque_z","force_x", "force_y", "force_z"};

// init PIDs
    pid_controllers_.resize(cartesian_parameters_names_.size());

    for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
    {
    // Load PIDs controllers using gains set on parameter server
        if (!pid_controllers_[i].init(ros::NodeHandle(nh_,  action_name_ + "/" + cartesian_parameters_names_[i] + "/pid")))
        {
            ROS_ERROR_STREAM("Failed to load PID parameters from " << action_name_ + "/" + cartesian_parameters_names_[i] + "/pid");
            return false;
        }
    }

    ROS_INFO("PID controllers initialised.");

// initialise subscriber /tf
    ros::SubscribeOptions tf_subscriber_options =
        ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
        "/tf", // topic name
        1, // queue length
        boost::bind(&puma01_controllers::ForceController::getCurrentTFCB,this,_1), // callback
        ros::VoidPtr(), // tracked object, we don't need one thus NULL
        &tf_queue_ // pointer to callback queue object
        );

// initialise subscriber /puma01_sim/ft_sensor
    ros::SubscribeOptions ft_sensor_subscriber_options =
        ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
        "/puma01_sim/ft_sensor", // topic name
        1, // queue length
        boost::bind(&puma01_controllers::ForceController::getCurrentWrenchCB,this,_1), // callback
        ros::VoidPtr(), // tracked object, we don't need one thus NULL
        &ft_sensor_queue_ // pointer to callback queue object
        );

    tf_sub_ = nh_.subscribe(tf_subscriber_options);
    sensor_sub_ = nh_.subscribe(ft_sensor_subscriber_options);

// standart definition of subscribers
    // tf_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, &ForceController::getCurrentTFCB, this); 
    // sensor_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/puma01_sim/ft_sensor", 1, &ForceController::getCurrentWrenchCB, this);

// reset ans start AsyncSpinners for subscribers' callbacks
    tf_spinner_.reset(new ros::AsyncSpinner(1, &tf_queue_));
    ft_sensor_spinner_.reset(new ros::AsyncSpinner(1, &ft_sensor_queue_));

    tf_spinner_->start();
    ft_sensor_spinner_->start();

// allocate data
    as_result_.output_torques.data.resize(6, 0.0); 

    for(size_t i= 0; i<6; i++) //MAGIC NUMBER!!!!!!!!!!!!!!!!!!!!!!!!!1
    {
        local_transforms_[i].setIdentity();
        transforms_[i].setIdentity();
    }

    filter_conv_.resize(filter_order_);
    for(size_t i= 0; i<filter_order_; i++) 
    {
        filter_conv_[i].fill(0.0);
    }

// to plot info for debug
    info_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/force_control_info",1);
    info_msg_.data.resize(7);

    ROS_INFO("force_controller initialized.");

    return true;
}

// taking goal from action client (hardware_interface in puma01) and performing control cycle action
void executeCB(const puma01_force::ForceControlGoalConstPtr &as_goal)
{
    
    // ROS_INFO("Executing goal.");

    bool succeed = true; // execution success mark

    std::array<double,6>    desired_wrench, wrench_error, tau, PI;

// get robot configuration, NO NEED cause of using /tf for computing jacobian
    // current_joint_angles_ = as_goal->current_joint_angles.data;
    // for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
    // {
    //     q_[i] = as_goal->current_joint_angles.data[i];
    // }

// get wrench command
    // desired_wrench = as_goal->desired_wrench;
    desired_wrench[0] = as_goal->desired_wrench.torque.x;
    desired_wrench[1] = as_goal->desired_wrench.torque.y;
    desired_wrench[2] = as_goal->desired_wrench.torque.z;
    desired_wrench[3] = as_goal->desired_wrench.force.x;
    desired_wrench[4] = as_goal->desired_wrench.force.y;
    desired_wrench[5] = as_goal->desired_wrench.force.z;

// computing tau = J^T * F
    double time_now = ros::Time::now().toSec();
    ros::Duration cycle_period(time_now - time_last_);
    time_last_ = time_now; 
    
    for(size_t i = 0; i<6; i++)  // MAGIC number!!!!!!!!!!!!1
    {
        if(i<3) // compute force error only
        {
            wrench_error[i] = 0.0; 
        }
        else
        {
            wrench_error[i] = desired_wrench[i]-force_[i-3]; 
        }

        PI[i] = pid_controllers_[i].computeCommand(wrench_error[i], cycle_period); // compute wrench PI output
    }

    for(size_t i = 0; i<6; i++)  // MAGIC number!!!!!!!!!!!!1
    {
    // shortened version of  tau = J^T * F
        // tau[i] = jacobian_w_[i][0]*PI[0] + jacobian_w_[i][1]*PI[1] + jacobian_w_[i][2]*PI[2]; // no need in torque control
        tau[i] += jacobian_v_[i][0]*PI[3] + jacobian_v_[i][1]*PI[4] + jacobian_v_[i][2]*PI[5];
        // tau[i] -= jacobian_v_[i][2]*force_[2]*0.05; 

    // set action result
        as_result_.output_torques.data[i] = tau[i];
    }

    // ROS_INFO("tau: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",tau[0],tau[1],tau[2],tau[3],tau[4],tau[5]);

    as_result_.header = as_goal->header;

// to plot info data
    info_msg_.data[3] = tau[0];
    info_msg_.data[4] = tau[1];
    info_msg_.data[5] = tau[2];
    info_pub_.publish(info_msg_);

// check force_controller operation success
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

    // ROS_INFO("Got current wrench.");
    geometry_msgs::Vector3    new_force = sensor_msg->wrench.force;

    // filter wrench data
    force_ = force_vector_filter(new_force);

    tf2::quatRotate(local_transforms_[5].getRotation(), force_);
    // projected_force_ = local_transforms_[5].getBasis()*force_;

    info_msg_.data[0] = force_[0];
    info_msg_.data[1] = force_[1];
    info_msg_.data[2] = force_[2];

}

// getting jacobian after we got global transforms
void getJacobian()
{

    tf2::Vector3    translation_diff;

    for(size_t i = 0; i<6; i++)  // MAGIC number!!
    {
    // J_w[i] = z_i
    // getting z_i and p_i_n = p_n - p_i 
        jacobian_w_[i] = transforms_[i].getBasis().getColumn(2); // jacobian for angular velocity
        translation_diff = transforms_[5].getOrigin()-transforms_[i].getOrigin();

    // J_v[i] = cross(z_i, p_i_n)
        jacobian_v_[i] = transforms_[i].getBasis().getColumn(2).cross(translation_diff); // jacobian for linear velocity
    }

    // printf jacobian in terminal
    // ROS_INFO("J_1: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][0],jacobian_w_[1][0],jacobian_w_[2][0],jacobian_w_[3][0],jacobian_w_[4][0],jacobian_w_[5][0]);
    // ROS_INFO("J_2: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][1],jacobian_w_[1][1],jacobian_w_[2][1],jacobian_w_[3][1],jacobian_w_[4][1],jacobian_w_[5][1]);
    // ROS_INFO("J_3: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][2],jacobian_w_[1][2],jacobian_w_[2][2],jacobian_w_[3][2],jacobian_w_[4][2],jacobian_w_[5][2]);
    // ROS_INFO("J_4: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][0],jacobian_v_[1][0],jacobian_v_[2][0],jacobian_v_[3][0],jacobian_v_[4][0],jacobian_v_[5][0]);
    // ROS_INFO("J_5: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][1],jacobian_v_[1][1],jacobian_v_[2][1],jacobian_v_[3][1],jacobian_v_[4][1],jacobian_v_[5][1]);
    // ROS_INFO("J_6: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][2],jacobian_v_[1][2],jacobian_v_[2][2],jacobian_v_[3][2],jacobian_v_[4][2],jacobian_v_[5][2]);

}

// getting current robot's transformations to compute jacobian
void getCurrentTFCB(const tf2_msgs::TFMessageConstPtr &tf_msg)
{

    tf2::Quaternion rotation_quat_;
    tf2::Vector3    translation_vec_;

    //getting local transformation matrices from /tf
    //computing transformation matrices

    for(size_t i= 0; i<6; i++) //MAGIC number!!!
    {

    // converting TF msg to tf2 objects
        tf2::convert(tf_msg->transforms[i].transform.rotation, rotation_quat_);
        tf2::convert(tf_msg->transforms[i].transform.translation, translation_vec_);

    // get local transforms
        local_transforms_[i].setRotation(rotation_quat_);
        local_transforms_[i].setOrigin(translation_vec_);

    // get global transforms
        if(i<1)
        {
            transforms_[i] = local_transforms_[i];
        }else{
            transforms_[i].mult(transforms_[i-1],local_transforms_[i]);
        }

    }

    getJacobian();  

}

// invoking callback queues' callbacks
void loop() 
{
    // tf_queue_.callOne();
    // ft_sensor_queue_.callOne();
}

// filter for the force vector
 tf2::Vector3 force_vector_filter(geometry_msgs::Vector3 & new_vector) 
{
    tf2::Vector3 filter_out; 
    
    filter_conv_[0][0] = new_vector.x; // getting new values
    filter_conv_[0][1] = new_vector.y; 
    filter_conv_[0][2] = new_vector.z; 

    for(size_t j = 0; j < 3; j++)
    {

        for(size_t i = 0; i < filter_order_; i++) // sum of convolution elements
        {
            filter_out[j] += filter_conv_[i][j];
        }

        filter_out[j] /= filter_order_; // computing filter out

        for(size_t i = filter_order_-1; i > 0; i--) // shift of convolution elements
        {
            filter_conv_[i][j] = filter_conv_[i-1][j];
        }

    }

    return filter_out;
}   


}; // class
} // namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_controller");

    puma01_controllers::ForceController force_controller(ros::this_node::getName());

    while(ros::ok())
    {
        force_controller.loop();
        ros::spinOnce();
    }

    return 0;
} // main