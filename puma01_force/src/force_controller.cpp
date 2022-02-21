#include <puma01_force/ForceControlAction.h>  
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_toolbox/pid.h>

#include <urdf/model.h>


namespace puma01_controllers
{

class ForceController 
{

private:

    ros::NodeHandle nh_; // = ros::NodeHandle("force_controller");

    actionlib::SimpleActionServer<puma01_force::ForceControlAction> action_server_;
    puma01_force::ForceControlResult as_result_;

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

    std::vector<std::string> cartesian_parameters_names_;

// PID
    std::vector<control_toolbox::Pid> pid_controllers_;

    // geometry_msgs::WrenchStamped current_wrench_;
    // geometry_msgs::Wrench desired_wrench_; 
    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_, tf_sub_;

    std::string action_name_;


    ros::Publisher info_pub_;
    std_msgs::Float64MultiArray info_msg_;

    std::array<double,6>    q_, 
                            wrench_, 
                            last_wrench_, last_last_wrench_, last_last_last_wrench_, last_last_last_last_wrench_,
                            desired_wrench_, wrench_error_, 
                            tau_,PI;

    tf2::Quaternion rotation_quat_;
    tf2::Matrix3x3  rotation_mat_;
    tf2::Vector3    translation_vec_, translation_diff_, force_, projected_force_;

    std::array<tf2::Transform,6>    local_transforms_, transforms_;

    std::array<tf2::Vector3,6>  jacobian_w_, jacobian_v_;  // components of jacobian 6x6
    
    ros::Duration   cycle_period_;
    double  time_last_ = 0.0;


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
        cartesian_parameters_names_ = {"torque_x", "torque_y", "torque_z","force_x", "force_y", "force_z"};

    // no need to load URDF while using /tf topic
    // get URDF from Parameter Server
        // urdf::Model urdf;
        // if (!urdf.initParamWithNodeHandle("robot_description", nh_))
        // {
        //     ROS_ERROR("Failed to parse URDF file.");
        //     return false;
        // }

        // ROS_INFO("Loaded URDF.");

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

        tf_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, &ForceController::getCurrentTFCB, this);

    // allocate data
        as_result_.output_torques.data.resize(6, 0.0); 

        for(int i = 0; i<6; i++) //MAGIC
        {
            local_transforms_[i].setIdentity();
            transforms_[i].setIdentity();
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

        // ROS_INFO("---");

        bool succeed = true; // execution success mark

        tau_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // get robot configuration
        // current_joint_angles_ = as_goal->current_joint_angles.data;
        for(std::size_t i=0; i<cartesian_parameters_names_.size(); i++)
        {
            q_[i] = as_goal->current_joint_angles.data[i];
        }

    // get wrench command
        // desired_wrench_ = as_goal->desired_wrench;
        desired_wrench_[0] = as_goal->desired_wrench.torque.x;
        desired_wrench_[1] = as_goal->desired_wrench.torque.y;
        desired_wrench_[2] = as_goal->desired_wrench.torque.z;
        desired_wrench_[3] = as_goal->desired_wrench.force.x;
        desired_wrench_[4] = as_goal->desired_wrench.force.y;
        desired_wrench_[5] = as_goal->desired_wrench.force.z;

    // computing tau = J^T * F
        double time_now = ros::Time::now().toSec();
        cycle_period_ = ros::Duration(time_now - time_last_);
        time_last_ = time_now; 

        for(unsigned int i=0; i<cartesian_parameters_names_.size(); i++)
        {
            for(unsigned int j=0; j<3; j++)  // MAGIC number!!! but obviously, it works for 6-dof manipulators
            {
                wrench_error_[j+3] = desired_wrench_[j+3]-projected_force_[j]; // calculate wrench error
                PI[i] = pid_controllers_[i].computeCommand(wrench_error_[i], cycle_period_); // compute wrench PI output
                tau_[i] += jacobian_w_[j][i]*PI[i] + jacobian_v_[j][i]*PI[i+3]; 
            }

            info_msg_.data[i] = tau_[i];
            as_result_.output_torques.data[i] = tau_[i];
        }

        // wrench_error_[5] = desired_wrench_[5]-projected_force_[1]; // calculate wrench error
        // PI[5] = pid_controllers_[5].computeCommand(wrench_error_[5], cycle_period_); // compute wrench PI output

        // for(unsigned int i=0; i<cartesian_parameters_names_.size(); i++)
        // {
        //     tau_[i] += jacobian_v_[2][i]*PI[5]; 
        //     as_result_.output_torques.data[i] = tau_[i];
        // }

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
        last_wrench_[0] = sensor_msg->wrench.torque.x;
        last_wrench_[1] = sensor_msg->wrench.torque.y;
        last_wrench_[2] = sensor_msg->wrench.torque.z;
        last_wrench_[3] = sensor_msg->wrench.force.x;
        last_wrench_[4] = sensor_msg->wrench.force.y;
        last_wrench_[5] = sensor_msg->wrench.force.z;

    // filter wrench data
        for(unsigned int i = 0; i<6; i++) //MAGIC number!!
        {
            // wrench_[i] = fabs(wrench_[i])<0.003 ? 0 : wrench_[i]; //saturation for very smaller values
            // if(wrench_[i] != 0)
            // { 
            last_last_last_last_wrench_[i] = last_last_last_wrench_[i]; 
            last_last_last_wrench_[i] = last_last_wrench_[i]; 
            last_last_wrench_[i] = last_wrench_[i];  
            // last_wrench_[i] = wrench_[i];   
            wrench_[i] = (last_last_last_last_wrench_[i] + last_last_last_wrench_[i] + last_last_wrench_[i]+last_wrench_[i])/4; 
            // }
        }

        force_[0] = wrench_[3];
        force_[1] = wrench_[4];
        force_[2] = wrench_[5];

        projected_force_ = local_transforms_[5].getBasis()*force_;

        info_msg_.data[0] = projected_force_[0];
        info_msg_.data[1] = projected_force_[1];
        info_msg_.data[2] = projected_force_[2];

    }

    void getJacobian()
    {
        for(size_t i = 0; i<6; i++)  // MAGIC number!!
        {
        // J_w[i] = z_i
        // getting z_i and p_i_n = p_n - p_i 
            jacobian_w_[i] = transforms_[i].getBasis().getColumn(2); // jacobian for angular velocity
            translation_diff_ = transforms_[5].getOrigin()-transforms_[i].getOrigin();

        // J_v[i] = cross(z_i, p_i_n)
            jacobian_v_[i] = transforms_[i].getBasis().getColumn(2).cross(translation_diff_); // jacobian for linear velocity
        }

        // printf jacobian in terminal
        ROS_INFO("J_1: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][0],jacobian_w_[1][0],jacobian_w_[2][0],jacobian_w_[3][0],jacobian_w_[4][0],jacobian_w_[5][0]);
        ROS_INFO("J_2: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][1],jacobian_w_[1][1],jacobian_w_[2][1],jacobian_w_[3][1],jacobian_w_[4][1],jacobian_w_[5][1]);
        ROS_INFO("J_3: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_w_[0][2],jacobian_w_[1][2],jacobian_w_[2][2],jacobian_w_[3][2],jacobian_w_[4][2],jacobian_w_[5][2]);
        ROS_INFO("J_4: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][0],jacobian_v_[1][0],jacobian_v_[2][0],jacobian_v_[3][0],jacobian_v_[4][0],jacobian_v_[5][0]);
        ROS_INFO("J_5: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][1],jacobian_v_[1][1],jacobian_v_[2][1],jacobian_v_[3][1],jacobian_v_[4][1],jacobian_v_[5][1]);
        ROS_INFO("J_6: %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f",jacobian_v_[0][2],jacobian_v_[1][2],jacobian_v_[2][2],jacobian_v_[3][2],jacobian_v_[4][2],jacobian_v_[5][2]);

    }

// getting current robot's transformations to compute jacobian
    void getCurrentTFCB(const tf2_msgs::TFMessageConstPtr &tf_msg)
    {
        //getting local transformation matrices from /tf
        //computing transformation matrices

        for(int i = 0; i<6; i++) //MAGIC number!!!
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

    // getting jacobian after we got global transforms
        getJacobian(); 
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