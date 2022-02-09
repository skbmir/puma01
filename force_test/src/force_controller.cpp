#include <force_test/ForceControlAction.h>  
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

    std::vector<std::string> cartesian_parameters_names_;

// PID
    std::vector<control_toolbox::Pid> pid_controllers_;

    // geometry_msgs::WrenchStamped current_wrench_;
    // geometry_msgs::Wrench desired_wrench_; 
    std::vector<double> current_joint_angles_;
    int joints_number_ = 6; // default value, for puma01 

    ros::Subscriber sensor_sub_, tf_sub_;

    std::string action_name_;


    ros::Publisher info_pub_;
    std_msgs::Float64MultiArray info_msg_;

    std::vector<double> q_, 
                        wrench_, 
                        last_wrench_, last_last_wrench_, 
                        desired_wrench_, wrench_error_, 
                        tau_;
    
    std::array<std::array<double,3>,6> translation_, rotation_z_, translation_diff_;  // sets of vectors 1x3
    std::array<double,3> cross_of_z_p_;

    std::array<std::array<double,6>,6> jacobian_;  // jacobian 6x6
    
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

        q_.resize(6, 0.0);  // MAGIC numbers???
        last_wrench_.resize(6, 0.0);
        last_last_wrench_.resize(6, 0.0);
        desired_wrench_.resize(6, 0.0);
        wrench_error_.resize(6, 0.0);
        tau_.resize(6, 0.0);

        tf_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, &ForceController::getCurrentTFCB, this);

        // to plot info for debug
        info_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/force_control_info",1);
        info_msg_.data.resize(7);

        ROS_INFO("force_controller initialized.");


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

    // compute jacobian   
        getJacobian();

    // computing tau = J^T * F
        double time_now = ros::Time::now().toSec();
        cycle_period_ = ros::Duration(time_now - time_last_);
        time_last_ = time_now; 

        for(unsigned int i=0; i<cartesian_parameters_names_.size(); i++)
        {
            wrench_error_[i] = desired_wrench_[i]-wrench_[i]; // calculate wrench error
            PI[i] = pid_controllers_[i].computeCommand(wrench_error_[i], cycle_period_); // compute wrench PI output

            for(unsigned int j=0; j<6; j++)  // MAGIC number!!! but obviously, it works for 6-dof manipulators
            {
                // tau_[i]+=jacobian_[j][i]*PI[i];
            }

            // info_msg_.data[i] = tau_[i];
            as_result_.output_torques.data[i] = 0; //kdl_tau_(i);
        }


        as_result_.header = as_goal->header;

        // info_pub_.publish(info_msg_);

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
        wrench_[0] = sensor_msg->wrench.torque.x;
        wrench_[1] = sensor_msg->wrench.torque.y;
        wrench_[2] = sensor_msg->wrench.torque.z;
        wrench_[3] = sensor_msg->wrench.force.x;
        wrench_[4] = sensor_msg->wrench.force.y;
        wrench_[5] = sensor_msg->wrench.force.z;

    // filter wrench data
        for(unsigned int i = 0; i<6; i++) //MAGIC number!!
        {
            // wrench_[i] = fabs(wrench_[i])<0.003 ? 0 : wrench_[i];
            // if(wrench_[i] != 0)
            // {
                wrench_[i] = (1*last_last_wrench_[i]+1*last_wrench_[i]+1*wrench_[i])/3;
                last_last_wrench_[i] = last_wrench_[i];
                last_wrench_[i] = wrench_[i];     
            // }
        }
        
    }

    void getJacobian()
    {
        //get J_[i,j] = z and i = rows from 0 to 2, and columns = j from 0 to 5
        //get J_v_i = cross(z,p)
        //form  J_

        for(size_t i = 0; i<6; i++) 
        {
        
        // J_w 
            jacobian_[i][0] = rotation_z_[i][0];
            jacobian_[i][1] = rotation_z_[i][1];
            jacobian_[i][2] = rotation_z_[i][2];

            translation_diff_[i][0] = translation_[5][0] - translation_[i][0];
            translation_diff_[i][1] = translation_[5][1] - translation_[i][1];
            translation_diff_[i][2] = translation_[5][2] - translation_[i][2];

        // cross product of z and p
        // a2*b3 - a3*b2
        // a3*b1 - a1*b3
        // a1*b2 - a2*b1
            cross_of_z_p_[0] = rotation_z_[i][1] * translation_diff_[i][2] - rotation_z_[i][2] * translation_diff_[i][1];
            cross_of_z_p_[1] = rotation_z_[i][2] * translation_diff_[i][0] - rotation_z_[i][0] * translation_diff_[i][2];
            cross_of_z_p_[2] = rotation_z_[i][0] * translation_diff_[i][1] - rotation_z_[i][1] * translation_diff_[i][0];

        // J_v
            jacobian_[i][3] = cross_of_z_p_[0];
            jacobian_[i][4] = cross_of_z_p_[1];
            jacobian_[i][5] = cross_of_z_p_[2];

        }
    

    }

// getting current robot's transformations to compute jacobian
    void getCurrentTFCB(const tf2_msgs::TFMessageConstPtr &tf_msg)
    {
        //get all 6 vectors of z = 3rd column of R matrix from quaternion
        //get all 6 vectors of p = translational vector
       
        for(size_t i = 0; i<6; i++) //MAGIC number!!!
        {
            //2*(x*z + w*y)
            //2*(y*z + w*x)
            //2*(w*w + z*z)-1

            rotation_z_[i][0] = 2*(tf_msg->transforms[i].transform.rotation.x * tf_msg->transforms[i].transform.rotation.z + tf_msg->transforms[i].transform.rotation.w * tf_msg->transforms[i].transform.rotation.y);
            rotation_z_[i][1] = 2*(tf_msg->transforms[i].transform.rotation.y * tf_msg->transforms[i].transform.rotation.z + tf_msg->transforms[i].transform.rotation.w * tf_msg->transforms[i].transform.rotation.x);
            rotation_z_[i][2] = 2*(tf_msg->transforms[i].transform.rotation.w * tf_msg->transforms[i].transform.rotation.w + tf_msg->transforms[i].transform.rotation.z * tf_msg->transforms[i].transform.rotation.z)-1;

            translation_[i][0] = tf_msg->transforms[i].transform.translation.x;
            translation_[i][1] = tf_msg->transforms[i].transform.translation.y;
            translation_[i][2] = tf_msg->transforms[i].transform.translation.z;
        }
            
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