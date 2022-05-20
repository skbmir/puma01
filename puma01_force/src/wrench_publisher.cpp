
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::array<tf2::Transform,6> _local_transforms, _transforms;
tf2::Vector3    tip_force_(0.0, 0.0, 0.0), goal_force_(1.0, 0.0, 0.0);

void get2DGoal(const geometry_msgs::PoseStamped &goal_msg) 
{
    tf2::Quaternion goal_quat;
    tf2::convert(goal_msg.pose.orientation, goal_quat);
    tf2::Matrix3x3 goal_rotation(goal_quat);
    tip_force_ = goal_rotation*goal_force_;
}

void getCurrentTFCB(const tf2_msgs::TFMessageConstPtr &tf_msg)
{
    tf2::Quaternion rotation_quat;
    tf2::Vector3    translation_vec;

    for(size_t i = 0; i<6; i++) //MAGIC number!!!
    {

    // converting TF msg to tf2 objects
        tf2::convert(tf_msg->transforms[i].transform.rotation, rotation_quat);
        tf2::convert(tf_msg->transforms[i].transform.translation, translation_vec);

    // get local transforms
        _local_transforms[i].setRotation(rotation_quat);
        _local_transforms[i].setOrigin(translation_vec);

    // get global transforms
        if(i<1)
        {
            _transforms[i] = _local_transforms[i];
        }
        else
        {
            _transforms[i].mult(_transforms[i-1],_local_transforms[i]);
        }

    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrench_pub");

    ros::NodeHandle nh;

    ros::Subscriber tf_sub, goal_sub;
    tf_sub =  nh.subscribe("/tf", 1, &getCurrentTFCB);
    goal_sub =  nh.subscribe("/goal", 1, &get2DGoal);

    for(size_t i= 0; i<6; i++) //MAGIC NUMBER!!!!!!!!!!!!!!!!!!!!!!!!!1
    {
        _local_transforms[i].setIdentity();
        _transforms[i].setIdentity();
    }

    tf2::Vector3    wrench_force(0.0, 0.0, 0.0);
    tf2::Transform  inv_transform;

    ros::Publisher wrench_pub;
    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/puma01_sim/ft_sensor",1);

    geometry_msgs::WrenchStamped wrench_msg;

    wrench_msg.header.frame_id = "link_T"; 
    wrench_msg.header.seq = 0;
    wrench_msg.header.stamp = ros::Time(ros::Time::now());

    wrench_msg.wrench.torque.x = 0.0;
    wrench_msg.wrench.torque.y = 0.0;
    wrench_msg.wrench.torque.z = 0.0;
    wrench_msg.wrench.force.x = wrench_force[0];
    wrench_msg.wrench.force.y = wrench_force[1];
    wrench_msg.wrench.force.z = wrench_force[2];

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        inv_transform = _transforms[5].inverse();
        wrench_force = inv_transform.getBasis()*tip_force_;

        wrench_msg.wrench.force.x = wrench_force[0];
        wrench_msg.wrench.force.y = wrench_force[1];
        wrench_msg.wrench.force.z = wrench_force[2];
        wrench_msg.header.stamp = ros::Time(ros::Time::now());
        wrench_msg.header.seq++;

        wrench_pub.publish(wrench_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} // main