#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "addfloor");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    moveit_msgs::CollisionObject object;
    object.header.frame_id = "floor";

    shape_msgs::Plane plane;
    plane.coef = {{0, 0, 1, 0}};

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    object.planes.push_back(plane);
    object.plane_poses.push_back(pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;

    planning_scene_diff_publisher.publish(planning_scene);

    ros::shutdown();

    return 0;
}