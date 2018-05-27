#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>

/// Need a Global Variable to Keep Track of Position and Orientation

projeto_semear::Pose g_pose;

// Services
bool gps(projeto_semear::GetPose::Request &req,
         projeto_semear::GetPose::Response &res)
{
    req.set ? g_pose = req.pose : res.pose = g_pose;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS");
    ros::NodeHandle node;

    // Initial Values
    g_pose.orientation = projeto_semear::Pose::ORIENTATION_TREM;
    g_pose.location = projeto_semear::Pose::DOCA_CENTRAL;

    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("gps", gps);

    ros::spin();
    return 0;
}
