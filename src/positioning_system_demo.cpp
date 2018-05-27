#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <cstdint>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_demo");
    ros::NodeHandle node;

    // Cria o serviço
    ros::ServiceClient pose_client = node.serviceClient<projeto_semear::GetPose>("gps");

    projeto_semear::GetPose srv;
    bool stop = false;
    while (!stop)
    {
        // Set Pose
        char temp;
        std::cout << " Set Location: ";
        std::cin >> srv.request.pose.location;

        std::cout << "Set orientation: ";
        std::cin >> srv.request.pose.orientation;

        srv.request.set = true;

        if (!pose_client.call(srv))
            ROS_ERROR_STREAM("NOT POSSIBLE TO SET");

        // Get pose
        srv.request.set = false;
        if (pose_client.call(srv))
            ROS_INFO_STREAM("Location: " << (std::uint8_t)srv.response.pose.location << "\t Orientation: " << (std::uint8_t)srv.response.pose.orientation);
        else
            ROS_ERROR("Failed to call gps");

        std::cout << "Stop (0/1) ? ";
        std::cin >> stop;
    }
    return 0;
}
