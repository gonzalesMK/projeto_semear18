#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <iostream>

/** Esse programa é um SERVIÇO responsável por manter a localização GLOBAL do robô.
 * Para os detalhes das localizações permitidas, ver a imagem location.png.
 * 
 * Definição do Serviço GetPose:
 *          bool set
 *          projeto_semear/Pose pose
 *              ---
 *          projeto_semear/Pose pose
 *
 *   set == True   -> a pose é atualizada
 *   set == False  -> o serviço retorna a pose atual
 * 
 * Nome do serviço: gps
 * */

projeto_semear::Pose g_pose;

// Overload of << for the Pose
std::ostream &operator<<(std::ostream &os, const projeto_semear::Pose &pose)
{
    os << "\t(";
    switch (pose.location)
    {
    case 0:
        os << "QUAD_CENTRAL";
        break;
    case 1:
        os << "QUAD_ESQ";
        break;
    case 2:
        os << "QUAD_DIR";
        break;
    case 3:
        os << "DOCA_VERDE";
        break;
    case 4:
        os << "DOCA_AZUL";
        break;
    case 5:
        os << "DOCA_CENTRAL";
        break;
    case 6:
        os << "TREM  ";
        break;
    case 255:
        os << "ERROR";
        break;
    default:
        os << "UKNOW";
    }

    os << ",\t ";
    switch (pose.orientation)
    {
    case 0:
        os << "ORIENTATION_TREM";
        break;
    case 1:
        os << "ORIENTATION_INICIO";
        break;
    case 2:
        os << "ORIENTATION_AZUL";
        break;
    case 3:
        os << "ORIENTATION_VERDE";
        break;
    case 255:
        os << "ERROR";
        break;
    default:
        os << "UNKOW";
    }

    os << ")";
    return os;
}

// Services
bool gps(projeto_semear::GetPose::Request &req,
         projeto_semear::GetPose::Response &res)
{
    req.set ? g_pose = req.pose : res.pose = g_pose;

    if (req.set)
        ROS_INFO_STREAM("Set New Pose: " << g_pose);

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
    ros::Publisher gps_publisher = node.advertise<projeto_semear::Pose>("/AMR/pose", 1);
    ros::Rate rate(10);

    while (ros::ok())
    {
        gps_publisher.publish(g_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
