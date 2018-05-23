#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

// Especifica a orientação do robô
enum Orientation
{
    norte,
    sul,
    leste,
    oeste
};
enum Orientation2
{
    trem,
    inicio,
    azul,
    verde
};

// Especifica as posições do robô
enum Location
{
    quadrante_central,
    quadrante_esquerdo,
    quadrante_direito,
    doca_verde,
    doca_azul,
    doca_central,
    trem
};

struct Pose{
    Orientation orientation;
    Location location; 
}

/// Need a Global Variable to Keep Track of Position and Orientation
Orientation g_orient;
Position g_pose;

// Serv
bool get_pose(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveEletroima");
    ros::NodeHandle node;

    // Initial Values
    g_orient = norte;
    g_pose = doca_central;

    // Cria o serviço
    ros::ServiceServer service = n.advertiseService("get_pos_and_orient", get_pose);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}
