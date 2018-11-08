#include <projeto_semear/kine_control.h>
#include <geometry_msgs/Twist.h>

void cmd_callback(const geometry_msgs::TwistConstPtr &msg)
{
  
  kineControl::robot robot;
  geometry_msgs::Twist vel;
  vel = *msg;
  robot.setVelocity(vel);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::robot robot;

    geometry_msgs::Twist velocidade;

    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_callback); 

    ros::spin();
/*    // Ir para trás
    velocidade.linear.x = -0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(0.5).sleep();
/*
    // Girar 90 Graus
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 3.14 / 3;
    robot.setVelocity(velocidade);
    ros::Duration(3).sleep();

    // Andar para frente
    velocidade.linear.x = 0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
*/

    return 0;
}