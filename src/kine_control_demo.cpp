#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::robot robot;

    geometry_msgs::Twist velocidade;

    
    // Ir para trás
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