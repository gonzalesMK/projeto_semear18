#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::motorControl motor;
    
    // Parâmetro de entrada do motor
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 1;
    velocidade.linear.y = 1;
    velocidade.angular.z = 1;

    motor.setVelocity(velocidade);
    
    return 0;
}