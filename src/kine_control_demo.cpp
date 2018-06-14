#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::motorControl motor;
    
    //** Como usar o motor para mudar a velocidade :

    // Parâmetro de entrada do motor
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 1;
    velocidade.linear.y = 1;
    velocidade.angular.z = 1;

    motor.setVelocity(velocidade);
     

    //** Como usar o motor para ler o sensor 
    while(ros::ok()){
        ROS_INFO_STREAM(" COLOR IN BR: " << motor.colorBR_ );
        ros::Duration(0.1);
        ROS_INFO_STREAM(" COLOR IN BR: " << motor.colorBR_ );
        ros::spinOnce();
    }
    return 0;
}