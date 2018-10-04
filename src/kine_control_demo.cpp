#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;
    //kineControl::linha_preta(motor);
    kineControl::alinhar_para_tras(motor);
//    kineControl::esquerda(motor);
    //kineControl::alinhar_direita(motor);
    //kineControl::direita(motor);
    //kineControl::direita(motor);
  //  kineControl::alinhar_para_frente(motor);

    //** Como usar o motor para mudar a velocidade :

    // Parâmetro de entrada do motor
    /*geometry_msgs::Twist velocidade;
    velocidade.linear.x = 1;
    velocidade.linear.y = 1;
    velocidade.angular.z = 1;

    motor.setVelocity(velocidade);
    ROS_INFO_STREAM(" COLOR IN BR: ");

    //** Como usar o motor para ler o sensor 
    while (ros::ok())
    {
        ROS_INFO_STREAM(" COLOR IN BR: " << motor.get_colorBL());
    }*/
    
    return 0;
}