#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::robot robot;
    //kineControl::linha_preta(motor);
    // kineControl::alinhar_para_tras(motor);
    int erro1=0, erro2=0;
  /*
    while (ros::ok())
    {
         erro1 = kineControl::erro_sensores_esquerda_com_branco(robot, erro1);
        erro2 = kineControl::erro_sensores_direita_com_branco(robot, erro2);
        ROS_INFO_STREAM("ERRO ESQUERDA: " << erro1  << " ERRO DIREITA: " << erro2);
    }*/
    //    kineControl::esquerda(motor);
    //kineControl::alinhar_direita(motor);
    //kineControl::direita(motor);
    //kineControl::direita(motor);
    //kineControl::alinhar_para_tras(robot);
    kineControl::alinhar_direita(robot);

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