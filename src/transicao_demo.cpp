#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("LOL");
    kineControl::quadrante_central2(motor, projeto_semear::Pose::QUADRANTE_CENTRAL, projeto_semear::Pose::QUADRANTE_DIREITO);
    ROS_INFO("LOL");
    kineControl::quadrante_central2(motor, projeto_semear::Pose::QUADRANTE_CENTRAL, projeto_semear::Pose::QUADRANTE_ESQUERDO);
    
    return 0;
}