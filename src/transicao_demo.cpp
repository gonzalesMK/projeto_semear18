#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Teste");
    kineControl::linha_preta(motor);
    ROS_INFO("Teste");
 /*   ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_CENTRAL, projeto_semear::Pose::QUADRANTE_CENTRAL);
    ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_CENTRAL, projeto_semear::Pose::QUADRANTE_DIREITO);
    ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_DIREITO, projeto_semear::Pose::QUADRANTE_ESQUERDO);
    ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_CENTRAL, projeto_semear::Pose::QUADRANTE_ESQUERDO);
    ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_ESQUERDO, projeto_semear::Pose::QUADRANTE_ESQUERDO);
    ROS_INFO("Teste");
    kineControl::mudar_quadrante(motor, projeto_semear::Pose::QUADRANTE_ESQUERDO, projeto_semear::Pose::QUADRANTE_CENTRAL);
   */
    kineControl::esquerda(motor);     
    kineControl::direita(motor);     
    return 0;
}