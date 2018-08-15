#include <projeto_semear/kine_control.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Iniciando Teste");
<<<<<<< HEAD
    kineControl::pegar_container(motor);
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);   */  
       
=======
   /* kineControl::linha_preta(motor);
    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);     
     */
    kineControl::alinhar(motor);  
>>>>>>> master
}