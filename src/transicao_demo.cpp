#include <projeto_semear/kine_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Iniciando Teste");
<<<<<<< HEAD
    kineControl::linha_preta(motor);
=======
   /* kineControl::linha_preta(motor);
>>>>>>> origin
    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);     
<<<<<<< HEAD
       
=======
     */
    kineControl::alinhar(motor);  
>>>>>>> origin
}