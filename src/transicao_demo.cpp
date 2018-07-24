#include <projeto_semear/kine_control.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Distance to Goal" <<  feedback->distance) ;
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState& state,
            const projeto_semear::moveEletroimaResultConstPtr& result)
{
  ROS_INFO_STREAM("Finished in state" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Iniciando Teste");
<<<<<<< a9c83cc14bbcf51ca04c627fdba90a4672a077be
<<<<<<< 165ad2252775e1b3114de7edd16e75c12b0cb889
   /* kineControl::linha_preta(motor);
    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);     
     */
    kineControl::alinhar(motor);  
=======
    kineControl::linha_preta(motor);
=======
    kineControl::alinhar_containerdepositado(motor);
>>>>>>> Testes depositar
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);*/
    
    return 0;
>>>>>>> Primeiro commit
}