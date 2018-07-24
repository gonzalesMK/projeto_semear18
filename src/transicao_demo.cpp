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
    kineControl::alinhar_containerdepositado(motor);
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);*/
    
    return 0;
}