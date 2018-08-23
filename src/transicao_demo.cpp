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
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");
    ros::NodeHandle nh;

    kineControl::robot motor;
/*
    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Iniciando Teste");
    kineControl::linha_preta(motor);
    /*kineControl::esquerda(motor);  
    kineControl::pegar_container(motor);
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);   */  
       
   /* kineControl::linha_preta(motor);
    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);  */
/*
    Client client("moveEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();

    // Meta para posicionar a garra em cima do container
    projeto_semear::moveEletroimaGoal goal;
    goal.deslocamento.linear.x = 0;
    goal.deslocamento.linear.y = 0;
    goal.deslocamento.linear.z = -0.12;
    goal.deslocamento.angular.z = 0;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());

    ROS_INFO_STREAM("ligando o eletroima");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  
    std_msgs::Bool msg;
    msg.data = true;
    pub.publish(msg);

    goal.deslocamento.linear.z = 0.12;
 

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());

    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor); 
    */
    kineControl::alinhar_pilha(motor,0);

    return 0;
}