#include <ros/ros.h>
#include <projeto_semear/PegarContainer.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>

int code = 1;

/* Código para pegar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr& feedback)
{
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState& state,
            const projeto_semear::moveEletroimaResultConstPtr& result)
{
}

// Called once when the goal becomes active
void activeCb()
{
}

bool pegar_container(projeto_semear::pegarContainer::Request &req,
         projeto_semear::pegarContainer::Response &res)
{
  ros::NodeHandle nh;

    ROS_INFO_STREAM("ligando o eletroima");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
    
    std_msgs::Bool msg;
    msg.data = false;
    pub.publish(msg);

    // Espera-se que o código já saiba se deve pegar o container da direita ou da esquerda
    Client client("moveEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();

    // Meta para posicionar a garra em cima do container
    projeto_semear::moveEletroimaGoal goal;
    goal.deslocamento.linear.x = 0.01;
    goal.deslocamento.linear.y = -0.2;
    goal.deslocamento.linear.z = 0;
    goal.deslocamento.angular.z = 0;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());

    // Girar a guarra 90º 
    // Ligar o Eletroimã:
    ROS_INFO_STREAM("ligando o eletroima");
    msg.data = true;
    pub.publish(msg);


    client.waitForResult(ros::Duration());

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pegar_container");
    ros::NodeHandle node;
    
    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("pegar_container", pegar_container);

    ros::spin();

    return 0;
}