#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>

/** Esse programa faz uma ligação entre o tópico /cmd_vel publicado pelo teleop_twist_keyboard
 *  e o controle do eletroima. Assim, é possivel move-lo utilizando o teclado .
 * */

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
    //ROS_INFO_STREAM("Distance to Goal" << feedback->distance);
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
 //   ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
    // ROS_INFO("Goal just went active");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Eletroima_client");
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
    goal.deslocamento.linear.y = -0.15;
    goal.deslocamento.linear.z = 0;
    goal.deslocamento.angular.z = 0;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    client.waitForResult(ros::Duration());

    // Girar a guarra 90º 
    // Ligar o Eletroimã:
    ROS_INFO_STREAM("ligando o eletroima");
    msg.data = true;
    pub.publish(msg);

    goal.deslocamento.linear.z = 0.04;
    goal.deslocamento.linear.x = -0.01;
    goal.deslocamento.linear.y = 0.05;
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    client.waitForResult(ros::Duration());

    return 0;
}