#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
#include <projeto_semear/kine_control.h>

/* Código para depositar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
=======

>>>>>>> Primeiro commit
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Eletroima_client");
  ros::NodeHandle nh;

<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
  kineControl::robot motor;

  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
=======
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
<<<<<<< 2c46a2b66ba647f7d62e380a82543e05b67d08b3

>>>>>>> Primeiro commit
=======
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
>>>>>>> Arrumar CMake
  std_msgs::Bool msg;
  ROS_INFO_STREAM("ligando o eletroima");
  msg.data = true;
  pub.publish(msg);

  Client client("moveEletroima", true); 
  client.waitForServer();
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
  int code = 0; //variável que guarda quantos containers têm em uma pilha

  projeto_semear::moveEletroimaGoal goal;
  
  /*Code == 0: nenhum container depositado
    Code != 0: já existe um ou mais containers na pilha*/

  if(code == 0){
      goal.deslocamento.angular.z = 0;
      goal.deslocamento.linear.x = 0;
      goal.deslocamento.linear.y = 0;
      goal.deslocamento.linear.z = -0.137;
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
      code++;
  }else{
      //alinhar com o container de baixo
      kineControl::alinhar_containerdepositado(motor);
      goal.deslocamento.angular.z = 0;
      goal.deslocamento.linear.x = 0;
      goal.deslocamento.linear.y = 0;
      goal.deslocamento.linear.z = -0.137+(code*0.02); //0,2 chute da altura do container
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
      code++;
  }
  //andar uma distância determinada para fica no meio do container já depositado
  goal.deslocamento.angular.z = 1;
  goal.deslocamento.linear.z = 0;
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());
=======

  projeto_semear::moveEletroimaGoal goal;
  goal.deslocamento.angular.z = 10/4;
  goal.deslocamento.linear.x = 0;
  goal.deslocamento.linear.y = 0;
  goal.deslocamento.linear.z = 0;
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());

  /*goal.deslocamento.linear.z = -0.137;
  goal.deslocamento.angular.z = 0;
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());

  goal.deslocamento.angular.z = 1;
  goal.deslocamento.linear.z = 0;
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());*/
>>>>>>> Primeiro commit

  ROS_INFO_STREAM("desligando o eletroima");
  msg.data = false;
  pub.publish(msg);

<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
  client.waitForResult(ros::Duration());
  
=======

  client.waitForResult(ros::Duration());

>>>>>>> Primeiro commit
  return 0;
}