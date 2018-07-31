#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
=======
>>>>>>> Testes depositar
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
#include <projeto_semear/kine_control.h>

/* Código para depositar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
=======

>>>>>>> Primeiro commit
=======
>>>>>>> Testes depositar
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
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

<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
=======
>>>>>>> Testes depositar
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
  kineControl::robot motor;

  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
<<<<<<< HEAD
=======
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
<<<<<<< 2c46a2b66ba647f7d62e380a82543e05b67d08b3

>>>>>>> Primeiro commit
=======
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
>>>>>>> Arrumar CMake
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
  std_msgs::Bool msg;
  ROS_INFO_STREAM("ligando o eletroima");
  msg.data = true;
  pub.publish(msg);

  Client client("moveEletroima", true); 
  client.waitForServer();
<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
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
=======
  int code = 0; //variável que guarda quantos containers têm em uma pilha
>>>>>>> Testes depositar
=======
  int code = 0; //variável que guarda quantos containers têm em uma pilha
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe

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
<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
  client.waitForResult(ros::Duration());*/
>>>>>>> Primeiro commit
=======
  client.waitForResult(ros::Duration());
>>>>>>> Testes depositar
=======
  client.waitForResult(ros::Duration());
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe

  ROS_INFO_STREAM("desligando o eletroima");
  msg.data = false;
  pub.publish(msg);

<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
  client.waitForResult(ros::Duration());
  
=======

  client.waitForResult(ros::Duration());

>>>>>>> Primeiro commit
=======
  client.waitForResult(ros::Duration());
  
>>>>>>> Testes depositar
=======
  client.waitForResult(ros::Duration());
  
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
  return 0;
}