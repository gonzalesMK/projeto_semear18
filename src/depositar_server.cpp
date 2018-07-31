#include <ros/ros.h>
#include <projeto_semear/DepositarContainer.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>

<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< f3d0d2c15ea6e3b66448dc4772208d0b391f94ea
int code = 1;

=======
>>>>>>> Serviço depositar_container
=======
int code = 1;

>>>>>>> Arrumando o serviço
=======
int code = 1;

>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
/* Código para depositar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
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

bool depositar_container(projeto_semear::DepositarContainer::Request &req,
         projeto_semear::DepositarContainer::Response &res)
{

  ros::NodeHandle nh;

  kineControl::robot motor;
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
  std_msgs::Bool msg;
  ROS_INFO_STREAM("ligando o eletroima");
  msg.data = true;
  pub.publish(msg);

  Client client("moveEletroima", true); 
  client.waitForServer();

  projeto_semear::moveEletroimaGoal goal;
  
  /*Code == 0: nenhum container depositado
    Code != 0: já existe um ou mais containers na pilha*/

<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< f3d0d2c15ea6e3b66448dc4772208d0b391f94ea
  if(code == 0){
=======
  if(req.code == 0){
>>>>>>> Serviço depositar_container
=======
  if(code == 0){
>>>>>>> Arrumando o serviço
=======
  if(code == 0){
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
      goal.deslocamento.angular.z = 0;
      goal.deslocamento.linear.x = 0;
      goal.deslocamento.linear.y = 0;
      goal.deslocamento.linear.z = -0.137;
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< f3d0d2c15ea6e3b66448dc4772208d0b391f94ea
      code++;
=======
      req.code++;
>>>>>>> Serviço depositar_container
=======
      code++;
>>>>>>> Arrumando o serviço
=======
      code++;
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
  }else{
      //alinhar com o container de baixo
      kineControl::alinhar_containerdepositado(motor);
      goal.deslocamento.angular.z = 0;
      goal.deslocamento.linear.x = 0;
      goal.deslocamento.linear.y = 0;
<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< f3d0d2c15ea6e3b66448dc4772208d0b391f94ea
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
      goal.deslocamento.linear.z = -0.137+(code*0.02); //0,2 chute da altura do container
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
      code++;
<<<<<<< HEAD
=======
      goal.deslocamento.linear.z = -0.137+(req.code*0.02); //0,2 chute da altura do container
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
      req.code++;
>>>>>>> Serviço depositar_container
=======
      goal.deslocamento.linear.z = -0.137+(code*0.02); //0,2 chute da altura do container
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration());
      code++;
>>>>>>> Arrumando o serviço
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
  }

  ROS_INFO_STREAM("desligando o eletroima");
  msg.data = false;
  pub.publish(msg);

  client.waitForResult(ros::Duration());
  return 0;
}

<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< f3d0d2c15ea6e3b66448dc4772208d0b391f94ea

=======
>>>>>>> Serviço depositar_container
=======

>>>>>>> Arrumando o serviço
=======

>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depositar_container");
    ros::NodeHandle node;
    
    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("depositar_container", depositar_container);

    ros::spin();

    return 0;
}