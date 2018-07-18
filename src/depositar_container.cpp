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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Eletroima_client");
  ros::NodeHandle nh;

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

  ROS_INFO_STREAM("desligando o eletroima");
  msg.data = false;
  pub.publish(msg);


  client.waitForResult(ros::Duration());

  return 0;
}