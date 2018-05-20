#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>

/** Esse programa faz uma ligação entre o tópico /cmd_vel publicado pelo teleop_twist_keyboard
 *  e o controle do eletroima. Assim, é possivel move-lo utilizando o teclado .
 * */


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
  ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}


void cmd_callback(const geometry_msgs::TwistConstPtr &msg){

  Client client("moveEletroima", true); // true -> don't need ros::spin()
  client.waitForServer();
  projeto_semear::moveEletroimaGoal goal;

  // Filling goal here  
  goal.deslocamento.linear.x  = msg->linear.x;
  goal.deslocamento.linear.y  = msg->linear.y;
  goal.deslocamento.linear.z  = msg->linear.z;
  goal.deslocamento.angular.z = msg->angular.z;

  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  client.waitForResult(ros::Duration());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Eletroima_client");
  ros::NodeHandle nh;

  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_callback);

  ros::spin();

  return 0;
}