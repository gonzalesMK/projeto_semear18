#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <std_msgs/Float64.h>
#include <projeto_semear/Vel_Elevadores.h>

/** Esse programa faz uma ligação entre o tópico /cmd_vel publicado pelo teleop_twist_keyboard
 *  e o controle do eletroima. Assim, é possivel move-lo utilizando o teclado .
 * */
typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;
typedef actionlib::SimpleActionClient<projeto_semear::setEletroimaAction> Client2;

ros::Publisher H_Motor_pub;
ros::Publisher V_Motor_pub;

projeto_semear::Vel_Elevadores vel_msg;

void controlH_callback(const std_msgs::Float64ConstPtr &msg)
{
  vel_msg.CremalheiraHorizontal = msg->data;
}
void controlV_callback(const std_msgs::Float64ConstPtr &msg)
{
  vel_msg.CremalheiraVertical = msg->data;
}

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM("Distance to Goal" << feedback->distance);
}

void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM("Distance to Goal" << feedback->distance);
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
  ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Função executada quando a tarefa termina
void doneCb2(const actionlib::SimpleClientGoalState &state,
             const projeto_semear::setEletroimaResultConstPtr &result)
{
  ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{

  ROS_INFO("Goal just went active");
}

void cmd_callback(const geometry_msgs::TwistConstPtr &msg)
{

  Client client("moveEletroima", true); // true -> don't need ros::spin()
  client.waitForServer();
  projeto_semear::moveEletroimaGoal goal;

  // Filling goal here
  goal.deslocamento.linear.x = msg->linear.x;
  goal.deslocamento.linear.y = msg->linear.y;
  goal.deslocamento.linear.z = msg->linear.z;
  goal.deslocamento.angular.z = msg->angular.z;

  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  client.waitForResult(ros::Duration());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Eletroima_client");
  ros::NodeHandle nh;

  int p;
  std::cout << "Deseja testar o move(1) ou o set(2) ou PID(3): ";
  std::cin >> p;

  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_callback);
  if (p == 1)
  {
    ros::spin();
  }
  else if (p == 2)
  {
    Client2 client("setEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();
    projeto_semear::setEletroimaGoal goal;

    while (ros::ok())
    {
      ROS_INFO("Escolha a posição desejada (1 a 3, sair 5): ");
      std::cin >> goal.pose;
      if (goal.pose == 5)
      {
        return -1;
      }
      if (goal.pose == 1)
      {
        ROS_INFO("POSE == 1");
        goal.pose = goal.posicao_inicial;
      }
      client.sendGoal(goal, &doneCb2, &activeCb, &feedbackCb2);
      client.waitForResult(ros::Duration());
    }
  }
  else
  {
    ros::Publisher vel_pub = nh.advertise<projeto_semear::Vel_Elevadores>("/AMR/arduinoElevadoresInputVel", 10);
    ros::Subscriber controlH_sub = nh.subscribe<std_msgs::Float64>("AMR/H_PID/control_effort", 10, controlH_callback);
    ros::Subscriber controlV_sub = nh.subscribe<std_msgs::Float64>("AMR/V_PID/control_effort", 10, controlV_callback);

    while (ros::ok())
    {
      vel_pub.publish(vel_msg);
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }
  }

  return 0;
}