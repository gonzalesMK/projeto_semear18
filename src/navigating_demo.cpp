#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <sstream>      // std::ostringstream

/** This program is Testing the algoritm Dijkstra implemented in navigation_server;
 * So, it will ask for all paths and print it in the screen
 * */
typedef actionlib::SimpleActionClient<projeto_semear::navigationAction> Client;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::navigationFeedbackConstPtr &feedback)
{
  std::ostringstream foo; 
  for (auto it = feedback->path.begin(); it != feedback->path.end(); it++)
  {
    foo << (int) (*it) << " ";
  }
  ROS_INFO_STREAM(foo.str());
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::navigationResultConstPtr &result)
{
  //ROS_INFO_STREAM("Finished in sta te");
}

// Called once when the goal becomes active
void activeCb()
{
  //ROS_INFO("Goal just went active");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Navigation_client");
  ros::NodeHandle nh;

  // Instantiate Navigation Action Client
  Client client("navigation", true); // true -> don't need ros::spin()
  client.waitForServer();
  

  // Instantiate GPS Service Client
  ros::ServiceClient pose_client = nh.serviceClient<projeto_semear::GetPose>("gps");

  // create goal msg
  projeto_semear::navigationGoal goal;

  // Fill standard service msg
  projeto_semear::GetPose srv_msg;
  srv_msg.request.set = true;
  srv_msg.request.pose.location = 0;
  srv_msg.request.pose.orientation = 0;

  ROS_INFO("Starting Test For Dijkstra");
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      // Set initial State j
      ROS_INFO_STREAM("----"<<j << "----" << i);
      
      srv_msg.request.pose.location = j;

      if (!pose_client.call(srv_msg))
      {
        ROS_ERROR("ERROR IN SERVICE");
        return -1;
      }

      // Set goal i
      goal.goal_pose.location = i;
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration(5));

      // Make de Inverse Goal
      srv_msg.request.pose.location = i;

      if (!pose_client.call(srv_msg))
      {
        ROS_ERROR("ERROR IN SERVICE");
        return -1;
      }

      // Set goal i
      goal.goal_pose.location = j;
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
      client.waitForResult(ros::Duration(5));
      ros::Duration(0.1).sleep();
    }
  }

  return 0;
}