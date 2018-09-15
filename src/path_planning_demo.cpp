#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/PathPlanning.h>
#include <sstream>      // std::ostringstream
#include <vector>

/** This program is Testing the algoritm Dijkstra implemented in path_planning_Server;
 * So, it will ask for all paths and print it in the screen
 * */

const int N_PLACES = 7 ; // number of places in the map == number of lines in the Graph

// Function to format the path to the screen
void print_path(const std::vector<std::uint32_t> path);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathPlanningDemo");
  ros::NodeHandle nh;

  // Instantiate PathPlanning Service Client
  ros::ServiceClient path_client = nh.serviceClient<projeto_semear::PathPlanning>("pathPlanning");

  // Fill PathPlanning service msg
  projeto_semear::PathPlanning srv_msg;
  srv_msg.request.initial_pose.location = 0;
  srv_msg.request.initial_pose.orientation = 0;
  srv_msg.request.goal_pose.location = 0;
  srv_msg.request.goal_pose.orientation = 0;

  ROS_INFO("Starting Test For Dijkstra");
  for (int i = 0; i < N_PLACES; i++)
  {
    for (int j = i; j < N_PLACES ; j++)
    {
      ROS_INFO_STREAM("----"<< j << "----" << i);

      // Set initial State j and goal i
      srv_msg.request.initial_pose.location = j;
      srv_msg.request.goal_pose.location = i;
      
      if (!path_client.call(srv_msg)) ROS_ERROR("ERROR IN SERVICE");
      print_path(srv_msg.response.path);

      // Set initial State i and goal j
      srv_msg.request.initial_pose.location = i;
      srv_msg.request.goal_pose.location = j;

      if (!path_client.call(srv_msg)) ROS_ERROR("ERROR IN SERVICE");
      print_path(srv_msg.response.path);
    }
  }
  return 0;
}

void print_path(const std::vector<std::uint32_t> path)
{
  std::ostringstream foo; 
  for (auto it = path.begin(); it != path.end(); it++)
  {
    foo << (*it) << " ";
  }
  ROS_INFO_STREAM(foo.str());
}
