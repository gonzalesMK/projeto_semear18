#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>

typedef actionlib::SimpleActionClient<projeto_semear::navigationAction> Client;
void print_path(const std::vector<std::uint8_t> path);

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::navigationFeedbackConstPtr& feedback)
{
  print_path(feedback->path);
  ROS_INFO_STREAM("CODE: " << (int) feedback->code);
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState& state,
            const projeto_semear::navigationResultConstPtr& result)
{
  ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    Client client("navigation", true); // true -> don't need ros::spin()
    
    ROS_INFO("Waiting for Navigation Server");
    client.waitForServer();
    projeto_semear::navigationGoal goal;

    // Filling goal here
    goal.goal_pose.location = goal.goal_pose.QUADRANTE_DIREITO;
    goal.goal_pose.orientation = goal.goal_pose.LESTE;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());
    return 0;
}

void print_path(const std::vector<std::uint8_t> path)
{
  std::ostringstream foo; 
  for (auto it = path.begin(); it != path.end(); it++)
  {
    foo << (int) (*it) << " ";
  }
  ROS_INFO_STREAM(foo.str());
}
