#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/PathPlanning.h>
#include <projeto_semear/navigationAction.h>
#include <vector>

typedef actionlib::SimpleActionServer<projeto_semear::navigationAction> Server;

ros::ServiceClient gps_client;  // Cliente para o Serviço gps
ros::ServiceClient path_client; // Cliente para o Serviço path_planning

void execute(const projeto_semear::navigationGoalConstPtr &goal, Server *as)
{

    // Get Actual Pose of the Robot
    projeto_semear::GetPose pose_srv;
    pose_srv.request.set = false;
    if (!gps_client.call(pose_srv))
    {
        ROS_ERROR("Can't find gps server");
        return;
    }
    
    // Get PATH
    projeto_semear::PathPlanning path_srv;
    path_srv.request.goal_pose = goal->goal_pose;
    path_srv.request.initial_pose = pose_srv.response.pose;
    if (!gps_client.call(pose_srv))
    {
        ROS_ERROR("Can't find pathPlanning server");
        return;
    }

    std::vector<std::uint8_t> path = path_srv.response.path;
    
    projeto_semear::navigationFeedback feedback;
    feedback.path = path;
    
    while (!path.empty())
    {

        /** O código para o switch é composto por 2 dígitos: XY
         *  X = from
        *  Y = To 
        */
        auto it_pose = path.begin();
        auto it_actual_goal = it_pose + 1;

        unsigned int code = 10 * (*it_pose) + (*it_actual_goal);
        
        feedback.code = code;
        as->publishFeedback(feedback);
        
        switch (code)
        {
        case (01):
            break;
        case (02):
            break;
        case (05):
            break;
        case (10):
            break;
        case (13):
            break;
        case (20):
            break;
        case (24):
            break;
        case (26):
            break;
        case (31):
            break;
        case (42):
            break;
        case (50):
            break;
        case (62):
            break;
        }
    }
    
    projeto_semear::navigationResult result;
    result.succeed = true;

    as->setSucceeded(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigationServer");
    ros::NodeHandle node;

    gps_client = node.serviceClient<projeto_semear::GetPose>("gps");           // Requisita o serviço gps
    path_client = node.serviceClient<projeto_semear::GetPose>("pathPlanning"); // Requisita o serviço path_planning

    // Cria o action server
    Server server(node, "navigation", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}