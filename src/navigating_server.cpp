#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/PathPlanning.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/kine_control.h>

#include <vector>

/** Esse código é uma AÇÃO responsável por unir todo o código de navegação.
 * Ou seja, obter uma trajetória sobre o serviço pathPlanning e conseguir
 * movimentar o robô nessa trajetória.
 * 
 * Ainda não está implementado.
 * 
 * 

Allowed Movements:
 *                  QC0|QE1|QD2|DV3|DA4|DC5|Tr6|  
 * Quadrante Cen 0 | - | 1 | 1 | 0 | 0 | 1 | 0 |
 * Quadrante Esq 1 | 1 | - | 0 | 1 | 0 | 0 | 0 |
 * Quadrante Dir 2 | 1 | 0 | - | 0 | 1 | 0 | 1 |
 * Doca Verde    3 | 0 | 1 | 0 | - | 0 | 0 | 0 |
 * Doca Azul     4 | 0 | 0 | 1 | 0 | - | 0 | 0 |
 * Doca Central  5 | 1 | 0 | 0 | 0 | 0 | - | 0 |
 * Trem          6 | 0 | 0 | 1 | 0 | 0 | 0 | - |
*/

typedef actionlib::SimpleActionServer<projeto_semear::navigationAction> Server;

ros::ServiceClient gps_client;  // Cliente para o Serviço gps
ros::ServiceClient path_client; // Cliente para o Serviço path_planning

void execute(const projeto_semear::navigationGoalConstPtr &goal, Server *as, kineControl::robot &robot)
{
    // Get Actual Pose of the Robot
    projeto_semear::GetPose pose_srv;
    pose_srv.request.set = false;
    if (!gps_client.call(pose_srv))
    {
        ROS_ERROR("Can't find gps server");
        return;
    }

    // Use o serviço GPS para conseguir o PATH
    projeto_semear::PathPlanning path_srv;
    path_srv.request.goal_pose = goal->goal_pose;
    path_srv.request.initial_pose = pose_srv.response.pose;
    while (!path_client.call(path_srv))
    {
        ROS_ERROR("Can't find pathPlanning server");
        ros::Duration(1).sleep();
        path_client.waitForExistence();
        
        
    }
    std::vector<std::uint32_t> path = path_srv.response.path;

    // Debug
    //ROS_INFO_STREAM("GOAL: " << (int)path_srv.request.goal_pose.location << " INITIAL: " << (int)path_srv.request.initial_pose.location);

    // Envie um feedback preliminar com o caminho
    projeto_semear::navigationFeedback feedback;
    feedback.path = path;
    feedback.code = -1;
    as->publishFeedback(feedback);

    if (path.empty())
    {
        ROS_INFO("Empty path");
        return;
    }
    for (auto it = path.begin(); it != --path.end(); it++)
    {

        /** O código para o switch é composto por 2 dígitos: XY
         *  X = from
        *  Y = To 
        */
        auto it_pose = it;
        auto it_actual_goal = it_pose + 1;

        unsigned int code = 10 * (*it_pose) + (*it_actual_goal);
        //ROS_INFO_STREAM(code);
        feedback.code = code;
        as->publishFeedback(feedback);

        switch (code)
        {
        case (01):
            kineControl::esquerda(robot);
            break;
        case (02):
            kineControl::direita(robot);
            break;
        case (20):
            kineControl::esquerda(robot);
            break;
        case (10):
            kineControl::direita(robot);
            break;
        case (05):
            break;
        case (13):
            kineControl::ir_doca(robot);
            break;
        case (24):
            kineControl::ir_doca(robot);
            break;
        case (26):
            ROS_ERROR("NAO IMPLEMENTADO");
            break;
        case (31):
            kineControl::ir_quadrante(robot);
            break;
        case (42):
            kineControl::ir_quadrante(robot);
            break;
        case (50):
            kineControl::linha_preta(robot);
            break;
        case (62):
            ROS_ERROR("NAO IMPLEMENTADO");
            break;
        }

        // Update GPS
        pose_srv.request.set = true;
        pose_srv.request.pose.location = (*it_actual_goal);
        pose_srv.request.pose.orientation = pose_srv.request.pose.TREM;
        gps_client.call(pose_srv);
    }

    projeto_semear::navigationResult result;
    result.succeed = true;

    as->setSucceeded(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigationServer");
    ros::NodeHandle node;

    gps_client = node.serviceClient<projeto_semear::GetPose>("gps"); // Requisita o serviço gps
    gps_client.waitForExistence();

    path_client = node.serviceClient<projeto_semear::PathPlanning>("pathPlanning"); // Requisita o serviço path_planning
    path_client.waitForExistence();

    kineControl::robot robot;

    // Cria o action server
    Server server(node, "navigation", boost::bind(&execute, _1, &server, boost::ref(robot)), false);
    server.start();

    ros::spin();

    return 0;
}