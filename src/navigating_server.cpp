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

void execute(const projeto_semear::navigationGoalConstPtr &goal, Server *as, kineControl::robot& motor)
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
        case (02):
            kineControl::quadrante_central2(motor, (*it_pose), (*it_actual_goal));
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

    kineControl::robot robot;
    // Cria o action server
    Server server(node, "navigation", boost::bind(&execute, _1, &server, robot), false);
    server.start();

    ros::spin();

    return 0;
}