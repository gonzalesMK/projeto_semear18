#include <ros/ros.h>
#include <projeto_semear/DepositarContainer.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>
#include <cstdint>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "depositar_client");
    ros::NodeHandle node;

    // Cria o serviço
    ros::ServiceClient depositar_client = node.serviceClient<projeto_semear::DepositarContainer>("depositar_container");

    projeto_semear::DepositarContainer srv;

    depositar_client.call(srv);
    return 0;
}
