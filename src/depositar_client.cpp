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

<<<<<<< HEAD
<<<<<<< 895e33a0c017e2b2cc5b476424d7b09857cb7ef3
<<<<<<< 3b2c4a9aa6091ee9d756a04c3bbf9cbd71c3032f
=======
    srv.request.code = 1;
>>>>>>> Cliente teste depositar_container
=======
>>>>>>> Arrumando o serviço
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
    depositar_client.call(srv);
    return 0;
}
