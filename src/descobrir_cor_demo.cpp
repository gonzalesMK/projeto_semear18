#include <projeto_semear/kine_control.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/MoveContainer.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>

// Funções padrões para actionLib
void feedbackCb(const projeto_semear::navigationFeedbackConstPtr &feedback);
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::navigationResultConstPtr &result);
void activeCb();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");
    ros::NodeHandle nh;

    kineControl::robot motor;

    // Serviço de descobrir container
    ros::ServiceClient descobrir_cor_srv = nh.serviceClient<projeto_semear::DescobrirCor>("descobrir_cor");
    descobrir_cor_srv.waitForExistence();

    projeto_semear::DescobrirCor descobrir_container_msg; // mensagem para o serviço de descobrir container

    // Serviços do mapa de container
    ros::ServiceClient get_client = nh.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    ros::ServiceClient set_client = nh.serviceClient<projeto_semear::SetContainer>("setContainer");
    ros::ServiceClient move_client = nh.serviceClient<projeto_semear::MoveContainer>("moveContainer");

    projeto_semear::MoveContainer move_srv;
    projeto_semear::GetContainerInfo get_srv;
    projeto_semear::SetContainer set_srv;

    // Actionlib de navegação
    actionlib::SimpleActionClient<projeto_semear::navigationAction> navigation_client("navigation", true);
    navigation_client.waitForServer();

    projeto_semear::navigationGoal navigation_msg; // mensagem para a action lib de navegação

    // Indo para linha Preta
    navigation_msg.goal_pose.location = navigation_msg.goal_pose.QUADRANTE_CENTRAL;
    navigation_msg.goal_pose.orientation = navigation_msg.goal_pose.TREM;

    navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
    navigation_client.waitForResult();

    ROS_INFO("DESCOBRINDO COR");
    descobrir_cor_srv.call(descobrir_container_msg);
    move_srv.request.where = 2;
    move_client.call(move_srv);

    ROS_INFO("DESCOBRINDO COR COM O DA ESQUERDA MAIS ABAIXO");
    descobrir_cor_srv.call(descobrir_container_msg);

    move_srv.request.where = 3;
    move_client.call(move_srv);

    ROS_INFO("DESCOBRINDO COR");
    descobrir_cor_srv.call(descobrir_container_msg);

    move_srv.request.where = 3;
    move_client.call(move_srv);

    ROS_INFO("DESCOBRINDO COR COM O DA DIREITA MAIS ABAIXO");
    descobrir_cor_srv.call(descobrir_container_msg);
    
    return 0;
}

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::navigationFeedbackConstPtr &feedback)
{
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::navigationResultConstPtr &result)
{
}

// Função que é chamada quando a GOAL se torna ATIVA
void activeCb()
{
}
