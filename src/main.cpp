#include <projeto_semear/kine_control.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/DescobrirCor.h>

typedef Client;
void print_path(const std::vector<std::uint8_t> path);
void feedbackCb(const projeto_semear::navigationFeedbackConstPtr &feedback);
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::navigationResultConstPtr &result);
void activeCb();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    kineControl::robot motor;

    // Actionlib de navegação
    actionlib::SimpleActionClient<projeto_semear::navigationAction> navigation_client("navigation", true);
    navigation_client.waitForServer();
    
    projeto_semear::navigationGoal navigation_msg; // mensagem para a action lib de navegação

    // Serviço de descobrir container
    ros::ServiceClient descobrir_cor_srv = nh.serviceClient<projeto_semear::DescobrirCor>("descobrir_cor");
    descobrir_cor_srv.waitForExistence();

    projeto_semear::DescobrirCor descobrir_container_msg; // mensagem para o serviço de descobrir container


    // Indo para linha Preta
    kineControl::linha_preta(motor);

    bool fim = false;
    while (!fim)
    {
        // Alinhando com os containers

        // Descobrindo cores dos containers
        descobrir_cor_srv.call(descobrir_container_msg);
        
        // Escolhendo containers

        // Levando o container para doca correta
        goal.goal_pose.location = goal.goal_pose.DOCA_VERDE;
        goal.goal_pose.orientation = goal.goal_pose.LESTE;

        navigation_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        navigation_client.waitForResult();

        // Depositando  o container

        // Voltando para doca mais próxima
    }

    client.waitForResult(ros::Duration());
}