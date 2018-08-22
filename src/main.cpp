#include <projeto_semear/kine_control.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/EscolherContainer.h>

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

    // Serviço de escolher container
    ros::ServiceClient escolher_container_srv = nh.serviceClient<projeto_semear::EscolherContainer>("EscolherContainer"); 
    escolher_container_srv.waitForExistence();

    projeto_semear::EscolherContainer escolher_container_msg;


    // Indo para linha Preta
    navigation_msg.goal_pose.location = navigation_msg.goal_pose.QUADRANTE_CENTRAL;
    navigation_msg.goal_pose.location = navigation_msg.goal_pose.TREM;

    navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
    navigation_client.waitForResult();

    bool fim = false;
    while (!fim)
    {
        // Aproximar da pilha 
        //kineControl::aproximar_pilha(motor);


        // Escolhendo containers
        escolher_container_srv.call(escolher_container_msg);

        // Na primeira vez que der NENHUM, verificar se todos os containers estão descobertos
 /////////////////// Englobar tudo dentro de Descobrir container ////////////////////////////////////////////
        // Alinhando com o container da esquerda primeiro
        kineControl::alinhar_pilha(motor, 0);
        
        // Descobrir cor do container
        descobrir_cor_srv.call(descobrir_container_msg);

        // Alinhando com o container da direita
        kineControl::alinhar_pilha(motor, 1);

        // Descobrir cor do container
        descobrir_cor_srv.call(descobrir_container_msg);
//////////////////////////////////////////////////////////////////////
        
        // Escolhendo containers
        escolher_container_srv.call(escolher_container_msg);

        // Alinhar com a pilha escolhida
        kineControl::alinhar_pilha(motor, escolher_container_msg.response.container_escolhido);

        // Levando o container para doca correta
        navigation_msg.goal_pose.location = navigation_msg.goal_pose.DOCA_VERDE;
        navigation_msg.goal_pose.orientation = navigation_msg.goal_pose.LESTE;

        navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
        navigation_client.waitForResult();

        // Depositando  o container
        kineControl::

        // Voltando para doca mais próxima
    }

    navigation_client.waitForResult(ros::Duration());
}