#include <projeto_semear/kine_control.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/DepositarContainer.h>

void feedbackCb(const projeto_semear::navigationFeedbackConstPtr &feedback);
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::navigationResultConstPtr &result);
void activeCb();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    kineControl::robot robot;

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

    // Serviço GPS
    ros::ServiceClient gps_srv = nh.serviceClient<projeto_semear::GetPose>("gps");
    gps_srv.waitForExistence();

    projeto_semear::GetPose gps_msg;

    // Serviço de depositar
    ros::ServiceClient depositar_srv = nh.serviceClient<projeto_semear::DepositarContainer>("depositar_container");
    depositar_srv.waitForExistence();

    projeto_semear::DepositarContainer depositar_msg;

    // Indo para linha Preta
    ROS_INFO("Indo para linha preta!");
    navigation_msg.goal_pose.location = navigation_msg.goal_pose.QUADRANTE_CENTRAL;
    navigation_msg.goal_pose.orientation = navigation_msg.goal_pose.TREM;

    navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
    navigation_client.waitForResult();

    bool fim = false;
    int proximo_quadrante = -1;
    int antigo_quadrante = -1;

    // Descobrir cor do container
    ROS_INFO("Descobrindo containers");
    descobrir_cor_srv.call(descobrir_container_msg);

    while (!fim && ros::ok())
    {
        // Escolhendo containers
        ROS_INFO("ESCOLHER CONTAINER");
        escolher_container_srv.call(escolher_container_msg);

        if (escolher_container_msg.response.container_escolhido == 2)
        {
            ROS_INFO("Descobrindo containers");
            descobrir_cor_srv.call(descobrir_container_msg);
            // Escolhendo containers
            escolher_container_srv.call(escolher_container_msg);
        }

        // Caso não haja um container nesta pilha, ir para outra pilha.
        gps_msg.request.set = false;
        gps_srv.call(gps_msg);
        antigo_quadrante = gps_msg.response.pose.location;
        proximo_quadrante = gps_msg.response.pose.location;
        while (escolher_container_msg.response.container_escolhido == 2 && ros::ok())
        {
            ROS_INFO("Dentro do While nao encontrar container escolhido");
            gps_msg.request.set = false;
            gps_srv.call(gps_msg);

            if (gps_msg.response.pose.location == 1) // esquerda
            {
                proximo_quadrante = 0;
                antigo_quadrante = 1;
            }
            else if (gps_msg.response.pose.location == 0) // centro
            {
                if (proximo_quadrante == -1)
                {
                    proximo_quadrante = 1;
                    antigo_quadrante = 0;
                }
                else if (antigo_quadrante == 2)
                {
                    proximo_quadrante = 1;
                    antigo_quadrante = 0;
                }
                else if (antigo_quadrante == 1)
                {
                    proximo_quadrante = 2;
                    antigo_quadrante = 0;
                }
            }
            else if (gps_msg.response.pose.location == 2) // direita
            {
                proximo_quadrante = 0;
                antigo_quadrante = 2;
            }

            // Movendo o Robô para o próximo quadrante
            ROS_INFO("Procurando em outro quadrante");
            navigation_msg.goal_pose.location = antigo_quadrante;
            navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
            navigation_client.waitForResult();

            // Descobrir cor do container
            ROS_INFO("Descobrindo containers");
            descobrir_cor_srv.call(descobrir_container_msg);

            // Escolhendo containers
            ROS_INFO("ESCOLHENDO CONTAINER");
            escolher_container_srv.call(escolher_container_msg);
        }

        // Alinhar com a pilha escolhida
        ROS_INFO("Alinhancon com a pilha escolhida");
        kineControl::alinhar_pilha(robot, escolher_container_msg.response.container_escolhido);

        // Pegar container
        ROS_INFO("Pegando Container");
        kineControl::pegar_container(robot, escolher_container_msg.response.container_escolhido);

        // Levando o container para doca correta
        ROS_INFO("Indo para Doca Correta");
        navigation_msg.goal_pose.location = navigation_msg.goal_pose.DOCA_VERDE;
        navigation_msg.goal_pose.orientation = navigation_msg.goal_pose.LESTE;
        navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
        navigation_client.waitForResult();

        // Depositando  o container
        ROS_INFO("Depositando Container");
        depositar_msg.request.posicao_origem_do_container = proximo_quadrante;
        depositar_msg.request.dir_ou_esq = escolher_container_msg.response.container_escolhido;
        depositar_srv.call(depositar_msg);

        // Voltando para doca mais próxima
        ROS_INFO_STREAM("Voltando para o Quadrante Mais próximo: " <<proximo_quadrante);
        navigation_msg.goal_pose.location = proximo_quadrante;
        navigation_msg.goal_pose.orientation = navigation_msg.goal_pose.LESTE;
        navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
        navigation_client.waitForResult();
    }
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
