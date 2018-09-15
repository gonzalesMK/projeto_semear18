#include <projeto_semear/kine_control.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/navigationAction.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/DepositarContainer.h>
#include <projeto_semear/Strategy.h>

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

    // Serviço da Estratégia
    ros::ServiceClient estrategia_srv = nh.serviceClient<projeto_semear::Strategy>("Estrategia");
    estrategia_srv.waitForExistence();

    projeto_semear::Strategy estrategia_msg;

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

    // Descobrir cor do container
    ROS_INFO("Descobrindo containers");
    descobrir_cor_srv.call(descobrir_container_msg);

    while (!fim && ros::ok())
    {
        // Decidindo próximo passo
        ROS_INFO("Decidindo proximo passo");
        estrategia_srv.call(estrategia_msg);
        ROS_INFO_STREAM("Estrategia: cor - " << estrategia_msg.response.cor << " - container escolhido (0 - 1) : " << estrategia_msg.response.container_escolhido << " pilha:" << estrategia_msg.response.pilha << "To go: " << estrategia_msg.response.to_go);

        while (estrategia_msg.response.container_escolhido == 2)
        {
            navigation_msg.goal_pose = estrategia_msg.response.to_go;
            navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
            navigation_client.waitForResult();

            // Descobrir cor do container
            ROS_INFO("Descobrindo containers");
            descobrir_cor_srv.call(descobrir_container_msg);

            // Decidindo próximo passo
            ROS_INFO("Decidindo proximo passo");
            estrategia_srv.call(estrategia_msg);
        }

        // Alinhar com a pilha escolhida
        ROS_INFO("Alinhar com com a pilha escolhida");
        kineControl::alinhar_pilha(robot, estrategia_msg.response.container_escolhido);

        // Pegar container
        ROS_INFO("Pegando Container");
<<<<<<< HEAD
        kineControl::pegar_container(robot, escolher_container_msg.response.container_escolhido);
=======
        kineControl::pegar_container(robot, estrategia_msg.response.container_escolhido);
>>>>>>> estrategia

        // Levando o container para doca correta
        ROS_INFO("Indo para Doca Correta");
        navigation_msg.goal_pose = estrategia_msg.response.to_go;
        navigation_client.sendGoal(navigation_msg, &doneCb, &activeCb, &feedbackCb);
        navigation_client.waitForResult();

        // Depositando  o container
        ROS_INFO("Depositando Container");
<<<<<<< HEAD
        depositar_msg.request.posicao_origem_do_container = proximo_quadrante;
        depositar_msg.request.dir_ou_esq = escolher_container_msg.response.container_escolhido;
=======
        depositar_msg.request.posicao_origem_do_container = estrategia_msg.response.pilha;
        depositar_msg.request.dir_ou_esq = estrategia_msg.response.container_escolhido;
>>>>>>> estrategia
        depositar_srv.call(depositar_msg);

        // Voltando para doca mais próxima
        int mais_proximo = estrategia_msg.response.to_go.location == navigation_msg.goal_pose.DOCA_AZUL ? navigation_msg.goal_pose.QUADRANTE_DIREITO : navigation_msg.goal_pose.QUADRANTE_ESQUERDO;
        ROS_INFO_STREAM("Voltando para o Quadrante mais proximo: " << mais_proximo);
        navigation_msg.goal_pose.location = mais_proximo;
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
