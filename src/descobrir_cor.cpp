#include <ros/ros.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/Pose.h>
#include <std_msgs/Float32.h>

void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback); // Função de feedback do ActionLib
void doneCb(const actionlib::SimpleClientGoalState &state,                      // Função executada quando a tarefa termina
            const projeto_semear::moveEletroimaResultConstPtr &result);
void activeCb(); // Called once when the goal becomes active
void callbackGarraR(const std_msgs::Float32 &msg);
void callbackGarraL(const std_msgs::Float32 &msg);

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;

ros::ServiceClient get_client;
ros::ServiceClient set_client;
ros::ServiceClient pose_client;

ros::Subscriber subGarraR;
ros::Subscriber subGarraL;

int cor_garra_L, cor_garra_R;

// Service
bool descobrirCor(projeto_semear::DescobrirCor::Request &req,
                  projeto_semear::DescobrirCor::Response &res)
{
    // Cria o cliente do eletroima
    Client client("moveEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();

    // Mover garra para Frente:
    projeto_semear::moveEletroimaGoal goal;

    goal.deslocamento.linear.x = 0.15;
    goal.deslocamento.linear.y = 0;
    goal.deslocamento.linear.z = 0;
    goal.deslocamento.angular.z = 0;
    client.sendGoal(goal, doneCb, activeCb, feedbackCb);

    // Usar reconhecimento de cor para saber quando os containers começaram
    // A fazer

    // Pegar a posição do robô para saber qual pilha checar
    projeto_semear::GetPose pose_msg;
    pose_msg.request.set = false;
    pose_client.call(pose_msg);

    int esq, dir;

    // Converte a posição do robô para os valores das pilhas
    switch (pose_msg.response.pose.location)
    {
    case projeto_semear::Pose::QUADRANTE_DIREITO:
        esq = 0;
        dir = 1;
    case projeto_semear::Pose::QUADRANTE_CENTRAL:
        esq = 2;
        dir = 3;
    case projeto_semear::Pose::QUADRANTE_ESQUERDO:
        esq = 4;
        dir = 5;
    }

    // Pegar informação das pilhas
    projeto_semear::GetContainerInfo get_esq;
    projeto_semear::GetContainerInfo get_dir;

    get_esq.request.where = esq;
    get_client.call(get_esq);

    get_dir.request.where = dir;
    get_client.call(get_dir);

    int size_esq = get_esq.response.lista.size();
    int size_dir = get_dir.response.lista.size();

    int ultimo_container_esq;
    int ultimo_container_dir;

    if (size_esq > 0)
    {
        ultimo_container_esq = get_esq.response.lista.back();
    }
    else
    {
        ultimo_container_esq = -1;
    }

    if (size_dir > 0)
    {
        ultimo_container_dir = get_dir.response.lista.back();
    }
    else
    {
        ultimo_container_dir = -1;
    }

    if (ultimo_container_dir != get_dir.response.DESCONHECIDO && ultimo_container_esq != get_esq.response.DESCONHECIDO)
    {
        // Todos os containers são conhecidos
        return true;
    }

    // Dividir nos três casos: o da esquerda já é conhecido, o da direita ja é conhecido e nenhum é conhecido, levando em consideração
    // a altura das pilhas
    projeto_semear::SetContainer set_msg;

    if (ultimo_container_esq == get_esq.response.DESCONHECIDO && ultimo_container_dir == get_dir.response.DESCONHECIDO)
    {
        if (size_esq == size_dir)
        {
            ros::spinOnce(); // atualiza a leitura da garra
        }
        else
        {
            // Girar a garra 90graus antes de fazer a leitura de cado lado
        }
    }
    else if (ultimo_container_esq == get_esq.response.DESCONHECIDO)
    {
        // Girar a garra 90graus e fazer apenas a leitura da esquerda
    }
    else if (ultimo_container_dir == get_dir.response.DESCONHECIDO)
    {
        // Girar a garra 90graus e fazer apenas a leitura da direita
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Descobrir Cor");
    ros::NodeHandle node;

    // Requisita os serviços do Mapa de Containers
    get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    set_client = node.serviceClient<projeto_semear::SetContainer>("setContainer");

    // ActionServer para mover o Eletroima
    Client client("moveEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();

    // Serviço da posição do robô
    pose_client = node.serviceClient<projeto_semear::GetPose>("gps");

    // Leitura dos sensores da Garra
    subGarraR = node.subscribe("/image_converter/sensorGarraR", 1000, callbackGarraR);
    subGarraL = node.subscribe("/image_converter/sensorGarraL", 1000, callbackGarraL);

    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("descobrir_cor", descobrirCor);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Distance to Goal" << feedback->distance);
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
    ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}
void callbackGarraR(const std_msgs::Float32 &msg)
{
    cor_garra_R = msg.data;
}
void callbackGarraL(const std_msgs::Float32 &msg)
{
    cor_garra_L = msg.data;
}
