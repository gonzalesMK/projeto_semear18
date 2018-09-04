#include <ros/ros.h>
#include <ros/ros.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <projeto_semear/DescobrirCor.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>

/* Função responsável por, após o robô estar posicionado em relação aos containers, fazer a leitura das cores
dos containers superiores. 

Ele depende do posicionamento atual do robô, para saber em qual pilha olhar, e do mapa de containers, para saber
quais containers precisam ter as cores descobertas.

O serviço não tem entrada nem saida... poderia ser transformado numa função. */

// Funções padrões para actionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback); // Função de feedback do ActionLib
void doneCb(const actionlib::SimpleClientGoalState &state,                      // Função executada quando a tarefa termina
            const projeto_semear::moveEletroimaResultConstPtr &result);
void activeCb();
void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback); // Função de feedback do ActionLib
void doneCb2(const actionlib::SimpleClientGoalState &state,                      // Função executada quando a tarefa termina
            const projeto_semear::setEletroimaResultConstPtr &result);
void activeCb();
// Essas duas funções classificam a saída do sensor. RGB em suas cores respectivas
void callbackGarraR(const std_msgs::ColorRGBA &msg);
void callbackGarraL(const std_msgs::ColorRGBA &msg);

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> MoveClient;
typedef actionlib::SimpleActionClient<projeto_semear::setEletroimaAction> SetClient;

ros::ServiceClient get_client;
ros::ServiceClient set_client;
ros::ServiceClient pose_client;

ros::Subscriber subGarraR;
ros::Subscriber subGarraL;

ros::Publisher enable_eletroima_pub;

int cor_garra_L, cor_garra_R;

// Service
bool descobrirCor(projeto_semear::DescobrirCor::Request &req,
                  projeto_semear::DescobrirCor::Response &res)
{

    // Desativa o Eletroiman
    std_msgs::Bool msg;
    msg.data = false;
    enable_eletroima_pub.publish(msg);

    // Cria o cliente do eletroima
    MoveClient move_client("moveEletroima", true); // true -> don't need ros::spin()
    move_client.waitForServer();
    SetClient set_eletroima_client("setEletroima", true); // true -> don't need ros::spin()
    set_eletroima_client.waitForServer();
    // Usar reconhecimento de cor para saber quando os containers começaram
    // A fazer

    // Pegar a posição do robô para saber qual pilha checar
    projeto_semear::GetPose pose_msg;
    pose_msg.request.set = false;
    pose_client.call(pose_msg);

    kineControl::robot robot;

    int esq, dir;

    // Converte a posição do robô para os valores das pilhas
 //   ROS_INFO_STREAM("Location: " << (int)pose_msg.response.pose.location);
    switch (pose_msg.response.pose.location)
    {
    case projeto_semear::Pose::QUADRANTE_ESQUERDO:
        esq = 0;
        dir = 1;
        break;
    case projeto_semear::Pose::QUADRANTE_CENTRAL:
        esq = 2;
        dir = 3;
        break;
    case projeto_semear::Pose::QUADRANTE_DIREITO:
        esq = 4;
        dir = 5;
        break;
    default:
        ROS_ERROR(" Localizacao do robo pode estar errada! Nenhuma foi escolhida");
        return false;
    }

  //  ROS_INFO_STREAM("Esq: " << esq << " Dir: " << dir);

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

    // Mover garra para Frente:
    projeto_semear::moveEletroimaGoal move_goal;
    projeto_semear::setEletroimaGoal set_goal;

    ROS_INFO("Alinhando robô");
    kineControl::alinhar_pilha(robot, 2);
    ROS_INFO("Centralizando garra");
    set_goal.pose = set_goal.posicao_pegar_container_superior;
    set_eletroima_client.sendGoal(set_goal, doneCb2, activeCb, feedbackCb2);
    set_eletroima_client.waitForResult(ros::Duration()); /// Espera terminar a movimentação do container

    // Mensagem para atualizar o mapa de containers
    projeto_semear::SetContainer set_msg;

    // Dividir nos casos: quando a pilha tiver o mesmo tamanho, é possivel ler os dois containers de uma vez. Caso contrário, irá ler o da esquerda e o da direita separadamente
    if (size_esq == size_dir)
    {
        ROS_INFO("Verificando os dois containers com mesmo tamanho");

        // Desce até os containers
        move_goal.deslocamento.linear.x = 0.0;
        move_goal.deslocamento.linear.y = 0;
        move_goal.deslocamento.linear.z = -0.04 * (4 - size_esq);
        move_goal.deslocamento.angular.z = 0;
        move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
        move_client.waitForResult(ros::Duration());
        ros::spinOnce(); // atualiza a leitura da garra

        // Atualiza o container da esquerda
        set_msg.request.where = esq;
        set_msg.request.color = cor_garra_L;

        if (cor_garra_L == set_msg.request.DESCONHECIDO)
        {
            ROS_ERROR("Container da esquerda nao foi identificado");
        }
        set_client.call(set_msg);

        // Atualiza o container da direita
        set_msg.request.where = dir;
        set_msg.request.color = cor_garra_R;
        if (cor_garra_R == set_msg.request.DESCONHECIDO)
        {
            ROS_ERROR("Container da direita nao foi identificado");
        }
        set_client.call(set_msg);
    }
    else
    {
        if (ultimo_container_esq == get_esq.response.DESCONHECIDO)
        {
            ROS_INFO("Verificando o container da esquerda");

            // Girar garra 90 graus
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0;
            move_goal.deslocamento.angular.z = 3.14 / 2;

            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);

            // Alinhar com esquerda
            kineControl::alinhar_pilha(robot, 0);

            move_client.waitForResult(ros::Duration());

            // Desce até o container
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = -0.04 * (4 - size_esq);
            move_goal.deslocamento.angular.z = 0;

            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());

            // Realiza a leitura
            ros::spinOnce();

            // Atualiza o mapa de containers
            set_msg.request.where = esq;
            set_msg.request.color = cor_garra_L;
            if (cor_garra_L == set_msg.request.DESCONHECIDO)
            {
                ROS_ERROR("Container da esquerda nao foi identificado");
            }
            set_client.call(set_msg);

            // Levanta a garra
            move_goal.deslocamento.linear.x = 0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0.04 * (4 - size_esq);
            move_goal.deslocamento.angular.z = 0;
            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());

            // Ajeita a garra
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0;
            move_goal.deslocamento.angular.z = -3.14 / 2;
            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());
        }

        if (ultimo_container_dir == get_dir.response.DESCONHECIDO)
        {
            ROS_INFO("Verificando o container da direita");
            // Girar garra 90 graus
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0;
            move_goal.deslocamento.angular.z = 3.14 / 2;

            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);

            // Alinha com o container da direita
            kineControl::alinhar_pilha(robot, 1);
            move_client.waitForResult(ros::Duration());

            // Desce até o container
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = -0.04 * (4 - size_dir);
            move_goal.deslocamento.angular.z = 0;
            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());

            // Realiza a leitura
            ros::spinOnce();

            // Atualiza o mapa de containers
            set_msg.request.where = dir;
            set_msg.request.color = cor_garra_R;
            if (cor_garra_R == set_msg.request.DESCONHECIDO)
            {
                ROS_ERROR("Container da direita nao foi identificado");
            }
            set_client.call(set_msg);

            // Levanta a Garra
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0.04 * (4 - size_dir);
            move_goal.deslocamento.angular.z = 0;
            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());

            // Ajeita a garra
            move_goal.deslocamento.linear.x = 0.0;
            move_goal.deslocamento.linear.y = 0;
            move_goal.deslocamento.linear.z = 0;
            move_goal.deslocamento.angular.z = -3.14 / 2;
            move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
            move_client.waitForResult(ros::Duration());
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "descobrirCor");
    ros::NodeHandle node;

    // Requisita os serviços do Mapa de Containers
    get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    set_client = node.serviceClient<projeto_semear::SetContainer>("setContainer");

    get_client.waitForExistence();
    set_client.waitForExistence();

    // Serviço da posição do robô
    pose_client = node.serviceClient<projeto_semear::GetPose>("gps");

    // Para desligar o eletroimã:
    enable_eletroima_pub = node.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);

    // Leitura dos sensores da Garra
    subGarraR = node.subscribe("/image_converter/sensorGarraR", 1000, callbackGarraR);
    subGarraL = node.subscribe("/image_converter/sensorGarraL", 1000, callbackGarraL);

    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("descobrir_cor", descobrirCor);

    ros::spin();

    return 0;
}

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
    ROS_INFO_STREAM("Finished in state" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

int reconhece_cor(const std_msgs::ColorRGBA &msg)
{
    std::vector<double> dist;
    dist.push_back(sqrt(pow(msg.r - 255, 2) + pow(msg.g, 2) + pow(msg.b, 2)));             // Distância até o vermelho
    dist.push_back(sqrt(pow(msg.r, 2) + pow(msg.g - 255, 2) + pow(msg.b, 2)));             // Distância até o verde
    dist.push_back(sqrt(pow(msg.r, 2) + pow(msg.g, 2) + pow(msg.b - 255, 2)));             // Distância até o azul
    dist.push_back(sqrt(pow(msg.r - 255, 2) + pow(msg.g - 255, 2) + pow(msg.b - 255, 2))); // Distância até o branco
    dist.push_back(sqrt(pow(msg.r, 2) + pow(msg.g, 2) + pow(msg.b, 2)));                   // Distância até o preto

    int argMin = std::min_element(dist.begin(), dist.end()) - dist.begin(); // Retorna a posição do menor elemento

    if (argMin == 0)
    {
        return projeto_semear::SetContainer::Request::VERMELHO;
    }
    else if (argMin == 1)
    {
        return projeto_semear::SetContainer::Request::VERDE;
    }
    else if (argMin == 2)
    {
        return projeto_semear::SetContainer::Request::AZUL;
    }
    else
    {
        return projeto_semear::SetContainer::Request::DESCONHECIDO;
    }
}
void callbackGarraR(const std_msgs::ColorRGBA &msg)
{
    cor_garra_R = reconhece_cor(msg);
}
void callbackGarraL(const std_msgs::ColorRGBA &msg)
{
    cor_garra_L = reconhece_cor(msg);
}
