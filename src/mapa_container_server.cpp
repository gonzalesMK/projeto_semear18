#include <ros/ros.h>
#include <vector>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <projeto_semear/MoveContainer.h>

// Cores dos containers
enum cores
{
    AZUL = 13,
    VERDE = 12,
    VERMELHO = 14,
    DESCONHECIDO = 255
};

// Função para checar se o número enviado para a posição da pilha está entre 0 e 14
bool checar_limites(int inteiro)
{
    if (inteiro >= 0 && inteiro <= 14)
    {
        return true;
    }
    ROS_ERROR_STREAM("LUGAR FORA DOS LIMITES DE 0 a 14: " << inteiro);

    return false;
}

// Função para checar se a cor enviada é VERMELHO, AZUL, VERDE ou DESCONHECIDO
bool checar_cores(std::uint32_t cor)
{
    if (!(cor != cores::VERMELHO && cor != cores::AZUL && cor != cores::VERDE  && cor != cores::DESCONHECIDO))
    {
        return true;
    }
    ROS_ERROR_STREAM("Essa cor nao existe ! : " << (int) cor);


    return false;
}

// Vetor para armazenar o conhecimento sobre os containers
std::vector<std::vector<std::uint32_t>> MAPA =
    {
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 0
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 1
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 2
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 3
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 4
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 5
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 6
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 7
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 8
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 9
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 10
        {cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO, cores::DESCONHECIDO}, // 11
        {},                                                                                   // 12 - Doca
        {},                                                                                   // 13
        {},                                                                                   // 14
};

/* Serviço para pegar  informações sobre a pilha do container:

    Request:
        where   -> qual pilha você quer receber informação
    Response:
        lista   -> lista (vetor) dos containers que estão na pilha requisitada, com as cores
*/
bool getContainerInfo(projeto_semear::GetContainerInfo::Request &req,
                      projeto_semear::GetContainerInfo::Response &res)
{
    if (!checar_limites(req.where))
    {
        return false;
    }
    res.lista = MAPA[req.where];

    return true;
}

/* Serviço para colocar a cor dos containers:

    Request:
        where   -> qual  a pilha do container que você vai enviar informação da cor (o código muda a cor do ultimo container da pilha automaticamente)
*/
bool setContainer(projeto_semear::SetContainer::Request &req,
                  projeto_semear::SetContainer::Response &res)
{
    if (!checar_limites(req.where))
    {
        return false;
    }
    else if (!checar_cores(req.color))
    {
        return false;
    }

    MAPA[req.where].back() = req.color;
    return true;
}

/* Serviço para avisar que o container mudou de posição:

    Request:
        where   -> qual a pilha do container que você vai enviar mover ( o código move automaticamente o último container da pilha para o local correto)
*/
bool moveContainer(projeto_semear::MoveContainer::Request &req,
                   projeto_semear::MoveContainer::Response &res)
{

    if (!checar_limites(req.where))
    {
        return false;
    }
    unsigned char cor = MAPA[req.where].back();

    if (cor == cores::DESCONHECIDO)
    {
        ROS_ERROR("Cuidado!! tentando mover um container de cor desconhecida, identifique o container!!");
        return false;
    }
    else
    {
        MAPA[cor].push_back(cor);
    }
    MAPA[req.where].pop_back();

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mapa_dos_Containers");
    ros::NodeHandle node;

    // Cria os serviços
    ros::ServiceServer get_service = node.advertiseService("getContainerInfo", getContainerInfo);
    ros::ServiceServer set_service = node.advertiseService("setContainer", setContainer);
    ros::ServiceServer move_service = node.advertiseService("moveContainer", moveContainer);

    ros::spin();

    return 0;
}
