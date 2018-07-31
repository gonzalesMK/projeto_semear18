#include <ros/ros.h>
#include <vector>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <projeto_semear/MoveContainer.h>

/** 
*/

enum cores
{
    AZUL = 13,
    VERDE = 12,
    VERMELHO = 14,
    DESCONHECIDO = 255
};

std::vector<std::vector<std::uint8_t>> MAPA =
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
        {},                                                                                   // 12
        {},                                                                                   // 13
        {},                                                                                   // 14
};

bool getContainerInfo(projeto_semear::GetContainerInfo::Request &req,
                      projeto_semear::GetContainerInfo::Response &res)
{
    res.lista = MAPA[req.where];
    return true;
}

bool setContainer(projeto_semear::SetContainer::Request &req,
                  projeto_semear::SetContainer::Response &res)
{

    MAPA[req.where].back() = req.color;
    return true;
}

bool moveContainer(projeto_semear::MoveContainer::Request &req,
                   projeto_semear::MoveContainer::Response &res)
{
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
