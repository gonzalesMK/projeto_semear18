/*
responsavel por definir a estratégia

a ordem resumida do que ele fará será o seguinte:

-INICIO:sabe a cor dos dois containers à frente e a posição
-chama o serviço escolher_container
-SE(pegou um verde)
    -va para a doca verde
-SE(pegou um azul)
    -va para a doca azul
-SE(ta no meio E retornou 2)
    -SE(a esquerda não tem só vermelho)
        -vá para a esquerda
    -SE(a esquerda só tem vermelho)
        -vá para a direita
-SE(ta na direita E retornou 2)
    -SE(o meio não tem só vermelho)
        -vá para o meio
    -SE(o meio tem só vermelho)
        -vá para a direita
-SE(ta na esquerda E retornou 2)
    -SE(o meio não tem só vermelho)
        -vá para o meio
    -SE(o meio tem só vermelho)
        -vá para a esquerda
-

*/

#include <ros/ros.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/Colors.h>
#include <vector>
#include <cstdint>

projeto_semear::Pose pose;
ros::ServiceClient get_client;

bool escolha(projeto_semear::Strategy::Request &req,
             projeto_semear::Strategy::Response &res)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "escolher_container");
    ros::NodeHandle node;

    // Inicializa o cliente
    get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");

    ros::ServiceServer choose_service = node.advertiseService("EscolherContainer", escolha); // Requisita o serviço EscolherContainer
    ROS_INFO("Preparado para escolher o container");

    ros::spinOnce();

    return 0;
}