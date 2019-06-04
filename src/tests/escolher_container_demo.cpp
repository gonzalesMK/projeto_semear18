#include <ros/ros.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/Colors.h>
#include <vector>
#include <cstdint>

int pilha;
int cor;
int aux;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "escolher_container_demo");
    ros::NodeHandle node;

    // Cria os serviços
    ros::ServiceClient choose_client = node.serviceClient<projeto_semear::EscolherContainer>("escolher_container");
    ros::ServiceClient set_client = node.serviceClient<projeto_semear::SetContainer>("setContainer");

    projeto_semear::EscolherContainer srv;
    projeto_semear::SetContainer set_srv;


    bool stop = false;
    while (!stop)
    {

        //Set Pose
        std::cout << "Set Location(quadrante)";
        std::cin >> srv.request.Posicao.location;

        for(aux=0;aux=2;aux++) //só precisa de duas pilhas
        {

            std::cout << "digite a pilha de container a ser informado";
            std::cin >> pilha;

            set_srv.request.where = pilha;

            std::cout << "Digite a cor: AZUL (13), VERDE (12) ou VERMELHO(14)";
            std::cin >> cor;

            set_srv.request.color = cor;

            set_client.call(set_srv);

        }

    //agora temos a posição e a cor de dois containers, so falta chamar o servico e testá-lo



    }
    return 0;
}