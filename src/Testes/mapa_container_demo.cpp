#include <ros/ros.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/MoveContainer.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/SetContainer.h>
#include <vector>
#include <cstdint>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MAPA_demo");
    ros::NodeHandle node;

    // Cria o serviço
    ros::ServiceClient get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    ros::ServiceClient set_client = node.serviceClient<projeto_semear::SetContainer>("setContainer");
    ros::ServiceClient move_client = node.serviceClient<projeto_semear::MoveContainer>("moveContainer");

    projeto_semear::MoveContainer move_srv;
    projeto_semear::GetContainerInfo get_srv;
    projeto_semear::SetContainer set_srv;

    int p;
    int cor;

    while (ros::ok())
    {

        std::cout << "Deseja: mover(1), setar(2) ou pegar informacoes (3): ";
        std::cin >> p;
        
        switch (p)
        {
        case 1:
            std::cout << "Digite a pilha do container a ser movido: ";
            std::cin >> p;

            move_srv.request.where = p;
            move_client.call(move_srv);

            break;
        case 2:
            std::cout << "Digite a pilha do container a ser informado: ";
            std::cin >> p;

            set_srv.request.where = p;

            std::cout << "Digite a cor: AZUL (13), VERDE (12) ou VERMELHO(14)";
            std::cin >> cor;

            set_srv.request.color = cor;

            set_client.call(set_srv);
            break;

        case 3:
            std::cout << "Digite a pilha requisitada: ";
            std::cin >> p;

            get_srv.request.where = p;

            get_client.call(get_srv);

            std::vector<std::uint32_t> vec = get_srv.response.lista;
            for (auto i = vec.begin(); i != vec.end(); i++)
            {
                ROS_INFO_STREAM("Container: " << (int)*i);
            }
        }
    }

    return 0;
}
