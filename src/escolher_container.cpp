/** Esse programa é um SERVIÇO responsável por escolher qual container deve pegar
 * 
 * Definição do serviço EscolherContainer: 
 *         cor do da esquerda
 *         cor do da direita
 *         ---
 *         retorno
 * 
 *  As entradas são as cores do container da esquerda e da direita logo à frente do container
 *  A saída é o container escolhido
 * 
 * Nome do serviço : EscolherContainer
 * */

//esse código considera que o navio verde estará NECESSARIAMENTE à esquerda
#include "ros/ros.h"
#include "projeto_semear/EscolherContainer.h"
#include "projeto_semear/Colors.h"


bool escolha(projeto_semear18::EscolherContainer::Request &req,
         projeto_semear18::EscolherContainer::Response &res)
    {
        if(req.Cor_Esquerda == 1){return 0}
        else if(req.Cor_Esquerda == 2){return 1}
        else if(req.Cor_Esquerda == 2 && req.Cor_Direita == 1){return 0}
        else if(req.Cor_Esquerda == 2 && req.Cor_Direita == 0){return 0}
        else if(req.Cor_Esquerda == 0 && req.Cor_Direita == 2){return 1}
        else if(req.Cor_Esquerda == 0 && req.Cor_Direita == 0){return 2}
        
    }

/**
 * 
 * int main(int argc, char **argv)
{
    ros::init(argc, argv, "escolher_container");
    ros::NodeHandle node;

    colors_client = node.serviceClient<projeto_semear::Colors>("EscolherContainer"); // Requisita o serviço EscolherContainer

    // Cria o action server
    Server server(node, "navigation", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}

**/


