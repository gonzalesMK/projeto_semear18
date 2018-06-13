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


void escolha(projeto_semear18::EscolherContainer::Request &req,
         projeto_semear18::EscolherContainer::Response &res)
    {
        if(req.Cor_Esquerda.cor == req.Cor_Esquerda.GREEN){res.container_escolhido = 0}
        else if(req.Cor_Direita.cor == req.Cor_Direita.BLUE){res.container_escolhido = 1}
        else if(req.Cor_Esquerda.cor == req.Cor_Esquerda.BLUE && req.Cor_Direita.cor == req.Cor_Direita.GREEN){res.container_escolhido = 0}
        else if(req.Cor_Esquerda.cor == req.Cor_Esquerda.BLUE && req.Cor_Direita.cor == req.Cor_Direita.RED){res.container_escolhido = 0}
        else if(req.Cor_Esquerda.cor == req.Cor_Esquerda.RED && req.Cor_Direita.cor == req.Cor_Direita.BLUE){res.container_escolhido = 1}
        else if(req.Cor_Esquerda.cor == req.Cor_Esquerda.RED && req.Cor_Direita.cor == req.Cor_Esquerda.RED){res.container_escolhido = 2}
        
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


