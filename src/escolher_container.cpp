/** Esse programa é um SERVIÇO responsável por escolher qual container deve pegar
 * 
 * Definição do serviço EscolherContainer: 
 *         cor do da esquerda
 *         cor do da direita
 *         ---
 *         retorno(0=pega o da esquerda/1=pega o da direita/2=mude de setor)
 * 
 *  As entradas são as cores do container da esquerda e da direita logo à frente do container
 *  A saída é o container escolhido
 * 
 * Nome do serviço : EscolherContainer
 * */

//esse código considera que o navio verde estará NECESSARIAMENTE à esquerda
#include <ros/ros.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/Colors.h>

bool escolha(projeto_semear::EscolherContainer::Request &req,
             projeto_semear::EscolherContainer::Response &res)
{
    if (req.Cor_Esquerda.cor == req.Cor_Esquerda.GREEN)
    {
        res.container_escolhido = 0;
    } //caso o da esquerda seja verde, ja pega ele , por facilidade pois a doca verde esta na esquerda
    else if (req.Cor_Direita.cor == req.Cor_Direita.BLUE)
    {
        res.container_escolhido = 1;
    } //caso o da direita seja azul, ja pega ele
    else if (req.Cor_Esquerda.cor == req.Cor_Esquerda.BLUE && req.Cor_Direita.cor == req.Cor_Direita.GREEN)
    {
        res.container_escolhido = 0;
    }
    else if (req.Cor_Esquerda.cor == req.Cor_Esquerda.BLUE && req.Cor_Direita.cor == req.Cor_Direita.RED)
    {
        res.container_escolhido = 0;
    }
    else if (req.Cor_Esquerda.cor == req.Cor_Esquerda.RED && req.Cor_Direita.cor == req.Cor_Direita.BLUE)
    {
        res.container_escolhido = 1;
    }
    else if (req.Cor_Esquerda.cor == req.Cor_Esquerda.RED && req.Cor_Direita.cor == req.Cor_Esquerda.RED)
    {
        res.container_escolhido = 2;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "escolher_container");
    ros::NodeHandle node;

    ros::ServiceServer colors_client = node.advertiseService("EscolherContainer", escolha); // Requisita o serviço EscolherContain
    ROS_INFO("Preparado para escolher o container");
    ros::spinOnce();

    return 0;
}


