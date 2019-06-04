/** Esse programa é um SERVIÇO responsável por escolher qual container deve pegar
 * 
 * Definição do serviço EscolherContainer: 
 *         posição
 *         ---
 *         retorno(0=pega o da esquerda/1=pega o da direita/2=mude de setor)
 * 
 *  A entrada é a posição do robô e a partir do mapa, serão adquiridas as cores do container da esquerda e da direita logo à frente do container
 *  A saída é o container escolhido
 * 
 * Nome do serviço : EscolherContainer
 * */

//esse código considera que o navio verde estará NECESSARIAMENTE à esquerda

#include <ros/ros.h>
#include <projeto_semear/EscolherContainer.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/Colors.h>
#include <vector>
#include <cstdint>

projeto_semear::Pose pose;

ros::ServiceClient get_client;
ros::ServiceClient pose_client;

bool escolha(projeto_semear::EscolherContainer::Request &req,
             projeto_semear::EscolherContainer::Response &res)
{
    projeto_semear::GetContainerInfo get_srv;
    int dir,esq;
    switch (req.Posicao.location)
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

    get_srv.request.where =  esq;
    get_client.call(get_srv);

    std::vector<std::uint32_t> vec = get_srv.response.lista;
    std::uint32_t cor_esquerda = vec.back(); // O último container é o primeiro da pilha

    res.container_esq_esta_vazio = cor_esquerda == 0;

    get_srv.request.where = dir;
    get_client.call(get_srv);

    vec = get_srv.response.lista; 
    std::uint32_t cor_direita = vec.back(); // O último container é o primeiro da pilha

    enum ESCOLHIDO{
            ESQUERDA = 0,
            DIREITA = 1,
            NENHUM = 2, //pq só tem vermelho
            DESCONHECIDO = 3 //não pega nenhum pq não sabe qual a cor
            
    };

    enum cores
    {
    AZUL = 13,
    VERDE = 12,
    VERMELHO = 14,
    NENHUMA = 254,
    DESCONHECIDA = 255
    };


    if(req.Posicao.location == 1) //esquerda
    {
        if(cor_esquerda == VERDE)
        {
            res.container_escolhido = ESQUERDA;
            res.cor = VERDE;
            res.pilha = 0;
        }
        else if(cor_direita == VERDE)
        {
            res.container_escolhido = DIREITA;
            res.cor = VERDE;
            res.pilha = 1;
        }
        else if(cor_direita == AZUL)
        {
            res.container_escolhido = DIREITA;
            res.cor = AZUL;
            res.pilha = 1;
        }
        else if(cor_esquerda == AZUL)
        {
            res.container_escolhido = ESQUERDA;
            res.cor = AZUL;
            res.pilha = 0;
        }
        else if (cor_direita == VERMELHO && cor_esquerda == VERMELHO)
        {
            res.container_escolhido = NENHUM;
            res.cor = NENHUMA;
        }
        else
        {
            res.container_escolhido = DESCONHECIDO;
            res.cor = DESCONHECIDA;
        }
    }


    else if(req.Posicao.location == 2) //direita
    {
        if(cor_direita == AZUL)
        {
            res.container_escolhido = DIREITA;
            res.cor = AZUL;
            res.pilha = 5;
        }
        else if(cor_esquerda == AZUL)
        {
            res.container_escolhido = ESQUERDA;
            res.cor = AZUL;
            res.pilha = 4;
        }
        else if(cor_esquerda == VERDE)
        {
            res.container_escolhido = ESQUERDA;
            res.cor = VERDE;
            res.pilha = 4;
        }
        else if(cor_direita == VERDE)
        {
            res.container_escolhido = DIREITA;
            res.cor = VERDE;
            res.pilha = 5;
        }
        else if (cor_direita == VERMELHO && cor_esquerda == VERMELHO)
        {
            res.container_escolhido = NENHUM;
            res.cor = NENHUMA;
        }
        else
        {
            res.container_escolhido = DESCONHECIDO;
            res.cor = DESCONHECIDA;
        }
    }

                                         //meio
    else if (cor_esquerda == VERDE)
    {
        res.container_escolhido = ESQUERDA;
        res.cor = VERDE;
        res.pilha = 2;
    } //caso o da esquerda seja verde, ja pega ele , por facilidade pois a doca verde esta na esquerda
    else if (cor_direita == AZUL)
    {
        res.container_escolhido = DIREITA;
        res.cor = AZUL;
        res.pilha = 3;
    } //caso o da direita seja azul, ja pega ele
    else if (cor_esquerda == AZUL)
    {
        res.container_escolhido = ESQUERDA;
        res.cor = AZUL;
        res.pilha = 2;
    }
    else if (cor_direita == VERDE)
    {
        res.container_escolhido = DIREITA;
        res.cor = VERDE;
        res.pilha = 3;
    }
    else if (cor_direita == VERMELHO && cor_esquerda == VERMELHO)
    {
        res.container_escolhido = NENHUM;
        res.cor = NENHUMA;
    }
    else
    {
        res.container_escolhido = DESCONHECIDO;
        res.cor = DESCONHECIDA;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "escolher_container");
    ros::NodeHandle node;

    // Inicializa o cliente
    get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");

    // Serviço da posição do robô
    pose_client = node.serviceClient<projeto_semear::GetPose>("gps");

    ros::ServiceServer choose_service = node.advertiseService("EscolherContainer", escolha); // Requisita o serviço EscolherContainer
    ROS_INFO("Preparado para escolher o container");

    ros::spin();

    return 0;
}







