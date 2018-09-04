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
#include <projeto_semear/Strategy.h>
#include <vector>
#include <cstdint>

projeto_semear::Pose pose;
ros::ServiceClient get_client;

enum pose
{
    QUADRANTE_CENTRAL=0,
    QUADRANTE_ESQUERDO=1,
    QUADRANTE_DIREITO=2,
    DOCA_VERDE=3,
    DOCA_AZUL=4
};




bool estrategia(projeto_semear::Strategy::Request &req,
                projeto_semear::Strategy::Response &res)
             
{
    projeto_semear::EscolherContainer escolher_container_srv;
    int cor,posicao,retorno;

    escolher_container_srv.response.cor = cor;
    get_client.call(escolher_container_srv);

    escolher_container_srv.request.Posicao.location= posicao;
    get_client.call(escolher_container_srv);

    escolher_container_srv.response.container_escolhido = retorno;
    get_client.call(escolher_container_srv);

    enum cores
    {
    AZUL = 13,
    VERDE = 12,
    VERMELHO = 14,
    NENHUMA = 254,
    DESCONHECIDO = 255
    };

    //abaixo daqui é para atribuir o valor dos containers que estão em cima à suas respectivas cores

    projeto_semear::GetContainerInfo get_srv;
    /*
    int Qesq_esq, Qesq_dir; //os dois do quadrante esquerdo
    int Qcen_esq, Qcen_dir; //os dois do quadrante central
    int Qdir_esq, Qdir_dir; //os dois do quadrante direito
    */
    get_srv.request.where = 0;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec0 = get_srv.response.lista;
    std::uint8_t Qesq_esq = vec0.back();

    get_srv.request.where = 1;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec1 = get_srv.response.lista;
    std::uint8_t Qesq_dir = vec1.back();

    get_srv.request.where = 2;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec2 = get_srv.response.lista;
    std::uint8_t Qcen_esq = vec2.back();

    get_srv.request.where = 3;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec3 = get_srv.response.lista;
    std::uint8_t Qcen_dir = vec3.back();

    get_srv.request.where = 4;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec4 = get_srv.response.lista;
    std::uint8_t Qdir_esq = vec4.back();

    get_srv.request.where = 5;
    get_client.call(get_srv);

    std::vector<std::uint8_t> vec5 = get_srv.response.lista;
    std::uint8_t Qdir_dir = vec5.back();

    //abaixo daqui é a estratégia 

    if(cor == VERDE)
    {
        res.to_go.location = DOCA_VERDE;
        res.container_escolhido = retorno;
        res.cor = cor;
        res.pilha = posicao;
    }
    else if(cor == AZUL)
    {
        res.to_go.location = DOCA_AZUL;
        res.container_escolhido = retorno;
        res.cor = cor;
        res.pilha = posicao;
    }
    else if(posicao == 0 && retorno == 2)
    {
        if(Qesq_esq != 0 || Qesq_dir != 0)
        {
            res.to_go.location = QUADRANTE_ESQUERDO;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
        else
        {
            res.to_go.location = QUADRANTE_DIREITO;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
    }
    else if(posicao == 2 && retorno == 2)
    {
        if(Qcen_esq != 0 || Qcen_dir != 0)
        {
            res.to_go.location = QUADRANTE_CENTRAL;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
        else
        {
            res.to_go.location = QUADRANTE_DIREITO;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
    }
    else if(posicao == 1 && retorno == 2)
    {
        if(Qcen_esq != 0 || Qcen_dir != 0)
        {
            res.to_go.location = QUADRANTE_CENTRAL;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
        else
        {
            res.to_go.location = QUADRANTE_ESQUERDO;
            res.container_escolhido = retorno;
            res.cor = cor;
            res.pilha = posicao;
        }
    }


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "estrategia_server");
    ros::NodeHandle node;

    // Inicializa o cliente
    get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");

    ros::ServiceClient escolher_container_srv = node.serviceClient<projeto_semear::EscolherContainer>("EscolherContainer"); // Requisita o serviço EscolherContainer
    escolher_container_srv.waitForExistence();

    ros::ServiceServer choose_service = node.advertiseService("Estrategia", estrategia);
    ROS_INFO("Pensando...");

    ros::spin();

    return 0;
}