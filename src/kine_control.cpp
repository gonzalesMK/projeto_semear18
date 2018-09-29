#include "projeto_semear/kine_control.h"
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <projeto_semear/GetContainerInfo.h>
#include <vector>
#include <projeto_semear/GetPose.h>
#include <std_msgs/Bool.h>
#include <boost/bind.hpp>
#include <stdlib.h>

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)

/** Implementação do Objeto que abstrai o robô. **/

void callback(const std_msgs::Float32ConstPtr &msg, kineControl::color &color)
{
    //ROS_INFO_STREAM("PRETO:" << kineControl::MAIOR_QUE_PRETO << "data: " << msg->data);
    if (msg->data > kineControl::MAIOR_QUE_VERDE)
    {
        color = kineControl::color::BRANCO;
    }
    else if (msg->data > kineControl::MAIOR_QUE_PRETO)
    {
        color = kineControl::color::AZUL_VERDE;
    }
    else
    {

        color = kineControl::color::PRETO;
    }
}

void distance_callback(const std_msgs::Float32ConstPtr &msg, double &variable)
{
    variable = msg->data;
}

// Construtor do robô
// Aqui são feitas as criações dos tópicos de leitura dos sensores e os parâmetros são lidos
kineControl::robot::robot()
{
    FR_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorFRSpeed", 1);
    FL_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorFLSpeed", 1);
    BR_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorBRSpeed", 1);
    BL_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorBLSpeed", 1);

    lineSensorFL_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorFL", 1, boost::bind(callback, _1, boost::ref(colorFL_)));
    lineSensorBL_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorBL", 1, boost::bind(callback, _1, boost::ref(colorBL_)));
    lineSensorFR_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorFR", 1, boost::bind(callback, _1, boost::ref(colorFR_)));
    lineSensorBR_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorBR", 1, boost::bind(callback, _1, boost::ref(colorBR_)));
    lateralSensor_ = nh_.subscribe<std_msgs::Float32>("/AMR/sensor_lateral", 1, boost::bind(distance_callback, _1, boost::ref(lateral_distance_)));

    frontalSensor_ = nh_.subscribe<std_msgs::Float32>("/image_converter/frontalSensor", 1, boost::bind(callback, _1, boost::ref(colorFF_)));

    ColorSensorR0_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR0", 1, boost::bind(callback, _1, boost::ref(colorR0_)));
    ColorSensorR1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR1", 1, boost::bind(callback, _1, boost::ref(colorR1_)));
    ColorSensorR2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR2", 1, boost::bind(callback, _1, boost::ref(colorR2_)));
    ColorSensorR3_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR3", 1, boost::bind(callback, _1, boost::ref(colorR3_)));
    ColorSensorL0_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL0", 1, boost::bind(callback, _1, boost::ref(colorL0_)));
    ColorSensorL1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL1", 1, boost::bind(callback, _1, boost::ref(colorL1_)));
    ColorSensorL2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL2", 1, boost::bind(callback, _1, boost::ref(colorL2_)));
    ColorSensorL3_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL3", 1, boost::bind(callback, _1, boost::ref(colorL3_)));

    // Necessário um tempo para inicializar os nós
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // Espera o motor ter os tópicos publicados
    while (colorBR_ == -1 || colorFL_ == -1 || colorBL_ == -1 || colorFR_ == -1)
    {
        ROS_INFO("Waiting 4 Topics");
        update(0.5);
    }

    if (!nh_.param("MAIOR_QUE_PRETO", MAIOR_QUE_PRETO, 59.0))
    {
        ROS_ERROR("Failed to get param 'MAIOR_QUE_PRETO'");
    }

    if (!nh_.param("MAIOR_QUE_VERDE", MAIOR_QUE_VERDE, 299.0))
    {
        ROS_ERROR("Failed to get param 'MAIOR_QUE_VERDE'");
    }

    if (!nh_.param("TEMPO_MEIA_VOLTA", TEMPO_MEIA_VOLTA, 3.1))
    {
        ROS_ERROR("Failed to get param 'TMEPO_MEIA_VOLTA'");
    }
    if (!nh_.param("VEL_Y", VEL_Y, 0.15))
    {
        ROS_ERROR("Failed to get param 'VEL_Y'");
    }
    if (!nh_.param("TEMPO_DIREITA_ESQUERDA", TEMPO_DIREITA_ESQUERDA, 1.5))
    {
        ROS_ERROR("Failed to get param 'TEMPO_DIREITA_ESQUERDA'");
    }

    if (!nh_.param("VEL_Z", VEL_Z, 0.2))
    {
        ROS_ERROR("Failed to get param 'VEL_Z'");
    }

    if (!nh_.param("VEL_X", VEL_X, 0.025))
    {
        ROS_ERROR("Failed to get param 'VEL_X'");
    }

    if (!nh_.param("TEMPO_ALINHAR_ESQUERDA", TEMPO_ALINHAR_ESQUERDA, 0.2))
    {
        ROS_ERROR("Failed to get param 'TEMPO_ALINHAR_ESQUERDA'");
    }

    if (!nh_.param("PRECISAO_DIST_ALINHAR_PILHA", PRECISAO_DIST_ALINHAR_PILHA, 0.005))
    {
        ROS_ERROR("Failed to get param 'PRECISAO_DIST_ALINHAR_PILHA'");
    }

    if (!nh_.param("FREQUENCIA_ROS", FREQUENCIA_ROS, 10.0))
    {
        ROS_ERROR("Failed to get param 'FREQUENCIA_ROS'");
    }

    if (!nh_.param("TEMPO_DIREITA", TEMPO_DIREITA, 0.2))
    {
        ROS_ERROR("Failed to get param 'TEMPO_DIREITA'");
    }
}

bool kineControl::robot::setVelocity(const geometry_msgs::Twist &vel)
{
    // Ângulo XY do vetor velocidade
    double theta = atan2(vel.linear.y, vel.linear.x);

    // Módulo do vetor velocidade em XY
    double w_module = sqrt(pow(vel.linear.x, 2) + pow(vel.linear.y, 2)) / (DIAMETRO);
    double conversao_angular = (LDIAG) / (DIAMETRO); // Converte o Rad/s em metros/s depois em Rad/s para as rodas

    std_msgs::Float32 Wfl;
    std_msgs::Float32 Wfr;
    std_msgs::Float32 Wbl;
    std_msgs::Float32 Wbr;

    // Modelo omnidirecional da base
    Wfl.data = ((w_module)*sin(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // !!!
    Wfr.data = ((w_module)*cos(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // !!!
    Wbl.data = ((w_module)*cos(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // !!!
    Wbr.data = ((w_module)*sin(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // !!!

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

// Função para alinhar os sensores de baixo do robô. Os dois sensores da frente deverão ficar sobre a linha preta ou azul, equanto os de trás devem
// ficar fora da linha.
void kineControl::alinhar_frontal(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - alinhar_frontal() - a linha preta esta atras");
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Duration time(1 / FREQUENCIA_ROS);
    ros::spinOnce();
    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    while ((robot.colorFR_ == BRANCO || robot.colorFL_ == BRANCO) && ros::ok())
    {
        code = 0;
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;
        //ROS_INFO_STREAM("\nFL " << robot.colorFL_ << " FR " << robot.colorFR_);
        // Caso 0: todos os sensores no branco. Supõe-se que o robô ultrapassou o alinhamento necessário. Garantir isso no resto do código
        // Caso 1: Caso o sensor BackRight esteja marcando verde, mas o BackLeft não -> girar positivo
        // Caso 2: Caso o sensor BackLeft esteja marcando verde, mas o BackRight não -> girar negativo
        // Caso 3: Caso o sensor FrontRight esteja marcando verde, mas o FrontLeft não -> girar positivo
        // Caso 4: Caso o sensor FrontLeft esteja marcando verde, mas o  FrontRight não -> girar negativo
        //ROS_INFO_STREAM("\nFL " << colorFL << "FR " << colorFR << "\nBL " << colorBL << "BR " << colorBR);

        if (robot.colorFL_ == BRANCO && robot.colorFR_ == BRANCO)
            code = 0;
        else if (robot.colorFL_ != PRETO && robot.colorFR_ == PRETO)
            code = 1;
        else if (robot.colorFL_ == PRETO && robot.colorFR_ != PRETO)
            code = 2;

        //ROS_INFO_STREAM("\nCase: " << code);
        switch (code)
        {
        case 0:
            velocidade.linear.x = -0.05;
            break;
        case 1:
            velocidade.angular.z = VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = -VEL_ANG;
            break;
        }

        robot.setVelocity(velocidade);
        ros::spinOnce();
        time.sleep();
    }
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

// Função para alinhar os sensores de baixo do robô. Os dois sensores de trás deverão ficar sobre a linha preta ou azul, equanto os da frente devem
// ficar fora da linha.
void kineControl::alinhar_traseiro(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - alinha_traseiro() - a linha preta esta na frente");
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Rate rate(FREQUENCIA_ROS);

    ros::spinOnce();

    double off_set = 0;
    bool alinhado = !(robot.colorBR_ == BRANCO || robot.colorBL_ == BRANCO || robot.colorFR_ != BRANCO || robot.colorFL_ != BRANCO);
    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    while (!alinhado && ros::ok())
    {
        velocidade.linear.y = 0;

        velocidade.linear.x = ((int)(robot.colorFL_ != BRANCO) + (int)(robot.colorFR_ != BRANCO) - (int)(robot.colorBL_ == BRANCO) - (int)(robot.colorBR_ == BRANCO) + off_set) * VEL_X;
        velocidade.angular.z = -((int)(robot.colorFL_ != BRANCO) - (int)(robot.colorFR_ != BRANCO) + (int)(robot.colorBL_ == BRANCO) - (int)(robot.colorBR_ == BRANCO)) * VEL_Z;
        //        velocidade.linear.x = ((int)(robot.colorFL_ == BRANCO) + (int)(robot.colorFR_ == BRANCO) - (int)(robot.colorBL_ != BRANCO) - (int)(robot.colorBR_ != BRANCO) + off_set) * VEL_X;
        //       velocidade.angular.z = ((int)(robot.colorFL_ == BRANCO) - (int)(robot.colorFR_ == BRANCO) + (int)(robot.colorBL_ == BRANCO) - (int)(robot.colorBR_ == BRANCO)) * VEL_Z;
        robot.setVelocity(velocidade);
        rate.sleep();

        ros::spinOnce();

        alinhado = !(robot.colorBR_ == BRANCO || robot.colorBL_ == BRANCO || robot.colorFR_ != BRANCO || robot.colorFL_ != BRANCO);
        ROS_INFO_STREAM(" X: " << velocidade.linear.x << " Z: " << velocidade.angular.z << "Alinhado:" << alinhado);
        if (!alinhado && velocidade.linear.x == 0 && velocidade.angular.z == 0)
        {
            ROS_ERROR(" O Robo travou, ligando ofsset");
            off_set = 2;
        }
        else
        {
            off_set = 0;
        }
    }
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::alinhar_doca(kineControl::robot &robot)
{

    ROS_INFO("KINECONTROL - alinhar_doca");

    geometry_msgs::Twist velocidade;

    int code = 0;
    ros::spinOnce();
    ros::Duration time(1 / FREQUENCIA_ROS);

    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    alinhar_traseiro(robot);

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::alinhar_depositar_esquerda(kineControl::robot &robot)
{
    kineControl::alinhar_traseiro(robot);

    ROS_INFO_STREAM("KINECONTROL - alinhar_esquerda() ");

    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_ROS);
    ros::spinOnce();

    while (robot.colorFF_ != AZUL_VERDE)
    {
        // Andar uma distância predefinida
        velocidade.linear.x = ((int)(robot.colorFL_ != BRANCO) + (int)(robot.colorFR_ != BRANCO) - (int)(robot.colorBL_ == BRANCO) - (int)(robot.colorBR_ == BRANCO)) * VEL_X;
        velocidade.angular.z = (-(int)(robot.colorFL_ != BRANCO) + (int)(robot.colorFR_ != BRANCO) + (int)(robot.colorBL_ == BRANCO) - (int)(robot.colorBR_ == BRANCO)) * VEL_Z;
        velocidade.linear.y = -VEL_Y;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}
void kineControl::esquerda(kineControl::robot &robot)
{
    kineControl::alinhar_esquerda(robot);
    ROS_INFO_STREAM("KINECONTROL - esquerda");

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_ROS);
    while (now - begin < ros::Duration(TEMPO_DIREITA_ESQUERDA))
    {
        // Andar uma distância predefinida
        ros::spinOnce();
        velocidade.linear.x = (-(int)(robot.colorFL_ != PRETO) - (int)(robot.colorFR_ != PRETO) + (int)(robot.colorBL_ != PRETO) + (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.linear.y = -VEL_Y;
        velocidade.angular.z = 0;
        robot.setVelocity(velocidade);
        now = ros::Time::now();
        rate.sleep();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    kineControl::alinhar_frontal(robot);
}

void kineControl::ir_doca(kineControl::robot &robot)
{
    kineControl::alinhar_frontal(robot);

    ROS_INFO_STREAM("KINECONTROL - ir_doca()");

    geometry_msgs::Twist velocidade;

    // Ir para trás
    velocidade.linear.x = -0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(2).sleep();

    // Girar 90 Graus
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = PI / 3;
    robot.setVelocity(velocidade);
    ros::Duration(3).sleep();

    // Andar para frente
    velocidade.linear.x = 0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    // É possível alinhar com a linha verde, se necessário

    // Alinhar
    kineControl::alinhar_doca(robot);
}

void kineControl::ir_quadrante(kineControl::robot &robot)
{

    ROS_INFO_STREAM("KINECONTROL - ir_quadrante");
    geometry_msgs::Twist velocidade;

    // Ir para trás
    velocidade.linear.x = -0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(1).sleep();

    // Girar 90 Graus
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = PI / 3;
    robot.setVelocity(velocidade);
    ros::Duration(TEMPO_MEIA_VOLTA).sleep();

    /* Girar 90 Graus
    velocidade.linear.x = -0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(1).sleep();
    */

    // É possível alinhar com a linha verde, se necessário

    kineControl::linha_preta(robot);
    //ROS_INFO("CHEGOU NO QUADRANTE");
}

void kineControl::direita(kineControl::robot &robot)
{
    kineControl::alinhar_direita(robot);

    ROS_INFO_STREAM("KINECONTROL - direita() ");

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_ROS);
    while (now - begin < ros::Duration(TEMPO_DIREITA_ESQUERDA))
    {
        // Andar uma distância predefinida
        ros::spinOnce();
        velocidade.linear.x = (-(int)(robot.colorFL_ != PRETO) - (int)(robot.colorFR_ != PRETO) + (int)(robot.colorBL_ != PRETO) + (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.linear.y = VEL_Y;
        velocidade.angular.z = 0;
        robot.setVelocity(velocidade);
        now = ros::Time::now();
        rate.sleep();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    kineControl::alinhar_traseiro(robot);
}

void kineControl::linha_preta(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - linha_preta - a linha preta esta a frente");
    const double VEL_ANG = kineControl::VEL_ANG;

    ros::Duration time(0.01);
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Time begin = ros::Time::now();
    ros::spinOnce();

    while ((robot.colorBR_ != PRETO || robot.colorBL_ != PRETO) && ros::ok())
    {
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;

        //Caso 0: Nenhum dos sensores na faixa pretav -> anda para frente
        //Caso 1: Sensor FrontRight no preto e FrontLeft não -> girar para direita
        //Caso 2: Sensor FrontLeft no preto e FrontRight não -> girar para esquerda
        //ROS_INFO_STREAM("\nFL " << robot.colorBL_ << " FR " << robot.colorBR_);
        if (robot.colorBL_ != PRETO && robot.colorBR_ != PRETO)
            code = 0;
        else if (robot.colorBL_ != PRETO && robot.colorBR_ == PRETO)
            code = 1;
        else if (robot.colorBL_ == PRETO && robot.colorBR_ != PRETO)
            code = 2;
        //ROS_INFO_STREAM("Case: " << code);
        switch (code)
        {
        case 0:
            velocidade.linear.x = 0.15;
            break;
        case 1:
            velocidade.angular.z = -VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
            break;
        }

        robot.setVelocity(velocidade);
        time.sleep();
        ros::spinOnce();
    }
    ros::Time end = ros::Time::now();
    float distance = (end.sec - begin.sec) * 0.05;
    //ros::Duration(3).sleep();
    //ROS_INFO_STREAM("distancia percorrida " << distance);
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);
}

void kineControl::alinhar_pilha(kineControl::robot &robot, int dir)
{

    // 0.07 -> container da esquerda 0
    // 0.01   -> container da direita 1
    ROS_INFO("KINECONTROL - alinhar_pilha");
    ros::spinOnce();
    robot.lateral_distance_;
    ros::Rate rate(FREQUENCIA_ROS);
    geometry_msgs::Twist velocidade;

    // Alinhar para frente
    bool alinhado = robot.colorFL_ != PRETO && robot.colorFR_ != PRETO && robot.colorBL_ == PRETO && robot.colorBR_ == PRETO;

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    double off_set = 0;

    while (!alinhado && ros::ok())
    {
        velocidade.linear.y = 0;
        velocidade.linear.x = ((int)(robot.colorFL_ == PRETO) + (int)(robot.colorFR_ == PRETO) - (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO) + off_set) * VEL_X;
        velocidade.angular.z = ((int)(robot.colorFL_ == PRETO) - (int)(robot.colorFR_ == PRETO) + (int)(robot.colorBL_ == PRETO) - (int)(robot.colorBR_ == PRETO)) * VEL_Z;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
        alinhado = robot.colorFL_ != PRETO && robot.colorFR_ != PRETO && robot.colorBL_ == PRETO && robot.colorBR_ == PRETO;

        if (!alinhado && velocidade.linear.x == 0 && velocidade.angular.z == 0)
        {
            ROS_ERROR(" O Robo travou, ligando ofsset");
            off_set = 2;
        }
        else
        {
            off_set = 0;
        }
    }
    velocidade.linear.y = 0;
    velocidade.linear.x = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    // esquerda é 0 e direita é 1 e 2 é o centro
    double dist;
    if (dir == 0)
    {
        dist = 0.071;
    }
    else if (dir == 1)
    {
        dist = 0.01;
    }
    else if (dir == 2)
    {
        dist = 0.04;
    }

    // Alinhar lateralmente
    double dif = dist - robot.lateral_distance_;
    while (std::fabs(dif) > PRECISAO_DIST_ALINHAR_PILHA && ros::ok())
    {
        // Andar uma distância predefinida
        velocidade.linear.y = -dif;
        velocidade.linear.x = ((int)(robot.colorFL_ == PRETO) + (int)(robot.colorFR_ == PRETO) - (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.angular.z = ((int)(robot.colorFL_ == PRETO) - (int)(robot.colorFR_ == PRETO) + (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_Z;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
        alinhado = robot.colorFL_ != PRETO && robot.colorFR_ != PRETO && robot.colorBL_ == PRETO && robot.colorBR_ == PRETO;
        dif = dist - robot.lateral_distance_;
    }

    velocidade.linear.y = 0;
    velocidade.linear.x = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::alinhar_containerdepositado(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - alinhar_containerdepositado");
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Duration time(1 / FREQUENCIA_ROS);
    ros::spinOnce();

    // condição de não alinhamento: o robo não detecta azul ou verde
    while ((robot.colorR0_ != AZUL_VERDE || robot.colorL0_ != AZUL_VERDE) && ros::ok())
    {
        code = 0;
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;
        //ROS_INFO_STREAM("\nFL " << robot.colorFL_ << " FR " << robot.colorFR_);
        // Caso 0: Sensor da esquerda detecta verde/azul, mas o da direita não -> ir para direita
        // Caso 1: Sensor da direita detecta verde/azul, mas o da esquerda não -> ir para esquerda
        if (robot.colorL0_ == AZUL_VERDE && robot.colorR0_ != AZUL_VERDE)
            code = 0;
        else if (robot.colorL0_ != AZUL_VERDE && robot.colorR0_ == AZUL_VERDE)
            code = 1;

        //ROS_INFO_STREAM("\nCase: " << code);

        switch (code)
        {
        case 0:
            //ROS_INFO("direita");
            //ROS_INFO_STREAM("\nlacor: " << AZUL_VERDE);
            //ROS_INFO_STREAM("\nlacor: " << robot.colorL0_);
            //OS_INFO_STREAM("\nlacor: " << robot.colorR0_);
            velocidade.linear.y = 0.05;
            break;
        case 1:
            //ROS_INFO("esquerda");
            velocidade.linear.y = -0.05;
            break;
        }

        robot.setVelocity(velocidade);
        ros::spinOnce();
        time.sleep();
    }
    switch (code)
    {
    case 0:
        velocidade.linear.y = 0.05;
        ros::Duration(1).sleep();
        break;
    case 1:
        velocidade.linear.y = -0.05;
        ros::Duration(1).sleep();
        break;
    }
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> MoveClient;
typedef actionlib::SimpleActionClient<projeto_semear::setEletroimaAction> SetClient;

void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback) {}
void doneCb(const actionlib::SimpleClientGoalState &state, const projeto_semear::moveEletroimaResultConstPtr &result) {}
void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback) {}
void doneCb2(const actionlib::SimpleClientGoalState &state, const projeto_semear::setEletroimaResultConstPtr &result) {}
void activeCb() {}

void kineControl::pegar_container(kineControl::robot &robot, char lado_escolhido)
{
    // Espera-se que o código já saiba se deve pegar o container da direita ou da esquerda
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
    ros::ServiceClient pose_client = nh.serviceClient<projeto_semear::GetPose>("gps");
    ros::ServiceClient get_client = nh.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    MoveClient move_client("moveEletroima", true); // true -> don't need ros::spin()
    SetClient set_client("setEletroima", true);    // true -> don't need ros::spin()

    ROS_INFO_STREAM("KINECONTROL - pegar_container");

    // Desligando Eletroima
    std_msgs::Bool msg;
    msg.data = false;
    pub.publish(msg);

    // Pegar a posição do robô
    projeto_semear::GetPose pose_msg;
    pose_msg.request.set = false;
    pose_client.call(pose_msg);

    // Encontrar qual a pilha
    double esq, dir;
    switch (pose_msg.response.pose.location)
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
        return;
    }
    char lado;
    if (lado_escolhido == 0)
    {
        lado = esq;
    }
    else if (lado_escolhido == 1)
    {
        lado = dir;
    }
    else
    {
        ROS_ERROR("Nenhum lado foi escolhido. Erro em pegar container");
    }

    // Meta para posicionar a garra em cima do container
    projeto_semear::moveEletroimaGoal goal;
    projeto_semear::setEletroimaGoal set_goal;
    move_client.waitForServer();
    set_client.waitForServer();

    //ROS_INFO_STREAM("KINECONTROL - pegar_container() - Centralizar garra no container superior da posicao: " << (int) lado);

    set_goal.pose = set_goal.posicao_pegar_container_superior;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    // Verificar o número de containers na pilha
    projeto_semear::GetContainerInfo get_container_info_msg;
    get_container_info_msg.request.where = lado;

    get_client.call(get_container_info_msg);
    double altura = get_container_info_msg.response.lista.size();

    // Ligar o Eletroimã:
    //ROS_INFO_STREAM("PEGAR CONTAINER - Ligando o eletroima");
    msg.data = true;
    pub.publish(msg);

    // Girar a guarra 90º
    //ROS_INFO_STREAM("PEGAR CONTAINER - Descendo Garra, altura: " << altura);
    projeto_semear::moveEletroimaGoal move_goal;
    move_goal.deslocamento.linear.x = 0.0;
    move_goal.deslocamento.linear.y = 0;
    move_goal.deslocamento.linear.z = -0.045 * (4 - altura);
    move_goal.deslocamento.angular.z = 0;
    move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
    move_client.waitForResult(ros::Duration());

    //ROS_INFO_STREAM("PEGAR CONTAINER - Erguer Container");
    set_goal.pose = set_goal.posicao_segurar_container;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    //ROS_INFO_STREAM("PEGAR CONTAINER - Rotacionar Container em cima");
    set_goal.pose = set_goal.posicao_segurar_container_rotacionado;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());
}

// Concerning is not properly working yet, need to improve the math
// Concerning é quando o robô gira em torno de uma das suas rodas.
bool kineControl::robot::concerning(const wheel w, double modulo_vel)
{
    std_msgs::Float32 Wfl;
    std_msgs::Float32 Wfr;
    std_msgs::Float32 Wbl;
    std_msgs::Float32 Wbr;
    modulo_vel = modulo_vel / DIAMETRO * PI;

    switch (w)
    {
    case kineControl::FL:
        Wfl.data = 0;
        Wfr.data = -modulo_vel;
        Wbl.data = 0;
        Wbr.data = -modulo_vel;
    case kineControl::BL:
        Wfl.data = 0;
        Wfr.data = modulo_vel;
        Wbl.data = 0;
        Wbr.data = modulo_vel;
    case kineControl::FR:
        Wfl.data = -modulo_vel;
        Wfr.data = 0;
        Wbl.data = -modulo_vel;
        Wbr.data = 0;
    case kineControl::BR:
        Wfl.data = modulo_vel;
        Wfr.data = 0;
        Wbl.data = modulo_vel;
        Wbr.data = 0;
    }
    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

void kineControl::alinhar_esquerda(kineControl::robot &robot)
{
    kineControl::alinhar_frontal(robot);

    ROS_INFO_STREAM("KINECONTROL - alinhar_esquerda() ");

    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_ROS);
    ros::spinOnce();

    while (robot.colorFF_ != PRETO)
    {
        // Andar uma distância predefinida
        velocidade.linear.x = ((int)(robot.colorFL_ == PRETO) + (int)(robot.colorFR_ == PRETO) - (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.angular.z = ((int)(robot.colorFL_ == PRETO) - (int)(robot.colorFR_ == PRETO) + (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_Z;
        velocidade.linear.y = -VEL_Y;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    /*while (now - begin < ros::Duration(TEMPO_ALINHAR_ESQUERDA))
    {
        // Andar uma distância predefinida
        velocidade.linear.x = ((int)(robot.colorFL_ == PRETO) + (int)(robot.colorFR_ == PRETO) - (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.angular.z = ((int)(robot.colorFL_ == PRETO) - (int)(robot.colorFR_ == PRETO) + (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_Z;
        velocidade.linear.y = VEL_Y;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
        now = ros::Time::now();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);*/
}

void kineControl::alinhar_direita(kineControl::robot &robot)
{
    kineControl::alinhar_traseiro(robot);

    ROS_INFO_STREAM("KINECONTROL - alinhar_direita() ");

    geometry_msgs::Twist velocidade;
    ros::Rate rate(10);
    ros::spinOnce();

    while (robot.colorFF_ != PRETO)
    {
        // Andar uma distância predefinida
        velocidade.linear.x = ((int)(robot.colorFL_ == BRANCO) + (int)(robot.colorFR_ == BRANCO) - (int)(robot.colorBL_ != BRANCO) - (int)(robot.colorBR_ != BRANCO)) * VEL_X;
        velocidade.angular.z = (-(int)(robot.colorFL_ == BRANCO) + (int)(robot.colorFR_ == BRANCO) - (int)(robot.colorBL_ != BRANCO) + (int)(robot.colorBR_ != BRANCO)) * VEL_Z;
        velocidade.linear.y = VEL_Y;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    /*while (now - begin < ros::Duration(TEMPO_ALINHAR_ESQUERDA))
    {
        // Andar uma distância predefinida
        velocidade.linear.x = ((int)(robot.colorFL_ == PRETO) + (int)(robot.colorFR_ == PRETO) - (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_X;
        velocidade.angular.z = ((int)(robot.colorFL_ == PRETO) - (int)(robot.colorFR_ == PRETO) + (int)(robot.colorBL_ != PRETO) - (int)(robot.colorBR_ != PRETO)) * VEL_Z;
        velocidade.linear.y = VEL_Y;
        robot.setVelocity(velocidade);
        rate.sleep();
        ros::spinOnce();
        now = ros::Time::now();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);*/
}

kineControl::color kineControl::robot::get_colorFL()
{
    ros::spinOnce();
    return this->colorFL_;
}
kineControl::color kineControl::robot::get_colorBL()
{
    ros::spinOnce();
    return this->colorBL_;
}
kineControl::color kineControl::robot::get_colorFR()
{
    ros::spinOnce();
    return this->colorFR_;
}
kineControl::color kineControl::robot::get_colorBR()
{
    ros::spinOnce();
    return this->colorBR_;
}
kineControl::color kineControl::robot::get_colorR0()
{
    ros::spinOnce();
    return this->colorR0_;
}
kineControl::color kineControl::robot::get_colorR1()
{
    ros::spinOnce();
    return this->colorR1_;
}
kineControl::color kineControl::robot::get_colorR2()
{
    ros::spinOnce();
    return this->colorR2_;
}
kineControl::color kineControl::robot::get_colorR3()
{
    ros::spinOnce();
    return this->colorR3_;
}
kineControl::color kineControl::robot::get_colorL0()
{
    ros::spinOnce();
    return this->colorL0_;
}
kineControl::color kineControl::robot::get_colorL1()
{
    ros::spinOnce();
    return this->colorL1_;
}
kineControl::color kineControl::robot::get_colorL2()
{
    ros::spinOnce();
    return this->colorL2_;
}
kineControl::color kineControl::robot::get_colorL3()
{
    ros::spinOnce();
    return this->colorL3_;
}
kineControl::color kineControl::robot::get_colorFF()
{
    ros::spinOnce();
    return this->colorFF_;
}
