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
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

const double PI = 3.141592653589793238463;
const double LX = 0.06099;   // Comprimento do eixo X
const double LY = 0.0991225; // Comprimento do eixo Y

void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback) {}
void doneCb(const actionlib::SimpleClientGoalState &state, const projeto_semear::moveEletroimaResultConstPtr &result) {}
void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback) {}
void doneCb2(const actionlib::SimpleClientGoalState &state, const projeto_semear::setEletroimaResultConstPtr &result) {}
void activeCb() {}

void callback(const std_msgs::UInt16ConstPtr &msg, kineControl::color &color)
{
    //ROS_INFO_STREAM("PRETO:" << kineControl::MAIOR_QUE_PRETO << "data: " << msg->data);
    if (msg->data > kineControl::MAIOR_QUE_PRETO)
    {
        color = kineControl::color::PRETO;
    }
    else if (msg->data > kineControl::MAIOR_QUE_VERDE)
    {
        color = kineControl::color::AZUL_VERDE;
    }
    else
    {
        color = kineControl::color::BRANCO;
    }
}

void distance_callback(const std_msgs::UInt32ConstPtr &msg, uint32_t &variable)
{
    variable = msg->data;
}

// Construtor do robô
// Aqui são feitas as criações dos tópicos de leitura dos sensores e os parâmetros são lidos
kineControl::robot::robot()
{
    FR_Motor_ = nh_.advertise<std_msgs::Float64>("/AMR/FR_PID/setpoint", 1);
    FL_Motor_ = nh_.advertise<std_msgs::Float64>("/AMR/FL_PID/setpoint", 1);
    BR_Motor_ = nh_.advertise<std_msgs::Float64>("/AMR/BR_PID/setpoint", 1);
    BL_Motor_ = nh_.advertise<std_msgs::Float64>("/AMR/BL_PID/setpoint", 1);

    lineSensorE0_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorE0", 1, boost::bind(callback, _1, boost::ref(colorE0_)));
    lineSensorE1_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorE1", 1, boost::bind(callback, _1, boost::ref(colorE1_)));
    lineSensorD0_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorD0", 1, boost::bind(callback, _1, boost::ref(colorD0_)));
    lineSensorD1_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorD1", 1, boost::bind(callback, _1, boost::ref(colorD1_)));

    lineSensorE2_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorE2", 1, boost::bind(callback, _1, boost::ref(colorE2_)));
    lineSensorE3_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorE3", 1, boost::bind(callback, _1, boost::ref(colorE3_)));
    lineSensorD2_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorD2", 1, boost::bind(callback, _1, boost::ref(colorD2_)));
    lineSensorD3_ = nh_.subscribe<std_msgs::UInt16>("/AMR/lineSensorD3", 1, boost::bind(callback, _1, boost::ref(colorD3_)));

    lateralSensor_ = nh_.subscribe<std_msgs::UInt32>("/AMR/sonar", 1, boost::bind(distance_callback, _1, boost::ref(lateral_distance_)));
    /*
    frontalSensorEsq_ = nh_.subscribe<std_msgs::Float32>("/image_converter/frontalSensorEsq", 1, boost::bind(callback, _1, boost::ref(colorFE_)));
    frontalSensorDir_ = nh_.subscribe<std_msgs::Float32>("/image_converter/frontalSensorDir", 1, boost::bind(callback, _1, boost::ref(colorFD_)));

    ColorSensorR0_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR0", 1, boost::bind(callback, _1, boost::ref(colorR0_)));
    ColorSensorR1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR1", 1, boost::bind(callback, _1, boost::ref(colorR1_)));
    ColorSensorR2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR2", 1, boost::bind(callback, _1, boost::ref(colorR2_)));
    ColorSensorR3_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorR3", 1, boost::bind(callback, _1, boost::ref(colorR3_)));
    ColorSensorL0_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL0", 1, boost::bind(callback, _1, boost::ref(colorL0_)));
    ColorSensorL1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL1", 1, boost::bind(callback, _1, boost::ref(colorL1_)));
    ColorSensorL2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL2", 1, boost::bind(callback, _1, boost::ref(colorL2_)));
    ColorSensorL3_ = nh_.subscribe<std_msgs::Float32>("/image_converter/ColorSensorL3", 1, boost::bind(callback, _1, boost::ref(colorL3_)));
*/
    // Necessário um tempo para inicializar os nós
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    /* Espera o motor ter os tópicos publicados
    while (colorBR_ == -1 || colorFL_ == -1 || colorBL_ == -1 || colorFR_ == -1)
    {
        ROS_INFO("Waiting 4 Topics of sensors");
        update(0.5);
    }*/

    if (!nh_.param("MAIOR_QUE_PRETO", MAIOR_QUE_PRETO, 59))
    {
        ROS_ERROR("Failed to get param 'MAIOR_QUE_PRETO'");
    }

    if (!nh_.param("MAIOR_QUE_VERDE", MAIOR_QUE_VERDE, 299))
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
    if (!nh_.param("TEMPO_TRANSICAO_ESQUERDA", TEMPO_TRANSICAO_ESQUERDA, 0.4))
    {
        ROS_ERROR("Failed to get param 'TEMPO_TRANSICAO_ESQUERDA'");
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

    if (!nh_.param("FREQUENCIA_PARA_ALINHAR", FREQUENCIA_PARA_ALINHAR, 30.0))
    {
        ROS_ERROR("Failed to get param 'FREQUENCIA_PARA_ALINHAR'");
    }

    if (!nh_.param("TEMPO_DIREITA", TEMPO_DIREITA, 0.2))
    {
        ROS_ERROR("Failed to get param 'TEMPO_DIREITA'");
    }

    if (!nh_.param("Kp", Kp, 0.3))
    {
        ROS_ERROR("Failed to get param 'Kp'");
    }
    if (!nh_.param("LDIAG", LDIAG, 0.116383204))
    {
        ROS_ERROR("Failed to get param 'LDIAG'");
    }
    if (!nh_.param("DIAMETRO_RODA", DIAMETRO, 0.099060))
    {
        ROS_ERROR("Failed to get param 'DIAMETRO_RODA'");
    }
    if (!nh_.param("VEL_MAX", VEL_MAX, 2.5))
    {
        ROS_ERROR("Failed to get param 'VEL_MAX'");
    }
    if (!nh_.param("DIST_DIR_SONAR", DIST_DIR_SONAR, 7.0))
    {
        ROS_ERROR("Failed to get param 'DIST_DIR_SONAR'");
    }
    if (!nh_.param("DIST_DIR_SONAR", DIST_ESQ_SONAR, 10.0))
    {
        ROS_ERROR("Failed to get param 'DIST_ESQ_SONAR'");
    }
    if (!nh_.param("DIST_CEN_SONAR", DIST_CEN_SONAR, 3.0))
    {
        ROS_ERROR("Failed to get param 'DIST_CEN_SONAR'");
    }
}

/* Essa função envia a velocidade para os tópicos dos PIDs,
  ** a saída da velocidade deve ser em RAD/s **/
bool kineControl::robot::setVelocity(const geometry_msgs::Twist &vel)
{
    // Ângulo XY do vetor velocidade
    double theta = atan2(vel.linear.y, vel.linear.x);

    // Módulo do vetor velocidade em XY
    double w_module = sqrt(pow(vel.linear.x, 2) + pow(vel.linear.y, 2)) / (DIAMETRO);
    double conversao_angular = (LDIAG) / (DIAMETRO); // Converte o Rad/s em metros/s depois em Rad/s para as rodas

    std_msgs::Float64 Wfl;
    std_msgs::Float64 Wfr;
    std_msgs::Float64 Wbl;
    std_msgs::Float64 Wbr;

    // Modelo omnidirecional da base
    Wfl.data = ((w_module)*sin(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // Rad/s
    Wfr.data = ((w_module)*cos(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // Rad/s
    Wbl.data = ((w_module)*cos(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // Rad/s
    Wbr.data = ((w_module)*sin(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // Rad/s

    double maior_velocidade = VEL_MAX;
    bool alguma_maior_que_max = false;

    ROS_INFO_STREAM("Wfl: " << Wfl.data);
    if (fabs(Wfl.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wfl.data);
    }
    if (fabs(Wfr.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wfr.data);
    }
    if (fabs(Wbl.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wbl.data);
    }
    if (fabs(Wbr.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wbr.data);
    }

    if (alguma_maior_que_max)
    {
        Wfl.data = Wfl.data / maior_velocidade * VEL_MAX;
        Wfr.data = Wfr.data / maior_velocidade * VEL_MAX;
        Wbl.data = Wbl.data / maior_velocidade * VEL_MAX;
        Wbr.data = Wbr.data / maior_velocidade * VEL_MAX;
    }
    ROS_INFO_STREAM("Wfl: " << Wfl.data);
    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

/* Essa função envia a velocidade para os tópicos dos PIDs, a diferença é que ela usa um controle mais fácil de ser interfaceado com 
 * as funções de seguir linha
 * 
  ** a saída da velocidade deve ser em RAD/s **/
bool kineControl::robot::setVelocityPID(float velL, float velR, geometry_msgs::Twist *vel)
{
    std_msgs::Float64 Wfl;
    std_msgs::Float64 Wfr;
    std_msgs::Float64 Wbl;
    std_msgs::Float64 Wbr;

    // Modelo simples da base
    Wfl.data = velL;
    Wfr.data = velR;
    Wbl.data = velL;
    Wbr.data = velR;

    if (vel != 0)
    {
        // Ângulo XY do vetor velocidade
        double theta = atan2(vel->linear.y, vel->linear.x);

        // Módulo do vetor velocidade em XY
        double w_module = sqrt(pow(vel->linear.x, 2) + pow(vel->linear.y, 2)) / (DIAMETRO);
        double conversao_angular = (LDIAG) / (DIAMETRO); // Converte o Rad/s em metros/s depois em Rad/s para as rodas

        // Modelo omnidirecional da base
        Wfl.data += ((w_module)*sin(PI / 4 + theta) + (vel->angular.z) * conversao_angular) * PI; // Rad/s
        Wfr.data += ((w_module)*cos(PI / 4 + theta) - (vel->angular.z) * conversao_angular) * PI; // Rad/s
        Wbl.data += ((w_module)*cos(PI / 4 + theta) + (vel->angular.z) * conversao_angular) * PI; // Rad/s
        Wbr.data += ((w_module)*sin(PI / 4 + theta) - (vel->angular.z) * conversao_angular) * PI; // Rad/s
    }

    double maior_velocidade = VEL_MAX;
    bool alguma_maior_que_max = false;

    if (fabs(Wfl.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wfl.data);
    }
    if (fabs(Wfr.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wfr.data);
    }
    if (fabs(Wbl.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wbl.data);
    }
    if (fabs(Wbr.data) > maior_velocidade)
    {
        alguma_maior_que_max = true;
        maior_velocidade = fabs(Wbr.data);
    }

    if (alguma_maior_que_max)
    {
        Wfl.data = Wfl.data / maior_velocidade * VEL_MAX;
        Wfr.data = Wfr.data / maior_velocidade * VEL_MAX;
        Wbl.data = Wbl.data / maior_velocidade * VEL_MAX;
        Wbr.data = Wbr.data / maior_velocidade * VEL_MAX;
    }

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

// Função para alinhar os sensores de baixo do robô quando a linha presta está à frente. Os dois sensores da frente deverão ficar sobre a linha preta ou azul, equanto os de trás devem
// ficar fora da linha.
void kineControl::alinhar_frente(kineControl::robot &robot, int initial_erro)
{
    ROS_INFO("KINECONTROL - alinhar_frente() - alinhar os sensores da frente");

    geometry_msgs::Twist velocidade;

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();
    ros::spinOnce();
    int erro1 = initial_erro, erro2 = initial_erro;
    double velE, velD;
    while (erro1 != 0 || erro2 != 0)
    {
        // Movimentar lateralmente
        velocidade.linear.y = 0;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E2E3(robot, erro1);
        erro2 = kineControl::erro_sensores_D2D3(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
    }
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}
// Função para alinhar os sensores de baixo do robô quando a linha presta está atrás. Os dois sensores da frente deverão ficar sobre a linha preta ou azul, equanto os de trás devem
// ficar fora da linha.
void kineControl::alinhar_atras(kineControl::robot &robot)

{
    ROS_INFO("KINECONTROL - alinhar_atras() - a linha preta esta a frente");

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = -5, erro2 = -5;
    double velE, velD;

    rate.sleep();

    while (erro1 != 0 || erro2 != 0)
    {
        erro1 = kineControl::erro_sensores_E0E1(robot, erro1);
        erro2 = kineControl::erro_sensores_D0D1(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD);

        rate.sleep();
    }

    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::alinhar_doca(kineControl::robot &robot)
{

    ROS_INFO("KINECONTROL - alinhar_doca");

    geometry_msgs::Twist velocidade;

    ros::spinOnce();

    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    alinhar_atras(robot);

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::alinhar_depositar_esquerda(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - alinhar_depositar_esquerda() ");

    kineControl::alinhar_atras(robot);

    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();

    int erro1 = 0, erro2 = 0;
    double velE = 0, velD = 0;

    ros::spinOnce();
    while (robot.colorFE_ == AZUL_VERDE)
    {
        // Andar uma distância predefinida
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;
        velocidade.linear.y = -VEL_Y;

        erro1 = kineControl::erro_sensores_E0E1(robot, erro1);
        erro2 = kineControl::erro_sensores_D0D1(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::esquerda(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - esquerda");

    kineControl::alinhar_esquerda(robot);

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();

    geometry_msgs::Twist velocidade;

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = 0, erro2 = 0;
    double velE, velD;

    while (now - begin < ros::Duration(TEMPO_TRANSICAO_ESQUERDA))
    {
        // Movimentar lateralmente
        velocidade.linear.y = -VEL_Y;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E2E3(robot, erro1);
        erro2 = kineControl::erro_sensores_D2D3(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
        ros::spinOnce();
        now = ros::Time::now();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    kineControl::alinhar_frente(robot);
}

void kineControl::direita(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - direita() ");

    kineControl::alinhar_esquerda(robot, -1);

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();

    geometry_msgs::Twist velocidade;

    int erro1 = 0, erro2 = 0;
    double velE, velD;

    while (now - begin < ros::Duration(TEMPO_DIREITA))
    {
        // Movimentar lateralmente
        velocidade.linear.y = VEL_Y;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E2E3(robot, erro1);
        erro2 = kineControl::erro_sensores_D2D3(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
        now = ros::Time::now();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    kineControl::alinhar_frente(robot);
}

// Função para detectar as linhas pretas durante as transições para ESQUERDA
// dir_esq -> 1 para ir para esquerda e -1 para ir para direita
void kineControl::alinhar_esquerda(kineControl::robot &robot, int dir_esq)
{

    ROS_INFO_STREAM("KINECONTROL - alinhar_esquerda() ");

    geometry_msgs::Twist velocidade;
    ros::spinOnce();

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = 0, erro2 = 0;
    double velE, velD;

    while (robot.colorFE_ != PRETO)
    {
        // Movimentar lateralmente
        velocidade.linear.y = -VEL_Y * dir_esq;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E2E3(robot, erro1);
        erro2 = kineControl::erro_sensores_D2D3(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();

    kineControl::alinhar_frente(robot);
}

// Função para detectar as linhas pretas durante as transições para Direita
void kineControl::alinhar_direita(kineControl::robot &robot)
{

    ROS_INFO_STREAM("KINECONTROL - alinhar_direita() ");

    //  kineControl::alinhar_atras(robot);
    geometry_msgs::Twist velocidade;

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();

    int erro1 = 0, erro2 = 0;
    double velE, velD;
    ros::spinOnce();
    while (robot.colorFD_ != PRETO)
    {
        // Movimentar lateralmente
        velocidade.linear.y = VEL_Y;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E2E3(robot, erro1);
        erro2 = kineControl::erro_sensores_D2D3(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;
        //ROS_INFO_STREAM("Erro 1: " << erro1 << " Erro 2:" << erro2);
        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

// Faz o robô girar 90 graus utilizando o encoder
/*
void kineControl::girar90graus(kineControl::robot &robot){

}
*/
void kineControl::ir_doca(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - ir_doca()");

    kineControl::alinhar_frente(robot);

    geometry_msgs::Twist velocidade;

    // Ir para trás
    velocidade.linear.x = -0.1;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(0.5).sleep();

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

    // Alinhar com a linha verde
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
    ros::Duration(0.5).sleep();

    // Girar 90 Graus
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = PI / 3;
    robot.setVelocity(velocidade);
    ros::Duration(TEMPO_MEIA_VOLTA).sleep();

    // É possível alinhar com a linha verde, se necessário

    kineControl::linha_preta(robot);
    //ROS_INFO("CHEGOU NO QUADRANTE");
}

// Falta comentar aqui
void kineControl::linha_preta(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - linha_preta - a linha preta esta a frente");

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();
    double VelE, VelD;
    int erro1 = -5, erro2 = -5;

    geometry_msgs::Twist velocidade;
    velocidade.linear.x = VEL_X;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(1).sleep();

    while ((erro1 != 0 || erro2 != 0) && ros::ok())
    {
        erro1 = kineControl::erro_sensores_esquerda_com_preto(robot, erro1);
        erro2 = kineControl::erro_sensores_direita_com_preto(robot, erro2);

        VelE = -erro1 * Kp;
        VelD = -erro2 * Kp;

        robot.setVelocityPID(VelE, VelD);

        rate.sleep();
    }

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);
}

void kineControl::alinhar_containerdepositado(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - alinhar_containerdepositado");
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Duration time(1 / FREQUENCIA_PARA_ALINHAR);
    ros::spinOnce();
    /*
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

    */
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> MoveClient;
typedef actionlib::SimpleActionClient<projeto_semear::setEletroimaAction> SetClient;

void kineControl::alinhar_pilha(kineControl::robot &robot, int dir, bool container_esq_esta_vazio)
{

    // 0.07 -> container da esquerda 0
    // 0.01   -> container da direita 1
    ROS_INFO("KINECONTROL - alinhar_pilha");

    robot.lateral_distance_;
    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);

    // Alinhar para frente
    double velE, velD;
    int erro1 = -5, erro2 = -5;
    ros::spinOnce();

    while ((erro1 != 0 || erro2 != 0) && ros::ok())
    {
        erro1 = kineControl::erro_sensores_E0E1(robot, erro1);
        erro2 = kineControl::erro_sensores_D0D1(robot, erro2);

        velE = -(erro1)*Kp;
        velD = -(erro2)*Kp;

        robot.setVelocityPID(velE, velD);

        rate.sleep();
        ros::spinOnce();
    }

    geometry_msgs::Twist velocidade;

    velocidade.linear.y = 0;
    velocidade.linear.x = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);

    // esquerda é 0 e direita é 1 e 2 é o centro
    double dist;
    if (dir == 0)
    {
        dist = DIST_ESQ_SONAR; //10

        if (container_esq_esta_vazio)
        {
            dist = 0.071 + 0.061;
        }
    }
    else if (dir == 1)
    {
        dist = DIST_DIR_SONAR; // 3

        if (container_esq_esta_vazio)
        {
            dist = 0.01 + 0.061;
        }
    }
    else if (dir == 2) // 7
    {
        dist = DIST_CEN_SONAR;

        if (container_esq_esta_vazio)
        {
            dist = 0.04 + 0.061;
        }
    }
    ROS_INFO("KINECONTROL - alinhar_pilha");
    // Movimentar lateralmente
    double dif = dist - robot.lateral_distance_;
    while (std::fabs(dif) > PRECISAO_DIST_ALINHAR_PILHA && ros::ok())
    {
        // Andar uma distância predefinida
        velocidade.linear.y = -dif;
        velocidade.linear.x = 0;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_E0E1(robot, erro1);
        erro2 = kineControl::erro_sensores_D0D1(robot, erro2);

        velE = -(erro1)*Kp;
        velD = -(erro2)*Kp;

        robot.setVelocityPID(velE, velD, &velocidade);

        rate.sleep();
        ros::spinOnce();
        dif = dist - robot.lateral_distance_;
    }

    // Re-alinhar
    ros::spinOnce();
    erro1 = 0;
    erro2 = 0;
    ROS_INFO("KINECONTROL - alinhar_pilha");
    while ((erro1 != 0 || erro2 != 0) && ros::ok())
    {
        erro1 = kineControl::erro_sensores_E0E1(robot, erro1);
        erro2 = kineControl::erro_sensores_D0D1(robot, erro2);

        velE = -erro1 * Kp;
        velD = -erro2 * Kp;

        robot.setVelocityPID(velE, velD);

        rate.sleep();
        ros::spinOnce();
    }

    velocidade.linear.y = 0;
    velocidade.linear.x = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::pegar_container(kineControl::robot &robot, char lado_escolhido)
{
    // Espera-se que o código já saiba se deve pegar o container da direita ou da esquerda
    ros::NodeHandle nh;

    // Conectando com o enable do Eletroima
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);

    // Conectando com as interfaces de mover o eletroima
    MoveClient move_client("moveEletroima", true); // true -> don't need ros::spin()
    SetClient set_client("setEletroima", true);    // true -> don't need ros::spin()

    // Conectando com o GPS
    ros::ServiceClient pose_client = nh.serviceClient<projeto_semear::GetPose>("gps");

    // Conectando com o getContainerInfo
    ros::ServiceClient get_client = nh.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");

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

    set_goal.pose = set_goal.posicao_pegar_container_superior;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    // Verificar o número de containers na pilha
    projeto_semear::GetContainerInfo get_container_info_msg;
    get_container_info_msg.request.where = lado;

    get_client.call(get_container_info_msg);
    double altura = get_container_info_msg.response.lista.size();

    // Ligar o Eletroimã:
    msg.data = true;
    pub.publish(msg);

    // Girar a guarra 90º
    projeto_semear::moveEletroimaGoal move_goal;
    move_goal.deslocamento.linear.x = 0.0;
    move_goal.deslocamento.linear.y = 0;
    move_goal.deslocamento.linear.z = -0.045 * (4 - altura);
    move_goal.deslocamento.angular.z = 0;
    move_client.sendGoal(move_goal, doneCb, activeCb, feedbackCb);
    move_client.waitForResult(ros::Duration());

    // Erguendo container
    set_goal.pose = set_goal.posicao_segurar_container;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    // Deixando o container perpendicular à posição inicial
    set_goal.pose = set_goal.posicao_segurar_container_rotacionado;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    // Alinha novamente com a linha preta
    kineControl::alinhar_frente(robot, 5);
}

// Posicionamento ideal dos 3 sensores:    S1<--15mm-->S2<--15mm-->S3
// Erro positivo -> o robô precisa ir para trás
// Função deve ser usada apenas para se movimentar lateralmente quando os dois sensores do meio estiverem no preto
int kineControl::erro_sensores_E0E1(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE0_ == BRANCO && robot.colorE1_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorE0_ != BRANCO)
        return 1;

    if (robot.colorE0_ == BRANCO && robot.colorE1_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorE1_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!

    return temp_erro;
}
int kineControl::erro_sensores_E2E3(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE2_ == BRANCO && robot.colorE3_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorE2_ != BRANCO)
        return 1;

    if (robot.colorE2_ == BRANCO && robot.colorE3_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorE3_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!

    return temp_erro;
}
int kineControl::erro_sensores_D0D1(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD0_ == BRANCO && robot.colorD1_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorD0_ != BRANCO)
        return 1;

    if (robot.colorD0_ == BRANCO && robot.colorD1_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorD1_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
int kineControl::erro_sensores_D2D3(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD2_ == BRANCO && robot.colorD3_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorD2_ != BRANCO)
        return 1;

    if (robot.colorD2_ == BRANCO && robot.colorD3_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorD3_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
int kineControl::erro_sensores_esquerda_com_preto(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE2_ != PRETO && robot.colorE3_ == PRETO)
    {
        return 0;
    }

    if (robot.colorE2_ == PRETO)
        return 1;

    if (robot.colorE2_ != PRETO && robot.colorE3_ != PRETO && temp_erro > 0)
        return 2;

    if (robot.colorE3_ != PRETO)
        return -1;
}

int kineControl::erro_sensores_direita_com_preto(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD2_ != PRETO && robot.colorD3_ == PRETO)
    {
        return 0;
    }

    if (robot.colorD2_ == PRETO)
        return 1;

    if (robot.colorD2_ != PRETO && robot.colorD3_ != PRETO && temp_erro > 0)
        return 2;

    if (robot.colorD3_ != PRETO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
