#include "projeto_semear/kine_control.h"
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <projeto_semear/GetContainerInfo.h>
#include <vector>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/MoveContainer.h>
#include <std_msgs/Bool.h>
#include <boost/bind.hpp>
#include <stdlib.h>

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)

void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback) {}
void doneCb(const actionlib::SimpleClientGoalState &state, const projeto_semear::moveEletroimaResultConstPtr &result) {}
void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback) {}
void doneCb2(const actionlib::SimpleClientGoalState &state, const projeto_semear::setEletroimaResultConstPtr &result) {}
void activeCb() {}

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

    lineSensorE1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorE1", 1, boost::bind(callback, _1, boost::ref(colorE1_)));
    lineSensorE2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorE2", 1, boost::bind(callback, _1, boost::ref(colorE2_)));

    lineSensorD1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorD1", 1, boost::bind(callback, _1, boost::ref(colorD1_)));
    lineSensorD2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorD2", 1, boost::bind(callback, _1, boost::ref(colorD2_)));

    lineSensorF1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorF1", 1, boost::bind(callback, _1, boost::ref(colorF1_)));
    lineSensorF2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorF2", 1, boost::bind(callback, _1, boost::ref(colorF2_)));

    lineSensorB1_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorB1", 1, boost::bind(callback, _1, boost::ref(colorF1_)));
    lineSensorB2_ = nh_.subscribe<std_msgs::Float32>("/image_converter/lineSensorB2", 1, boost::bind(callback, _1, boost::ref(colorF2_)));

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

    /* Espera o motor ter os tópicos publicados
    while (colorBR_ == -1 || colorFL_ == -1 || colorBL_ == -1 || colorFR_ == -1)
    {
        ROS_INFO("Waiting 4 Topics of sensors");
        update(0.5);
    }*/

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
}

/* Controla a velocidade do robô:

Input: 
    Twist Vel  -> vetor da velocidade, só levamos em consideracao velocidade linear X e Y e a angular Z
*/
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
    Wfl.data = ((w_module)*sin(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // !!!
    Wfr.data = ((w_module)*cos(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // !!!
    Wbl.data = ((w_module)*cos(PI / 4 + theta) + (vel.angular.z) * conversao_angular) * PI; // !!!
    Wbr.data = ((w_module)*sin(PI / 4 + theta) - (vel.angular.z) * conversao_angular) * PI; // !!!

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

/* Controla a velocidade do robô para ser usada com as funções PID -> ver função alinhar_esquerda

A vantagem de usar essa interface é que, enquanto o robô se movimenta para esquerda ou para direita, podemos sobrepor ao seu movimento lateral
um movimento ascendente ou descendente de maneira a manter os sensores da esquerda ou da direita alinhados. Exemplo:

        Suponha que o robô esteja seguindo linha para direita e os 2 sensores da esquerda se desalinham para cima. Basta o PID avisar que há um erro e
        somar à velocidade das rodas da esquerda uma velocidade para baixo. Assim, o robô conserta seu alinhamento sem parar de se movimentar.
    
Input:
    float velR -> velocidade a ser somada nas rodas da direita
    float velL -> velocidade a ser somada nas rodas da esquerda
    Twist Vel  -> vetor da velocidade, só levamos em consideracao velocidade linear X e Y e a angular Z

*/
bool kineControl::robot::setVelocityPID(float velL, float velR, geometry_msgs::Twist *vel)
{
    std_msgs::Float32 Wfl;
    std_msgs::Float32 Wfr;
    std_msgs::Float32 Wbl;
    std_msgs::Float32 Wbr;

    // Modelo omnidirecional da base
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
        Wfl.data += ((w_module)*sin(PI / 4 + theta) + (vel->angular.z) * conversao_angular) * PI; // !!!
        Wfr.data += ((w_module)*cos(PI / 4 + theta) - (vel->angular.z) * conversao_angular) * PI; // !!!
        Wbl.data += ((w_module)*cos(PI / 4 + theta) + (vel->angular.z) * conversao_angular) * PI; // !!!
        Wbr.data += ((w_module)*sin(PI / 4 + theta) - (vel->angular.z) * conversao_angular) * PI; // !!!
    }

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}

// Função para alinhar os sensores de baixo do robô quando a linha presta está à frente. Os dois sensores da frente deverão ficar sobre a linha preta ou azul, equanto os de trás devem
// ficar fora da linha.
//FEITO
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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//FEITO
void kineControl::alinhar_atras(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - alinhar_atras() - a linha preta esta a frente");

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = -5, erro2 = -5;
    double velE, velD;

    rate.sleep();

    while (erro1 != 0 || erro2 != 0)
    {
        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//FEITO
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
//FEITO
void kineControl::alinhar_depositar_esquerda(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - alinhar_depositar_esquerda() ");
    
    kineControl::alinhar_atras(robot);

    geometry_msgs::Twist velocidade;
    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();
    
    int erro1=0, erro2=0;
    double velE=0, velD=0;
    
    ros::spinOnce();
    while (robot.colorFE_ == AZUL_VERDE)
    {
        // Andar uma distância predefinida
        velocidade.linear.x = 0 ;
        velocidade.angular.z = 0 ;
        velocidade.linear.y = -VEL_Y;
        
        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//FEITO
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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//FEITO
void kineControl::direita(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - direita() ");

    kineControl::alinhar_esquerda(robot, - 1);

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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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

void kineControl::frente(kineControl::robot &robot)
{
    ROS_INFO_STREAM("KINECONTROL - esquerda");

    kineControl::alinhar_frente1(robot);

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

        erro1 = kineControl::erro_sensores_F1F2(robot, erro1);
        erro2 = kineControl::erro_sensores_B1B2(robot, erro2);

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

// Função para detectar as linhas pretas durante as transições para ESQUERDA
// dir_esq -> 1 para ir para esquerda e -1 para ir para direita
//FEITO
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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//FEITO
void kineControl::alinhar_frente1(kineControl::robot &robot, int dir_esq)
{

    ROS_INFO_STREAM("KINECONTROL - alinhar_frente() ");

    geometry_msgs::Twist velocidade;
    ros::spinOnce();

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = 0, erro2 = 0;
    double velE, velD;

    while (robot.colorFE_ != PRETO)
    {
        // Movimentar lateralmente
        velocidade.linear.y = 0;
        velocidade.linear.x = VEL_Y * dir_esq;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_F1F2(robot, erro1);
        erro2 = kineControl::erro_sensores_B1B2(robot, erro2);

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
//FEITO
void kineControl::alinhar_tras1(kineControl::robot &robot, int dir_esq)
{

    ROS_INFO_STREAM("KINECONTROL - alinhar_tras() ");

    geometry_msgs::Twist velocidade;
    ros::spinOnce();

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    int erro1 = 0, erro2 = 0;
    double velE, velD;

    while (robot.colorFE_ != PRETO)
    {
        // Movimentar lateralmente
        velocidade.linear.y = 0;
        velocidade.linear.x = -VEL_Y * dir_esq;
        velocidade.angular.z = 0;

        erro1 = kineControl::erro_sensores_F1F2(robot, erro1);
        erro2 = kineControl::erro_sensores_B1B2(robot, erro2);

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
//FEITO
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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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
//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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
//FEITO
void kineControl::linha_preta(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - linha_preta - a linha preta esta a frente");

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();
    double VelE, VelD;
    int erro1 = -5, erro2 = -5;

    while ((erro1 != 0 || erro2 != 0) && ros::ok())
    {
        erro1 = kineControl::erro_sensores_esquerda_com_preto(robot, erro1);
        erro2 = kineControl::erro_sensores_direita_com_preto(robot, erro2);

        VelE = -erro1 * Kp;
        VelD = -erro2 * Kp;

        robot.setVelocityPID(VelE, VelD);

        rate.sleep();
    }

    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);
}
//FEITO
void kineControl::linha_verde(kineControl::robot &robot)
{
    ROS_INFO("KINECONTROL - linha_verde - a linha verde esta a frente");

    ros::Rate rate(FREQUENCIA_PARA_ALINHAR);
    rate.sleep();
    double VelE, VelD;
    int erro1 = -5, erro2 = -5;

    while ((erro1 != 0 || erro2 != 0) && ros::ok())
    {
        erro1 = kineControl::erro_sensores_esquerda_com_verde(robot, erro1);
        erro2 = kineControl::erro_sensores_direita_com_verde(robot, erro2);

        VelE = -erro1 * Kp;
        VelD = -erro2 * Kp;

        robot.setVelocityPID(VelE, VelD);

        rate.sleep();
    }

    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);
}

//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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


//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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
        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
        dist = 0.071;

        if (container_esq_esta_vazio)
        {
            dist = 0.071 + 0.061;
        }
    }
    else if (dir == 1)
    {
        dist = 0.01;

        if (container_esq_esta_vazio)
        {
            dist = 0.01 + 0.061;
        }
    }
    else if (dir == 2)
    {
        dist = 0.04;

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

        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
        erro1 = kineControl::erro_sensores_E1E2(robot, erro1);
        erro2 = kineControl::erro_sensores_D1D2(robot, erro2);

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
//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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

    //ROS_INFO_STREAM("PEGAR CONTAINER - Erguer Container");
    set_goal.pose = set_goal.posicao_segurar_container;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    //ROS_INFO_STREAM("PEGAR CONTAINER - Rotacionar Container em cima");
    set_goal.pose = set_goal.posicao_segurar_container_rotacionado;
    set_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_client.waitForResult(ros::Duration());

    kineControl::alinhar_frente(robot, 5);
}

// Concerning is not properly working yet, need to improve the math
// Concerning é quando o robô gira em torno de uma das suas rodas.
//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
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
//NÃO SEI AINDA, PERGUNTAR PARA O GONZALES
void kineControl::depositar_container(kineControl::robot &robot, uint32_t cor, uint32_t posicao_origem_do_container)
{

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
    MoveClient move_eletro_client("moveEletroima", true); // true -> don't need ros::spin()
    SetClient set_eletro_client("setEletroima", true);    // true -> don't need ros::spin()

    ROS_INFO_STREAM("KINECONTROL - depositar_container");


    std_msgs::Bool msg;
    msg.data = true;
    pub.publish(msg);

    move_eletro_client.waitForServer();
    set_eletro_client.waitForServer();

    projeto_semear::moveEletroimaGoal move_goal;
    projeto_semear::setEletroimaGoal set_goal;

    // Centraliza a garra
    set_goal.pose = set_goal.posicao_inicial_rotacionada;
    set_eletro_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_eletro_client.waitForResult();

    // Pegar a localização do robô
    ros::ServiceClient pose_client = nh.serviceClient<projeto_semear::GetPose>("gps");
    projeto_semear::GetPose srv;
    pose_client.call(srv);
    int localizacao_aux = (std::uint32_t)srv.response.pose.location;

    // Transformando a localização_aux para localização correta
    int localizacao;
    switch (localizacao_aux)
    {
    case 3:
        localizacao = 12;

    case 4:
        localizacao = 13;
    }

    // Utilizando o servico de mapa_container
    ros::ServiceClient get_client = nh.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
    ros::ServiceClient move_container_client = nh.serviceClient<projeto_semear::MoveContainer>("moveContainer");

    projeto_semear::GetContainerInfo get_srv;
    projeto_semear::MoveContainer move_container_srv;
    srv.request.set = false;

    // Informando a localizacao da pilha onde o robo se encontra
    get_srv.request.where = cor;
    ROS_INFO_STREAM("DEPOSITAR CONTAINER - Posicao do robo: " << get_srv.request.where);
    get_client.call(get_srv);

    // Pegando o vetor que contem os containers depositados
    std::vector<std::uint32_t> vec = get_srv.response.lista;

    int code = vec.size(); //variável que guarda quantos containers têm em uma pilha

    if(vec.back() == 0 ){
        code = 0;
    } else{
        ROS_INFO_STREAM("DEPOSITAR CONTAINER - Ultimo container tem código: " << vec.back());
    }

    ROS_INFO_STREAM("DEPOSITAR CONTAINER - Valor do code:" << code);

    /*Code == 0: nenhum container depositado
        Code != 0: já existe um ou mais containers na pilha*/

    kineControl::alinhar_depositar_esquerda(robot);
    // Alinhar com o container de baixo
    if (code != 0)
        kineControl::alinhar_containerdepositado(robot);

    move_goal.deslocamento.angular.z = 0;
    move_goal.deslocamento.linear.x = 0;
    move_goal.deslocamento.linear.y = 0;
    move_goal.deslocamento.linear.z = -0.137 + (code * 0.042); //0,2 chute da altura do container
    move_eletro_client.sendGoal(move_goal, &doneCb, &activeCb, &feedbackCb);
    move_eletro_client.waitForResult(ros::Duration());

    // Desligando o Eletroima
    msg.data = false;
    pub.publish(msg);

    // Atualiza que o container foi depositado
    move_container_srv.request.where = posicao_origem_do_container;
    move_container_client.call(move_container_srv);

    set_goal.pose = set_goal.posicao_inicial_rotacionada;
    set_eletro_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
    set_eletro_client.waitForResult();

}

// Posicionamento ideal dos 3 sensores:    S1<--15mm-->S2<--15mm-->S3
// Erro positivo -> o robô precisa ir para trás
// Função deve ser usada apenas para se movimentar lateralmente quando os dois sensores do meio estiverem no preto

//APAGAR AS FUNÇÕES QUANDO ESTIVER TUDO CERTO
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

//FEITO
int kineControl::erro_sensores_D1D2(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD1_ == BRANCO && robot.colorD2_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorD1_ != BRANCO)
        return 1;

    if (robot.colorD1_ == BRANCO && robot.colorD2_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorD2_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
//FEITO
int kineControl::erro_sensores_E1E2(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE1_ == BRANCO && robot.colorE2_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorE1_ != BRANCO)
        return 1;

    if (robot.colorE1_ == BRANCO && robot.colorE2_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorE2_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
//FEITO
int kineControl::erro_sensores_F1F2(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorF1_ == BRANCO && robot.colorF2_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorF1_ != BRANCO)
        return 1;

    if (robot.colorF1_ == BRANCO && robot.colorF2_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorF2_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
//FEITO
int kineControl::erro_sensores_B1B2(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorB1_ == BRANCO && robot.colorB2_ != BRANCO)
    {
        return 0;
    }

    if (robot.colorB1_ != BRANCO)
        return 1;

    if (robot.colorB1_ == BRANCO && robot.colorB2_ == BRANCO && temp_erro > 0)
        return 2;

    if (robot.colorB2_ == BRANCO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}

//FEITO
int kineControl::erro_sensores_esquerda_com_preto(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE1_ != PRETO && robot.colorE2_ == PRETO)
    {
        return 0;
    }

    if (robot.colorE1_ == PRETO)
        return 1;

    if (robot.colorE1_ != PRETO && robot.colorE2_ != PRETO && temp_erro > 0)
        return 2;

    if (robot.colorE2_ != PRETO)
        return -1;
}
//FEITO
int kineControl::erro_sensores_direita_com_preto(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD1_ != PRETO && robot.colorD2_ == PRETO)
    {
        return 0;
    }

    if (robot.colorD1_ == PRETO)
        return 1;

    if (robot.colorD1_ != PRETO && robot.colorD2_ != PRETO && temp_erro > 0)
        return 2;

    if (robot.colorD2_ != PRETO)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}

//FEITO
int kineControl::erro_sensores_esquerda_com_verde(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorE1_ != AZUL_VERDE && robot.colorE2_ == AZUL_VERDE)
    {
        return 0;
    }

    if (robot.colorE1_ == AZUL_VERDE)
        return 1;

    if (robot.colorE1_ != AZUL_VERDE && robot.colorE2_ != AZUL_VERDE && temp_erro > 0)
        return 2;

    if (robot.colorE2_ != AZUL_VERDE)
        return -1;
}
//FEITO
int kineControl::erro_sensores_direita_com_verde(kineControl::robot &robot, int temp_erro)
{
    ros::spinOnce();

    if (robot.colorD1_ != AZUL_VERDE && robot.colorD2_ == AZUL_VERDE)
    {
        return 0;
    }

    if (robot.colorD1_ == AZUL_VERDE)
        return 1;

    if (robot.colorD1_ != AZUL_VERDE && robot.colorD2_ != AZUL_VERDE && temp_erro > 0)
        return 2;

    if (robot.colorD2_ != AZUL_VERDE)
        return -1;

    // Caso chegue aqui ?!
    return temp_erro;
}
