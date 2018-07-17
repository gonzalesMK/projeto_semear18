#include "projeto_semear/kine_control.h"

#include <boost/bind.hpp>

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)

/** Implementação do Objeto que abstrai o robô. **/

void callback(const std_msgs::Float32ConstPtr &msg, kineControl::color &color)
{
    if (msg->data > kineControl::MAIOR_QUE_VERDE)
    {
        color = kineControl::color::BRANCO;
    }
    else if (msg->data > kineControl::MAIOR_QUE_PRETO)
    {
        color = kineControl::color::AZUL_VERDE;
    }
    else
        color = kineControl::color::PRETO;
}

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

    // Necessário um tempo para inicializar os nós
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // Espera o motor ter os tópicos publicados
    while (colorBR_ == -1 || colorFL_ == -1 || colorBL_ == -1 || colorFR_ == -1)
    {
        ROS_INFO("Waiting 4 Topics");
        update(0.5);
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

void kineControl::alinhar(kineControl::robot &robot)
{
    ROS_INFO("Alinhando com a linha Preta (a linha preta esta atras)");
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Duration time(0.05);
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
            velocidade.angular.z = -VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
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

void kineControl::alinhar_doca(kineControl::robot &robot)
{

    ROS_INFO("Alinhando com a linha da doca");

    geometry_msgs::Twist velocidade;

    int code = 0;
    ros::spinOnce();
    ros::Duration time(0.05);

    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    while ((robot.colorFR_ == BRANCO || robot.colorFL_ == BRANCO) && ros::ok())
    {
        code = 0;
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;

        // Caso 0: todos os sensores no branco. Supõe-se que o robô ultrapassou o alinhamento necessário. Garantir isso no resto do código
        // Caso 3: Caso o sensor FrontRight esteja marcando verde, mas o FrontLeft não -> girar positivo
        // Caso 4: Caso o sensor FrontLeft esteja marcando verde, mas o  FrontRight não -> girar negativo
        // ROS_INFO_STREAM("\nFL " << colorFL << "FR " << colorFR << "\nBL " << colorBL << "BR " << colorBR);

        if (robot.colorFL_ == BRANCO && robot.colorFR_ == BRANCO)
            code = 0;
        else if (robot.colorFL_ == BRANCO && robot.colorFR_ != BRANCO)
            code = 1;
        else if (robot.colorFL_ != BRANCO && robot.colorFR_ == BRANCO)
            code = 2;

        switch (code)
        {
        case 0:
            velocidade.linear.x = 0.05;
            break;
        case 1:
            velocidade.angular.z = -VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
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

void kineControl::esquerda(kineControl::robot &robot)
{
    kineControl::alinhar(robot);

    ROS_INFO_STREAM("Transicaoo do quadrante para ESQUERDA ");

    // Andar uma distância predefinida
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = -0.1;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(3).sleep();

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}

void kineControl::ir_doca(kineControl::robot &robot)
{
    kineControl::alinhar(robot);

    ROS_INFO_STREAM("Transicao do quadrante para docas ");

    geometry_msgs::Twist velocidade;

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

    // Alinhar
    kineControl::alinhar_doca(robot);
}

void kineControl::ir_quadrante(kineControl::robot &robot){
    
    ROS_INFO_STREAM("Transicao da Doca para Quadrante");
    geometry_msgs::Twist velocidade;

    // Girar 90 Graus
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = PI / 3;
    
    robot.setVelocity(velocidade);
    ros::Duration(3).sleep();

    kineControl::linha_preta(robot);
}
void kineControl::direita(kineControl::robot &robot)
{
    kineControl::alinhar(robot);

    ROS_INFO_STREAM("Transição do quadrante para DIREITA ");

    // Andar uma distância predefinida
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 0.1;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
    ros::Duration(3).sleep();

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;
    robot.setVelocity(velocidade);
}
/*
void kineControl::mudar_quadrante(kineControl::robot &robot, std::uint8_t from, std::uint8_t to)
{
    // Change names. Avoid have global variables
    double MAIOR_QUE_VERDE = kineControl::MAIOR_QUE_VERDE;
    double MAIOR_QUE_PRETO = kineControl::MAIOR_QUE_PRETO;
    const double VEL_ANG = kineControl::VEL_ANG;

    double colorBL = robot.get_colorBL();
    double colorBR = robot.get_colorBR();
    double colorFL = robot.get_colorFL();
    double colorFR = robot.get_colorFR();

    projeto_semear::Pose pose_from, pose_to;
    pose_from.location = from;
    pose_from.orientation = 0;
    pose_to.location = to;
    pose_to.orientation = 0;

    // Checa se o parametro quadrante to está correto
    if (from != projeto_semear::Pose::QUADRANTE_ESQUERDO && from != projeto_semear::Pose::QUADRANTE_CENTRAL && from != projeto_semear::Pose::QUADRANTE_DIREITO)
    {
        ROS_ERROR_STREAM("O Quadrante início deve ser o da esquerda (1), ou direito (2), ou centro (0), mas ele eh: " << pose_from);
    }
    if (to != projeto_semear::Pose::QUADRANTE_ESQUERDO && to != projeto_semear::Pose::QUADRANTE_CENTRAL && to != projeto_semear::Pose::QUADRANTE_DIREITO)
    {
        ROS_ERROR_STREAM("O Quadrante alvo deve conectado ao quadrante requisitado. : " << pose_to);
    }

    int direita_ou_esquerda = 0;
    switch (from)
    {
    case (projeto_semear::Pose::QUADRANTE_ESQUERDO):
        direita_ou_esquerda = 1;
        if (to == projeto_semear::Pose::QUADRANTE_ESQUERDO) // caso seja repetido, não há o que enviar
            return;
        break;

    case (projeto_semear::Pose::QUADRANTE_DIREITO):
        direita_ou_esquerda = -1;
        if (to == projeto_semear::Pose::QUADRANTE_DIREITO) // caso seja repetido, não há o que enviar
            return;
        break;

    case (projeto_semear::Pose::QUADRANTE_CENTRAL):
        direita_ou_esquerda = to == projeto_semear::Pose::QUADRANTE_DIREITO ? 1 : -1;
        if (to == projeto_semear::Pose::QUADRANTE_CENTRAL) // caso seja repetido, não há o que enviar
            return;
        break;
    }

    // condição de não alinhamento: o robo deve ter ultrapassado a linha preta
    ros::Duration time(0.05);
    geometry_msgs::Twist velocidade;
    int code = 0;

    ROS_INFO("Alinhando com a linha preta e verde");
    while ((colorBL > MAIOR_QUE_VERDE || colorBR > MAIOR_QUE_VERDE || colorFR > MAIOR_QUE_PRETO || colorFL > MAIOR_QUE_PRETO))
    {

        int code = 0;
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;

        // Caso 0: todos os sensores no branco. Supõe-se que o robô ultrapassou o alinhamento necessário. Garantir isso no resto do código
        // Caso 1: Caso o sensor BackRight esteja marcando verde, mas o BackLeft não -> girar positivo
        // Caso 2: Caso o sensor BackLeft esteja marcando verde, mas o BackRight não -> girar negativo
        // Caso 3: Caso o sensor FrontRight esteja marcando verde, mas o FrontLeft não -> girar positivo
        // Caso 4: Caso o sensor FrontLeft esteja marcando verde, mas o  FrontRight não -> girar negativo
        // ROS_INFO_STREAM("\nFL " << colorFL << "FR " << colorFR << "\nBL " << colorBL << "BR " << colorBR);

        if (colorBL > MAIOR_QUE_VERDE && colorBR > MAIOR_QUE_VERDE && colorFL > MAIOR_QUE_VERDE && colorFR > MAIOR_QUE_VERDE)
            code = 0;
        else if (colorBL > MAIOR_QUE_VERDE && colorBR < MAIOR_QUE_VERDE)
            code = 1;
        else if (colorBL < MAIOR_QUE_VERDE && colorBR > MAIOR_QUE_VERDE)
            code = 2;
        else if (colorFL > MAIOR_QUE_PRETO && colorFR < MAIOR_QUE_PRETO)
            code = 3;
        else if (colorFL < MAIOR_QUE_PRETO && colorFR > MAIOR_QUE_PRETO)
            code = 4;

        //ROS_INFO_STREAM("Case: " << code);

        switch (code)
        {
        case 0:
            velocidade.linear.x = -0.05;
        case 1:
            velocidade.angular.z = -VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
            break;
        case 3:
            velocidade.angular.z = -VEL_ANG;
            break;
        case 4:
            velocidade.angular.z = VEL_ANG;
            break;
        }

        robot.setVelocity(velocidade);
        ros::spinOnce();
        colorBL = robot.colorBL_;
        colorBR = robot.colorBR_;
        colorFL = robot.colorFL_;
        colorFR = robot.colorFR_;
        time.sleep();
    }

    ROS_INFO_STREAM("Transição do quadrante: " << direita_ou_esquerda);
    // Vá para direita até o sensor da direita atingir o fita preta
    velocidade.linear.x = 0;
    velocidade.linear.y = direita_ou_esquerda * 0.1;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);

    // Quinto, andar uma distância predefinida
    ros::Duration(3).sleep();

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    robot.setVelocity(velocidade);
}*/

void kineControl::linha_preta(kineControl::robot &robot)
{
    ROS_INFO("Alinhando com a linha preta (a linha preta esta a frente)");
    double MAIOR_QUE_PRETO = kineControl::MAIOR_QUE_PRETO;
    const double VEL_ANG = kineControl::VEL_ANG;

    ros::Duration time(0.05);
    geometry_msgs::Twist velocidade;
    int code = 0;
    ros::Time begin = ros::Time::now();
    ros::spinOnce();
    while ((robot.colorFR_ != PRETO || robot.colorFL_ != PRETO) && ros::ok())
    {
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;
        //Caso 0: Nenhum dos sensores na faixa pretav -> anda para frente
        //Caso 1: Sensor FrontRight no preto e FrontLeft não -> girar para direita
        //Caso 2: Sensor FrontLeft no preto e FrontRight não -> girar para esquerda
        //ROS_INFO_STREAM("\nFL " << robot.colorFL_ << " FR " << robot.colorFR_);
        if (robot.colorFL_ != PRETO && robot.colorFR_ != PRETO)
            code = 0;
        else if (robot.colorFL_ != PRETO && robot.colorFR_ == PRETO)
            code = 1;
        else if (robot.colorFL_ == PRETO && robot.colorFR_ != PRETO)
            code = 2;
        // ROS_INFO_STREAM("Case: " << code);
        switch (code)
        {
        case 0:
            velocidade.linear.x = 0.05;
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