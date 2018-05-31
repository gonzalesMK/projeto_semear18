#include "projeto_semear/kine_control.h"

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)

/** Implementação do Objeto que abstrai o motor. **/


kineControl::motorControl::motorControl()
{
    FR_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorFRSpeed", 1);
    FL_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorFLSpeed", 1);
    BR_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorBRSpeed", 1);
    BL_Motor_ = nh_.advertise<std_msgs::Float32>("/AMR/motorBLSpeed", 1);

    // Necessário um tempo para inicializar os nós
    ros::Duration(0.1).sleep();
}


bool kineControl::motorControl::setVelocity(const geometry_msgs::Twist &vel)
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
    Wfl.data = ((w_module)*sin(PI / 4 + theta) - (vel.angular.z) * conversao_angular ) * PI; // !!!
    Wfr.data = ((w_module)*cos(PI / 4 + theta) + (vel.angular.z) * conversao_angular ) * PI; // !!!
    Wbl.data = ((w_module)*cos(PI / 4 + theta) - (vel.angular.z) * conversao_angular ) * PI; // !!!
    Wbr.data = ((w_module)*sin(PI / 4 + theta) + (vel.angular.z) * conversao_angular ) * PI; // !!!

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);
}


// Concerning is not properly working yet, need to improve the math
// Concerning é quando o robô gira em torno de uma das suas rodas.
bool kineControl::motorControl::concerning(const wheel w, double modulo_vel)
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