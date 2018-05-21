#include "projeto_semear/kine_control.h"

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
    double vel_module = sqrt(pow(vel.linear.x, 2) + pow(vel.linear.y, 2));

    std_msgs::Float32 Wfl;
    std_msgs::Float32 Wfr;
    std_msgs::Float32 Wbl;
    std_msgs::Float32 Wbr;

    // Modelo omnidirecional da base
    Wfl.data = (vel_module)*sin(PI / 4 + theta) + (vel.angular.z); // !!!
    Wfr.data = (vel_module)*cos(PI / 4 + theta) - (vel.angular.z); // !!!     FALTA INCLUIR O EFEITO DO RAIO DA RODAS NA VELOCIDADE ANGULAR
    Wbl.data = (vel_module)*cos(PI / 4 + theta) + (vel.angular.z); // !!!
    Wbr.data = (vel_module)*sin(PI / 4 + theta) - (vel.angular.z); // !!!

    // Publicação para o motor
    FR_Motor_.publish(Wfr);
    FL_Motor_.publish(Wfl);
    BR_Motor_.publish(Wbr);
    BL_Motor_.publish(Wbl);

    ros::Duration(1).sleep();
}
