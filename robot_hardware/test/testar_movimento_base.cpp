#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>

/* Digitar 'i' e ',' para subir e descer e 'j' e 'l' para esquerda e direita

*/

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)

ros::Publisher pubMotorFR;
ros::Publisher pubMotorFL;
ros::Publisher pubMotorBR;
ros::Publisher pubMotorBL;

// Turn on/Off the infrared sensors in the base
void keyboard_cb(const geometry_msgs::TwistConstPtr &vel)
{
    // Ângulo XY do vetor velocidade
    double theta = atan2(vel->linear.y, vel->linear.x);

    // Módulo do vetor velocidade em XY
    double w_module = sqrt(pow(vel->linear.x, 2) + pow(vel->linear.y, 2)) / DIAMETRO;
    double conversao_angular = (LDIAG) / (DIAMETRO); // Converte o Rad/s em metros/s depois em Rad/s para as rodas

    std_msgs::Float64 Wfl;
    std_msgs::Float64 Wfr;
    std_msgs::Float64 Wbl;
    std_msgs::Float64 Wbr;

    // Modelo omnidirecional da base
    Wfl.data = ((w_module)*sin(PI / 4 + theta) + (vel->angular.z)) * 10;
    Wfr.data = ((w_module)*cos(PI / 4 - theta) - (vel->angular.z)) * 10;
    Wbl.data = ((w_module)*cos(PI / 4 + theta) + (vel->angular.z)) * 10;
    Wbr.data = ((w_module)*sin(PI / 4 - theta) - (vel->angular.z)) * 10;

    // Publicação para o motor
    pubMotorFR.publish(Wfr);
    pubMotorFL.publish(Wfl);
    pubMotorBR.publish(Wbr);
    pubMotorBL.publish(Wbl);

    // ROS_INFO_STREAM(" Wfr: " << Wfr <<" Wfl: " << Wfl <<" Wbr: "<< Wbr << " Wbl: "<< Wbl);
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarBase");
    ros::NodeHandle node;
    
    pubMotorFR = node.advertise<std_msgs::Float64>("/motorFR/pwm", 1);
    pubMotorFL = node.advertise<std_msgs::Float64>("/motorFL/pwm", 1);
    pubMotorBR = node.advertise<std_msgs::Float64>("/motorBR/pwm", 1);
    pubMotorBL = node.advertise<std_msgs::Float64>("/motorBL/pwm", 1);

    ros::Subscriber subKeyboard = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, keyboard_cb );

    ros::spin();
    
    return 0;	
}
