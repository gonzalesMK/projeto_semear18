#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>   
#include <robot_strategy/motorControlLib.h>

/* Digitar 'i' e ',' para subir e descer e 'j' e 'l' para esquerda e direita

*/

const double PI = 3.141592653589793238463;
const double DIAMETRO = 0.099060; // Diametro da RODA
const double LX = 0.06099;        // Comprimento do eixo X
const double LY = 0.0991225;      // Comprimento do eixo Y
const double LDIAG = 0.116383204; // Comprimento da diagonal do robõ  = sqrt(LX * LX + LY * LY)




// Turn on/Off the infrared sensors in the base
void keyboard_cb(const geometry_msgs::TwistConstPtr &vel, MotorControl& motor)
{
    // Ângulo XY do vetor velocidade
    double theta = atan2(vel->linear.y, vel->linear.x);

    // Módulo do vetor velocidade em XY
    double w_module = sqrt(pow(vel->linear.x, 2) + pow(vel->linear.y, 2)) / DIAMETRO;
    double conversao_angular = (LDIAG) / (DIAMETRO); // Converte o Rad/s em metros/s depois em Rad/s para as rodas

    // Modelo omnidirecional da base
    double W[4];
    
    W[Wheels::wFL]= ((w_module)*cos(PI / 4 - theta) - (vel->angular.z)) * 20;
    W[Wheels::wFR]= ((w_module)*cos(PI / 4 + theta) + (vel->angular.z)) * 20;
    W[Wheels::wBL]= ((w_module)*cos(PI / 4 + theta) - (vel->angular.z)) * 20;
    W[Wheels::wBR]= ((w_module)*cos(PI / 4 - theta) + (vel->angular.z)) * 20;

    // ROS_INFO_STREAM("Sending Vel: \nFL: "<< W[Wheels::FL] << " \tFR: "<< W[Wheels::FR] << " \nBL: "<< W[Wheels::BL] << " \tBR: "<< W[Wheels::BR] );
    motor.align(W);
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarBase");
    ros::NodeHandle node;
    
    MotorControl motor(100, 1, 0, 0, 0, 0);

    ros::Subscriber subKeyboard = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind( keyboard_cb, _1, boost::ref(motor) ));

    ros::spin();
    
    return 0;	
}
