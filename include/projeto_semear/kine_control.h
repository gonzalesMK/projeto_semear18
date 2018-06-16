/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* \mainpage hokuyo_node
 *  \htmlinclude manifest.html
 */

#ifndef KINECONTROL_HH
#define KINECONTROL_HH

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <projeto_semear/Pose.h>
#include <stdlib.h>
//! A namespace containing the hokuyo device driver
namespace kineControl
{

const double MAIOR_QUE_PRETO = 60;  // Constante para marcar o valor do preto
const double MAIOR_QUE_VERDE = 300; // Constate para marcar o valor do verde
const double MENOR_QUE_VERDE = 100; //  ""  ""
const double VEL_ANG = 0.1;         // Constante para marcar a velocidade angular

const double PI = 3.141592653589793238463;

enum wheel
{
  BR,
  BL,
  FR,
  FL
};

class robot
{

protected:
  ros::NodeHandle nh_;

public:
  float colorFL_;
  float colorBL_;
  float colorFR_;
  float colorBR_;

  ros::Publisher FR_Motor_;
  ros::Publisher FL_Motor_;
  ros::Publisher BR_Motor_;
  ros::Publisher BL_Motor_;

  ros::Subscriber lineSensorFL_;
  ros::Subscriber lineSensorFR_;
  ros::Subscriber lineSensorBL_;
  ros::Subscriber lineSensorBR_;

  robot();

  //! Set the velocity of the robot.
  bool setVelocity(const geometry_msgs::Twist &vel);

  // Quando o robô gira em torno de uma de suas rodas - Ainda não funciona
  bool concerning(const wheel w, double modulo_vel);

  // Encapsula o ros::spinOnce para as variáveis do robô
  void update(double periodo = 0)
  {
    ros::spinOnce();
    if(periodo != 0) ros::Duration(periodo).sleep();
  }

  float get_colorFL();
  float get_colorBL();
  float get_colorFR();
  float get_colorBR();
};

void mudar_quadrante(kineControl::robot &robot, std::uint8_t from, std::uint8_t to);

} // namespace kineControl

#endif