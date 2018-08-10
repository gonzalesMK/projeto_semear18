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

namespace kineControl
{

const double MAIOR_QUE_PRETO = 60;  // Constante para marcar o valor do preto
const double MAIOR_QUE_VERDE = 300; // Constate para marcar o valor do verde
const double VEL_ANG = 0.1;         // Constante para marcar a velocidade angular

const double PI = 3.141592653589793238463;

enum wheel
{
    BR,
    BL,
    FR,
    FL
};

enum color
{
    PRETO,
    AZUL_VERDE,
    BRANCO,
    UNKNOWN
};

class robot
{

protected:
  ros::NodeHandle nh_;

public:
  color colorFL_= BRANCO;
  color colorBL_= BRANCO ;
  color colorFR_= BRANCO ;
  color colorBR_= BRANCO ;

  ros::Publisher FR_Motor_;
  ros::Publisher FL_Motor_;
  ros::Publisher BR_Motor_;
  ros::Publisher BL_Motor_;

  ros::Subscriber lineSensorFL_;
  ros::Subscriber lineSensorFR_;
  ros::Subscriber lineSensorBL_;
  ros::Subscriber lineSensorBR_;

  public:
    color colorFL_ = BRANCO;
    color colorBL_ = BRANCO;
    color colorFR_ = BRANCO;
    color colorBR_ = BRANCO;

  // Encapsula o ros::spinOnce e ros::Duration
  void update(double periodo = 0)
  {
    ros::spinOnce();
    if(periodo != 0) ros::Duration(periodo).sleep();
  }

    ros::Subscriber lineSensorFL_;
    ros::Subscriber lineSensorFR_;
    ros::Subscriber lineSensorBL_;
    ros::Subscriber lineSensorBR_;

    robot();

    //! Set the velocity of the robot.
    bool setVelocity(const geometry_msgs::Twist &vel);

    // Quando o robô gira em torno de uma de suas rodas - Ainda não funciona
    bool concerning(const wheel w, double modulo_vel);

    // Encapsula o ros::spinOnce e ros::Duration
    void update(double periodo = 0)
    {
        ros::spinOnce();
        if (periodo != 0)
            ros::Duration(periodo).sleep();
    }

    // Funções que chama o ros::spinOnce e devolve o valor da variável
    color get_colorFL();
    color get_colorBL();
    color get_colorFR();
    color get_colorBR();
};

// Função para mudar o quadrante do robô - Deve funcionar como seguidor de linha
// Apenas util para os 3 quadrantes.
// MUDAS APENAS 1 QUADRANTE POR VEZ !! SE MANDAR DIREITA E ESQUERDA, VAI PARAR NO CENTRO.
// void mudar_quadrante(kineControl::robot &robot, std::uint8_t from, std::uint8_t to);
void linha_preta(kineControl::robot &robot);
void esquerda(kineControl::robot &robot);
void direita(kineControl::robot &robot);
void alinhar(kineControl::robot &robot);
void ir_doca(kineControl::robot &robot);
void ir_quadrante(kineControl::robot &robot);
void alinhar_doca(kineControl::robot &robot);
void descobrir_cor(kineControl::robot &robot);

} // namespace kineControl

// Overload of << for the Pose
std::ostream &operator<<(std::ostream &os, const projeto_semear::Pose &pose)
{
    os << "\t(";
    switch (pose.location)
    {
    case 0:
        os << "QUAD_CENTRAL";
        break;
    case 1:
        os << "QUAD_ESQ";
        break;
    case 2:
        os << "QUAD_DIR";
        break;
    case 3:
        os << "DOCA_VERDE";
        break;
    case 4:
        os << "DOCA_AZUL";
        break;
    case 5:
        os << "DOCA_CENTRAL";
        break;
    case 6:
        os << "TREM  ";
        break;
    case 255:
        os << "ERROR";
        break;
    default:
        os << "UKNOW";
    }

    os << ",\t ";
    switch (pose.orientation)
    {
    case 0:
        os << "ORIENTATION_TREM";
        break;
    case 1:
        os << "ORIENTATION_INICIO";
        break;
    case 2:
        os << "ORIENTATION_AZUL";
        break;
    case 3:
        os << "ORIENTATION_VERDE";
        break;
    case 255:
        os << "ERROR";
        break;
    default:
        os << "UNKOW";
    }

    os << ")";
    return os;
}

#endif