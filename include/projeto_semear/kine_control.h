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
#include <std_msgs/Float64.h>
#include <projeto_semear/Pose.h>
#include <stdlib.h>

namespace kineControl
{

int MAIOR_QUE_PRETO;             // Constante para marcar o valor do preto
int MAIOR_QUE_VERDE;             // Constate para marcar o valor do verde
double TEMPO_MEIA_VOLTA;            // Tempo para o robô girar 90º Graus
double TEMPO_ALINHAR_ESQUERDA;      // Depois que o robô alinha com a esquerda, ele anda uma distância predefinida por esse tempo
double TEMPO_TRANSICAO_ESQUERDA;    // Tempo para andar durante a transição esquerda-direita
double PRECISAO_DIST_ALINHAR_PILHA; // Precisão quando alinha lateralmente com os containers
double TEMPO_DIREITA;
double VEL_Y;
double VEL_Z;
double VEL_X;
double Kp; // Constante para controle Proporcional
const double VEL_ANG = 0.1; // Constante para marcar a velocidade angular
double FREQUENCIA_PARA_ALINHAR;
double LDIAG;
double DIAMETRO;
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
    color colorE0_ = BRANCO;
    color colorE1_ = BRANCO;
    color colorE2_ = BRANCO;
    color colorE3_ = BRANCO;

    color colorD0_ = BRANCO;
    color colorD1_ = BRANCO;
    color colorD2_ = BRANCO;
    color colorD3_ = BRANCO;

    color colorFE_ = BRANCO;
    color colorFD_ = BRANCO;

    color colorR0_ = BRANCO;
    color colorR1_ = BRANCO;
    color colorR2_ = BRANCO;
    color colorR3_ = BRANCO;
    color colorL0_ = BRANCO;
    color colorL1_ = BRANCO;
    color colorL2_ = BRANCO;
    color colorL3_ = BRANCO;

    double lateral_distance_ = 1;

    ros::Publisher FR_Motor_;
    ros::Publisher FL_Motor_;
    ros::Publisher BR_Motor_;
    ros::Publisher BL_Motor_;

    ros::Subscriber lineSensorE0_;
    ros::Subscriber lineSensorE1_;
    ros::Subscriber lineSensorE2_;
    ros::Subscriber lineSensorE3_;

    ros::Subscriber lineSensorD0_;
    ros::Subscriber lineSensorD1_;
    ros::Subscriber lineSensorD2_;
    ros::Subscriber lineSensorD3_;

    ros::Subscriber lateralSensor_;

    ros::Subscriber ColorSensorR0_;
    ros::Subscriber ColorSensorR1_;
    ros::Subscriber ColorSensorR2_;
    ros::Subscriber ColorSensorR3_;
    ros::Subscriber ColorSensorL0_;
    ros::Subscriber ColorSensorL1_;
    ros::Subscriber ColorSensorL2_;
    ros::Subscriber ColorSensorL3_;

    ros::Subscriber frontalSensorEsq_;
    ros::Subscriber frontalSensorDir_;

    robot();

    //! Set the velocity of the robot.
    bool setVelocity(const geometry_msgs::Twist &vel);
    bool setVelocityPID(float velL, float velR, geometry_msgs::Twist* vel = 0 );
    // Quando o robô gira em torno de uma de suas rodas - Ainda não funciona
    bool concerning(const wheel w, double modulo_vel);

    // Encapsula o ros::spinOnce e ros::Duration
    void update(double periodo = 0)
    {
        ros::spinOnce();
        if (periodo != 0)
            ros::Duration(periodo).sleep();
    }

};

// Função para mudar o quadrante do robô - Deve funcionar como seguidor de linha
// Apenas util para os 3 quadrantes.
// MUDAS APENAS 1 QUADRANTE POR VEZ !! SE MANDAR DIREITA E ESQUERDA, VAI PARAR NO CENTRO.
// void mudar_quadrante(kineControl::robot &robot, std::uint8_t from, std::uint8_t to);
void linha_preta(kineControl::robot &robot);
void esquerda(kineControl::robot &robot);
void direita(kineControl::robot &robot);
void alinhar_atras(kineControl::robot &robot);
void alinhar_frente(kineControl::robot &robot, int initial_erro = -5);
void ir_doca(kineControl::robot &robot);
void ir_quadrante(kineControl::robot &robot);
void alinhar_doca(kineControl::robot &robot);
void alinhar_containerdepositado(kineControl::robot &robot);
void descobrir_cor(kineControl::robot &robot);
void pegar_container(kineControl::robot &robot, char lado_escolhido);
void alinhar_pilha(kineControl::robot &robot, int dir, bool container_esq_esta_vazio = false);
void alinhar_esquerda(kineControl::robot &robot, int dir_esq = 1);
void alinhar_depositar_esquerda(kineControl::robot &robot);
void alinhar_direita(kineControl::robot &robot);
void alinhar_adiantado(kineControl::robot &robot);

int erro_sensores_esquerda_com_preto(kineControl::robot &robot, int temp_erro = 0);
int erro_sensores_direita_com_preto(kineControl::robot &robot, int temp_erro = 0);

int erro_sensores_D0D1(kineControl::robot &robot, int temp_erro);
int erro_sensores_D2D3(kineControl::robot &robot, int temp_erro);
int erro_sensores_E0E1(kineControl::robot &robot, int temp_erro);
int erro_sensores_E2E3(kineControl::robot &robot, int temp_erro);
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