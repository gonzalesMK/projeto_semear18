#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <projeto_semear/Vel.h>
#include <projeto_semear/Infra_Placa_Elevadores.h>
#include <projeto_semear/Enable_Placa_Elevadores.h>
#include <std_msgs/Bool.h>

projeto_semear::Vel vel_msg;

/* Subscribers e Publishers para os PIDs */
ros::Publisher stateFR_pub;
ros::Subscriber controlFR_sub;
ros::Publisher stateBR_pub;
ros::Subscriber controlBR_sub;
ros::Publisher stateBL_pub;
ros::Subscriber controlBL_sub;
ros::Publisher stateFL_pub;
ros::Subscriber controlFL_sub;

/** Esse código é responsável pela interface Arduino <-> ROS
 * Por isso, ele vai ficar responsável pelas 3 placas:
 *  Placa dos motores:
 *      4 motores com encoder  <-> PID
 
 * Placa do elevador:
 *      2 motores com encoder <-> PID
 *      4 Infravermelhos <-> Leitura
 *      1 RGB  <-> Leitura
 *      2 eletroima (rele)  <->  Enable
*/

/* Faz a interface entre o arduino -> PID  para os estados da seguinte maneira:
 *  Cada nó de PID recebe o estado do motor (velocidade angular) em seu respectivo tópico: /AMR/<motor>_PID/state
 *  O arduino publica os estados dos 4 motores em 1 tópico apenas
 */

void state_callback(const projeto_semear::VelConstPtr &msg)
{
    std_msgs::Float64 goal;

    // Filling goal here
    goal.data = (double) msg->wFR;
    stateFR_pub.publish(goal);

    goal.data = (double) msg->wBR;
    stateBR_pub.publish(goal);

    goal.data = (double) msg->wFL;
    stateFL_pub.publish(goal);

    goal.data = (double) msg->wBL;
    stateBL_pub.publish(goal);
}

/* As 4 funções abaixo fazem a interface PIDs -> Arduino  para o controle:
 * Cada um dos 4 nós publica o controle a ser efetivado, enquanto o arduino recebe em 1 tópico. Assim, cada função coloca o valor
 * na mensagem, que é enviada a 20Hz.
 */
void controlFR_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.wFR = (int16_t) msg->data;
}
void controlFL_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.wFL = (int16_t) msg->data;
}
void controlBL_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.wBL = (int16_t) msg->data;
}
void controlBR_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.wBR = (int16_t) msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    vel_msg.wFR = 0;
    vel_msg.wBL = 0;
    vel_msg.wBL = 0;
    vel_msg.wBR = 0;

    /* Os seguintes tópicos são dos PIDs */
    stateFR_pub = nh.advertise<std_msgs::Float64>("AMR/FR_PID/state", 1);
    controlFR_sub = nh.subscribe<std_msgs::Float64>("AMR/FR_PID/control_effort", 1, controlFR_callback);
    stateBR_pub = nh.advertise<std_msgs::Float64>("AMR/BR_PID/state", 1);
    controlBR_sub = nh.subscribe<std_msgs::Float64>("AMR/BR_PID/control_effort", 1, controlBR_callback);
    stateBL_pub = nh.advertise<std_msgs::Float64>("AMR/BL_PID/state", 1);
    controlBL_sub = nh.subscribe<std_msgs::Float64>("AMR/BL_PID/control_effort", 1, controlBL_callback);
    stateFL_pub = nh.advertise<std_msgs::Float64>("AMR/FL_PID/state", 1);
    controlFL_sub = nh.subscribe<std_msgs::Float64>("AMR/FL_PID/control_effort", 1, controlFL_callback);
    ros::Publisher vel_pub = nh.advertise<projeto_semear::Vel>("/AMR/InputVelBase", 1);

    
    double FREQUENCIA_ARDUINO = 20;
    if (!nh.param("FREQUENCIA_ARDUINO_MOTORES", FREQUENCIA_ARDUINO, 20.0))
    {
        ROS_ERROR("Failed to get param 'FREQUENCIA_ARDUINO_MOTORES'");
    }
    ros::Rate rate(FREQUENCIA_ARDUINO);

    while (ros::ok())
    {
        ros::spinOnce();
        vel_pub.publish(vel_msg);

        rate.sleep();
    }
}
