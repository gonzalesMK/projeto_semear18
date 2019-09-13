#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include "robot_hardware/arduinoInterfaceLib.h"

#include <sstream>
#include <sys/ioctl.h>

#include <poll.h> // Poll funcionallity

#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

// #include <termio.h>


/* Esse código é responsável pelos seguintes sensores/atuadores:
 *
 * A) Motor DC da garra: 
 *    1 Encoder com canais A e B
 *    2 Inputs da Ponte H para controlar o sentido
 *    1 Enable para controlar PWM
 *    2 Fim de cursos
 *  -> Enviar os valores {-62, 61, ..., 0, ..., 61, 62} para controlar o PWM (que varia de {-255, 255})
 * 
 * B) Eletroima:
 *    1 Sinal de controle (para o BJT)
 *
 * C) 6 Sensores Pololu
 *    2 Para cada um dos 4 lados do robô
 *
 * D) 2 sensores Digitais de Infravermelho
 *    Para detectar os containers
 *
 * E) 1 Servo motor para controlar a rotação da garra
 *     -> Enviar um inteiro entre 0 e 180 para controlar o ângulo
 *
 * Para controlar o arduino, é necessário utilizar comunicação serial.
 *
 * Envie um INT com o seguinte valor para conseguir a funcionalidade:
 *  64: Ligar o Eletroima
 *  65: Desliga Eletroima
 *  66: Liga controle do servo. 
 *    -> Sequentemente, você precisa enviar a posição do servo
 *  67: Ligar o feedback do encoder, fim de curso e motor da garra. 
 *  68: Desligar o feedback do encoder, fim de curso e o motor da garra
 *  69: Ligar os sensores de linha
 *  70: Desligar sensores de linha
 *  71: Ligar os sensores de containers
 *  72: Desligar os sensores de containers
 *
 *  Exemplo:   
 *      Caso o motor tenha sido ativado (enviando previamente 67), o PWM e o sentido podem ser controlados enviando valores entre {-62, ..., 62}
 *   
 * O feedback dos motores são 2 Serial.write. Primeiro, envia-se um LONG INT com os ticks do motor e, posteriormente, um BOOL para avisar se algum fim de curso foi ativado.
 *
 * O feedback dos sensores são os 6 sensores da pololu seguido pelos 2 sensores digitais
 */

const uint8_t ELETROIMA_ON_CODE = 64;
const uint8_t ELETROIMA_OFF_CODE = 65;
const uint8_t SERVO_ON_CODE = 66;
const uint8_t CLAW_ON_CODE = 67;
const uint8_t CLAW_OFF_CODE = 68;
const uint8_t LINESENSOR_ON_CODE = 69;
const uint8_t LINESENSOR_OFF_CODE = 70;
const uint8_t CONTAINERSENSOR_ON_CODE = 71;
const uint8_t CONTAINERSENSOR_OFF_CODE = 72;

int fd;

bool setElectromagnet = false;
int8_t clawPWM = 0;
bool turnOnClaw = false;
bool setLineFollower = false;
bool setContainer = false;
uint8_t servoPose = 0;

// Turn On/Off the claw's electromagnet
void setElectromagnet_cb(const std_msgs::BoolConstPtr &msg)
{
    setElectromagnet = msg->data;

    if (setElectromagnet)
    {
        write(fd, &ELETROIMA_ON_CODE, 1);
        ROS_INFO_STREAM("Turning On electromagnet: " << ELETROIMA_ON_CODE);
    }
    else
    {
        write(fd, &ELETROIMA_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off electromagnet: " << ELETROIMA_OFF_CODE);
    }
}

// Send position command to the claw's DC motor  
void clawPWM_cb(const std_msgs::Float64ConstPtr &msg)
{
    if (turnOnClaw)
    {

        if (abs(msg->data) < 255)
        {
	    clawPWM = (int8_t)msg->data/4;            
            write(fd, &clawPWM, 1);
            ROS_INFO_STREAM("Moving Gear and Pinion: " << (int) clawPWM);
        }
        else
        {
            ROS_ERROR_STREAM("Not Sending ILEGAL claw PWM. should be between [-255,255], but is: " << msg->data);
        }
    }
    else
    {
        ROS_INFO_STREAM("Not moving Gear and Pinion: " << (int) clawPWM);
    }
}

// Turn On/Off the claw's DC motor interface
void turnOnClaw_cb(const std_msgs::BoolConstPtr &msg)
{
    turnOnClaw = msg->data;
    if (turnOnClaw)
    {
        write(fd, &CLAW_ON_CODE, 1);
        ROS_INFO_STREAM("Turning On Gear and Pinion: " << (int) CLAW_ON_CODE);
    }
    else
    {
        write(fd, &CLAW_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off Gear and Pinion: " << CLAW_OFF_CODE);
    }
}

// Turn on/Off the infrared sensors in the base
void setLineFollower_cb(const std_msgs::BoolConstPtr &msg)
{
    setLineFollower = msg->data;

    if (setLineFollower)
    {
        write(fd, &LINESENSOR_ON_CODE, 1);
        ROS_INFO_STREAM("Turning On setLineFollower: " << LINESENSOR_ON_CODE);
    }
    else
    {
        write(fd, &LINESENSOR_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off setLineFollower: " << LINESENSOR_OFF_CODE);
    }
}

// Turn on/Off the infrared sensors in the container
void setContainer_cb(const std_msgs::BoolConstPtr &msg)
{
    setContainer = msg->data;

    if (setContainer)
    {
        write(fd, &CONTAINERSENSOR_ON_CODE, 1);
        ROS_INFO_STREAM("Turning On setContainer: " << CONTAINERSENSOR_ON_CODE);
    }
    else
    {
        write(fd, &CONTAINERSENSOR_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off setContainer: " << CONTAINERSENSOR_OFF_CODE);
    }
}

// Send position command to the Servo motor  
void servoPose_cb(const std_msgs::UInt8ConstPtr &msg)
{
    servoPose = (uint8_t) msg->data;

    if( servoPose > 180 or servoPose < 0){

	ROS_ERROR_STREAM("O valor de servoPose deve estar em [0, 180], mas é : " << servoPose);
    }
    write(fd, &SERVO_ON_CODE, 1);
    ROS_INFO_STREAM("Turning On Servo: " << SERVO_ON_CODE);

    ros::Duration(0.005).sleep();

    write(fd, &servoPose, 1);
    ROS_INFO_STREAM("Servo Pose: " << servoPose);
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "SensorArduinoInterface");
    ros::NodeHandle node;

    char str[] = "/dev/ttyUSB0";
    Arduino arduino(str);
    fd = arduino.fd;

    ros::Subscriber subSetElectro = node.subscribe<std_msgs::Bool>("/claw/enableElectromagnet", 1, setElectromagnet_cb);
    ros::Subscriber subClaw = node.subscribe<std_msgs::Float64>("/claw/pwm", 1, clawPWM_cb);
    ros::Subscriber subTurnOnClaw = node.subscribe<std_msgs::Bool>("/claw/enableFB", 1, turnOnClaw_cb);
    ros::Subscriber subServo = node.subscribe<std_msgs::UInt8>("/claw/servoPose", 1, servoPose_cb);
    
    ros::Publisher pubEncoder = node.advertise<std_msgs::Int64>("/claw/height", 1);
    
    ros::Subscriber subLine = node.subscribe<std_msgs::Bool>("/turnOnPololuSensors", 1, setLineFollower_cb);
    ros::Subscriber subContainer = node.subscribe<std_msgs::Bool>("/turnOnContainerSensors", 1, setContainer_cb);

    ros::Publisher pubLineSensors = node.advertise<std_msgs::UInt8>("/pololuSensor", 1);
    ros::Publisher pubContainers = node.advertise<std_msgs::UInt8>("/containerSensor", 1);

    struct pollfd arduino_fds[1];
    arduino_fds[0].fd = fd;
    arduino_fds[0].events = POLLIN;

    int timeout = 0; // ms
    int nread = 0;
    char b[50];

    std_msgs::UInt8 msg;
    std_msgs::Int64 msg64;
    while (ros::ok())
    {

        int tmp = poll(arduino_fds, 1, timeout);

        if (tmp > 0)
        {
            if (arduino_fds[0].revents & POLLIN)
            {
                nread = read(arduino_fds[0].fd, b, 32);

                if (nread > 0)
                {
                    int i = 0;

                   // ROS_INFO_STREAM("Buffer " << nread << ": " << (int)b[0] << ":" << (int)b[1] << ":" << (int)b[2] << ":" << (int)b[3] << ":" << (int)b[4] << ":" << (int)b[5] << ":" << (int)b[6] << ":");
                    if (setLineFollower)
                    {
                        msg.data = b[i++];
                        pubLineSensors.publish(msg);
                    }

                    if (setContainer)
                    {
                        msg.data = b[i++];
                        pubContainers.publish(msg);
                    }

                    if (turnOnClaw)
                    {
                        msg.data = b[i++];

                        std::string s(&b[i], nread - i);
                        //ROS_INFO_STREAM("DEBUG STRING " << s);
                        msg64.data = std::stoi(s) ;
                        pubEncoder.publish(msg64);
                    }
                }
            }
        }

        ros::spinOnce();
    }

    ROS_INFO("Closing communication");
    close(fd);
}
