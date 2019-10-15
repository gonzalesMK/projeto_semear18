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
 * 
 * Contagens por revolução: 8245.92 counts per revolution quando se usa os 2 canais subida e descida. No nosso caso, temos 4122.96 voltas. O diâmetro primitivo é 18mm
 * 
 * Fim de cursos: b[2] & 1 -> botão de baixo e  b[2] & 4 -> botão de cima
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
        ROS_INFO_STREAM("Turning On electromagnet: " << (int)ELETROIMA_ON_CODE);
    }
    else
    {
        write(fd, &ELETROIMA_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off electromagnet: " << (int)ELETROIMA_OFF_CODE);
    }
}

// Send position command to the claw's DC motor
void clawPWM_cb(const std_msgs::Float64ConstPtr &msg)
{
    if (abs(msg->data) < 255)
    {
        clawPWM = (int8_t)(msg->data / 4);
        write(fd, &clawPWM, 1);
        ROS_INFO_STREAM("Moving Gear and Pinion: " << (int)clawPWM);
    }
    else
    {
        ROS_ERROR_STREAM("Not Sending ILEGAL claw PWM. should be between [-255,255], but is: " << msg->data);
    }
}

// Send position command to the Servo motor
void servoPose_cb(const std_msgs::UInt8ConstPtr &msg)
{
    servoPose = (uint8_t)msg->data;

    if (servoPose > 180 or servoPose < 0)
    {

        ROS_ERROR_STREAM("O valor de servoPose deve estar em [0, 180], mas é : " << servoPose);
    }
    write(fd, &SERVO_ON_CODE, 1);
    ROS_INFO_STREAM("Turning On Servo: " << (int) SERVO_ON_CODE);

    ros::Duration(0.005).sleep();

    write(fd, &servoPose, 1);
    ROS_INFO_STREAM("Servo Pose: " << (int) servoPose);
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
    ros::Subscriber subServo = node.subscribe<std_msgs::UInt8>("/claw/servoPose", 1, servoPose_cb);

    ros::Publisher pubLimitSwitchs = node.advertise<std_msgs::UInt8>("/claw/limitSwitchs", 1);
    ros::Publisher pubEncoder = node.advertise<std_msgs::Int64>("/claw/height", 1);
    ros::Publisher pubLineSensors = node.advertise<std_msgs::UInt8>("/pololuSensor", 1);
    ros::Publisher pubContainers = node.advertise<std_msgs::UInt8>("/containerSensor", 1);

    int nread = 0;
    uint8_t b[50];

    std_msgs::UInt8 msg;
    std_msgs::Int64 msg64;
    std_msgs::Bool msg_bool;
    ros::Rate r(1000);

    while (ros::ok())
    {

        nread = read(fd, b, 7);
        if (nread > 0)
        {
            // Line Follower
            msg.data = b[0];
            pubLineSensors.publish(msg);

            // Container
            msg.data = b[1];
            pubContainers.publish(msg);
            
            /**
            for (int i = 0; i < 7; i++)
            {
                ROS_INFO_STREAM("b[" << i << "] = " << (int)b[i]);
            }
            //*/
            
            // Limit Switchs
            msg.data = b[2];
            pubLimitSwitchs.publish(msg);

            // Encoder reading
            msg64.data = -((int64_t)b[3] + (int64_t)b[4] * 256 + (int64_t)b[5] * 65536 + (int64_t)b[6] * 16777216 - (int64_t)2147483648);
            pubEncoder.publish(msg64);
        }

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Closing communication");
    close(fd);
}
