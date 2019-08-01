#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include "projeto_semear/arduino_interface.h"

#include <sstream>
#include <sys/ioctl.h>

#include <poll.h> // Poll funcionallity

#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

// #include <termio.h>

/* Interface ROS -> Arduino 
 *
 * This code receives the PWM command from the topic <motor>/pwm and sends it to the arduino. Check the arduino code for more details on this.
 * 
 * The arduino in the Linux system is recognized as a TTY file. 
 * 
 * The arduino Receives one char in the range [-127, 127] for each motor
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
int8_t clawPose = 0;
bool turnOnClaw = false;
bool setLineFollower = false;
bool setContainer = false;
uint8_t servoPose = 0;

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
/* Position of the gear and pinion
*
*   One of {0,1,2,3,4,5}
*/

void clawPose_cb(const std_msgs::Float64ConstPtr &msg)
{
    if (turnOnClaw)
    {
        clawPose = (int8_t)msg->data;
        if (clawPose > -64 && clawPose < 64)
        {
            write(fd, &clawPose, 1);
            ROS_INFO_STREAM("Moving Gear and Pinion: " << (int) clawPose);
        }
        else
        {
            ROS_ERROR_STREAM("Not Sending ILEGAL claw position. should be between [-63,63], but is: " << (int) clawPose);
        }
    }
    else
    {
        ROS_INFO_STREAM("Not moving Gear and Pinion: " << (int) clawPose);
    }
}

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

void servoPose_cb(const std_msgs::Float64ConstPtr &msg)
{
    servoPose = (uint8_t)msg->data;

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
    Arduino::Arduino arduino(str);
    fd = arduino.fd;

    // Create 4 topics to receive motor speed
    ros::Subscriber subSetElectro = node.subscribe<std_msgs::Bool>("/turnOnElectromagnet", 1, setElectromagnet_cb);
    ros::Subscriber subClaw = node.subscribe<std_msgs::Float64>("/setClawVel", 1, clawPose_cb);
    ros::Subscriber subTurnOnClaw = node.subscribe<std_msgs::Bool>("/turnOnClaw", 1, turnOnClaw_cb);
    ros::Subscriber subLine = node.subscribe<std_msgs::Bool>("/turnOnPololuSensors", 1, setLineFollower_cb);
    ros::Subscriber subContainer = node.subscribe<std_msgs::Bool>("/turnOnContainerSensors", 1, setContainer_cb);
    ros::Subscriber subServo = node.subscribe<std_msgs::Float64>("/setServoPose", 1, servoPose_cb);

    // Publisher for the information acquired from the arduino
    ros::Publisher pubEncoder = node.advertise<std_msgs::Int64>("/clawEncoder", 1);
    ros::Publisher pubLineSensors = node.advertise<std_msgs::UInt8>("/pololuSensor", 1);
    ros::Publisher pubContainers = node.advertise<std_msgs::UInt8>("/containerSensor", 1);

    struct pollfd arduino_fds[1];
    arduino_fds[0].fd = fd;
    arduino_fds[0].events = POLLIN;

    int timeout = 0; // ms
    int nread = 0;
    char b[50];

    //ros::Duration dur2(0.1);
    //int tmp = 1;
    std_msgs::UInt8 msg;
    std_msgs::Int64 msg64;
    // std::ostringstream oss;
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
                        ROS_INFO_STREAM("DEBUG STRING " << s);
                        msg64.data = std::stoi(s);
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
