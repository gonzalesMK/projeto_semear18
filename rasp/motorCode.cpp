#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "projeto_semear/arduino_interface.h"

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

char vel[4];
char stop[4] = {0, 0, 0, 0};

bool turnOnBaseMotor = false;

void motor_cb(const std_msgs::Float64ConstPtr &msg, char &var)
{

    var = (char)msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MotorArduinoInterface");
    ros::NodeHandle node;

    // Create 4 topics to receive motor speed
    ros::Subscriber motor_sub1 = node.subscribe<std_msgs::Float64>("/motorFR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[0])));
    ros::Subscriber motor_sub2 = node.subscribe<std_msgs::Float64>("/motorFL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[1])));
    ros::Subscriber motor_sub3 = node.subscribe<std_msgs::Float64>("/motorBR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[2])));
    ros::Subscriber motor_sub4 = node.subscribe<std_msgs::Float64>("/motorBL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[3])));

    // Open arduino
    char str[] = "/dev/ttyUSB1";

    Arduino::Arduino arduino(str);

    ros::Rate rate(50);

    while (ros::ok())
    {

        ros::spinOnce();

        write(arduino.fd, vel, 4);

        rate.sleep();
    }

    ROS_INFO("Closing communication - Arduino Motor");
    close(arduino.fd);
}
