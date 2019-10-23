#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "robot_hardware/arduinoInterfaceLib.h"

#include <boost/bind.hpp>

// #include <termio.h>

/* Interface ROS -> Arduino 
 *
 * This code receives the PWM command from the topic <motor>/pwm and sends it to the arduino. Check the arduino code for more details on this.
 * 
 * The arduino in the Linux system is recognized as a TTY file. 
 * 
 * The arduino Receives one char in the range [-120, 120] for each motor
 */

/*
class Wheels(IntEnum):
    FL = 0
    FR = 1
    BL = 2
    BR = 3

    def __int__(self):
        return self.value
*/
#define FL 3
#define FR 0 
#define BL 1 
#define BR 2  

char vel[4];
char stop[4] = {0, 0, 0, 0};

bool turnOnBaseMotor = false;

void motor_cb(const std_msgs::Float64ConstPtr &msg, char &var)
{

    double temp = abs( msg->data ) > 120 ? 120 * ( msg->data / fabs( msg->data ) ) : msg->data ;  

    var = (int8_t) ( (int16_t) temp);
   // ROS_INFO_STREAM("Received: " << msg->data << " " << ( msg->data / fabs( msg->data )) << " Now: " << (int) var);
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MotorArduinoInterface");
    ros::NodeHandle node;

    // Create 4 topics to receive motor speed
    ros::Subscriber motor_sub1 = node.subscribe<std_msgs::Float64>("/motorFR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[FR])));
    ros::Subscriber motor_sub2 = node.subscribe<std_msgs::Float64>("/motorFL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[FL])));
    ros::Subscriber motor_sub3 = node.subscribe<std_msgs::Float64>("/motorBR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[BR])));
    ros::Subscriber motor_sub4 = node.subscribe<std_msgs::Float64>("/motorBL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[BL])));

    // Open arduino
    char str[] = "/dev/ttyUSB1";
    Arduino arduino(str);

    ros::Rate rate(50);

    while (ros::ok())
    {

        ros::spinOnce();

        // ROS_INFO_STREAM("FL: " << (int) vel[0] << "FR: " << (int) vel[1] << " " << (int) vel[2] << " " << (int) vel[3] << " ");
        write(arduino.fd, vel, 4);

        rate.sleep();
    }

    ROS_INFO("Closing communication - Arduino Motor");
    close(arduino.fd);
}
