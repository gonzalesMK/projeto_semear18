#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>

/* Digitar 'i' e ',' para subir e descer e 'j' e 'l' para esquerda e direita

*/

double angular_z=90;
double linear_z;
int64_t height=0;   

ros::Publisher pubClaw;
ros::Publisher pubServo;

// Turn on/Off the infrared sensors in the base
void keyboard_cb(const geometry_msgs::TwistConstPtr &msg)
{
	angular_z += msg->angular.z * 10;
	linear_z = msg->linear.x * 80;	

	std_msgs::UInt8 servo;
	servo.data = angular_z;

	std_msgs::Float64 claw;
	claw.data = linear_z;
	pubClaw.publish(claw);
	pubServo.publish(servo);
}

void height_cb(const std_msgs::Int64ConstPtr &msg)
{
	ROS_INFO_STREAM("Height: " << msg->data);
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarGarra");
    ros::NodeHandle node;
    
    pubClaw = node.advertise<std_msgs::Float64>("/claw/pwm", 1);
    pubServo = node.advertise<std_msgs::UInt8>("/claw/servoPose", 1);
    
    ros::Subscriber subEncoder = node.subscribe<std_msgs::Int64>("/claw/height", 1, height_cb);
    ros::Subscriber subKeyboard = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, keyboard_cb );

    ros::spin();
    
    return 0;	
}
