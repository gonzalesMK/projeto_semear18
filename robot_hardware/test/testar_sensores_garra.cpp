#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>

#include <iostream>
#include <stdio.h>

uint8_t pololu = 0;
uint8_t container = 0;


void pololu_cb(const std_msgs::UInt8ConstPtr &msg)
{
	pololu = msg->data;
	ROS_INFO_STREAM(
		" FL: " << ((pololu & (1 << 0)) != 0) << 
		" FR: " << ((pololu & (1 << 1)) != 0) << 
		" BL: " << ((pololu & (1 << 2)) != 0) << 
		" BR: " << ((pololu & (1 << 3)) != 0) << 
		" LF: " << ((pololu & (1 << 4)) != 0) << 
		" LB: " << ((pololu & (1 << 5)) != 0) << 
		" RF: " << ((pololu & (1 << 6)) != 0) << 
		" RB: " << ((pololu & (1 << 7)) != 0));
}

void container_cb(const std_msgs::UInt8ConstPtr &msg)
{
	container = msg->data;
	// Print
	ROS_INFO_STREAM(
		" Digi1: " << ((container & (1 << 0)) != 0) << " Digi2: " << ((container & (1 << 1)) != 0));
}

void encoder_cb(const std_msgs::Int64ConstPtr &msg)
{
	// Print
	ROS_INFO_STREAM(
		" Height: " << msg->data);
}
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "testarGarra");
	ros::NodeHandle node;

	ros::Subscriber subPololu = node.subscribe<std_msgs::UInt8>("/pololuSensor", 1, pololu_cb);
	ros::Subscriber subContainer = node.subscribe<std_msgs::UInt8>("/containerSensor", 1, container_cb);
	ros::Subscriber subEncoder = node.subscribe<std_msgs::Int64>("/claw/height", 1, encoder_cb);


	ros::spin();


	return 0;
}
