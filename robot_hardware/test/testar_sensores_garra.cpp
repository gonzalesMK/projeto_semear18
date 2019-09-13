#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <iostream>
#include <stdio.h>

uint8_t pololu=0;
uint8_t container=0;


void pololu_cb(const std_msgs::UInt8ConstPtr &msg)
{
 pololu = msg->data;
}


void container_cb(const std_msgs::UInt8ConstPtr &msg)
{
 container = msg->data;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarGarra");
    ros::NodeHandle node;
    
    ros::Subscriber subPololu = node.subscribe<std_msgs::UInt8>("/pololuSensor",1, pololu_cb);
    ros::Subscriber subContainer = node.subscribe<std_msgs::UInt8>("/containerSensor", 1, container_cb );

   ros::Rate r(20);
	while(ros::ok())
{
	// Print 
	ROS_INFO_STREAM(

	" A0: " << ( (pololu & (1 << 0)) != 0) <<
	" A1: " << ((pololu & (1 << 1)) != 0 ) <<
	" A2: " << ((pololu & (1 << 2)) != 0 ) <<
	" A3: " << ((pololu & (1 << 3)) != 0 ) <<
	" A4: " << ((pololu & (1 << 4)) != 0 ) <<
	" A5: " << ((pololu & (1 << 5)) != 0 ) <<
	" A6: " << ((pololu & (1 << 6)) != 0 ) <<
	" A7: " << ((pololu & (1 << 7)) != 0 ) <<
	" Digi1: " << ((container & (1 << 0)) != 0) <<
	" Digi2: " << ((container & (1 << 1)) != 0)

	);

	
   	ros::spinOnce();
	r.sleep();

}
 
    
    return 0;	
}
