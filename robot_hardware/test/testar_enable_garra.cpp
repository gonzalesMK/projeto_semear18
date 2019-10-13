#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarEnableGarra");
    ros::NodeHandle node;
    
    ros::Publisher pubSetElectro = node.advertise<std_msgs::Bool>("/claw/enableElectromagnet", 1);
    ros::Publisher pubTurnOnClaw = node.advertise<std_msgs::Bool>("/claw/enableFB", 1);
	ros::Publisher pubTurnOnInfr = node.advertise<std_msgs::Bool>("/turnOnPololuSensors", 1);
    std_msgs::Bool msg;

    while (ros::ok())
    {	char command;
	std::cout << "Type a command : \n(1) Turn On eletromagnet\n(2) Turn off eletromagnet\n(3) Turn on Claw\n(4) Turn off claw\n(5) Turn On Infra\n(6) Turn On Infra\n(7) Quit\n\tCommand: ";
	std::cin >> command;
	switch(command){
		case '1':
			ROS_INFO("Turning On Eletromagnet");
			msg.data = true;
			pubSetElectro.publish(msg);
			break;
		case '2':
			ROS_INFO("Turning Off Eletromagnet");
			msg.data = false;
			pubSetElectro.publish(msg);
			break;
		case '3':
			ROS_INFO("Turning On Motor");
			msg.data = true;
			pubTurnOnClaw.publish(msg);
			break;
		case '4':
			ROS_INFO("Turning Off Motor");
			msg.data = false;
			pubTurnOnClaw.publish(msg);
			break;
		case '5':
			ROS_INFO("Turning On Infras");
			msg.data = true;
			pubTurnOnInfr.publish(msg);
			break;
		case '6':
			ROS_INFO("Turning Off Infrqas");
			msg.data = false;
			pubTurnOnInfr.publish(msg);
			break;
		case '7':
			return 0;
			
	}
        ros::spinOnce();
    }

    
    return 0;	
}
