#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>
#include "robot_strategy/lineSensorLib.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testarGarra");

    ros::NodeHandle node;

    LineSensor linesensor;
    ROS_INFO_STREAM("Arduino Conected");
    int cm;
    double height;
    uint8_t servoPose;
    while (ros::ok())
    {
        std::cout << "Enter Command: \n(1) Pick Container\n(2) Drop Container\n(3) Set Servo Pose\n(4) Set Eletromagnet\n(5) Turn Off Eletromagnet\nChoice: ";
        std::cin >> cm;

        switch (cm)
        {
        case 1:
            linesensor.pickContainer();
            break;
        case 2:
            std::cout << "Enter height: " << std::endl;
            std::cin >> height;
            linesensor.dropContainer(height);
            break;
        case 3:
            std::cout << "Enter angle: " << std::endl;
            std::cin >> height;
            linesensor.setServoPose(servoPose);
            break;
        case 4:
            linesensor.setElectromagnet(true);
            break;
        case 5:
            linesensor.setElectromagnet(false);
            break;
        case 0:
            return 0;
        }
    }
    return 0;
}
