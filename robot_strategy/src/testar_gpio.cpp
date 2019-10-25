#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>
#include <robot_strategy/stateLib.h>
#include <pigpio.h>

int GPIO_PIN = 15;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testarUtils");
    ros::NodeHandle node;

    if (gpioInitialise() < 0)
        return 1;

    gpioSetMode(GPIO_PIN, PI_INPUT);

    bool start = false;

    while (!start and ros::ok())
    {
        start = gpioRead(GPIO_PIN);
        ROS_INFO_STREAM("Button: " << start);
        ros::Duration(1).sleep();
    }

    return 0;
}
