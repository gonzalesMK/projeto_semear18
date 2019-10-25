#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>
#include <robot_strategy/stateLib.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarUtils");
    ros::NodeHandle node;
    ros::Duration(1).sleep();
    LineSensor linesensor;
    ROS_INFO_STREAM("Motor");
    MotorControl motorControl;
    ros::Duration(3).sleep();
    int cm;

    Positions position = Positions::GreenIntersection;

    double tmp;
    if (node.getParam("/testar_states/Kp1", tmp))
    {   
        ROS_INFO_STREAM("Received KP1: " << tmp);
        motorControl.Kp1 = tmp;
    }
    if (node.getParam("/testar_states/Kd1", tmp))
    {   
        ROS_INFO_STREAM("Received Kd1: " << tmp);
        motorControl.Kd1 = tmp;
    }
        if (node.getParam("/testar_states/momentum1", tmp))
    {   
        ROS_INFO_STREAM("Received momentum1: " << tmp);
        motorControl.momentum1 = tmp;
    }
    if (node.getParam("/testar_states/Kp2", tmp))
    {   
        ROS_INFO_STREAM("Received Kp2: " << tmp);
        motorControl.Kp2 = tmp;
    }
    if (node.getParam("/testar_states/Kd2", tmp))
    {   
        ROS_INFO_STREAM("Received Kd2: " << tmp);
        motorControl.Kd2 = tmp;
    }
    if (node.getParam("/testar_states/momentum2", tmp))
    {   
        ROS_INFO_STREAM("Received momentum2: " << tmp);
        motorControl.momentum2 = tmp;
    }
    if (node.getParam("/testar_states/Kp3", tmp))
    {   
        ROS_INFO_STREAM("Received Kp3: " << tmp);
        motorControl.Kp3 = tmp;
    }
    if (node.getParam("/testar_states/Kd3", tmp))
    {   
        ROS_INFO_STREAM("Received Kd3: " << tmp);
        motorControl.Kd3 = tmp;
    }
    if (node.getParam("/testar_states/momentum3", tmp))
    {   
        ROS_INFO_STREAM("Received momentum3: " << tmp);
        motorControl.momentum3 = tmp;
    }


    while (ros::ok())
    {
        std::cout << "Enter Command: " << std::endl;
        std::cin >> cm;

        switch (cm)
        {
        case 1:
            firstPose(linesensor, motorControl);
            break;
        case 2:
            to_container(linesensor, motorControl);
            break;
        case 3:
            container_to_intersection(linesensor, motorControl);
            break;
        case 4:
            to_dock(linesensor, motorControl, Colors::Green);
            break;
        case 5:
            dock_to_intersection(linesensor, motorControl, Colors::Green);
        case 6:
            change_intersection(linesensor, motorControl, position);
            position = position == Positions::GreenIntersection ? Positions::BlueIntersection : Positions::GreenIntersection;
            break;
        case 0:
            return 0;
        }
    }
    return 0;
}
