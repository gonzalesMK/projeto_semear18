#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <robot_strategy/lineSensorLib.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "testarSensor");
    ros::NodeHandle node;

    LineSensor lineSensors;

    lineSensors.reset();
    
    ros::Rate r(500);

    ros::Publisher pubLineSensors1 = node.advertise<std_msgs::UInt8>("/pololuSensorFL", 1);
    ros::Publisher pubLineSensors2 = node.advertise<std_msgs::UInt8>("/pololuSensorFR", 1);
    ros::Publisher pubLineSensors3 = node.advertise<std_msgs::UInt8>("/pololuSensorBL", 1);
    ros::Publisher pubLineSensors4 = node.advertise<std_msgs::UInt8>("/pololuSensorBR", 1);
    ros::Publisher pubLineSensors5 = node.advertise<std_msgs::UInt8>("/pololuSensorLF", 1);
    ros::Publisher pubLineSensors6 = node.advertise<std_msgs::UInt8>("/pololuSensorLB", 1);
    ros::Publisher pubLineSensors7 = node.advertise<std_msgs::UInt8>("/pololuSensorRF", 1);
    ros::Publisher pubLineSensors8 = node.advertise<std_msgs::UInt8>("/pololuSensorRB", 1);

    ros::Publisher pubContainer = node.advertise<std_msgs::UInt8>("/container", 1);
    ros::Publisher pubL = node.advertise<std_msgs::Float64>("/sensorL", 1);
    ros::Publisher pubR = node.advertise<std_msgs::Float64>("/sensorR", 1);
    ros::Publisher pubF = node.advertise<std_msgs::Float64>("/sensorF", 1);
    ros::Publisher pubB = node.advertise<std_msgs::Float64>("/sensorB", 1);
    ros::Duration(3).sleep();
    std::vector<double> *sensor;
    uint8_t b[50];
    std_msgs::UInt8 msg;
    int nread;
    while (ros::ok())
    {

        sensor = lineSensors.readLines();

        msg.data = lineSensors.raw_readings[sensorSides::FL];
        pubLineSensors1.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::FR];
        pubLineSensors2.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::BL];
        pubLineSensors3.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::BR];
        pubLineSensors4.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::LF];
        pubLineSensors5.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::LB];
        pubLineSensors6.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::RF];
        pubLineSensors7.publish(msg);
        msg.data = lineSensors.raw_readings[sensorSides::RB];
        pubLineSensors8.publish(msg);

        msg.data = 10 * lineSensors.digi;
        pubContainer.publish(msg);

        std_msgs::Float64 msgL;
        std_msgs::Float64 msgR;
        std_msgs::Float64 msgF;
        std_msgs::Float64 msgB;

        msgL.data = sensor->at(Sides::LEFT);
        msgR.data = sensor->at(Sides::RIGHT);
        msgF.data = sensor->at(Sides::FRONT);
        msgB.data = sensor->at(Sides::BACK);

        pubL.publish(msgL);
        pubR.publish(msgR);
        pubF.publish(msgF);
        pubB.publish(msgB);

        r.sleep();
    }
}