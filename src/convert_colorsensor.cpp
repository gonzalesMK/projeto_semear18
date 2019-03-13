#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <cmath>

using namespace cv;
using namespace std;

/** Esse código é responsável por converter os sensores divulgados por ROS para um valor em
 * float para que seja mais facilmente utilizado.
 * 
 * Especificamente, os sensores enviam um pixel RGB, que é transformado em uma escala de cinza
 * */

ros::Publisher pubE1;
ros::Publisher pubE2;

ros::Publisher pubD1;
ros::Publisher pubD2;

ros::Publisher pubB1;
ros::Publisher pubF1;
ros::Publisher pubB2;
ros::Publisher pubF2;

void callbackD1(const sensor_msgs::ImageConstPtr &msg);
void callbackD2(const sensor_msgs::ImageConstPtr &msg);

void callbackE1(const sensor_msgs::ImageConstPtr &msg);
void callbackE2(const sensor_msgs::ImageConstPtr &msg);

void callbackB1(const sensor_msgs::ImageConstPtr &msg);
void callbackB2(const sensor_msgs::ImageConstPtr &msg);
void callbackF1(const sensor_msgs::ImageConstPtr &msg);
void callbackF2(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converColorSensor");

    ros::NodeHandle n;

    ros::Subscriber subE1 = n.subscribe("/AMR/lineSensorE1", 1000, callbackE1);
    ros::Subscriber subE2 = n.subscribe("/AMR/lineSensorE2", 1000, callbackE2);

    ros::Subscriber subD1 = n.subscribe("/AMR/lineSensorD1", 1000, callbackD1);
    ros::Subscriber subD2 = n.subscribe("/AMR/lineSensorD2", 1000, callbackD2);

    ros::Subscriber subB1 = n.subscribe("/AMR/lineSensorB1", 1000, callbackB1);
    ros::Subscriber subB2 = n.subscribe("/AMR/lineSensorB2", 1000, callbackB2);

    ros::Subscriber subF1 = n.subscribe("/AMR/lineSensorF1", 1000, callbackF1);
    ros::Subscriber subF2 = n.subscribe("/AMR/lineSensorF2", 1000, callbackF2);

    pubE1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE1", 1);
    pubE2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE2", 1);
    pubD1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD1", 1);
    pubD2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD2", 1);


    pubB1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorB1", 1);
    pubB2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorB2", 1);
    pubF1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorF1", 1);
    pubF2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorF2", 1);

    ros::spin();

    return 0;
}

void callbackE1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubE1.publish(message);
}

void callbackE2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubE2.publish(message);
}


void callbackD1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubD1.publish(message);
}

void callbackD2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubD2.publish(message);
}


void callbackB1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubB1.publish(message);
}

void callbackB2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubB2.publish(message);
}

void callbackF1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubF1.publish(message);
}

void callbackF2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubF2.publish(message);
}