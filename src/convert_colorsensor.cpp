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
ros::Publisher pubBR;
ros::Publisher pubFR;
ros::Publisher pubBL;
ros::Publisher pubFL;
ros::Publisher pubGarraR;
ros::Publisher pubGarraL;

void callbackBR(const sensor_msgs::ImageConstPtr &msg);
void callbackBL(const sensor_msgs::ImageConstPtr &msg);
void callbackFR(const sensor_msgs::ImageConstPtr &msg);
void callbackFL(const sensor_msgs::ImageConstPtr &msg);
void callbackGarraR(const sensor_msgs::ImageConstPtr &msg);
void callbackGarraL(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converColorSensor");

    ros::NodeHandle n;

    ros::Subscriber subBL = n.subscribe("/AMR/lineSensorBL", 1000, callbackBL);
    ros::Subscriber subBR = n.subscribe("/AMR/lineSensorBR", 1000, callbackBR);
    ros::Subscriber subFR = n.subscribe("/AMR/lineSensorFR", 1000, callbackFR);
    ros::Subscriber subFL = n.subscribe("/AMR/lineSensorFL", 1000, callbackFL);
    ros::Subscriber subGarraR = n.subscribe("/AMR/ColorSensorGarraR", 1000, callbackGarraR);
    ros::Subscriber subGarraL = n.subscribe("/AMR/ColorSensorGarraL", 1000, callbackGarraL);

    pubBL = n.advertise<std_msgs::Float32>("/image_converter/lineSensorBL", 1);
    pubBR = n.advertise<std_msgs::Float32>("/image_converter/lineSensorBR", 1);
    pubFR = n.advertise<std_msgs::Float32>("/image_converter/lineSensorFR", 1);
    pubFL = n.advertise<std_msgs::Float32>("/image_converter/lineSensorFL", 1);
    pubGarraR = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraR", 1);
    pubGarraL = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraL", 1);
    

    ros::spin();

    return 0;
}

void callbackBR(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubBR.publish(message);
}
void callbackBL(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubBL.publish(message);
}
void callbackFR(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubFR.publish(message);
}
void callbackFL(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubFL.publish(message);
}

void callbackGarraR(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::ColorRGBA message;
    message.r = msg->data[0];
    message.g = msg->data[1];
    message.b = msg->data[2];

    pubGarraR.publish(message);
}
void callbackGarraL(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::ColorRGBA message;
    message.r = msg->data[0];
    message.g = msg->data[1];
    message.b = msg->data[2];

    pubGarraL.publish(message);
}
