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
ros::Publisher pubE0;
ros::Publisher pubE1;
ros::Publisher pubE2;
ros::Publisher pubE3;

ros::Publisher pubD0;
ros::Publisher pubD1;
ros::Publisher pubD2;
ros::Publisher pubD3;

ros::Publisher pubBR;
ros::Publisher pubFR;
ros::Publisher pubBL;
ros::Publisher pubFL;

ros::Publisher pubGarraR;
ros::Publisher pubGarraL;

ros::Publisher pubFE;
ros::Publisher pubFD;

ros::Publisher pubR0;
ros::Publisher pubR1;
ros::Publisher pubR2;
ros::Publisher pubR3;
ros::Publisher pubL0;
ros::Publisher pubL1;
ros::Publisher pubL2;
ros::Publisher pubL3;

void callbackGarraR(const sensor_msgs::ImageConstPtr &msg);
void callbackGarraL(const sensor_msgs::ImageConstPtr &msg);

void callbackR0(const sensor_msgs::ImageConstPtr &msg);
void callbackR1(const sensor_msgs::ImageConstPtr &msg);
void callbackR2(const sensor_msgs::ImageConstPtr &msg);
void callbackR3(const sensor_msgs::ImageConstPtr &msg);

void callbackL0(const sensor_msgs::ImageConstPtr &msg);
void callbackL1(const sensor_msgs::ImageConstPtr &msg);
void callbackL2(const sensor_msgs::ImageConstPtr &msg);
void callbackL3(const sensor_msgs::ImageConstPtr &msg);

void callbackE0(const sensor_msgs::ImageConstPtr &msg);
void callbackE1(const sensor_msgs::ImageConstPtr &msg);
void callbackE2(const sensor_msgs::ImageConstPtr &msg);
void callbackE3(const sensor_msgs::ImageConstPtr &msg);

void callbackD0(const sensor_msgs::ImageConstPtr &msg);
void callbackD1(const sensor_msgs::ImageConstPtr &msg);
void callbackD2(const sensor_msgs::ImageConstPtr &msg);
void callbackD3(const sensor_msgs::ImageConstPtr &msg);


void callbackFE(const sensor_msgs::ImageConstPtr &msg);
void callbackFD(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converColorSensor");

    ros::NodeHandle n;

    ros::Subscriber subE0 = n.subscribe("/AMR/lineSensorL0", 1000, callbackE0);
    ros::Subscriber subE1 = n.subscribe("/AMR/lineSensorL1", 1000, callbackE1);
    ros::Subscriber subE2 = n.subscribe("/AMR/lineSensorL2", 1000, callbackE2);
    ros::Subscriber subE3 = n.subscribe("/AMR/lineSensorL3", 1000, callbackE3);

    ros::Subscriber subD0 = n.subscribe("/AMR/lineSensorD0", 1000, callbackD0);
    ros::Subscriber subD1 = n.subscribe("/AMR/lineSensorD1", 1000, callbackD1);
    ros::Subscriber subD2 = n.subscribe("/AMR/lineSensorD2", 1000, callbackD2);
    ros::Subscriber subD3 = n.subscribe("/AMR/lineSensorD3", 1000, callbackD3);

    ros::Subscriber subGarraR = n.subscribe("/AMR/ColorSensorGarraR", 1000, callbackGarraR);
    ros::Subscriber subGarraL = n.subscribe("/AMR/ColorSensorGarraL", 1000, callbackGarraL);

    ros::Subscriber subR0 = n.subscribe("/AMR/ColorSensorR0", 1000, callbackR0);
    ros::Subscriber subL0 = n.subscribe("/AMR/ColorSensorL0", 1000, callbackL0);

    ros::Subscriber subFE = n.subscribe("/AMR/frontalSensor_esq0", 1000, callbackFE);
    ros::Subscriber subFD = n.subscribe("/AMR/frontalSensor_dir0", 1000, callbackFD);

    pubE0 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE0", 1);
    pubE1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE1", 1);
    pubE2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE2", 1);
    pubE3 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorE3", 1);
    pubD0 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD0", 1);
    pubD1 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD1", 1);
    pubD2 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD2", 1);
    pubD3 = n.advertise<std_msgs::Float32>("/image_converter/lineSensorD3", 1);

    pubGarraR = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraR", 1);
    pubGarraL = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraL", 1);

    pubFE = n.advertise<std_msgs::Float32>("/image_converter/frontalSensorEsq", 1);
    pubFD = n.advertise<std_msgs::Float32>("/image_converter/frontalSensorDir", 1);

    pubR0 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorR0", 1);
    pubR1 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorR1", 1);
    pubR2 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorR2", 1);
    pubR3 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorR3", 1);
    pubL0 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorL0", 1);
    pubL1 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorL1", 1);
    pubL2 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorL2", 1);
    pubL3 = n.advertise<std_msgs::Float32>("/image_converter/ColorSensorL3", 1);
    ros::spin();

    return 0;
}

void callbackL0(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubR0.publish(message);
}

void callbackL1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubR1.publish(message);
}

void callbackL2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubR2.publish(message);
}

void callbackL3(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubR3.publish(message);
}

void callbackR0(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubL0.publish(message);
}

void callbackR1(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubL1.publish(message);
}

void callbackR2(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubL2.publish(message);
}

void callbackR3(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubL3.publish(message);
}

void callbackE0(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubE0.publish(message);
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

void callbackE3(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubE3.publish(message);
}

void callbackD0(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubD0.publish(message);
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

void callbackD3(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubD3.publish(message);
}

void callbackFE(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubFE.publish(message);
}

void callbackFD(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubFD.publish(message);
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

