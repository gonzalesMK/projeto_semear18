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

ros::Publisher pubFr;

ros::Publisher pubR0;
ros::Publisher pubR1;
ros::Publisher pubR2;
ros::Publisher pubR3;
ros::Publisher pubL0;
ros::Publisher pubL1;
ros::Publisher pubL2;
ros::Publisher pubL3;

void callbackBR(const sensor_msgs::ImageConstPtr &msg);
void callbackBL(const sensor_msgs::ImageConstPtr &msg);
void callbackFR(const sensor_msgs::ImageConstPtr &msg);
void callbackFL(const sensor_msgs::ImageConstPtr &msg);
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

void callbackFr(const sensor_msgs::ImageConstPtr &msg);

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

    ros::Subscriber subR0 = n.subscribe("/AMR/ColorSensorR0", 1000, callbackR0);
    ros::Subscriber subR1 = n.subscribe("/AMR/ColorSensorR1", 1000, callbackR1);
    ros::Subscriber subR2 = n.subscribe("/AMR/ColorSensorR2", 1000, callbackR2);
    ros::Subscriber subR3 = n.subscribe("/AMR/ColorSensorR3", 1000, callbackR3);
    ros::Subscriber subL0 = n.subscribe("/AMR/ColorSensorL0", 1000, callbackL0);
    ros::Subscriber subL1 = n.subscribe("/AMR/ColorSensorL1", 1000, callbackL1);
    ros::Subscriber subL2 = n.subscribe("/AMR/ColorSensorL2", 1000, callbackL2);
    ros::Subscriber subL3 = n.subscribe("/AMR/ColorSensorL3", 1000, callbackL3);

    ros::Subscriber subFr = n.subscribe("/AMR/FrontalSensor", 1000, callbackFr);
    

    pubBL = n.advertise<std_msgs::Float32>("/image_converter/lineSensorBL", 1);
    pubBR = n.advertise<std_msgs::Float32>("/image_converter/lineSensorBR", 1);
    pubFR = n.advertise<std_msgs::Float32>("/image_converter/lineSensorFR", 1);
    pubFL = n.advertise<std_msgs::Float32>("/image_converter/lineSensorFL", 1);
    pubGarraR = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraR", 1);
    pubGarraL = n.advertise<std_msgs::ColorRGBA>("/image_converter/sensorGarraL", 1);
    
    pubFr = n.advertise<std_msgs::Float32>("/image_converter/FrontalSensor", 1);

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

void callbackFr(const sensor_msgs::ImageConstPtr &msg)
{
    std_msgs::Float32 message;
    message.data = sqrt(pow(msg->data[0], 2) + pow(msg->data[1], 2) + pow(msg->data[2], 2));

    pubL3.publish(message);
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
