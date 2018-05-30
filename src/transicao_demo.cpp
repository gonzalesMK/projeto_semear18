#include <ros/ros.h>
#include <projeto_semear/kine_control.h>
#include <std_msgs/Float32.h>

// Code that takes the robot from de Central to the Right
void callbackBL(const std_msgs::Float32ConstPtr &msg);
void callbackBR(const std_msgs::Float32ConstPtr &msg);
void callbackFL(const std_msgs::Float32ConstPtr &msg);
void callbackFR(const std_msgs::Float32ConstPtr &msg);

double colorBL = -1;
double colorBR = -1;
double colorFL = -1;
double colorFR = -1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicaoDemo");

    ros::NodeHandle nh;

    kineControl::motorControl motor;    

    ros::Subscriber lineSensorBL = nh.subscribe("/image_converter/lineSensorBL", 1, callbackBL);
    ros::Subscriber lineSensorBR = nh.subscribe("/image_converter/lineSensorBR", 1, callbackBR);
    ros::Subscriber lineSensorFL = nh.subscribe("/image_converter/lineSensorFL", 1, callbackFL);
    ros::Subscriber lineSensorFR = nh.subscribe("/image_converter/lineSensorFR", 1, callbackFR);

    ROS_INFO("Waiting 4 Topics");
    while (colorBR == -1 || colorFL == -1 || colorBL == -1 || colorFR == -1)
    {
        ros::spinOnce(); // It is importante to get topics callback
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("STARTING TRANSITION");
    // Vá para direita até o sensor da direita atingir o fita preta
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 0.1;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);

    // Primeiro, o sensor da direita toca a linha preta
    while (colorFR > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // Segundo, o sensor da esquerda toca a linha preta
    while (colorFL > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // Terceiro, o sensor da esquerda toca a linha preta
    while (colorFL > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // Quarto, o sensor da esquerda sai da linha preta
    while (colorFL < 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // Quinto, andar uma distância predefinida
    ros::Duration(2.5).sleep();

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);

    return 0;
}

void callbackBL(const std_msgs::Float32ConstPtr &msg)
{
    colorBL = msg->data;
}

void callbackBR(const std_msgs::Float32ConstPtr &msg)
{
    colorBR = msg->data;
}

void callbackFL(const std_msgs::Float32ConstPtr &msg)
{
    colorFL = msg->data;
}

void callbackFR(const std_msgs::Float32ConstPtr &msg)
{
    colorFR = msg->data;
}
