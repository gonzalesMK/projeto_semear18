#include <ros/ros.h>
#include <projeto_semear/kine_control.h>
#include <std_msgs/Float32.h>

// Code that takes the robot from de Central to the Right

void callbackBL(std_msgs::Float32ConstPtr &msg);
void callbackBR(std_msgs::Float32ConstPtr &msg);
void callbackFL(std_msgs::Float32ConstPtr &msg);
void callbackFR(std_msgs::Float32ConstPtr &msg);

double colorBL = -1;
double colorBR = -1;
double colorFL = -1;
double colorFR = -1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::motorControl motor;

    // Parâmetro de entrada do motor
    geometry_msgs::Twist velocidade;
    velocidade.linear.x = 0;
    velocidade.linear.y = 1;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);

    ros::Subscriber lineSensorBL = nh.subscribe("/AMR/lineSensorBL", 1, callbackBL);
    ros::Subscriber lineSensorBR = nh.subscribe("/AMR/lineSensorBR", 1, callbackBR);
    ros::Subscriber lineSensorFL = nh.subscribe("/AMR/lineSensorFL", 1, callbackFL);
    ros::Subscriber lineSensorFR = nh.subscribe("/AMR/lineSensorFR", 1, callbackFR);

    while (colorBR == -1 || colorFL == -1 || colorBL == -1 || colorFR == -1)
    {
        ros::spinOnce(); // It is importante to get topics callback
        ros::Duration(0.5).sleep();
    }

    // First, get the right sensor to the black tape
    //while( )
    // Second, get the left sensor to the black tape

        // Third, walk a predefined length

        return 0;
}

void callbackBL(std_msgs::Float32ConstPtr &msg)
{
    colorBL = msg->data;
}

void callbackBR(std_msgs::Float32ConstPtr &msg)
{
    colorBR = msg->data;
}

void callbackFL(std_msgs::Float32ConstPtr &msg)
{
    colorFL = msg->data;
}

void callbackFR(std_msgs::Float32ConstPtr &msg)
{
    colorFR = msg->data;
}
