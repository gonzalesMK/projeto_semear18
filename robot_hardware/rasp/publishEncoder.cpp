#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <stdio.h>

#include <pigpio.h>

#include "robot_hardware/encoderLib.h"

#include <boost/bind.hpp>

// http://wiki.ros.org/pid

/* Interface Encoder and ROS
 *
 * We have a class to update the position of each encoder using the Raspberry GPIO. This class is going to
 * publish the position of the encoder into the topic <motor>/encoder.
 * 
 * This topic is going to be subscribed by the PID topic.
 * 
 * One can turn the publish on by setting the PID enable topic to TRUE
 */
bool enable = false;
void pid_cb(std_msgs::Bool bool_msg)
{
    enable = bool_msg.data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "encoderInterface");
    ros::NodeHandle node;

    if (gpioInitialise() < 0)
        return 1;

    // ROXO e CINZA | FR
    re_decoder dec1(20, 21); // GPIO 26 | GPIO 13
    // VERDE e AZUL |  BR
    re_decoder dec2(16, 5); // GPIO 6 | GPIO 5
    // AMARELO e LARANJA | FL
    re_decoder dec3(13, 6); // GPIO 12 | GPIO 16
    // MARROM e VERMELHO | BL
    re_decoder dec4(19, 26); // GPIO 23| GPIO 24

    //    sleep(1000);

    ros::Publisher pub1 = node.advertise<std_msgs::Float64>("/motorFR/encoderVelocity", 1);
    ros::Publisher pub2 = node.advertise<std_msgs::Float64>("/motorBR/encoderVelocity", 1);
    ros::Publisher pub3 = node.advertise<std_msgs::Float64>("/motorFL/encoderVelocity", 1);
    ros::Publisher pub4 = node.advertise<std_msgs::Float64>("/motorBL/encoderVelocity", 1);
    ros::Publisher pub1_del = node.advertise<std_msgs::Int64>("/motorFR/encoderDesl", 1);
    ros::Publisher pub2_del = node.advertise<std_msgs::Int64>("/motorBR/encoderDesl", 1);
    ros::Publisher pub3_del = node.advertise<std_msgs::Int64>("/motorFL/encoderDesl", 1);
    ros::Publisher pub4_del = node.advertise<std_msgs::Int64>("/motorBL/encoderDesl", 1);
    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;
    std_msgs::Float64 msg4;
    std_msgs::Int64 msg1_del;
    std_msgs::Int64 msg2_del;
    std_msgs::Int64 msg3_del;
    std_msgs::Int64 msg4_del;

    ros::Rate rate(100);
    double current_time = ros::Time::now().toSec();
    double old_time = ros::Time::now().toSec();
    double dt = 0;

    int32_t old_position1 = 0;
    int32_t old_position2 = 0;
    int32_t old_position3 = 0;
    int32_t old_position4 = 0;

    while (ros::ok())
    {
        current_time = ros::Time::now().toSec();
        dt = current_time - old_time;

        msg1.data = (dec1.position - old_position1) / dt;
        msg2.data = (dec2.position - old_position2) / dt;
        msg3.data = (dec3.position - old_position3) / dt;
        msg4.data = (dec4.position - old_position4) / dt;

        msg1_del.data = dec1.position;
        msg2_del.data = dec2.position;
        msg3_del.data = dec3.position;
        msg4_del.data = dec4.position;
        pub1.publish(msg1);
        pub2.publish(msg2);
        pub3.publish(msg3);
        pub4.publish(msg4);
        
        pub1_del.publish(msg1_del);
        pub2_del.publish(msg2_del);
        pub3_del.publish(msg3_del);
        pub4_del.publish(msg4_del);

        old_position1 = dec1.position;
        old_position2 = dec2.position;
        old_position3 = dec3.position;
        old_position4 = dec4.position;

        old_time = current_time;

        ros::spinOnce();
        rate.sleep();
    }

    dec1.re_cancel();
    dec2.re_cancel();
    dec3.re_cancel();
    dec4.re_cancel();

    gpioTerminate();
}
