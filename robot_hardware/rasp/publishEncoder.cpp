#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <stdio.h>

#include <pigpio.h>

#include "projeto_semear/encoderLib.h"

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

    re_decoder dec1(37,33); // GPIO 26 | GPIO 13
    re_decoder dec2(31, 29); // GPIO 6 | GPIO 5
    re_decoder dec3(32, 36); // GPIO 12 | GPIO 16 
    re_decoder dec4(16, 18); // GPIO 23| GPIO 24    

    //    sleep(1000);

    ros::Publisher pub1 = node.advertise<std_msgs::Int32>("/motorFR/encoderVelocity", 1);
    ros::Publisher pub2 = node.advertise<std_msgs::Int32>("/motorFL/encoderVelocity", 1);
    ros::Publisher pub3 = node.advertise<std_msgs::Int32>("/motorBR/encoderVelocity", 1);
    ros::Publisher pub4 = node.advertise<std_msgs::Int32>("/motorBL/encoderVelocity", 1);

    ros::Subscriber pid_sub = node.subscribe<std_msgs::Bool>("/encoder_enable", 1, pid_cb);

    std_msgs::Int32 msg1;
    std_msgs::Int32 msg2;
    std_msgs::Int32 msg3;
    std_msgs::Int32 msg4;

    ros::Rate rate(800);
    while (ros::ok())
    {
        if (enable)
        {
            msg1.data = dec1.position;
            msg2.data = dec2.position;
            msg3.data = dec3.position;
            msg4.data = dec4.position;

            pub1.publish(msg1);
            pub2.publish(msg2);
            pub3.publish(msg3);
            pub4.publish(msg4);
        }
        ros::spinOnce();

        rate.sleep();
    }

    dec1.re_cancel();
    dec2.re_cancel();
    dec3.re_cancel();
    dec4.re_cancel();

    gpioTerminate();
}
