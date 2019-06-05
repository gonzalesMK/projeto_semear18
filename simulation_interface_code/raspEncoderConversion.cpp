#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <stdio.h>

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
bool volatile enable = false;

void pid_cb(std_msgs::Bool bool_msg)
{   
    //ROS_INFO_STREAM("Encoder Enable: *********************************************************" << (int) bool_msg.data);
    enable = bool_msg.data;
}


void motor_cb(const std_msgs::Float64ConstPtr &msg, std_msgs::Float64 &var){

   var.data = msg->data / 127 * 0.36;
}

std_msgs::Float64 msg1;
std_msgs::Float64 msg2;
std_msgs::Float64 msg3;
std_msgs::Float64 msg4;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "encoderInterface");
    ros::NodeHandle node;

    ros::Publisher pub1 = node.advertise<std_msgs::Float64>("/motorFR/encoderVelocity", 1);
    ros::Publisher pub2 = node.advertise<std_msgs::Float64>("/motorFL/encoderVelocity", 1);
    ros::Publisher pub3 = node.advertise<std_msgs::Float64>("/motorBR/encoderVelocity", 1);
    ros::Publisher pub4 = node.advertise<std_msgs::Float64>("/motorBL/encoderVelocity", 1);
    ros::Subscriber pid_sub = node.subscribe<std_msgs::Bool>("/encoder_enable", 1, pid_cb);

    ros::Subscriber motor_sub1 = node.subscribe<std_msgs::Float64>("/motorFR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(msg1)));
    ros::Subscriber motor_sub2 = node.subscribe<std_msgs::Float64>("/motorFL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(msg2)));
    ros::Subscriber motor_sub3 = node.subscribe<std_msgs::Float64>("/motorBR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(msg3)));
    ros::Subscriber motor_sub4 = node.subscribe<std_msgs::Float64>("/motorBL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(msg4)));

    ros::Rate rate(10);

    msg1.data=0;
    msg2.data=0;
    msg3.data=0;
    msg4.data=0;

    while (ros::ok())
    {
        if (enable)
        {
            pub1.publish(msg1);
            pub2.publish(msg2);
            pub3.publish(msg3);
            pub4.publish(msg4);
        }
        ros::spinOnce();

        //  ROS_INFO_STREAM("Encoder Enable: " << (int) enable);
        rate.sleep();
        
    }

}
