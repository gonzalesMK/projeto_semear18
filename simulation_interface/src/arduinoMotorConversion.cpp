#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

// #include <termio.h>

/* Interface ROS -> Arduino 
 *
 * This code receives the PWM command from the topic <motor>/pwm and sends it to the arduino. Check the arduino code for more details on this.
 * 
 * The arduino in the Linux system is recognized as a TTY file. 
 * 
 * The arduino Receives one char in the range [-127, 127] for each motor
 */

double vel[4];

void motor_cb(const std_msgs::Float64ConstPtr &msg, double &var)
{

   var = msg->data / 50;
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "MotorArduinoInterface");
   ros::NodeHandle node;

   // Create 4 topics to receive motor speed
   ros::Subscriber motor_sub1 = node.subscribe<std_msgs::Float64>("/motorFR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[0])));
   ros::Subscriber motor_sub2 = node.subscribe<std_msgs::Float64>("/motorFL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[1])));
   ros::Subscriber motor_sub3 = node.subscribe<std_msgs::Float64>("/motorBR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[2])));
   ros::Subscriber motor_sub4 = node.subscribe<std_msgs::Float64>("/motorBL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[3])));

   ros::Publisher pub[4];
   pub[0] = node.advertise<std_msgs::Float32>("/AMR/motorFRSpeed", 1);
   pub[1] = node.advertise<std_msgs::Float32>("/AMR/motorFLSpeed", 1);
   pub[2] = node.advertise<std_msgs::Float32>("/AMR/motorBRSpeed", 1);
   pub[3] = node.advertise<std_msgs::Float32>("/AMR/motorBLSpeed", 1);

   std_msgs::Float32 msg[4];
   ros::Rate rate(20);
   while (ros::ok())
   {
      for (int i = 0; i < 4; i++)
      {
         msg[i].data = vel[i];
         pub[i].publish(msg[i]);
      }

      ros::spinOnce();
      rate.sleep();
   }

}
