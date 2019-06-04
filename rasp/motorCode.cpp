#include <ros/ros.h>
#include <std_msgs/Float64.h>

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

char vel[4];

void motor_cb(const std_msgs::Float64ConstPtr &msg, char &var){

   var = (char) msg->data;
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "MotorArduinoInterface");
   ros::NodeHandle node;
   
   // Create 4 topics to receive motor speed
   ros::Subscriber motor_sub1 = node.subscribe<std_msgs::Float64>("/motorFR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[0])) );
   ros::Subscriber motor_sub2 = node.subscribe<std_msgs::Float64>("/motorFL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[1])) );
   ros::Subscriber motor_sub3 = node.subscribe<std_msgs::Float64>("/motorBR/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[2])) );
   ros::Subscriber motor_sub4 = node.subscribe<std_msgs::Float64>("/motorBL/pwm", 1, boost::bind(motor_cb, _1, boost::ref(vel[3])) );

   // Open arduino
   std::FILE *file = std::fopen("/dev/ttyACM0", "w");


   while( ros::ok() ){

      std::fprintf( file,"%c", vel[0]);
      std::fprintf( file,"%c", vel[1]);
      std::fprintf( file,"%c", vel[2]);
      std::fprintf( file,"%c", vel[3]);

      ros::spinOnce();
      sleep(10);
   }

   std::fclose(file);
}
