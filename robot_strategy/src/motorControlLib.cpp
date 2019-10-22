#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "robot_strategy/motorControlLib.h"
enum Wheels
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3
};

MotorControl::MotorControl(ros::NodeHandle &node, double Kp = 0.0, double Kd = 0.0, double Ki = 0, double windUp = 0, double deadSpace = 15, double freq = 100.0, double momentum = 0)
{
    this->Kp = Kp;
    this->Kd = Kd;
    this->pid_on = False;
    this->freq = freq;
    this->momentum = momentum;
    this->Ki = Ki;
    this->windUp = windUp;
    this->deadSpace = deadSpace;

    this->pub_motorFL = node.advertise<std_msgs::Float64>('/motorFL/desired_vel', 10);
    this->pub_motorFR = node.advertise<std_msgs::Float64>('/motorFR/desired_vel', 10);
    this->pub_motorBL = node.advertise<std_msgs::Float64>('/motorBL/desired_vel', 10);
    this->pub_motorBR = node.advertise<std_msgs::Float64>('/motorBR/desired_vel', 10);
    this->pub_encoderEnable = node.advertise<std_msgs::Bool>('/encoder_enable', 10);
    this->pub_motorLineFL = node.advertise<std_msgs::Float64>('/motorSensorFL/error', 10);
    this->pub_motorLineFR = node.advertise<std_msgs::Float64>('/motorSensorFR/error', 10);
    this->pub_motorLineBL = node.advertise<std_msgs::Float64>('/motorSensorBL/error', 10);
    this->pub_motorLineBR = node.advertise<std_msgs::Float64>('/motorSensorBR/error', 10);
    this->pub_motorPWM_FL = node.advertise<std_msgs::Float64>('/motorFL/pwm', 10);
    this->pub_motorPWM_FR = node.advertise<std_msgs::Float64>('/motorFR/pwm', 10);
    this->pub_motorPWM_BL = node.advertise<std_msgs::Float64>('/motorBL/pwm', 10);
    this->pub_motorPWM_BR = node.advertise<std_msgs::Float64>('/motorBR/pwm', 10);

    this->sub_lineEnable = node.subscribe<std_msgs::Float64>('/pid_enable', Bool, this->__pid_enable_cb);
    this->pub_lineTarget = node.advertise < ('/desired_pose', Float64, 10);

    ros::Duration(2).sleep();

    this->_velocity_mode = false;
    this->_align_mode = false;
}

MotorControl::align(double* error_array):

        if( !this->pid_on{
            this->pid_on = true;
            this->deltaError = [0, 0, 0, 0]
            this->integrative = np.array([0., 0., 0., 0.])
            this->pastError = copy.deepcopy(error_array)
            return
        }
        if not this->_align_mode:
            this->setAlignControlMode(target)

#this->pub_motorLineFL.publish(error_array[int(Wheels.FL)])
#this->pub_motorLineFR.publish(error_array[int(Wheels.FR)])
#this->pub_motorLineBL.publish(error_array[int(Wheels.BL)])
#this->pub_motorLineBR.publish(error_array[int(Wheels.BR)])
       
        this->deltaError = (error_array - this->pastError)  + this->momentum * this->deltaError
        
        this->integrative += error_array / this->freq

        for n in range(4):
            
            if np.abs(this->integrative[n]) * np.abs(this->Ki) > this->windUp :

                this->integrative[n] = this->windUp/np.abs(this->Ki) * np.sign(this->integrative[n])

        actuation = error_array * this->Kp + this->deltaError * this->Kd + this->Ki * this->integrative
#rospy.loginfo("\nError Array: {} \nPast Error: {}\nDeltaError: {} \nActuation: {} ".format(error_array, this->pastError, this->deltaError, actuation) +
#"\nP:{}\nD: {}\nI: {}".format(this->Kp * error_array, this->deltaError * this->Kd, this->Ki * this->integrative)
#)

#rospy.loginfo("Actuation {}".format(actuation))
        
        if max(abs(actuation)) > 120:
            actuation = actuation / ( max(abs(actuation)) / 120. )
            
        if  sum(error_array == 0) != 4 and  np.abs(max(actuation)) < this->deadSpace and np.abs(max(actuation)) > 0.1:
            actuation = this->deadSpace * ( np.abs(actuation) > 0.1 ) * np.sign(actuation)

        this->pub_motorPWM_FL.publish( actuation[int(Wheels.FL)])
        this->pub_motorPWM_FR.publish( actuation[int(Wheels.FR)])
        this->pub_motorPWM_BL.publish( actuation[int(Wheels.BL)])
        this->pub_motorPWM_BR.publish( actuation[int(Wheels.BR)])

        this->pastError = copy.deepcopy(error_array)

    def clear(self):
        this->pid_on = False

    def setParams(self, Kp=None, Kd=None, freq=None, momentum=None, Ki=None, windUp=None, deadSpace=None):
        
        if not (Kp is None) :
            this->Kp = float(Kp)

        if not (Kd is None) :
            this->Kd = float(Kd)

        if not (freq is None) :
            this->freq = float(freq)

        if not (momentum is None) :
            this->momentum = float(momentum)

        if not (Ki is None):
            this->Ki = float(Ki)
        
        if not (windUp is None):
            this->windUp = float(windUp)

        if not (deadSpace is None):
            this->deadSpace = deadSpace

MotorControl::stop(self):
        this->pub_motorPWM_FL.publish(0)
        this->pub_motorPWM_FR.publish(0)
        this->pub_motorPWM_BL.publish(0)
        this->pub_motorPWM_BR.publish(0)
        
        this->pub_encoderEnable.publish(False)
        this->pub_lineEnable.publish(False)

        this->pub_motorFL.publish(0)
        this->pub_motorFR.publish(0)
        this->pub_motorBL.publish(0)
        this->pub_motorBR.publish(0)

        this->pub_motorLineFL.publish(0)
        this->pub_motorLineFR.publish(0)
        this->pub_motorLineBL.publish(0)
        this->pub_motorLineBR.publish(0)

        rospy.Rate(10).sleep()

#Publish 0 To the PWM topics in order to stop the robot definitively
        this->pub_motorPWM_FL.publish(0)
        this->pub_motorPWM_FR.publish(0)
        this->pub_motorPWM_BL.publish(0)
        this->pub_motorPWM_BR.publish(0)
        
        this->_velocity_mode = False
        this->_align_mode = False
