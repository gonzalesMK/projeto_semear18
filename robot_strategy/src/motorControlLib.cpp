#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "robot_strategy/motorControlLib.h"
#include <math.h>
enum Wheels
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3
};

MotorControl::MotorControl(ros::NodeHandle &node, double Kp = 0.0, double Kd = 0.0, double Ki = 0, double windUp = 0, double freq = 100.0, double momentum = 0)
{
    this->Kp = Kp;
    this->Kd = Kd;
    this->pid_on = false;
    this->freq = freq;
    this->momentum = momentum;
    this->Ki = Ki;
    this->windUp = windUp;
    this->deadSpace = deadSpace;

    // Settup arduino
    char str[] = "/dev/ttyUSB1";
    Arduino arduino(str);
    this->arduino = *arduino;
}

void MotorControl::align(double *error_array)
{
    double actuation[4];
    
    // Start PID
    if (!this->pid_on)
    {
        this->pid_on = true;
        this->deltaError = {0, 0, 0, 0};
        this->integrative = {0., 0., 0., 0.};
        this->pastError = copy.deepcopy(error_array);
        return;
    };

    // Integrative and Derivative Error
    for(int i = 0; i < 4; i++){
        this->deltaError[i] = (error_array[i] - this->pastError[i]) + this->momentum * this->deltaError;
        this->integrative[i] += error_array[i] / this->freq;

        // Windup
        if (abs( this->integrative[i] ) * abs(this->Ki) > this->windUp){
            this->integrative[i] = this->windUp/ this->Ki * this->integrative / abs(this->integrative);
        }

        // Calculate PID
        actuation[i] = error_array[i] * this->Kp + this->deltaError[i] * this->Kd + this->Ki * this->integrative[i];

        // Clip limits
        if (actuation[i] >  120) actuation[i] =  120;
        if (actuation[i] < -120) actuation[i] = -120;

        // Save actual error 
        this->pastError[i] = error_array[i];   
    }

    // Send Velocity
    write(this->arduino->fd, actuation, 4);

}
         
void MotorControl::align(){
    this->pid_on = false;
}    


void MotorControl:setParams( double Kp=0, Kd=None, freq=None, momentum=None, Ki=None, windUp=None, deadSpace=None) :
    def setParams(self, 
        
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
*/