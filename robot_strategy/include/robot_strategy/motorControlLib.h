#include <ros/ros.h>
#include "robot_strategy/arduinoInterfaceLib.h"


class MotorControl{


    public:

    double Kp;
    double Kd;
    double pid_on;
    double freq;
    double momentum;
    double Ki;
    double windUp;
    
    bool _velocity_mode = false;
    bool _align_mode = false;
    bool _pid_on = false;
    
    Arduino* arduino;

    double deltaError[4] = {0, 0, 0 ,0};

    MotorControl(ros::NodeHandle &node, double Kp, double Kd, double Ki, double windUp, double freq, double momentum);

    
};

