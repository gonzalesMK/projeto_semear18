#include "robot_strategy/arduinoInterfaceLib.h"

#ifndef MOTORCONTROL_LIB
#define MOTORCONTROL_LIB

class MotorControl
{

public:
    double Kp;
    double Kd;
    double pid_on;
    double freq;
    double momentum;
    double Ki;
    double windUp;
    double pastError[4] = {0, 0, 0, 0};
    double integrative[4]{0, 0, 0, 0};
    bool _velocity_mode = false;
    bool _align_mode = false;
    bool _pid_on = false;

    Arduino *arduino;

    double deltaError[4] = {0, 0, 0, 0};

    MotorControl(double freq = 100.0, double Kp = 0.0, double Kd = 0.0, double Ki = 0, double windUp = 0, double momentum = 0);

    void clear();
    void stop();
    void align(double *error_array);

    // Variables to tune some functions
    double Kp1 = 30;
    double Kd1 = 30;
    double momentum1 = 0.6;

    double Kp2 = 30;
    double Kd2 = 30;
    double momentum2 = 0.6;
    
    double Kp3 = 30;
    double Kd3 = 30;
    double momentum3 = 0.6;
};

enum Wheels
{
    wFL = 0,
    wFR = 1,
    wBL = 2,
    wBR = 3
};

#endif