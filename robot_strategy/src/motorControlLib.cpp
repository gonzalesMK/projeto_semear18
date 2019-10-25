#include "robot_strategy/motorControlLib.h"
//#include <math.h>
#include <stdlib.h> /* abs */
#include <math.h>   /* fabs */

MotorControl::MotorControl(double freq, double Kp, double Kd, double Ki, double windUp, double momentum)
{
    this->Kp = Kp;
    this->Kd = Kd;
    this->pid_on = false;
    this->freq = freq;
    this->momentum = momentum;
    this->Ki = Ki;
    this->windUp = windUp;

    // Settup arduino
    //char str[] = "/dev/ttyUSB0";
    this->arduino = new Arduino(1);
}

void MotorControl::align(double *error_array)
{
    double actuation[4];
    int actuation_temp[4];
    int8_t actuation_int8_t[4];
    // Start PID
    if (!this->pid_on)
    {
        this->pid_on = true;

        for (int i = 0; i < 4; i++)
        {
            this->deltaError[i] = 0;
            this->integrative[i] = 0;
        }
        this->pastError[0] = error_array[0];
        this->pastError[1] = error_array[1];
        this->pastError[2] = error_array[2];
        this->pastError[3] = error_array[3];
        return;
    };

    // Integrative and Derivative Error
    for (int i = 0; i < 4; i++)
    {
        this->deltaError[i] = (error_array[i] - this->pastError[i]) + this->momentum * this->deltaError[i];
        this->integrative[i] += error_array[i] / this->freq;

        // Windup
        if (fabs(this->integrative[i]) * fabs(this->Ki) > this->windUp)
        {
            this->integrative[i] = this->windUp / this->Ki * this->integrative[i] / fabs(this->integrative[i]);
        }

        // Calculate PID
        actuation[i] = error_array[i] * this->Kp + this->deltaError[i] * this->Kd + this->Ki * this->integrative[i];

        // Clip limits
        if (actuation[i] > 120)
            actuation[i] = 120;
        if (actuation[i] < -120)
            actuation[i] = -120;

        // Save actual error
        this->pastError[i] = error_array[i];
        actuation_temp[i] = (int)actuation[i];
        actuation_int8_t[i] = ((int8_t)actuation_temp[i]);
    }

    // Send Velocity
    write(this->arduino->fd, actuation_int8_t, 4);
    /**
    ROS_INFO_STREAM("Received: \nFL: " << error_array[Wheels::FL] << " \tFR: " << error_array[Wheels::FR] << " \nBL: " << error_array[Wheels::BL] << " \tBR: " << error_array[Wheels::BR]);
    ROS_INFO_STREAM("Sending actuation: \nFL: " << actuation[Wheels::FL] << " \tFR: " << actuation[Wheels::FR] << " \nBL: " << actuation[Wheels::BL] << " \tBR: " << actuation[Wheels::BR]);
    ROS_INFO_STREAM("Sending char actuation: \nFL: " << (int)actuation_char[Wheels::FL] << " \tFR: " << (int)actuation_char[Wheels::FR] << " \nBL: " << (int)actuation_char[Wheels::BL] << " \tBR: " << (int)actuation_char[Wheels::BR]);
    */
    // ROS_INFO_STREAM("Received: \nFL: " << (int)c[Wheels::FL] << " \tFR: " << (int)c[Wheels::FR] << " \nBL: " << (int)c[Wheels::BL] << " \tBR: " << (int)c[Wheels::BR]);
}

void MotorControl::clear()
{
    this->pid_on = false;
}

void MotorControl::stop()
{
    double actuation[4] = {0, 0, 0, 0};
    write(this->arduino->fd, actuation, 4);
}
