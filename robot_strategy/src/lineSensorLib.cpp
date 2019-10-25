#include "robot_strategy/lineSensorLib.h"
#include <stdlib.h> /* abs */
#include <math.h>   /* fabs */
#include <unistd.h>

const uint8_t ELETROIMA_ON_CODE = 64;
const uint8_t ELETROIMA_OFF_CODE = 65;
const uint8_t SERVO_ON_CODE = 66;

LineSensor::LineSensor()
{

    //char str[] = "/dev/ttyUSB1";
    this->arduino = new Arduino(2);

    this->_error = new std::vector<double>;
    this->_error->push_back(0);
    this->_error->push_back(0);
    this->_error->push_back(0);
    this->_error->push_back(0);
}

std::vector<double> *LineSensor::readLines()
{
    this->talkToArduino();
    //double calibrated_readings[8];
    for (int i = 0; i < 8; i++)
    {
        if (this->_max_readings[i] <= this->raw_readings[i])
            this->_max_readings[i] = this->raw_readings[i];

        if (this->_min_readings[i] >= this->raw_readings[i])
            this->_min_readings[i] = this->raw_readings[i];

        this->calibrated_readings[i] = ((double)this->raw_readings[i] - this->_min_readings[i]) / ((double)this->_max_readings[i] - this->_min_readings[i]) * 100;
        
    }

    bool sensorValueA[4];
    bool sensorValueB[4];

    for (int i = 0; i < 4; i++)
    {

        if (this->calibrated_readings[2 * i] > 20 || this->calibrated_readings[2 * i + 1] > 20)
        {
            this->_error->at(i) = 2 * this->calibrated_readings[2 * i + 1] / (this->calibrated_readings[2 * i] + this->calibrated_readings[2 * i + 1]) - 1;
        }
        else
        {
            this->_error->at(i) = this->_error->at(i) > 0 ? 1 : -1;
        }
    }

    return this->_error;
}

void LineSensor::talkToArduino()
{
    char a[1] = {68};
    write(this->arduino->fd, a, 1); // Send one byte and wait for response

    uint8_t b[14];
    int nread;
    do
    {
        nread = read(this->arduino->fd, b, 14);
        usleep(1000);
    } while (!(nread > 0));

    for(int i =0 ; i < 8; i++){
        this->raw_readings[i] = b[i];
    }
    // Container
    this->digi = !(b[8] & 1);

    // Limits
    this->upperLimit = (b[9] & 1);
    this->lowerLimit = (b[9] & 4);

    // Encoder reading
    this->encoder = -((int64_t)b[10] + (int64_t)b[11] * 256 + (int64_t)b[12] * 65536 + (int64_t)b[13] * 16777216 - (int64_t)2147483648);
}
void LineSensor::reset(bool resetToMinimun)
{
    this->_error->at(0) = resetToMinimun ? -1 : 1;
    this->_error->at(1) = resetToMinimun ? -1 : 1;
    this->_error->at(2) = resetToMinimun ? -1 : 1;
    this->_error->at(3) = resetToMinimun ? -1 : 1;
}

// PWM in [-255, 255]
void LineSensor::writeClawPWM(int pwm)
{

    if (abs(pwm) < 255)
    {
        int8_t clawPWM = (int8_t)(pwm / 4);
        write(this->arduino->fd, &clawPWM, 1);
        ROS_INFO_STREAM("Moving Gear and Pinion: " << (int)clawPWM);
    }
    else
    {
        ROS_ERROR_STREAM("Not Sending ILEGAL claw PWM. should be between [-255,255], but is: " << pwm);
    }
}

// Controle da Cremalheira
void LineSensor::pickContainer()

{   this->talkToArduino();
    this->writeClawPWM(-this->__fastVel);
    ros::Rate r(200);

    while (ros::ok() && this->lowerLimit)
    {
        this->talkToArduino();
    }

    this->writeClawPWM(0);
    this->setElectromagnet(true);

    ROS_INFO_STREAM("Grabbed Container");
}

void LineSensor::dropContainer(double height)
{
    if (!this->__claw_is_referenced)
        ROS_INFO_STREAM("A cremalheira não foi referenciada. Cuidado! Chamar a função <> antes de usar o set Pose");

    this->talkToArduino();
    double error = (this->encoder - this->startPose) - height;
    ros::Rate r(200);
    if (error < 0)
    {
        while (fabs(error) > this->__claw_precision)
        {
            this->talkToArduino();

            error = (this->encoder - this->startPose) - height; // in cm

            if (fabs(error) > 1)
                this->writeClawPWM(this->__fastVel * error / fabs(error));
            else
                this->writeClawPWM(this->__slowVel * error / fabs(error));

            
        }
    }
    this->setElectromagnet(false);
    this->writeClawPWM(0);
}

void LineSensor::resetGearAndPinionPose()
{
    this->talkToArduino();
    this->writeClawPWM(this->__fastVel);

    while (this->upperLimit)
    {
        this->talkToArduino();
    }

    this->writeClawPWM(0);
    this->talkToArduino();

    this->startPose += this->encoder;
    this->__claw_is_referenced = true;
}
// Controle do Servo
void LineSensor::writeToServo(uint8_t servoPose)
{
    if (servoPose > 180)
    {
        ROS_ERROR_STREAM("O valor de servoPose deve estar em [0, 180], mas é : " << servoPose);
    }
    write(this->arduino->fd, &SERVO_ON_CODE, 1);
    ROS_INFO_STREAM("Turning On Servo: " << (int)SERVO_ON_CODE);

    ros::Duration(0.005).sleep();

    write(this->arduino->fd, &servoPose, 1);
    ROS_INFO_STREAM("Servo Pose: " << (int)servoPose);
}

void LineSensor::setServoPose(uint8_t servoPose)
{

    ros::Rate r(5);
    double step = ((int8_t)(servoPose - this->__servo_pose)) / 5;

    for (int i = 1; i < 5; i++)
    {
        this->writeToServo((uint8_t)this->__servo_pose + step * i);
        r.sleep();
    }
    this->writeToServo(servoPose);
    this->__servo_pose = servoPose;
}

// Controle do Eletroima
void LineSensor::setElectromagnet(bool turnOn)
{
    if (turnOn)
    {
        write(this->arduino->fd, &ELETROIMA_ON_CODE, 1);
        ROS_INFO_STREAM("Turning On electromagnet: " << (int)ELETROIMA_ON_CODE);
    }
    else
    {
        write(this->arduino->fd, &ELETROIMA_OFF_CODE, 1);
        ROS_INFO_STREAM("Turning Off electromagnet: " << (int)ELETROIMA_OFF_CODE);
    }
}
