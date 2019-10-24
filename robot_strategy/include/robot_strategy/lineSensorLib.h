#include "robot_strategy/arduinoInterfaceLib.h"
#include <vector>
#include <iostream>
#ifndef LINESENSOR_LIB
#define LINESENSOR_LIB

 
class LineSensor
{

public:
    std::vector<double>* _error;
    uint8_t _max_readings[8] = {100, 100, 100, 100};
    uint8_t _min_readings[8] = {20, 20, 20, 20};
    uint64_t encoder=0;
    bool upperLimit;
    bool lowerLimit;
    uint8_t digi;

    // Debug Purpose
    uint8_t raw_readings[8];
    double calibrated_readings[8];
    double servo_pose=90;
    // Cremalheira
    int __fastVel = 255;
    int __slowVel = 100;
    bool __claw_is_referenced=false;
    double gearAndPinionHeight=0;
    double __claw_precision=0.1;
    Arduino *arduino;

    LineSensor();
    // Controle Sensores de Linha
    std::vector<double>* readLines();
    void reset(bool resetToMinimun = false);

    // Controle da Cremalheira
    void pickContainer();
    void dropContainer(double height);
    void writeClawPWM(int pwm);
    
    // Controle do Servo
    void setServoPose(double pose);

    // Controle do Eletroima
    void setElectromagnet(bool turnOn=true);
    
    
};

enum Sides
{
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3
};

enum sensorSides
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3,
    LF = 4,
    LB = 5,
    RF = 6,
    RB = 7
};

#endif