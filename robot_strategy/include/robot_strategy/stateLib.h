#include "robot_strategy/lineSensorLib.h"
#include "robot_strategy/motorControlLib.h"

#ifndef STATE_LIB
#define STATE_LIB

enum Colors
{
    Red = 0,
    Green = 1,
    Blue = 2,
    Unknown = 3,
    Empty = 4
};

inline std::ostream &operator<<(std::ostream &Str, Colors c)
{
    switch (c)
    {
    case Colors::Red:
        return Str << "RED";
    case Colors::Green:
        return Str << "GREEN";
    case Colors::Blue:
        return Str << "BLUE";
    case Colors::Empty:
        return Str << "Empty";
    }
}
enum Positions
{
    Container1 = 0,
    Container2 = 1,
    Container3 = 2,
    Container4 = 3,
    Container5 = 4,
    Container6 = 5,
    Container7 = 6,
    Container8 = 7,
    GreenDock = 8,
    BlueDock = 9,
    GreenIntersection = 10,
    BlueIntersection = 11,
    StartPosition = 12,
    Unkown = 13
};

inline std::ostream &operator<<(std::ostream &Str, Positions c)
{
    switch (c)
    {
    case Positions::GreenIntersection:
        return Str << "GreenIntersection";
    case Positions::BlueIntersection:
        return Str << "BlueIntersection";
    case Positions::Container1:
        return Str << "Container" << 1;
    case Positions::Container2:
        return Str << "Container" << 2;
    case Positions::Container3:
        return Str << "Container" << 3;
    case Positions::Container4:
        return Str << "Container" << 4;
    case Positions::Container5:
        return Str << "Container" << 5;
    case Positions::Container6:
        return Str << "Container" << 6;
    case Positions::Container7:
        return Str << "Container" << 7;
    case Positions::Container8:
        return Str << "Container" << 8;
    default:
        return Str << "I don't know";
    }
}

void firstPose(LineSensor &linesensors, MotorControl &motorControl);
void to_container(LineSensor &linesensors, MotorControl &motorControl);
void container_to_intersection(LineSensor &linesensors, MotorControl &motorControl);
void to_dock(LineSensor &linesensors, MotorControl &motorControl, Colors containerColor);
void dock_to_intersection(LineSensor &linesensors, MotorControl &motorControl, Colors containerColor);
void change_intersection(LineSensor &linesensors, MotorControl &motorControl, Positions startingPose);
void functionPlanner(std::vector<boost::function<void()> >& functionList, LineSensor &linesensors, MotorControl &motorControl, std::vector<Colors> colorsToPick, std::vector<int> containersToPick, Positions initialPose);

#endif