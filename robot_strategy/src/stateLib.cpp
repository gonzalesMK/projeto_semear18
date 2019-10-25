#include "robot_strategy/stateLib.h"
#include "ros/ros.h"
#include "boost/bind.hpp"

// Falta passar pela linha preta
void firstPose(LineSensor &linesensors, MotorControl &motorControl)
{
    double error[4] = {-3, 3, 3, -3};
    std::vector<double> *sensors;
    double angular;
    double diretional;

    ROS_INFO_STREAM("1) Get Through the green line");

    linesensors.reset(false);
    motorControl.Kp = 40;
    while (ros::ok())
    {
        sensors = linesensors.readLines();
        motorControl.align(error);
        ROS_INFO_STREAM("L:" << sensors->at(Sides::LEFT) << " R:" << sensors->at(Sides::RIGHT));
        if (sensors->at(Sides::LEFT) == -1 && sensors->at(Sides::RIGHT) == -1)
        {
            break;
        }
    }
    ROS_INFO_STREAM("2) Align with the Next Black Line");

    linesensors.reset(false);
    motorControl.clear();
    motorControl.Kp = 30;
    motorControl.Kd = 100;
    motorControl.momentum = 0.5;

    int is_stable = 0;
    while (ros::ok())
    {

        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular;
        error[Wheels::wFR] = +diretional - angular;
        error[Wheels::wBL] = +diretional + angular;
        error[Wheels::wBR] = -diretional - angular;

        motorControl.align(error);

        if (fabs(sensors->at(Sides::LEFT)) < 0.3 && fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable += 1;
            if (is_stable >= 0)
                break;
        }
        else
            is_stable = 0;
    }

    motorControl.stop();

    ROS_INFO_STREAM("3) Go to the Left Following the Line");
    motorControl.clear();
    linesensors._error->at(Sides::FRONT) = -1;
    is_stable = 0;
    while (ros::ok())
    {

        sensors = linesensors.readLines();

        angular = (sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT)) * 0.5;
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wFR] = +diretional - angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wBL] = +diretional + angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wBR] = -diretional - angular - sensors->at(Sides::FRONT) * 1.5;

        if (fabs(sensors->at(Sides::FRONT)) < 0.3 && fabs(sensors->at(Sides::LEFT)) < 0.3 && fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable = is_stable + 1;

            if (is_stable >= 0)
            {
                break;
            }
        }
        //else
        //    (sensors->at(Sides::FRONT) != 0 and is_stable > 0){is_stable = 0};

        motorControl.align(error);
    }
    motorControl.stop();
}

// Ok
void to_container(LineSensor &linesensors, MotorControl &motorControl)
{
    double error[4] = {1, 1, 1, 1};
    std::vector<double> *sensors;
    double angular;
    double diretional;
    int is_stable = 0;

    ROS_INFO_STREAM("goToContainer: 1) Go straight Ahead Until Back sensors find blackline ");
    motorControl.Kp = 30;
    motorControl.Kd = 100;
    motorControl.momentum = 0.5;
    motorControl.freq = 100;

    motorControl.clear();
    error[Wheels::wFL] = -1;
    error[Wheels::wFR] = +1;
    error[Wheels::wBL] = +1;
    error[Wheels::wBR] = -1;

    motorControl.align(error);

    while (ros::ok())
    {
        motorControl.align(error);
        sensors = linesensors.readLines();

        if (fabs(sensors->at(Sides::BACK) < 0.3))
        {
            break;
        }
    }

    motorControl.stop();

    ROS_INFO_STREAM("goToContainer: 2) Keep going until find container ( or 3 sec)");
    motorControl.clear();

    is_stable = 0;
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = sensors->at(Sides::FRONT) - sensors->at(Sides::BACK);
        diretional = (sensors->at(Sides::FRONT) + sensors->at(Sides::BACK)) / 2;

        error[Wheels::wFL] = +(-diretional + angular) - ((double)1 - linesensors.digi);
        error[Wheels::wFR] = +(-diretional - angular) + ((double)1 - linesensors.digi);
        error[Wheels::wBL] = +(-diretional + angular) + ((double)1 - linesensors.digi);
        error[Wheels::wBR] = +(-diretional - angular) - ((double)1 - linesensors.digi);

        ROS_INFO_STREAM("angular: " << angular << "directional: " << diretional << "digi: " << (double)linesensors.digi);
        if (linesensors.digi)
        {
            is_stable += 1;
            if (is_stable >= 1)
            {
                break;
            }
        }
        motorControl.align(error);
    }
    ROS_INFO_STREAM("goToContainer: 3) Success");
    motorControl.stop();
}

// Ok
void container_to_intersection(LineSensor &linesensors, MotorControl &motorControl)
{
    double error[4] = {-3, 3, 3, -3};
    std::vector<double> *sensors;
    double angular;
    double diretional;
    int is_stable = 0;

    ROS_INFO_STREAM("goFromContainerToIntersection: 1) Go Back until lateral sensors find blackline");
    motorControl.clear();
    motorControl.Kp = 30;
    motorControl.Kd = 100;
    motorControl.momentum = 0.5;

    is_stable = 0;
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 3;

        error[Wheels::wFL] = +3;
        error[Wheels::wFR] = -3;
        error[Wheels::wBL] = -3;
        error[Wheels::wBR] = +3;

        motorControl.align(error);

        if (fabs(sensors->at(Sides::LEFT)) < 0.3 or fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable += 1;
            break;
        }
        else
        {
            is_stable = 0;
        }
    }
    ROS_INFO_STREAM("goFromContainerToIntersection: 2) Success");
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = (sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT));
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wFR] = +diretional - angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wBL] = +diretional + angular - sensors->at(Sides::FRONT) * 1.5;
        error[Wheels::wBR] = -diretional - angular - sensors->at(Sides::FRONT) * 1.5;

        if (fabs(sensors->at(Sides::FRONT)) < 0.3 and fabs(sensors->at(Sides::LEFT)) < 0.3 and fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable = is_stable + 1;
            if (is_stable >= 0)
            {
                break;
            }
        }

        motorControl.align(error);
    }

    motorControl.stop();
}

// Not working
void to_dock(LineSensor &linesensors, MotorControl &motorControl, Colors containerColor)
{
    double error[4] = {0, 0, 0, 0};
    std::vector<double> *sensors;
    double angular;
    double diretional;
    int is_stable = 0;

    motorControl.clear();
    motorControl.Kp = 30;
    motorControl.Kd = 50;
    motorControl.momentum = 0.5;
    double coef = containerColor == Colors::Green ? 1 : -1;

    ROS_INFO_STREAM("goToDock: 1) Go to the Side following the line during {}s | coef: " << coef);
    ros::Time begin = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - begin < ros::Duration(1)))
    {

        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular + coef * 1.5;
        error[Wheels::wFR] = +diretional - angular + coef * 1.5;
        error[Wheels::wBL] = +diretional + angular + coef * 1.5;
        error[Wheels::wBR] = -diretional - angular + coef * 1.5;

        motorControl.align(error);
    }

    ROS_INFO_STREAM("goToDock: 1) Align before moving down");
    is_stable = 0;
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular;
        error[Wheels::wFR] = +diretional - angular;
        error[Wheels::wBL] = +diretional + angular;
        error[Wheels::wBR] = -diretional - angular;

        motorControl.align(error);

        if (fabs(sensors->at(Sides::LEFT)) < 0.3 and fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable += 1;
            if (is_stable >= 50)
                break;
        }
        else
            is_stable = 0;
    }

    motorControl.stop();

    ROS_INFO_STREAM("goToDock: 2) Go DOWN until lateral sensors get out of black line ");

    error[Wheels::wFL] = 2;
    error[Wheels::wFR] = -2;
    error[Wheels::wBL] = -2;
    error[Wheels::wBR] = +2;
    while (ros::ok())
    {

        sensors = linesensors.readLines();
        motorControl.align(error);

        if (sensors->at(Sides::LEFT) > 0.99 && sensors->at(Sides::RIGHT) > 0.99)
            break;
    }

    ROS_INFO_STREAM("goToDock: 3) Go Down until find green line");
    linesensors.reset(true);

    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = (sensors->at(Sides::LEFT)) - (sensors->at(Sides::RIGHT));
        diretional = ((sensors->at(Sides::LEFT)) + (sensors->at(Sides::RIGHT))) / 2.;

        ROS_INFO_STREAM("here: " << sensors->at(Sides::LEFT));

        error[Wheels::wFL] = -diretional + angular;
        error[Wheels::wFR] = +diretional - angular;
        error[Wheels::wBL] = +diretional + angular;
        error[Wheels::wBR] = -diretional - angular;

        if (sensors->at(Sides::LEFT) != -1 && sensors->at(Sides::RIGHT) != -1)
        {
            motorControl.stop();
            break;
        }
        else
            is_stable = 0;

        motorControl.align(error);
    }

    motorControl.stop();
    ROS_INFO_STREAM("goToDock: 4) Success");
}

// Ok
void dock_to_intersection(LineSensor &linesensors, MotorControl &motorControl, Colors containerColor)
{
    double error[4] = {0, 0, 0, 0};
    std::vector<double> *sensors;
    double angular;
    double diretional;
    int is_stable = 0;

    motorControl.clear();
    motorControl.Kp = 30;
    motorControl.Kd = 100;
    motorControl.freq = 100;
    double coef = containerColor == Colors::Green ? -1 : 1;

    ROS_INFO_STREAM("goFromDockToIntersection: 1) Go straight Up Until Lateral Sensors get out of line");

    linesensors.reset(false);
    error[Wheels::wFL] = -2;
    error[Wheels::wFR] = +2;
    error[Wheels::wBL] = +2;
    error[Wheels::wBR] = -2;

    while (ros::ok())
    {

        sensors = linesensors.readLines();
        motorControl.align(error);

        ROS_INFO_STREAM("L: " << sensors->at(Sides::LEFT) << " R: " << sensors->at(Sides::LEFT));
        if (sensors->at(Sides::LEFT) == -1 && sensors->at(Sides::RIGHT) == -1)
            break;
    }

    ROS_INFO_STREAM("goFromDockToIntersection: 2) Go straight Up Until LEFT/RIGHT sensors find blackline");
    linesensors.reset(false);
    motorControl.clear();
    is_stable = 0;
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = (sensors->at(Sides::LEFT)) - (sensors->at(Sides::RIGHT));
        diretional = ((sensors->at(Sides::LEFT)) + (sensors->at(Sides::RIGHT))) / 2;

        error[Wheels::wFL] = -diretional + angular;
        error[Wheels::wFR] = +diretional - angular;
        error[Wheels::wBL] = +diretional + angular;
        error[Wheels::wBR] = -diretional - angular;

        motorControl.align(error);

        if (fabs(sensors->at(Sides::LEFT)) < 0.3 and fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable += 1;
            if (is_stable >= 10)
                break;
        }
        else
            is_stable = 0;
    }
    motorControl.stop();

    ROS_INFO_STREAM("goFromDockToIntersection: 3) Go to the side until find intersection");
    linesensors._error->at(Sides::FRONT) = containerColor == Colors::Green ? 1 : -1;

    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = (sensors->at(Sides::LEFT)) - (sensors->at(Sides::RIGHT)) * 0.5;
        diretional = ((sensors->at(Sides::LEFT)) + (sensors->at(Sides::RIGHT))) / 2;

        error[Wheels::wFL] = -diretional + angular + coef * 2;
        error[Wheels::wFR] = +diretional - angular + coef * 2;
        error[Wheels::wBL] = +diretional + angular + coef * 2;
        error[Wheels::wBR] = -diretional - angular + coef * 2;

        if (fabs(sensors->at(Sides::FRONT)) < 0.3)
        {
            is_stable = is_stable + 1;
            if (is_stable >= 20)
                break;
        }
        else
        {
            is_stable = 0;
        }
        motorControl.align(error);
    }
    motorControl.stop();

    ROS_INFO_STREAM("goFromDockToIntersection: 4) Success");
}

void change_intersection(LineSensor &linesensors, MotorControl &motorControl, Positions startingPose)
{

    double error[4] = {0, 0, 0, 0};
    std::vector<double> *sensors;
    double angular;
    double diretional;
    int is_stable = 0;

    motorControl.clear();
    motorControl.Kp = 30;
    motorControl.Kd = 100;
    motorControl.freq = 100;

    double coef;
    if (startingPose == Positions::GreenIntersection)
    {
        coef = -1;
        linesensors._error->at(Sides::FRONT) = -1;
        linesensors._error->at(Sides::BACK) = -1;
    }
    else
    {
        coef = 1;
        linesensors._error->at(Sides::FRONT) = 1;
        linesensors._error->at(Sides::BACK) = 1;
    }

    ROS_INFO_STREAM("changeIntersection: 1)  Get out of black line");
    sensors = linesensors.readLines();
    while (ros::ok() && (sensors->at(Sides::FRONT) != 1 * coef))
    {
        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular + coef * 1.5;
        error[Wheels::wFR] = +diretional - angular + coef * 1.5;
        error[Wheels::wBL] = +diretional + angular + coef * 1.5;
        error[Wheels::wBR] = -diretional - angular + coef * 1.5;

        motorControl.align(error);
    }

    ROS_INFO_STREAM("changeIntersection: 2) Keep going'til find next black ");

    motorControl.clear();
    if (coef == -1)
        linesensors.reset(false);
    else
        linesensors.reset(true);

    is_stable = 0;
    while (ros::ok())
    {
        sensors = linesensors.readLines();

        angular = sensors->at(Sides::LEFT) - sensors->at(Sides::RIGHT);
        diretional = (sensors->at(Sides::LEFT) + sensors->at(Sides::RIGHT)) / 2;

        error[Wheels::wFL] = -diretional + angular - sensors->at(Sides::FRONT) * 2;
        error[Wheels::wFR] = +diretional - angular - sensors->at(Sides::FRONT) * 2;
        error[Wheels::wBL] = +diretional + angular - sensors->at(Sides::FRONT) * 2;
        error[Wheels::wBR] = -diretional - angular - sensors->at(Sides::FRONT) * 2;

        if (fabs(sensors->at(Sides::FRONT)) < 0.3 and fabs(sensors->at(Sides::LEFT)) < 0.3 and fabs(sensors->at(Sides::RIGHT)) < 0.3)
        {
            is_stable = is_stable + 1;
            if (is_stable >= 20)
                break;
        }
        else
        {
            is_stable = 0;
        }
        motorControl.align(error);
    }
    motorControl.stop();

    ROS_INFO_STREAM("changeIntersection: 3) Success");
}

void pick_container(LineSensor &linesensors, MotorControl &motorControl, int container_number)
{
    // Go To Container
    to_container(linesensors, motorControl);

    // Pick container
    uint8_t angle_to_pick = container_number == 0 || container_number == 2 ? 0 : 160;

    linesensors.setServoPose(angle_to_pick);
    linesensors.pickContainer();
    linesensors.resetGearAndPinionPose();
    linesensors.setServoPose(90);

    container_to_intersection(linesensors, motorControl);
}

void drop_container(LineSensor &linesensors, MotorControl &motorControl, Colors containerColor, double height)
{

    to_dock(linesensors, motorControl, containerColor);

    linesensors.dropContainer(height);
    linesensors.resetGearAndPinionPose();

    dock_to_intersection(linesensors, motorControl, containerColor);
}

void functionPlanner(std::vector<boost::function<void()>> &functionList, LineSensor &linesensors, MotorControl &motorControl, std::vector<Colors> colorsToPick, std::vector<int> containersToPick, Positions initialPose)
{

    Positions position = initialPose;

    // Save all the functions that the robot should do
    //std::vector<boost::function<void()> > functionList;

    int n_green = 0;
    int n_blue = 0;
    int n_container = 0;
    double height = 0;
    int n_turns = colorsToPick.size();
    int container_number = 0;
    Positions container_pose;
    Colors color;
    for (int i = 0; i < n_turns; i++)
    {
        container_number = containersToPick[i];
        color = colorsToPick[i];

        // Change position to stay in the right intersection
        if (container_number == 0 || container_number == 1 || container_number == 5 || container_number == 6)
        {
            container_pose = Positions::GreenIntersection;
        }
        else
        {
            container_pose = Positions::BlueIntersection;
        }

        if (container_pose != position)
        {
            ROS_INFO_STREAM("Changing Pose: " << position);
            functionList.push_back(boost::bind(change_intersection, boost::ref(linesensors), boost::ref(motorControl), position));
            position = position == Positions::GreenIntersection ? Positions::BlueIntersection : Positions::GreenIntersection;
        }

        // # Go to Container | # Pick container # Go Back to Intersection
        ROS_INFO_STREAM("Picking Container  Number: " << container_number);
        functionList.push_back(boost::bind(pick_container, boost::ref(linesensors), boost::ref(motorControl), container_number));

        // # Change position to stay in the right intersection
        if (color == Colors::Green)
        {
            container_pose = Positions::BlueIntersection;
        }
        else
        {
            container_pose = Positions::GreenIntersection;
        }

        if (container_pose != position)
        {
            ROS_INFO_STREAM("Changing Pose to Deposit: " << position);
            functionList.push_back(boost::bind(change_intersection, boost::ref(linesensors), boost::ref(motorControl), position));
            position = position == Positions::GreenIntersection ? Positions::BlueIntersection : Positions::GreenIntersection;
        }

        // # Go To Dock | Deposit Container | Go back to Intersection
        if (color == Colors::Blue)
        {
            n_blue += 1;
            n_container = n_blue;
        }
        else if (color == Colors::Green)
        {
            n_green += 1;
            n_container = n_green;
        }
        else
        {
            ROS_INFO_STREAM("color of container is wrong");
            break;
        }

        switch (n_container)
        {
        case 1:
            height = linesensors.container1;
            break;
        case 2:
            height = linesensors.container2;
            break;
        case 3:
            height = linesensors.container3;
            break;
        case 4:
            height = linesensors.container4;
            break;
        case 5:
            height = linesensors.container5;
            break;
        default:
            ROS_INFO_STREAM("Default case wtf:: ?");
        }
        ROS_INFO_STREAM("Drop Container  Color: " << color << " Height: " << height);
        functionList.push_back(boost::bind(drop_container, boost::ref(linesensors), boost::ref(motorControl), color, height));
    }

    //return functionList;
}
/*
void strategyPlanner(std::vector<Colors> &container_list, Positions inital_pose, std::vector<Colors> &colorsToPick, std::vector<Positions> &containersToPick)
{

    Positions position = initial_pose;

    int green_value = 0;
    int blue_value = 0;
    Colors best_value;
    
    for (i = 0; i < 3; i++)
    {
        green_value = 0;
        blue_value = 0;
        // Checa melhor cor para pegar
        for (j = 0; j < colorsToPick.size(); j++)
        {
            if (colorsToPick[j] == Colors::Green)
            {
                green_value += 1;
            }
            else
            {
                if (colorsToPick[j] == Colors::Blue)
                    blue_value += 1;
            }
        }
        
        if (green_value > blue_value)
        {
            best_value = Colors::Green;
        }
        else
        {
            if (blue_value > green_value)
            {
                best_value = Colors::Blue;
            }
            else
            {
                if (position == Positions::GreenIntersection)
                {
                    best_value = Colors::Green;
                }
                else
                {
                    best_value = Colors::Blue;
                }
            }
        }

        // Dada a melhor cor, procurar se há algum container
    std::vector<bool> top_containers
    for(int k=0; k < container_list.size(); k++ )
    {
        if( container_list[k][0] == best_value){
            top_containers.push_back(1);
        }else{
            top_containers.push_back(0);
        }
    }
    //top_containers = np.array([ container[0]  for container in container_list[:4] ]) == best_value
}

/*
        if sum(top_containers) == 0 : # check if any is of the  containers is the best value color
            if best_value == Colors.Green:
                best_value = Colors.Blue
            else:
                best_value = Colors.Green

            top_containers = np.array([ container[0]  for container in container_list[:4] ]) == best_value    
        index = -1
        if position == Positions.GreenIntersection:
            if top_containers[0]:
                index = 0
            elif top_containers[1]:
                index = 1
            elif top_containers[2]:
                index = 2
            elif top_containers[3]:
                index = 3
        else:
            if top_containers[2]:
                index = 2
            elif top_containers[3]:
                index = 3
            elif top_containers[0]:
                index = 0
            elif top_containers[1]:
                index = 1

        if index == -1 :
            return colorsToPick

        colorsToPick.append( container_list[index][0] )
        containersToPick.append(index)

        del container_list[index][0]
        
        if not container_list[index]:
            container_list[index].append(Colors.Empty)  

        position =  Positions.GreenIntersection if colorsToPick[-1] == Colors.Green else Positions.GreenIntersection

    return colorsToPick, containersToPick
*/