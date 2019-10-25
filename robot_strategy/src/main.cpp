#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <boost/bind.hpp>
#include <robot_strategy/stateLib.h>
#include <pigpio.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <bits/stdc++.h> 
#include <vector>
int GPIO_PIN = 15;

std::string imageName("/media/gonzales/DATA/Semear/cad_2.JPG"); // by default

void get_containers(std::vector<Colors>& colorsToPick)
{

    // Read image
    cv::Mat src = imread(imageName, cv::IMREAD_COLOR);

    if (src.empty())
    {
        std::cerr << "No image supplied ..." << std::endl;
        return;
    }

    // Transform to HSV
    cv::cvtColor(src, src, cv::COLOR_BGR2HSV);

    // Get only H
    std::vector<cv::Mat> hsv_planes;

    // We need to do this for each container:
    // Container Esquerda 0 (Baixo)
    cv::split(src(cv::Range(100, 200), cv::Range(100, 200)), hsv_planes);

    double sum1 = cv::sum(hsv_planes[0])[0] / (100 * 100);

    cv::namedWindow("Cropped Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Cropped Image", 600, 600);
    cv::imshow("Cropped Image", src(cv::Range(100, 200), cv::Range(100, 200)));
    ROS_INFO_STREAM("HSV: " << sum1);
    cv::waitKey(0);
    // Container Esquerda 1

    // Container Esquerda 2
    // Container Esquerda 3

    // Container Direita 0 (Baixo)
    // Container Direita 1
    // Container Direita 2
    // Container Direita 3
}
std::string GetStdoutFromCommand(std::string cmd)
{
    std::string data;
    FILE *stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream)
    {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL)
                data.append(buffer);
        pclose(stream);
    }
    return data;
}

void callback(int gpio, int level, uint32_t tick, void *user){
    int *t = (int *) user;

    if(gpio == GPIO_PIN){
            (*t) = (int)level;
            ROS_INFO_STREAM("Changing Button to level " << level);
            if( (*t) == 1 && level == 0  ){
                ROS_INFO_STREAM("Estava ativado e foi desligado... matando o nó");
                system("rosnode kill /main"); 
            }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testarUtils");
    ros::NodeHandle node;

    if (gpioInitialise() < 0)
    return 1;

    LineSensor linesensor;
    MotorControl motorControl;

    int* start = new int;
    (*start) = 0;

    /* Isso está funcionando
    gpioSetMode(GPIO_PIN, PI_INPUT);
    ros::Duration(5).sleep();
    ros::Duration(10).sleep();
    gpioSetAlertFuncEx(GPIO_PIN, callback, start); // isso precisa testar    

    while (!start and ros::ok())
    {
        start = gpioRead(GPIO_PIN);
        ros::Rate(.5).sleep();
    }
     */
   
    /*
    firstPose(linesensor, motorControl);

    std::vector<Colors> colorsToPick;
    std::vector<Colors> containersToPick;
    colorsToPick.append(Colors::Green);
    colorsToPick.append(Colors::Blue);

    // Take Picture and process

    // Change side

    // Take Picture and process
    to_container(linesensor, motorControl);
*/
    /*
    //std::vector<boost::function<void()>> planner;
    functionPlanner(planner, linesensors, motorControl, colorsToPick, containersToPick, initialPose);
    
    for(int i =0; i < planner.size(); i++){
        planner[i]();
    }
    */
    return 0;
}
