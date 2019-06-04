#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <cmath>

using namespace cv;
using namespace std;

/** This code converts the sensors value from sensor_msgs::Image to std_msgs::UInt16. Besides convertion to the scale 0..1000 ( where black is 1000 and white is 0), we do some calculations 
 * to get the same results as the Pololu Arduino library for QTR sensors ( we are trying to mimic the function readLineBlack)
 */

const int LINE_TOP_LIMIT = 320; // values below are considered to be a detecting a line. MISSING CALIBRATION !!!

enum SensorsPosition
{
    L1,
    L2,
    R1,
    R2,
    F1,
    F2,
    B1,
    B2

};
enum LineSensors
{
    Left,
    Right,
    Front,
    Back
};

static bool values[8] = {0, 0, 0, 0, 0, 0, 0, 0};


void callback(const sensor_msgs::ImageConstPtr &msg, bool &value);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converColorSensor");

    ros::NodeHandle n;

    // Publishers
    ros::Publisher pubPololu = n.advertise<std_msgs::UInt8>("/pololuSensor", 1);

    // Subscribe to all the lineSensors in the base from the simulation
    ros::Subscriber subB1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorB1", 1, boost::bind(callback, _1, boost::ref(values[B1])));
    ros::Subscriber subB2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorB2", 1, boost::bind(callback, _1, boost::ref(values[B2])));
    ros::Subscriber subR1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorR1", 1, boost::bind(callback, _1, boost::ref(values[R1])));
    ros::Subscriber subR2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorR2", 1, boost::bind(callback, _1, boost::ref(values[R2])));
    ros::Subscriber subL1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorL1", 1, boost::bind(callback, _1, boost::ref(values[L1])));
    ros::Subscriber subL2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorL2", 1, boost::bind(callback, _1, boost::ref(values[L2])));
    ros::Subscriber subF1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorF1", 1, boost::bind(callback, _1, boost::ref(values[F1])));
    ros::Subscriber subF2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorF2", 1, boost::bind(callback, _1, boost::ref(values[F2])));

    std_msgs::UInt8 msg;
    ros::Rate rate(100);

    while( ros::ok() ){
        msg.data = 0 ;
        
        for(int i = 0; i < 8; i++){
            msg.data |= (values[i] << i); // 1 if is black
        }
        pubPololu.publish(msg);   
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

void callback(const sensor_msgs::ImageConstPtr &msg, bool &value)
{

    value = ( sqrt(pow((double)msg->data[0], 2) + pow((double)msg->data[1], 2) + pow((double)msg->data[2], 2))) < LINE_TOP_LIMIT ; // is black

}

/*    Pololu Code
    // Convert the sensors to the right measure and publish it - we are using the same values as the Pololu library
    pub[Back] = n.advertise<std_msgs::UInt16>("/lineSensorB_LinePosition", 1);
    pub[Right] = n.advertise<std_msgs::UInt16>("/lineSensorR_LinePosition", 1);
    pub[Left] = n.advertise<std_msgs::UInt16>("/lineSensorL_LinePosition", 1);
    pub[Front] = n.advertise<std_msgs::UInt16>("/lineSensorF_LinePosition", 1);
    
    ros::Rate rate(30);
    bool onLine = false;
    std::uint32_t avg = 0, sum = 0, lastPosition;
    std_msgs::UInt16 msg;
    while (ros::ok())
    {

        for (int j = 0; j < 4; j++)
        {
            onLine = false;
            avg = 0;
            sum = 0;

            for (int i = 0; i < 2; i++)
            {
                if (values[i + j * 2] > LINE_TOP_LIMIT)
                    onLine = true;

                avg += (uint32_t) values[i + j * 2] * (i * 1000);
                sum += (uint32_t) values[i + j * 2];
                
            }
            
            if(sum ==0) avg = 0;
            else avg = avg / sum;

            if (!onLine)
            {
                if (pastLineValues[j] < (2 - 1) * 1000 / 2)
                    msg.data = 0;
                else
                    msg.data = (2 - 1) * 1000;

                pub[j].publish(msg);

                continue;
            }
            
            msg.data = avg;
            pastLineValues[j] = avg;
            pub[j].publish(msg);

        }


        ros::spinOnce();
        rate.sleep();
        //ROS_INFO_STREAM( (int) values[0] << (int) values[1]);
    }
    */