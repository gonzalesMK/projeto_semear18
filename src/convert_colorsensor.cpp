#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt16.h>
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

const int LINE_TOP_LIMIT = 100; // values below are considered to be a detecting a line. MISSING CALIBRATION !!!

ros::Publisher pub[4]; // one publisher for each side of the robot - because we have one value for each 2 sensors

enum SensorsPosition
{
    B1,
    B2,
    R1,
    R2,
    L1,
    L2,
    F1,
    F2
};
enum LineSensors
{
    Back,
    Right,
    Left,
    Front
};

static std::uint16_t values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

std::uint16_t pastLineValues[4] = {0, 0, 0, 0};

void callback(const sensor_msgs::ImageConstPtr &msg, std::uint16_t &value, ros::Publisher &pub);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converColorSensor");

    ros::NodeHandle n;

    // Publishers
    ros::Publisher pubB1 = n.advertise<std_msgs::UInt16>("/lineSensorB1", 1);
    ros::Publisher pubB2 = n.advertise<std_msgs::UInt16>("/lineSensorB2", 1);
    ros::Publisher pubR1 = n.advertise<std_msgs::UInt16>("/lineSensorR1", 1);
    ros::Publisher pubR2 = n.advertise<std_msgs::UInt16>("/lineSensorR2", 1);
    ros::Publisher pubL1 = n.advertise<std_msgs::UInt16>("/lineSensorL1", 1);
    ros::Publisher pubL2 = n.advertise<std_msgs::UInt16>("/lineSensorL2", 1);
    ros::Publisher pubF1 = n.advertise<std_msgs::UInt16>("/lineSensorF1", 1);
    ros::Publisher pubF2 = n.advertise<std_msgs::UInt16>("/lineSensorF2", 1);

    // Subscribe to all the lineSensors in the base from the simulation
    ros::Subscriber subB1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorB1", 1, boost::bind(callback, _1, boost::ref(values[B1]), boost::ref(pubB1) ));
    ros::Subscriber subB2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorB2", 1, boost::bind(callback, _1, boost::ref(values[B2]), boost::ref(pubB2) ));
    ros::Subscriber subR1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorR1", 1, boost::bind(callback, _1, boost::ref(values[R1]), boost::ref(pubR1) ));
    ros::Subscriber subR2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorR2", 1, boost::bind(callback, _1, boost::ref(values[R2]), boost::ref(pubR2) ));
    ros::Subscriber subL1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorL1", 1, boost::bind(callback, _1, boost::ref(values[L1]), boost::ref(pubL1) ));
    ros::Subscriber subL2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorL2", 1, boost::bind(callback, _1, boost::ref(values[L2]), boost::ref(pubL2) ));
    ros::Subscriber subF1 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorF1", 1, boost::bind(callback, _1, boost::ref(values[F1]), boost::ref(pubF1) ));
    ros::Subscriber subF2 = n.subscribe<sensor_msgs::Image>("/AMR/lineSensorF2", 1, boost::bind(callback, _1, boost::ref(values[F2]), boost::ref(pubF2) ));

/*    
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
    ros::spin();
    return 0;
}

void callback(const sensor_msgs::ImageConstPtr &msg, std::uint16_t &value, ros::Publisher &pub)
{
    
    value = (std::uint16_t) sqrt(pow( (double) msg->data[0], 2) + pow( (double) msg->data[1], 2) + pow( (double) msg->data[2], 2));
    
    std_msgs::UInt16 data;
    data.data = (std::uint16_t)  (420.0 - (double) value) / (420.0 - 40) * 1000 ;  // We need to invert the value to mimic a phototransirtors
    
    pub.publish(data);
}
