#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/ArduinoRGB.h>
#include <std_msgs/ColorRGBA.h>
#include <projeto_semear/Infra_Placa_Sensores.h>

/* Subscribers e Publishers para os Infra */
ros::Subscriber infra_elevadores_sub;
ros::Publisher linesensor_D3;
ros::Publisher linesensor_D2;
ros::Publisher linesensor_D1;
ros::Publisher linesensor_D0;


/*  RGB   */
ros::Publisher pub_RGB; 

ros::Publisher enable_RGB_pub;
std_msgs::Bool enable_rgb;

/* Interface para os Infravermelhos, podem ser até 6 */
void infra_elevadores_callback(const projeto_semear::Infra_Placa_SensoresConstPtr &msg)
{
    std_msgs::UInt16 lineBBR;
    std_msgs::UInt16 lineBFR;
    std_msgs::UInt16 lineFBR;
    std_msgs::UInt16 lineFFR;

    lineBBR.data = msg->infraBBR;
    lineBFR.data = msg->infraBFR;
    lineFBR.data = msg->infraFBR;
    lineFFR.data = msg->infraFFR;

    linesensor_D3.publish(lineBBR);
    linesensor_D2.publish(lineBFR);
    linesensor_D1.publish(lineFBR);
    linesensor_D0.publish(lineFFR);
}

/* FALTA IMPLEMENTAR O ENABLE RGB NO CÓDIGO */
void enable_rgb_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enable_rgb.data = msg->data;
}

void rgb_callback(const projeto_semear::ArduinoRGBConstPtr &msg){

    std_msgs::ColorRGBA message;
    message.r = msg->red;
    message.g = msg->green;
    message.b = msg->blue;

    pub_RGB.publish(message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Interface_Arduino_Elevadores");

    ros::NodeHandle nh;

    /* Interface Infras */
    linesensor_D3 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_D0", 5);  // Infras
    linesensor_D2 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_D1", 5);
    linesensor_D1 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_D2", 5);
    linesensor_D0 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_D3", 5);
    infra_elevadores_sub = nh.subscribe<projeto_semear::Infra_Placa_Sensores>("/AMR/arduinoSensoresRGBInfras", 5, infra_elevadores_callback);
       
    /* RGB */
    ros::Subscriber sensor_rgb_sub = nh.subscribe<projeto_semear::ArduinoRGB>("/AMR/arduinoRGB", 5 , rgb_callback);  // RGB 
    pub_RGB = nh.advertise<std_msgs::ColorRGBA>("/AMR/RGB", 5); 
    enable_RGB_pub = nh.advertise<std_msgs::Bool>("/AMR/enableRGB", 5);

    enable_rgb.data = false;

    double FREQUENCIA_ARDUINO = 20;
    if (!nh.param("FREQUENCIA_ARDUINO_ELEVADORES", FREQUENCIA_ARDUINO, 20.0))
    {
        ROS_ERROR("Failed to get param 'FREQUENCIA_ARDUINO_ELEVADORES'");
    }
    ros::Rate rate(FREQUENCIA_ARDUINO);

    while (ros::ok())
    {
        ros::spinOnce();

        enable_RGB_pub.publish(enable_rgb);
        
        rate.sleep();
    }
}
