#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/ArduinoRGB.h>
#include <std_msgs/ColorRGBA.h>
#include <projeto_semear/Infra_Placa_Sensores.h>

/* Subscribers e Publishers para os Infra */
ros::Subscriber infra_elevadores_sub;
ros::Publisher linesensor_E0;
ros::Publisher linesensor_E1;
ros::Publisher linesensor_D0;
ros::Publisher linesensor_D1;


/*  RGB   */
ros::Publisher pub_RGB; 

ros::Publisher enable_RGB_pub;
std_msgs::Bool enable_rgb;

/* Interface para os Infravermelhos, podem ser até 6 */
void infra_elevadores_callback(const projeto_semear::Infra_Placa_SensoresConstPtr &msg)
{
    std_msgs::UInt16 lineBL;
    std_msgs::UInt16 lineBR;
    std_msgs::UInt16 lineFR;
    std_msgs::UInt16 lineFL;

    lineBL.data = msg->infraBL;
    lineBR.data = msg->infraBR;
    lineFR.data = msg->infraFR;
    lineFL.data = msg->infraFL;

    linesensor_E0.publish(lineBL);
    linesensor_E1.publish(lineFL);
    linesensor_D0.publish(lineBR);
    linesensor_D1.publish(lineFR);
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
    linesensor_E0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE0", 5);  // Infras
    linesensor_E1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE1", 5);
    linesensor_D0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD0", 5);
    linesensor_D1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD1", 5);
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
