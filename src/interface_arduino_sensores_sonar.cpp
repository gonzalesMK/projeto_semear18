#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <projeto_semear/Sonar_Infra_Placa_Sensores.h>

/* Subscribers e Publishers para os Infra */
ros::Subscriber infra_elevadores_sub;
ros::Publisher linesensor_E3;
ros::Publisher linesensor_E2;
ros::Publisher linesensor_E1;
ros::Publisher linesensor_E0;

/* Sonar */
ros::Publisher pub_sonar;

/* Interface para Infra e Sonar */
void infra_elevadores_callback(const projeto_semear::Sonar_Infra_Placa_SensoresConstPtr &msg)
{
    std_msgs::UInt16 lineBBL;
    std_msgs::UInt16 lineBFL;
    std_msgs::UInt16 lineFBL;
    std_msgs::UInt16 lineFFL;
    std_msgs::UInt32 sonar;

    lineBBL.data = msg->infraBBL;
    lineBFL.data = msg->infraBFL;
    lineFBL.data = msg->infraFBL;
    lineFFL.data = msg->infraFFL;
    sonar.data = msg->sonar;

    linesensor_E3.publish(lineBBL);
    linesensor_E2.publish(lineBFL);
    linesensor_E1.publish(lineFBL);
    linesensor_E0.publish(lineFFL);
    pub_sonar.publish(sonar);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Interface_Arduino_Elevadores");

    ros::NodeHandle nh;

    /* Interface Infras */
    linesensor_E3 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_E0", 5); // Infras
    linesensor_E2 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_E1", 5);
    linesensor_E1 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_E2", 5);
    linesensor_E0 = nh.advertise<std_msgs::UInt16>("/AMR/linesensor_E3", 5);
    pub_sonar = nh.advertise<std_msgs::UInt32>("/AMR/sonar", 5); // sonar
    infra_elevadores_sub = nh.subscribe<projeto_semear::Sonar_Infra_Placa_Sensores>("/AMR/arduinoSensoresUltrassom", 5, infra_elevadores_callback);
    ros::spin();
}
