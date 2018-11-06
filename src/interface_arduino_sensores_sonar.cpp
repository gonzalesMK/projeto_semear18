#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <projeto_semear/Sonar_Infra_Placa_Sensores.h>

/* Subscribers e Publishers para os Infra */
ros::Subscriber infra_elevadores_sub;
ros::Publisher linesensor_E0;
ros::Publisher linesensor_E1;
ros::Publisher linesensor_D0;
ros::Publisher linesensor_D1;

/* Sonar */
ros::Publisher pub_sonar;

/* Interface para Infra e Sonar */
void infra_elevadores_callback(const projeto_semear::Sonar_Infra_Placa_SensoresConstPtr &msg)
{
    std_msgs::UInt16 lineBL;
    std_msgs::UInt16 lineBR;
    std_msgs::UInt16 lineFR;
    std_msgs::UInt16 lineFL;
    std_msgs::UInt16 sonar;

    lineBL.data = msg->infraBL;
    lineBR.data = msg->infraBR;
    lineFR.data = msg->infraFR;
    lineFL.data = msg->infraFL;
    sonar.data = msg->sonar;

    linesensor_E0.publish(lineBL);
    linesensor_E1.publish(lineFL);
    linesensor_D0.publish(lineBR);
    linesensor_D1.publish(lineFR);
    pub_sonar.publish(sonar);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Interface_Arduino_Elevadores");

    ros::NodeHandle nh;

    /* Interface Infras */
    linesensor_E0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE0", 5); // Infras
    linesensor_E1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE1", 5);
    linesensor_D0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD0", 5);
    linesensor_D1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD1", 5);
    pub_sonar = nh.advertise<std_msgs::UInt16>("/AMR/sonar", 5); // sonar
    infra_elevadores_sub = nh.subscribe<projeto_semear::Sonar_Infra_Placa_Sensores>("/AMR/arduinoSensoresSonarInfras", 5, infra_elevadores_callback);
    ros::spin();
}
