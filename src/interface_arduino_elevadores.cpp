#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <projeto_semear/Vel.h>
#include <projeto_semear/Infra_Placa_Elevadores.h>
#include <projeto_semear/Enable_Placa_Elevadores.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/ArduinoRGB.h>
#include <std_msgs/ColorRGBA.h>
#include <projeto_semear/ServoPose.h>

/* Subscribers e Publishers para os Infra */
ros::Subscriber infra_elevadores_sub;
ros::Publisher linesensor_E0;
ros::Publisher linesensor_E1;
ros::Publisher linesensor_D0;
ros::Publisher linesensor_D1;

/* Subscribers e Publishers para o Eletroima (Rele) */
ros::Publisher arduino_eletro_enable_pub;
ros::Subscriber enable_eletro_ros_sub;

/* Publisher para os enables */
ros::Publisher enables_arduinos_pub;

projeto_semear::Enable_Placa_Elevadores enables_msg;

/* Publishers RGB */
ros::Publisher pubR_RGB;


/* Interface para os Infravermelhos */
void infra_elevadores_callback(const projeto_semear::Infra_Placa_ElevadoresConstPtr &msg)
{

    std_msgs::Float32 lineBL;
    std_msgs::Float32 lineBR;
    std_msgs::Float32 lineFR;
    std_msgs::Float32 lineFL;

    lineBL.data = msg->infraBL;
    lineBR.data = msg->infraBR;
    lineFR.data = msg->infraFR;
    lineFL.data = msg->infraFL;

    linesensor_E0.publish(lineBL);
    linesensor_E1.publish(lineFL);
    linesensor_D0.publish(lineBR);
    linesensor_D1.publish(lineFR);
}

/* Interface ROS -> Eletroima */
void enable_eletro_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enables_msg.enable_rele = msg->data;
}

/* FALTA IMPLEMENTAR O ENABLE RGB NO CÓDIGO */
void enable_rgb_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enables_msg.enable_rgb = msg->data;
}

/* FALTA IMPLEMENTAR O ENABLE SERVO NO CÓDIGO */
void servo_ros_callback(const projeto_semear::ServoPoseConstPtr &msg)
{
    enables_msg.enable_servo = msg->enable;
    enables_msg.servo_pwm = msg->pwm;
}

/* FALTA IMPLEMENTAR O ENABLE INFRA NO CÓDIGO */
void enable_infra_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enables_msg.enable_infra = msg->data;
}

/* FALTA IMPLEMENTAR O ENABLE Motores dos Elevadores NO CÓDIGO */
void enable_motores_elevadores_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enables_msg.enable_motor = msg->data;
}


void rgb_callback(const projeto_semear::ArduinoRGBConstPtr &msg){

    std_msgs::ColorRGBA message;
    message.r = msg->red;
    message.g = msg->green;
    message.b = msg->blue;

    pubR_RGB.publish(message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Interface_Arduino_Elevadores");

    ros::NodeHandle nh;

    /* Interface Infras */
    linesensor_E0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE0", 1);  // Infras
    linesensor_E1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorE1", 1);
    linesensor_D0 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD0", 1);
    linesensor_D1 = nh.advertise<std_msgs::UInt16>("/AMR/lineSensorD1", 1);
    infra_elevadores_sub = nh.subscribe<projeto_semear::Infra_Placa_Elevadores>("/AMR/arduinoElevadoresInfras", 1, infra_elevadores_callback);

    /* Interface dos Motores e do Servo */
        // código no eletroima_control_serve
        
    /* RGB */
    ros::Subscriber sensor_rgb_sub = nh.subscribe<projeto_semear::ArduinoRGB>("/AMR/arduinoElevadoresRGB", 1 , rgb_callback);  // RGB 
    pubR_RGB = nh.advertise<std_msgs::ColorRGBA>("/AMR/sensorGarraR", 1); // A escolha do RGB ser da garra direita é arbitrário

    /* Relés */
    enable_eletro_ros_sub = nh.subscribe<std_msgs::Bool>("/AMR/activateEletroima", 1, enable_eletro_ros_callback);
    
    /* Enables */
    enables_arduinos_pub = nh.advertise<projeto_semear::Enable_Placa_Elevadores>("/AMR/enableElevadores", 1);
    ros::Subscriber servo_enable_sub = nh.subscribe<projeto_semear::ServoPose>("/AMR/servoPwm", 1 , servo_ros_callback);  
    ros::Subscriber motor_enable_sub = nh.subscribe<std_msgs::Bool>("/AMR/enableMotoresElevador", 1 , enable_motores_elevadores_ros_callback);  

    enables_msg.enable_motor = false;
    enables_msg.enable_infra = false;
    enables_msg.enable_rele = false;
    enables_msg.enable_servo = false;
    enables_msg.servo_pwm = false;
    enables_msg.enable_rgb = false;

    double FREQUENCIA_ARDUINO = 20;
    if (!nh.param("FREQUENCIA_ARDUINO_ELEVADORES", FREQUENCIA_ARDUINO, 20.0))
    {
        ROS_ERROR("Failed to get param 'FREQUENCIA_ARDUINO_ELEVADORES'");
    }
    ros::Rate rate(FREQUENCIA_ARDUINO);

    while (ros::ok())
    {
        ros::spinOnce();

        enables_arduinos_pub.publish(enables_msg);


        rate.sleep();
    }
}
