#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <projeto_semear/Vel.h>
#include <projeto_semear/Enable_Placa_Elevadores.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <projeto_semear/ServoPose.h>

/* Subscribers e Publishers para o Eletroima (Rele) */
ros::Publisher arduino_eletro_enable_pub;
ros::Subscriber enable_eletro_ros_sub;

/* Publisher para os enables */
ros::Publisher enables_arduinos_pub;

projeto_semear::Enable_Placa_Elevadores enables_msg;

/* Interface ROS -> Eletroima */
void enable_eletro_ros_callback(const std_msgs::BoolConstPtr &msg)
{
    enables_msg.enable_rele = msg->data;
}

/* FALTA IMPLEMENTAR O ENABLE SERVO NO CÓDIGO */
void servo_ros_callback(const projeto_semear::ServoPoseConstPtr &msg)
{
    enables_msg.enable_servo = msg->enable;
    enables_msg.servo_pwm = msg->pwm;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Interface_Arduino_Elevadores");

    ros::NodeHandle nh;

    /* Interface dos Motores e do Servo */
        // código no eletroima_control_serve
        
    /* Relés */
    enable_eletro_ros_sub = nh.subscribe<std_msgs::Bool>("/AMR/activateEletroima", 1, enable_eletro_ros_callback);
    
    /* Enables */
    enables_arduinos_pub = nh.advertise<projeto_semear::Enable_Placa_Elevadores>("/AMR/enableElevadores", 1);
    ros::Subscriber servo_enable_sub = nh.subscribe<projeto_semear::ServoPose>("/AMR/servoPwm", 1 , servo_ros_callback);  

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
