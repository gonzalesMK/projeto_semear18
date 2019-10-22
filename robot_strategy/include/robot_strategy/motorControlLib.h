#include <ros/ros.h>



class MotorControl{


    public:

    double Kp;
    double Kd;
    double pid_on;
    double freq;
    double momentum;
    double Ki;
    double windUp;
    double deadSpace;
    
    bool _velocity_mode = false;
    bool _align_mode = false;
    bool _pid_on = false;
    
    double deltaError = {0, 0, 0 ,0};
    ros::Publisher pub_motorFL;
    ros::Publisher pub_motorFR;
    ros::Publisher pub_motorBL;
    ros::Publisher pub_motorBR;
    ros::Publisher pub_encoderEnable;
    ros::Publisher pub_motorLineFL;
    ros::Publisher pub_motorLineFR;
    ros::Publisher pub_motorLineBL;
    ros::Publisher pub_motorLineBR;
    ros::Publisher pub_motorPWM_FL;
    ros::Publisher pub_motorPWM_FR;
    ros::Publisher pub_motorPWM_BL;
    ros::Publisher pub_motorPWM_BR;
    ros::Publisher pub_lineEnable;
    ros::Publisher pub_lineTarget;

    ros::Subscribersub_lineEnable;

    MotorControl(ros::NodeHandle &node, double Kp=0.0, double Kd=0.0, double Ki = 0, double windUp = 0, double deadSpace = 15, double freq=100.0, double momentum=0)

};

