#include <projeto_semear/kine_control.h>
#include <std_msgs/Float64.h>


double VelE=0;
double VelD=0;

void PID_E_callback(const std_msgs::Float64ConstPtr msg){
    VelE = msg->data;
}
void PID_D_callback(const std_msgs::Float64ConstPtr msg){
    VelD = msg->data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kine_control_demo");

    ros::NodeHandle nh;

    kineControl::robot robot;

    ros::Publisher pubPID_E_state = nh.advertise<std_msgs::Float64>("/PIDL/state", 1);
    ros::Publisher pubPID_D_state = nh.advertise<std_msgs::Float64>("/PIDR/state", 1);

    ros::Publisher pubPID_set = nh.advertise<std_msgs::Float64>("/setpoint", 1);


    ros::Subscriber subPID_E = nh.subscribe<std_msgs::Float64>("/PIDL/control_effort", 1, PID_E_callback);
    ros::Subscriber subPID_D = nh.subscribe<std_msgs::Float64>("/PIDR/control_effort", 1, PID_D_callback);

    std_msgs::Float64 erro1;
    std_msgs::Float64 erro2;
    std_msgs::Float64 setpoint;

    setpoint.data = 0;
    pubPID_set.publish(setpoint);
    
    erro1.data = 2;
    erro2.data = 2;

    ros::Rate rate(20);

    while (ros::ok())
    {
        erro1.data = kineControl::erro_sensores_E2E3(robot, erro1.data);
        erro2.data = kineControl::erro_sensores_D2D3(robot, erro2.data);
        
        ROS_INFO_STREAM("ERRO ESQUERDA: " << erro1.data << " ERRO DIREITA: " << erro2);
        
        pubPID_E_state.publish(erro1);
        pubPID_D_state.publish(erro2);

        ros::spinOnce();
        ROS_INFO_STREAM("VELO ESQUERDA: " << VelE << " VELO DIREITA: " << VelD);
        
        robot.setVelocityPID(VelE, VelD);
        
        rate.sleep();
        pubPID_set.publish(setpoint);
    }

    //pubPID_e

    return 0;
}