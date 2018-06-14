//Ação responsável por levar o robô da posição inicial até a linha preta de referência

/*Como a posição inicial é definida,e pode escolher como o robo vai começar o round, 
considera-se que ele já esteja com a frente voltada para a linha preta (como na simulação)*/
#include <ros/ros.h>
#include <projeto_semear/kine_control.h>
#include <std_msgs/Float32.h>

void callbackFL(const std_msgs::Float32ConstPtr &msg);
void callbackFR(const std_msgs::Float32ConstPtr &msg);
 
// Variáveis para armazenar os valores dos sensores
double colorFL = -1;
double colorFR = -1;
geometry_msgs::Twist velocidade;

const double MAIOR_QUE_PRETO = 60;  // Constante para marcar o valor do preto
const double VEL_ANG = 0.1;         // Constante para marcar a velocidade angular

int main(int argc, char **argv){
    ros::init(argc, argv, "linhapreta");

    ros::NodeHandle nh;

    kineControl::robot motor;

    ros::Subscriber lineSensorFL = nh.subscribe("/image_converter/lineSensorFL", 1, callbackFL);
    ros::Subscriber lineSensorFR = nh.subscribe("/image_converter/lineSensorFR", 1, callbackFR);

    while (colorFL == -1 || colorFR == -1){
        //se entrar nesse while, significa que não está recebendo as mensagens do convert
        ROS_INFO("rosrun no convert, por favor");
        ros::spinOnce(); 
        ros::Duration(0.5).sleep();
    }

    ros::Duration time(0.05);
    geometry_msgs::Twist velocidade;
    int code = 0;

    ros::Time begin = ros::Time::now();        
    while ((colorFR > MAIOR_QUE_PRETO || colorFL > MAIOR_QUE_PRETO)){
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;
   
        //Caso 0: Nenhum dos sensores na faixa pretav -> anda para frente
        //Caso 1: Sensor FrontRight no preto e FrontLeft não -> girar para direita
        //Caso 2: Sensor FrontLeft no preto e FrontRight não -> girar para esquerda
        ROS_INFO_STREAM("\nFL " << colorFL << " FR " << colorFR);
        if(colorFL > MAIOR_QUE_PRETO && colorFR > MAIOR_QUE_PRETO)
            code = 0;
        else if (colorFL > MAIOR_QUE_PRETO && colorFR < MAIOR_QUE_PRETO)
            code = 1;
        else if (colorFL < MAIOR_QUE_PRETO && colorFR > MAIOR_QUE_PRETO)
            code = 2;
        ROS_INFO_STREAM("Case: " << code);     
        switch(code){
        case 0:
            velocidade.linear.x = 0.05;
            break;
        case 1:
            velocidade.angular.z = - VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
            break;
        }
        
        motor.setVelocity(velocidade);
        time.sleep();
        ros::spinOnce();
    }
    ros::Time end = ros::Time::now();
    float distance = (end.sec - begin.sec)*0.05;
    //ros::Duration(3).sleep();
    ROS_INFO_STREAM("distancia percorrida " << distance);
    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);
    return 0;
}


void callbackFL(const std_msgs::Float32ConstPtr &msg){
    colorFL = msg->data;
}

void callbackFR(const std_msgs::Float32ConstPtr &msg){
    colorFR = msg->data;
}
