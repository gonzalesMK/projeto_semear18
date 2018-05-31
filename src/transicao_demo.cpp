#include <ros/ros.h>
#include <projeto_semear/kine_control.h>
#include <std_msgs/Float32.h>



// Code that takes the robot from de Central to the Right
void callbackBL(const std_msgs::Float32ConstPtr &msg);
void callbackBR(const std_msgs::Float32ConstPtr &msg);
void callbackFL(const std_msgs::Float32ConstPtr &msg);
void callbackFR(const std_msgs::Float32ConstPtr &msg);

// Variáveis para armazenar os valores dos sensores
double colorBL = -1;
double colorBR = -1;
double colorFL = -1;
double colorFR = -1;

const double MAIOR_QUE_PRETO = 60;  // Constante para marcar o valor do preto
const double MAIOR_QUE_VERDE = 300; // Constate para marcar o valor do verde
const double MENOR_QUE_VERDe = 100; //  ""  ""
const double VEL_ANG = 0.1;         // Constante para marcar a velocidade angular

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicaoDemo");

    ros::NodeHandle nh;

    kineControl::motorControl motor;

    ros::Subscriber lineSensorBL = nh.subscribe("/image_converter/lineSensorBL", 1, callbackBL);
    ros::Subscriber lineSensorBR = nh.subscribe("/image_converter/lineSensorBR", 1, callbackBR);
    ros::Subscriber lineSensorFL = nh.subscribe("/image_converter/lineSensorFL", 1, callbackFL);
    ros::Subscriber lineSensorFR = nh.subscribe("/image_converter/lineSensorFR", 1, callbackFR);

    while (colorBR == -1 || colorFL == -1 || colorBL == -1 || colorFR == -1)
    {
        ROS_INFO("Waiting 4 Topics");
        ros::spinOnce(); // It is importante to get topics callback
        ros::Duration(0.5).sleep();
    }

    // condição de não alinhamento:
    ros::Duration time(0.05);
    geometry_msgs::Twist velocidade;
    int code = 0;
    
    while ((colorBL > MAIOR_QUE_VERDE || colorBR > MAIOR_QUE_VERDE || colorFR > MAIOR_QUE_PRETO || colorFL > MAIOR_QUE_PRETO) && nh.ok())
    {

        int code = 0;
        velocidade.linear.x = 0;
        velocidade.linear.y = 0;
        velocidade.angular.z = 0;

        // Caso 0: todos os sensores no branco. Supõe-se que o robô ultrapassou o alinhamento necessário. Garantir isso no resto do código
        // Caso 1: Caso o sensor BackRight esteja marcando verde, mas o BackLeft não -> girar positivo
        // Caso 2: Caso o sensor BackLeft esteja marcando verde, mas o BackRight não -> girar negativo
        // Caso 3: Caso o sensor FrontRight esteja marcando verde, mas o FrontLeft não -> girar positivo
        // Caso 4: Caso o sensor FrontLeft esteja marcando verde, mas o  FrontRight não -> girar negativo
        ROS_INFO_STREAM("\nFL " << colorFL << "FR " << colorFR << "\nBL " << colorBL << "BR " << colorBR );
        
        if( colorBL > MAIOR_QUE_VERDE && colorBR > MAIOR_QUE_VERDE && colorFL > MAIOR_QUE_VERDE && colorFR > MAIOR_QUE_VERDE)
            code = 0;
        else if (colorBL > MAIOR_QUE_VERDE && colorBR < MAIOR_QUE_VERDE)
            code = 1;
        else if (colorBL < MAIOR_QUE_VERDE && colorBR > MAIOR_QUE_VERDE)
            code = 2;
        else if (colorFL > MAIOR_QUE_PRETO && colorFR < MAIOR_QUE_PRETO)
            code = 3;
        else if (colorFL < MAIOR_QUE_PRETO && colorFR > MAIOR_QUE_PRETO)
            code = 4;

        ROS_INFO_STREAM("Case: " << code);

        switch (code)
        {
        case 0:
            velocidade.linear.x = -0.05;
        case 1:
            velocidade.angular.z = - VEL_ANG;
            break;
        case 2:
            velocidade.angular.z = VEL_ANG;
            break;
        case 3:
            velocidade.angular.z = - VEL_ANG;
            break;
        case 4:
            velocidade.angular.z = VEL_ANG;
            break;
        }
        
        motor.setVelocity(velocidade);
        time.sleep();
        ros::spinOnce();
    }

    ROS_INFO("STARTING TRANSITION");
    // Vá para direita até o sensor da direita atingir o fita preta
    velocidade.linear.x = 0;
    velocidade.linear.y = 0.1;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);

    // Alinhar os sensores fora da linha preta e em cima da linha verde
/*
    //O sensor da direita toca a linha preta
    while (colorFR > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // O sensor da esquerda toca a linha preta
    while (colorFL > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    //O sensor da esquerda toca a linha preta
    while (colorFL > 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // O sensor da esquerda sai da linha preta
    while (colorFL < 60 && nh.ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
*/
    // Quinto, andar uma distância predefinida
    ros::Duration(3).sleep();

    velocidade.linear.x = 0;
    velocidade.linear.y = 0;
    velocidade.angular.z = 0;

    motor.setVelocity(velocidade);

    return 0;
}

void callbackBL(const std_msgs::Float32ConstPtr &msg)
{
    colorBL = msg->data;
}

void callbackBR(const std_msgs::Float32ConstPtr &msg)
{
    colorBR = msg->data;
}

void callbackFL(const std_msgs::Float32ConstPtr &msg)
{
    colorFL = msg->data;
}

void callbackFR(const std_msgs::Float32ConstPtr &msg)
{
    colorFR = msg->data;
}
