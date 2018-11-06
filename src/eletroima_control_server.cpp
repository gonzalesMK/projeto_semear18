#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/constants/constants.hpp>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/Vel_Elevadores.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

/** Esse código é responsável por controlar o Eletroima do robô no V-REP
 * 
 * Mais especificamente, dentro do V-REP, há apenas a função para colocar a posição do
 * eletroima no mundo. Assim, para haver um movimento suave e linear, é preciso
 * controlar a velocidade e ir mudando a posição aos poucos
 * 
 * Embora a mensagem de ROS para posição seja geometry_msgs/Pose, ela é meio complexa por causa
 * do quartenion. Assim, por simplicidade, a posição também é controlada usando geometry_msgs/Twist.
 * 
 * A goal é o deslocamento do eletroima em relação AO ROBÔ
 * 
 * Para se comunicar com esse nó, ver exemplo no código: eletroima_control_demo
 * */

// Constants
const double FREQUENCIA = 10;                                            // Hertz
const double VEL_X = 0.05 / FREQUENCIA;                                  // metros/segundo
const double VEL_Y = 0.05 / FREQUENCIA;                                  // metros/segundo
const double VEL_Z = 0.05 / FREQUENCIA;                                  // metros/segundo
const double W = boost::math::constants::pi<double>() / 10 / FREQUENCIA; // Rad/ segundo
const double PI = boost::math::constants::pi<double>();
const double K = 0.1;

double posicao_V = 0;
double posicao_H = 0;

double posicao_Servo = 0;

// Precisa de uma função para inicializar as cremalheiras
// Quando alguma interrupção dos botões for atingida, a variável de estado deve ser Zerada.

ros::Publisher eletro_twist;
ros::Publisher servo_pub;
ros::Publisher enable_motor_pub;
/* Subscrivers e Publishers para os Motores */
ros::Subscriber controlV_sub;
ros::Subscriber controlH_sub;
ros::Publisher stateV_pub; // publica o Estado (Posição)
ros::Publisher stateH_pub; // publica o Estado (Posição)

ros::Publisher H_Motor_pub; // publica o setPoint
ros::Publisher V_Motor_pub; // publica o setPoint

ros::Publisher vel_pub; // Publica a velocidade requerida para o arduino
projeto_semear::Vel_Elevadores vel_msg;


double ERRO_POSICAO_MOTOR=0.01;

/* Falta os enables dos motores **/
// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
    //ROS_INFO_STREAM("Distance to Goal" << feedback->distance);
}

// Função executada quando a tarefa termina
void doneCb(const actionlib ::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
    //ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
    //ROS_INFO("Goal just went active");
}

/* Envia o estado (Posição) de cado motor para o tópico do PID */
void state_callback(const projeto_semear::Vel_ElevadoresConstPtr &msg)
{
    std_msgs::Float64 horizontal;
    std_msgs::Float64 vertical;

    horizontal.data = msg->CremalheiraHorizontal;
    vertical.data = msg->CremalheiraVertical;

    posicao_H = msg->CremalheiraHorizontal;
    posicao_V = msg->CremalheiraVertical;

    stateV_pub.publish(vertical);
    stateH_pub.publish(horizontal);
}

/* Recebe a resposta do PID e coloca na mensagem a ser enviada para o Arduino */
void controlH_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.CremalheiraHorizontal = msg->data;
}
void controlV_callback(const std_msgs::Float64ConstPtr &msg)
{
    vel_msg.CremalheiraVertical = msg->data;
}

void move_eletroima(const projeto_semear::moveEletroimaGoalConstPtr &goal, actionlib::SimpleActionServer<projeto_semear::moveEletroimaAction> *as, tf::TransformListener &listener)
{
    std_msgs::Float64 Hmotor_pose_msg;
    std_msgs::Float64 Vmotor_pose_msg;

    // Distância a ser percorrida em cada direção
    Hmotor_pose_msg.data = goal->deslocamento.linear.y + posicao_H;
    Vmotor_pose_msg.data = goal->deslocamento.linear.z + posicao_V;

    // Mensagem para enviar feedback
    projeto_semear::moveEletroimaFeedback feedback;
    
    // Ligar enable do motor e do Servo
    std_msgs::Int16 servo_msg;
    
    servo_msg.data = goal->deslocamento.angular.z;
    servo_pub.publish(servo_msg);

    // Enviar SetPoint
    H_Motor_pub.publish(Hmotor_pose_msg);
    V_Motor_pub.publish(Vmotor_pose_msg);
    
    ros::Rate r(20);

    double erro_V=100000, erro_H=10000;
    
    while (ros::ok() && ! (erro_V < ERRO_POSICAO_MOTOR && erro_H < ERRO_POSICAO_MOTOR ))
    {
        erro_V = fabs(posicao_V - Vmotor_pose_msg.data );
        erro_H = fabs(posicao_H - Hmotor_pose_msg.data );

        vel_pub.publish(vel_msg);
        ros::spinOnce();
       
        r.sleep();
    }
    
    // Desligar a movimentação do servo
    servo_msg.data = -1;
    servo_pub.publish(servo_msg);

    // Desligar enable do motor
    as->setSucceeded();
}

void set_eletroima(const projeto_semear::setEletroimaGoalConstPtr &goal, actionlib::SimpleActionServer<projeto_semear::setEletroimaAction> *as, tf::TransformListener &listener)
{

    geometry_msgs::Twist final_pose_msg;

    actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> client("moveEletroima", true);

    if (goal->pose == goal->posicao_inicial)  // Garra retraída e com 90º
    {
        final_pose_msg.linear.x = 0;
        final_pose_msg.linear.z = +1.0816e-1;
        final_pose_msg.linear.y = -1.4999e-1;
        final_pose_msg.angular.x = 0;
        final_pose_msg.angular.y = .0;
        final_pose_msg.angular.z = 94;
    }
    else if (goal->pose == goal->posicao_pegar_container_superior) // Garra para frente e com 0º
    {
        final_pose_msg.linear.x = 0;
        final_pose_msg.linear.y = -0.2;
        final_pose_msg.linear.z = +1.0e-1;
        final_pose_msg.angular.x = 0;
        final_pose_msg.angular.y = .0;
        final_pose_msg.angular.z = 4;
    }
    else if (goal->pose == goal->posicao_inicial_rotacionada)   // Garra para frente e com 
    {
        final_pose_msg.linear.x = 0;
        final_pose_msg.linear.y = -1.4999e-1;
        final_pose_msg.linear.z = +1.0816e-1;
        final_pose_msg.angular.x = .0;
        final_pose_msg.angular.y = .0;
        final_pose_msg.angular.z = 94;
    }
    else if (goal->pose == goal->posicao_segurar_container)
    {
        final_pose_msg.linear.x = 0;
        final_pose_msg.linear.y = -0.2;
        final_pose_msg.linear.z = +2.0816e-1;
        final_pose_msg.angular.x = .0;
        final_pose_msg.angular.y = .0;
        final_pose_msg.angular.z = 4;
    }
    else if (goal->pose == goal->posicao_segurar_container_rotacionado) // Gara para frente e 90º
    {
        final_pose_msg.linear.x = 0;
        final_pose_msg.linear.y = -0.2;
        final_pose_msg.linear.z = +2.0816e-1;
        final_pose_msg.angular.x = .0;
        final_pose_msg.angular.y = .0;
        final_pose_msg.angular.z = 94;
    }
    else
    {
        ROS_ERROR_STREAM("Posicao nao conhecida: " << goal->pose);
    }

    std_msgs::Float64 Hmotor_pose_msg;
    std_msgs::Float64 Vmotor_pose_msg;

    // Posição a ser atingida
    Hmotor_pose_msg.data = final_pose_msg.linear.y + posicao_H;
    Vmotor_pose_msg.data = final_pose_msg.linear.z + posicao_V;

    // Mensagem para enviar feedback
    projeto_semear::moveEletroimaFeedback feedback;

    // Ligar enable do motor e do Servo
    std_msgs::Int16 servo_msg;
    
    servo_msg.data = final_pose_msg.angular.z;
    servo_pub.publish(servo_msg);
    
    // Enviar SetPoint
    H_Motor_pub.publish(Hmotor_pose_msg);
    V_Motor_pub.publish(Vmotor_pose_msg);

    ros::Rate r(20);

    double erro_V=10000, erro_H=10000;
    while (ros::ok() && ( erro_V > ERRO_POSICAO_MOTOR || erro_H > ERRO_POSICAO_MOTOR ))
    {
        erro_V = fabs(posicao_V - Vmotor_pose_msg.data );
        erro_H = fabs(posicao_H - Hmotor_pose_msg.data );

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        r.sleep();
    }
    
    // Desativar servo
    servo_msg.data= -1;
    servo_pub.publish(servo_msg);
    
    as->setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveEletroima");
    ros::NodeHandle node;

    // Falta o servomotor

    // Esse publisher irá se comunicar com o V-REP: irá controlar a posição do Eletroimã
    tf::TransformListener listener1;
    tf::TransformListener listener2;

    actionlib::SimpleActionServer<projeto_semear::moveEletroimaAction> server1(node, "moveEletroima", boost::bind(&move_eletroima, _1, &server1, std::ref(listener1)), false);
    actionlib::SimpleActionServer<projeto_semear::setEletroimaAction> server2(node, "setEletroima", boost::bind(&set_eletroima, _1, &server2, std::ref(listener2)), false);

    /* Interface para PID dos Motores */
    stateV_pub = node.advertise<std_msgs::Float64>("AMR/V_PID/state", 10); //publica o estado para o PID, que foi lido pelo tópico arduinoElevadoresOutputVel
    stateH_pub = node.advertise<std_msgs::Float64>("AMR/H_PID/state", 10);

    H_Motor_pub = node.advertise<std_msgs::Float64>("/AMR/H_PID/setpoint", 10); // publica o setpoint do código para o pid
    V_Motor_pub = node.advertise<std_msgs::Float64>("/AMR/V_PID/setpoint", 10);

    controlH_sub = node.subscribe<std_msgs::Float64>("AMR/H_PID/control_effort", 10, controlH_callback);
    controlV_sub = node.subscribe<std_msgs::Float64>("AMR/V_PID/control_effort", 10, controlV_callback);

    /* Servo */
    servo_pub = node.advertise<std_msgs::Int16>("/AMR/servo", 10);


    ros::Subscriber arduino_state_motor_sub = node.subscribe<projeto_semear::Vel_Elevadores>("/AMR/arduinoElevadoresOutputDisplacement", 10, state_callback); // Output Vel dos Motores
    vel_pub = node.advertise<projeto_semear::Vel_Elevadores>("/AMR/arduinoElevadoresInputVel", 10);

    server1.start();
    server2.start();

    ros::spin();
    return 0;
}
