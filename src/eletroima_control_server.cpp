#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/constants/constants.hpp>


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

typedef actionlib::SimpleActionServer<projeto_semear::moveEletroimaAction> Server;

// Constants
const double FREQUENCIA = 20;           // Hertz
const double VEL_X = 0.1 / FREQUENCIA; // metros/segundo
const double VEL_Y = 0.1 / FREQUENCIA; // metros/segundo
const double VEL_Z = 0.1 / FREQUENCIA; // metros/segundo
const double W =  boost::math::constants::pi<double>() / 4 / FREQUENCIA ;      // Rad/ segundo

ros::Publisher eletro_twist;

void execute(const projeto_semear::moveEletroimaGoalConstPtr &goal, Server *as)
{
    // Recebe a posição atual do eletroima
    tf::TransformListener listener;
    tf::StampedTransform pose_transform, orientation_transform;

    while (true)
    {
        try
        {
            // Pega a posição do Eletroima em relação ao robô.
            listener.lookupTransform("/AMR", "/eletroima",
                                     ros::Time(0), pose_transform);

            break;
        }
        catch (tf::TransformException ex)
        {
            ros::Duration(1.0).sleep();
        }
    }

    // Create Pose message to send to V-REP
    geometry_msgs::Twist pose_msg;
    pose_msg.angular.z = 0; 
    pose_msg.angular.y = 0;
    pose_msg.angular.x = 0;
    pose_msg.linear.x = pose_transform.getOrigin().x();
    pose_msg.linear.y = pose_transform.getOrigin().y();
    pose_msg.linear.z = pose_transform.getOrigin().z();
    
    // Distância a ser percorrida em cada direção
    double dist_x = fabs(goal->deslocamento.linear.x);
    double dist_y = fabs(goal->deslocamento.linear.y);
    double dist_z = fabs(goal->deslocamento.linear.z);
    double dist_w = fabs(goal->deslocamento.angular.z);

    // Sentido a ser percorrido em cada direção
    double sent_x = boost::math::sign(goal->deslocamento.linear.x); //devolve o sinal
    double sent_y = goal->deslocamento.linear.y > 0 ? 1 : -1;       // Pode ser assim também
    double sent_z = boost::math::sign(goal->deslocamento.linear.z);
    double sent_w = boost::math::sign(goal->deslocamento.angular.z);

    // Mensagem para enviar feedback
    projeto_semear::moveEletroimaFeedback feedback;

    // helper variables
    ros::Rate r(FREQUENCIA);
    bool succeed = false;
    ros::NodeHandle nh;
    while (!succeed && nh.ok())
    {

        // Preenche a mensagem a ser publicada
        if (dist_w >= W)
        {
            pose_msg.angular.z = sent_w * W; // Soma à posição atual um deslocamento no sentido correto
            dist_w += -W;                     // Variável que controla o deslocamento
        }
        if (dist_x >= VEL_X)
        {
            pose_msg.linear.x += sent_x * VEL_X;
            dist_x += -VEL_X;
        }
        if (dist_y >= VEL_Y)
        {
            pose_msg.linear.y += sent_y * VEL_Y;
            dist_y += -VEL_Y;
        }
        if (dist_z >= VEL_Z)
        {
            pose_msg.linear.z += sent_z * VEL_Z;
            dist_z += -VEL_Z;
        }

        // Eletroima next pose publication
        eletro_twist.publish(pose_msg);

        // Send feedback message:
        feedback.distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2) + pow(dist_w, 2));
        as->publishFeedback(feedback);
        ROS_INFO_STREAM("W: " << dist_w <<  "vel W: " << pose_msg.angular.z);
        // Check if Final Pose is reached.
        if (dist_w < W && dist_x < VEL_X && dist_y < VEL_Y && dist_z < VEL_Z)
            succeed = true;
        else
            r.sleep();
    }

    as->setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveEletroima");
    ros::NodeHandle node;

    // Esse publisher irá se comunicar com o V-REP: irá controlar a posição do Eletroimã
    eletro_twist = node.advertise<geometry_msgs::Twist>("/AMR/setEletroimaPose", 1);

    Server server(node, "moveEletroima", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
    return 0;
}
