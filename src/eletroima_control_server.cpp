#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/constants/constants.hpp>
#include <actionlib/client/simple_action_client.h>

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
const double FREQUENCIA = 20;                                           // Hertz
const double VEL_X = 0.2 / FREQUENCIA;                                  // metros/segundo
const double VEL_Y = 0.2 / FREQUENCIA;                                  // metros/segundo
const double VEL_Z = 0.2 / FREQUENCIA;                                  // metros/segundo
const double W = boost::math::constants::pi<double>() / FREQUENCIA; // Rad/ segundo

ros::Publisher eletro_twist;

void move_eletroima(const projeto_semear::moveEletroimaGoalConstPtr &goal, actionlib::SimpleActionServer<projeto_semear::moveEletroimaAction> *as)
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
            // A rotação é em relação ao mundo
            listener.lookupTransform("/world", "/eletroima",
                                     ros::Time(0), orientation_transform);

            break;
        }
        catch (tf::TransformException ex)
        {
            ros::Duration(1.0).sleep();
        }
    }

    // Create Pose message to send to V-REP
    // A mensagem para o vrep envia a posição absoluta em que queremos o eletroimã, mas não envia o angulo absoluto. O ãngulo enviado é um deslocamento angular.
    geometry_msgs::Twist pose_msg;
    tf::Matrix3x3(orientation_transform.getRotation()).getRPY(pose_msg.angular.x, pose_msg.angular.y, pose_msg.angular.z);
    pose_msg.linear.x = pose_transform.getOrigin().x();
    pose_msg.linear.y = pose_transform.getOrigin().y();
    pose_msg.linear.z = pose_transform.getOrigin().z();

    // Distância a ser percorrida em cada direção
    double goal_x = goal->deslocamento.linear.x + pose_msg.linear.x;
    double goal_y = goal->deslocamento.linear.y + pose_msg.linear.y;
    double goal_z = goal->deslocamento.linear.z + pose_msg.linear.z;
    double goal_w = goal->deslocamento.angular.z + pose_msg.angular.z;

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

    double dist_x, dist_y, dist_z, dist_w;
    bool parou = true;
    while (!succeed && nh.ok())
    {
        listener.lookupTransform("/AMR", "/eletroima", ros::Time(0), pose_transform);
        listener.lookupTransform("/world", "/eletroima", ros::Time(0), orientation_transform);

        tf::Matrix3x3(orientation_transform.getRotation()).getRPY(pose_msg.angular.x, pose_msg.angular.y, pose_msg.angular.z);
        pose_msg.linear.x = pose_transform.getOrigin().x();
        pose_msg.linear.y = pose_transform.getOrigin().y();
        pose_msg.linear.z = pose_transform.getOrigin().z();

        dist_x = goal_x - pose_msg.linear.x;
        dist_y = goal_y - pose_msg.linear.y;
        dist_z = goal_z - pose_msg.linear.z;
        dist_w = goal_w - pose_msg.angular.z;

        parou = true;
        // Preenche a mensagem a ser publicada
        if (dist_w * sent_w >= W)
        {
            pose_msg.angular.z += sent_w * W; // Soma à posição atual um deslocamento no sentido correto
            dist_w += -W;                     // Variável que controla o deslocamento
            parou = false;
        }
        if (dist_x * sent_x >= VEL_X)
        {
            pose_msg.linear.x += sent_x * VEL_X;
            dist_x += -VEL_X;
            parou = false;
        }
        if (dist_y * sent_y >= VEL_Y)
        {
            pose_msg.linear.y += sent_y * VEL_Y;
            dist_y += -VEL_Y;
            parou = false;
        }
        if (dist_z * sent_z >= VEL_Z)
        {
            pose_msg.linear.z += sent_z * VEL_Z;
            dist_z += -VEL_Z;
            parou = false;
        }

        // Eletroima next pose publication
        eletro_twist.publish(pose_msg);

        // Send feedback message:
        feedback.distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2) + pow(dist_w, 2));
        as->publishFeedback(feedback);
        ROS_INFO_STREAM("W: " << dist_w << " Pose W: " << pose_msg.angular.z);
        // Check if Final Pose is reached.
        if (parou)
            succeed = true;
        else
            r.sleep();
    }

    as->setSucceeded();
}

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Distance to Goal" <<  feedback->distance) ;
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState& state,
            const projeto_semear::moveEletroimaResultConstPtr& result)
{
  ROS_INFO_STREAM("Finished in sta te" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

void set_eletroima(const projeto_semear::setEletroimaGoalConstPtr &goal, actionlib::SimpleActionServer<projeto_semear::setEletroimaAction>  *as)
{
    
    geometry_msgs::Twist pose_msg;

    actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> client("moveEletroima", true);

    if (goal->pose == goal->posicao_pegar_container_superior)
    {
        pose_msg.linear.x = 0;
        pose_msg.linear.y = -.190;
        pose_msg.linear.z = .170;
        pose_msg.angular.x = .0;
        pose_msg.angular.y = .0;
        pose_msg.angular.z = .0;
    }

    projeto_semear::moveEletroimaGoal move_msg;
    move_msg.deslocamento = pose_msg;
    
    client.sendGoal(move_msg, &doneCb, &activeCb, &feedbackCb);
    client.waitForResult(ros::Duration());

    as->setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveEletroima");
    ros::NodeHandle node;

    // Esse publisher irá se comunicar com o V-REP: irá controlar a posição do Eletroimã
    eletro_twist = node.advertise<geometry_msgs::Twist>("/AMR/setEletroimaPose", 1);

    actionlib::SimpleActionServer<projeto_semear::moveEletroimaAction> server1(node, "moveEletroima", boost::bind(&move_eletroima, _1, &server1), false);
    actionlib::SimpleActionServer<projeto_semear::setEletroimaAction> server2(node, "setEletroima", boost::bind(&set_eletroima, _1, &server2), false);

    server1.start();
    server2.start();

    ros::spin();
    return 0;
}
