#include <projeto_semear/kine_control.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> Client;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Distance to Goal" <<  feedback->distance) ;
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState& state,
            const projeto_semear::moveEletroimaResultConstPtr& result)
{
  ROS_INFO_STREAM("Finished in state" << state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transicion_demo");

    ros::NodeHandle nh;

    kineControl::robot motor;

    //** Como usar o motor para mudar a velocidade :
    ROS_INFO("Iniciando Teste");
<<<<<<< HEAD
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
<<<<<<< a9c83cc14bbcf51ca04c627fdba90a4672a077be
<<<<<<< 165ad2252775e1b3114de7edd16e75c12b0cb889
=======
=======
<<<<<<< 81310585d7bccfbcff7c41ea327bea39e2a48e00
>>>>>>> Testes depositar
<<<<<<< 2164c25943927943ea9c6b1da4b86d38d7ee859f
>>>>>>> Primeiro commit
   /* kineControl::linha_preta(motor);
    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);     
     */
    kineControl::alinhar(motor);  
=======
    kineControl::linha_preta(motor);
<<<<<<< 55d7f5e5ba1bbef79f46538f84abed9470ab4f00
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
=======
    kineControl::alinhar_containerdepositado(motor);
>>>>>>> Testes depositar
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);*/
=======
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);  */

    Client client("moveEletroima", true); // true -> don't need ros::spin()
    client.waitForServer();

    // Meta para posicionar a garra em cima do container
    projeto_semear::moveEletroimaGoal goal;
    goal.deslocamento.linear.x = 0;
    goal.deslocamento.linear.y = 0;
    goal.deslocamento.linear.z = -0.12;
    goal.deslocamento.angular.z = 0;

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());

    ROS_INFO_STREAM("ligando o eletroima");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  
    std_msgs::Bool msg;
    msg.data = true;
    pub.publish(msg);

    goal.deslocamento.linear.z = 0.12;
 

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    client.waitForResult(ros::Duration());

    kineControl::esquerda(motor);  
    kineControl::ir_doca(motor); 
>>>>>>> Primeiro commit
=======
=======
    kineControl::alinhar_containerdepositado(motor);
>>>>>>> Testes depositar
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);*/
>>>>>>> Testes depositar
    
    return 0;
>>>>>>> Primeiro commit
=======
    kineControl::alinhar_containerdepositado(motor);
    /*kineControl::esquerda(motor);  
    kineControl::ir_doca(motor);  
    kineControl::ir_quadrante(motor);  
    kineControl::direita(motor);*/
    
    return 0;
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
}