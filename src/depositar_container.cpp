#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>

/* Código para depositar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Eletroima_client");
  ros::NodeHandle nh;

  kineControl::robot motor;

  ros::Publisher pub = nh.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  
  std_msgs::Bool msg;
  ROS_INFO_STREAM("ligando o eletroima");
  msg.data = true;
  pub.publish(msg);

  Client client("moveEletroima", true); 
  client.waitForServer();

  projeto_semear::moveEletroimaGoal goal;



  //utilizando o servico de gps para encontrar a localização
  ros::ServiceClient pose_client = node.serviceClient<projeto_semear::GetPose>("gps");
  projeto_semear::GetPose srv;

  //pegando a localização
  int localizacao = (std::uint8_t)srv.response.pose.location;

  //utilizando o servico de mapa_container
  ros::ServiceClient get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
  projeto_semear::GetContainerInfo get_srv;

  //informando a localizacao da pilha onde o robo se encontra
  get_srv.request.where = localizacao;
  get_client.call(get_srv);

  //pegando o vetor que contem os containers depositados
  std::vector<std::uint8_t> vec = get_srv.response.lista;

  int code = 0; //variável que guarda quantos containers têm em uma pilha

  for (auto i = vec.begin(); i != vec.end(); i++)
  {
    if(vec[i]!=255) code++;
  }  
  
  /*Code == 0: nenhum container depositado
    Code != 0: já existe um ou mais containers na pilha*/



  //alinhar com o container de baixo
  if(code != 0) kineControl::alinhar_containerdepositado(motor);
  
  goal.deslocamento.angular.z = 0;
  goal.deslocamento.linear.x = 0;
  goal.deslocamento.linear.y = 0;
  goal.deslocamento.linear.z = -0.137+(code*0.02); //0,2 chute da altura do container
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());
  }
  //andar uma distância determinada para fica no meio do container já depositado
  goal.deslocamento.angular.z = 1;
  goal.deslocamento.linear.z = 0;
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration());

  ROS_INFO_STREAM("desligando o eletroima");
  msg.data = false;
  pub.publish(msg);

  client.waitForResult(ros::Duration());

  return 0;
}