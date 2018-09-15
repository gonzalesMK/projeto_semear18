#include <ros/ros.h>
#include <projeto_semear/DepositarContainer.h>
#include <actionlib/client/simple_action_client.h>
#include <projeto_semear/moveEletroimaAction.h>
#include <projeto_semear/setEletroimaAction.h>
#include <std_msgs/Bool.h>
#include <projeto_semear/kine_control.h>
#include <projeto_semear/GetContainerInfo.h>
#include <projeto_semear/MoveContainer.h>
#include <vector>
#include <projeto_semear/GetPose.h>

int code = 0;

/* Código para depositar o container na doca correta.
  Para execução do código, considera-se que o robô já está alinhado à doca certa e que 
  o container já está na posição correta na garra para ser depositado.
*/
typedef actionlib::SimpleActionClient<projeto_semear::moveEletroimaAction> moveClient;
typedef actionlib::SimpleActionClient<projeto_semear::setEletroimaAction> setClient;

// Função de feedback do ActionLib
void feedbackCb(const projeto_semear::moveEletroimaFeedbackConstPtr &feedback)
{
}
void feedbackCb2(const projeto_semear::setEletroimaFeedbackConstPtr &feedback)
{
}

// Função executada quando a tarefa termina
void doneCb(const actionlib::SimpleClientGoalState &state,
            const projeto_semear::moveEletroimaResultConstPtr &result)
{
}
void doneCb2(const actionlib::SimpleClientGoalState &state,
             const projeto_semear::setEletroimaResultConstPtr &result)
{
}

// Called once when the goal becomes active
void activeCb()
{
}

bool depositar_container(projeto_semear::DepositarContainer::Request &req,
                         projeto_semear::DepositarContainer::Response &res)
{
  ros::NodeHandle node;

  kineControl::robot motor;

  ros::Publisher pub = node.advertise<std_msgs::Bool>("/AMR/activateEletroima", 1);
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  std_msgs::Bool msg;
  msg.data = true;
  pub.publish(msg);

  moveClient move_eletro_client("moveEletroima", true);
  setClient set_eletro_client("setEletroima", true);
  move_eletro_client.waitForServer();
  set_eletro_client.waitForServer();

  projeto_semear::moveEletroimaGoal move_goal;
  projeto_semear::setEletroimaGoal set_goal;

  // Centraliza a garra
  set_goal.pose = set_goal.posicao_inicial_rotacionada;
  set_eletro_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
  set_eletro_client.waitForResult();

  // Pegar a localização do robô
  ros::ServiceClient pose_client = node.serviceClient<projeto_semear::GetPose>("gps");
  projeto_semear::GetPose srv;
  pose_client.call(srv);
  int localizacao_aux = (std::uint32_t)srv.response.pose.location;

  // Transformando a localização_aux para localização correta
  int localizacao;
  switch (localizacao_aux)
  {
  case 3:
    localizacao = 12;

  case 4:
    localizacao = 13;
  }

  // Utilizando o servico de mapa_container
  ros::ServiceClient get_client = node.serviceClient<projeto_semear::GetContainerInfo>("getContainerInfo");
  ros::ServiceClient move_container_client = node.serviceClient<projeto_semear::MoveContainer>("moveContainer");

  projeto_semear::GetContainerInfo get_srv;
  projeto_semear::MoveContainer move_container_srv;
  srv.request.set = false;

  // Informando a localizacao da pilha onde o robo se encontra
  get_srv.request.where = localizacao;
  get_client.call(get_srv);

  // Pegando o vetor que contem os containers depositados
  std::vector<std::uint32_t> vec = get_srv.response.lista;

  int code = vec.size(); //variável que guarda quantos containers têm em uma pilha

<<<<<<< HEAD
=======
  if(vec.back() == 0 ){
    code = 0;
  }
>>>>>>> estrategia
  ROS_INFO_STREAM("DEPOSITAR CONTAINER: Valor do code:" << code);

  /*Code == 0: nenhum container depositado
    Code != 0: já existe um ou mais containers na pilha*/

  // Alinhar com o container de baixo
  if (code != 0)
    kineControl::alinhar_containerdepositado(motor);

  move_goal.deslocamento.angular.z = 0;
  move_goal.deslocamento.linear.x = 0;
  move_goal.deslocamento.linear.y = 0;
  move_goal.deslocamento.linear.z = -0.137 + (code * 0.04); //0,2 chute da altura do container
  move_eletro_client.sendGoal(move_goal, &doneCb, &activeCb, &feedbackCb);
  move_eletro_client.waitForResult(ros::Duration());

  //andar uma distância determinada para fica no meio do container já depositado
  //goal.deslocamento.angular.z = 1;
  //goal.deslocamento.linear.z = 0;
  //client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  //client.waitForResult(ros::Duration());

  // Desligando o Eletroima
  msg.data = false;
  pub.publish(msg);

  // Atualiza que o container foi depositado
  int pose;
  switch (req.posicao_origem_do_container)
  {
  case projeto_semear::Pose::QUADRANTE_ESQUERDO:
    pose = 0;
    break;
  case projeto_semear::Pose::QUADRANTE_CENTRAL:
    pose = 2;
    break;
  case projeto_semear::Pose::QUADRANTE_DIREITO:
    pose = 4;
    break;
  default:
    ROS_ERROR(" Localizacao do robo pode estar errada! Nenhuma foi escolhida");
    return false;
  }
  move_container_srv.request.where = pose + req.dir_ou_esq;
  move_container_client.call(move_container_srv);

  set_goal.pose = set_goal.posicao_inicial_rotacionada;
  set_eletro_client.sendGoal(set_goal, &doneCb2, &activeCb, &feedbackCb2);
  set_eletro_client.waitForResult();

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depositar_container");
  ros::NodeHandle node;

  // Cria o serviço
  ros::ServiceServer service = node.advertiseService("depositar_container", depositar_container);

  ros::spin();

  return 0;
}