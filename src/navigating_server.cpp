#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/navigationAction.h>
#include <vector>
#include <queue>
#include <forward_list>

std::vector<std::uint8_t> dijkstra(std::uint8_t vertice_alvo, std::uint8_t vertice_inicio);
// Class to represent the vertices in the Map Graph
class Vertice
{
    std::uint8_t number;

  public:
    int distance;
    /*Vertice(std::uint8_t vertice_number, std::uint8_t _distance) : number(vertice_number), distance(_distance)
    {

    }*/
    Vertice(std::uint8_t vertice_number, std::uint8_t _distance)
    {
        number = vertice_number;
        distance = _distance;
    }
    Vertice(){};
    std::uint8_t getDistance() const { return distance; }
    std::uint8_t getNumber() const { return number; }
};

// To compare two Vertices in inverted ordem. So, is true for the vertice with the smallest distance.
class myComparator
{
  public:
    int operator()(const Vertice *p1, const Vertice *p2)
    {
        return p1->getDistance() < p2->getDistance();
    }
};

// for Map identification, see image

/** Identification:
 * QUADRANTE_CENTRAL=0
 * QUADRANTE_ESQUERDO=1
 * QUADRANTE_DIREITO=2
 * DOCA_VERDE=3
 * DOCA_AZUL=4
 * DOCA_CENTRAL=5
 * TREM=6

Allowed Movements:
 *                  QC0|QE1|QD2|DV3|DA4|DC5|Tr6|  
 * Quadrante Cen 0 | - | 1 | 1 | 0 | 0 | 1 | 0 |
 * Quadrante Esq 1 | 1 | - | 0 | 1 | 0 | 0 | 0 |
 * Quadrante Dir 2 | 1 | 0 | - | 0 | 1 | 0 | 1 |
 * Doca Verde    3 | 0 | 1 | 0 | - | 0 | 0 | 0 |
 * Doca Azul     4 | 0 | 0 | 1 | 0 | - | 0 | 0 |
 * Doca Central  5 | 1 | 0 | 0 | 0 | 0 | - | 0 |
 * Trem          6 | 0 | 0 | 1 | 0 | 0 | 0 | - |
*/

const std::forward_list<std::forward_list<std::uint8_t>> GRAPH =
    {{0, 1, 1, 0, 0, 1, 0},
     {1, 0, 0, 1, 0, 0, 0},
     {1, 0, 0, 0, 1, 0, 1},
     {0, 1, 0, 0, 0, 0, 0},
     {0, 0, 1, 0, 0, 0, 0},
     {1, 0, 0, 0, 0, 0, 0},
     {0, 0, 1, 0, 0, 0, 0}};

const int N_VERTICES = 7;
const std::uint8_t INFINITY_ = N_VERTICES + 10; // A value big enough to be considered INFINITY_

typedef actionlib::SimpleActionServer<projeto_semear::navigationAction> Server;

ros::ServiceClient pose_client;

void execute(const projeto_semear::navigationGoalConstPtr &goal, Server *as)
{
    /** O código para o switch é composto por 2 dígitos: XY
     *  X = from
     *  Y = To 
     */

    // Get Actual Pose of the Robot
    projeto_semear::GetPose srv;
    srv.request.set = false;
    if (!pose_client.call(srv))
    {
        // Sent Failed Result
        return;
    }
    projeto_semear::Pose robot_pose = srv.response.pose;

    // Get Goal Pose
    projeto_semear::Pose goal_pose = goal->goal_pose;

    projeto_semear::navigationFeedback feedback;

    // Get PATH
    feedback.path = dijkstra(goal_pose.location, robot_pose.location);

    as->publishFeedback(feedback);

    projeto_semear::navigationResult result;
    result.succeed = true;

    as->setSucceeded(result);
    /*
    unsigned int code = 10 * robot_pose.location + goal_pose.location;
    switch (code)
    {
    case (01):
        break;
    case (02):
        break;
    case (05):
        break;
    case (10):
        break;
    case (13):
        break;
    case (20):
        break;
    case (24):
        break;
    case (26):
        break;
    case (31):
        breakb
    case (42):
        break;
    case (50):
        break;
    case (62):
        break;
    }
    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NavigationServer");
    ros::NodeHandle node;

    // Requisita o serviço
    pose_client = node.serviceClient<projeto_semear::GetPose>("gps");

    // Cria o action server
    Server server(node, "navigation", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}

std::vector<std::uint8_t> dijkstra(std::uint8_t vertice_alvo, std::uint8_t vertice_inicio)
{
    std::vector<std::vector<Vertice *>> adjacent_list(N_VERTICES);//, std::vector<Vertice *>(1, nullptr)); // adjacent_list[v] = Vetor com as conexões de V
    std::vector<std::uint8_t> path(N_VERTICES, -1);                                                    // path[v] = Vetor com o caminho de V até o alvo
    std::vector<Vertice> vertices;

    // The minimun distance value is the first in this qeue
    std::priority_queue<Vertice *, std::vector<Vertice *>, myComparator> min_heap;

    // Create vertices
    for (std::uint8_t i = 0; i < N_VERTICES; i++)
    {
        vertices.push_back(Vertice(i, -1));
    }

    // Convert connection Graph to adjacent List
    std::uint8_t i = 0;
    for (auto links_list : GRAPH)
    {
        vertices[i].distance = i == vertice_inicio ? 0 : INFINITY_;
        if (i == vertice_inicio)
        {
            // Initialize min_heap
            min_heap.push(&vertices[i]);
        }

        // Set the vertice postd::uint8_ter in the Adjacent list for vertice i
        std::uint8_t j = 0;
        for (std::uint8_t &v : links_list)
        {
            if (v == 1)
                adjacent_list[i].push_back(&vertices[j]);
            j++;
        }

        i++;
    }
    
    /*  Dijkstra
    enquanto Q ≠ ø
         u ← extrair-mín(Q)                     //Q ← Q - {u}
         para cada v adjacente a u
              se d[v] > d[u] + w(u, v)          //relaxe (u, v)
                 então d[v] ← d[u] + w(u, v)
                       π[v] ← u

    */

    Vertice *u;
    while (!min_heap.empty())
    {
        u = min_heap.top();
        min_heap.pop();
        std::vector<Vertice *> links((adjacent_list[u->getNumber()]));
        for (std::vector<Vertice *>::iterator it = links.begin(); it != links.end(); ++it)
        {
            if ((*it)->getDistance() > u->getDistance() + 1)
            {

                (*it)->distance = u->getDistance() + 1;
                path[(*it)->getNumber()] = u->getNumber();

                if ((*it)->getNumber() == vertice_alvo)
                {
                    std::vector<std::uint8_t> final_path;
                    final_path.push_back((*it)->getNumber());

                    while (final_path.back() != vertice_inicio)
                    {
                        final_path.push_back(path[(final_path.back())]);
                    }

                    std::reverse(final_path.begin(), final_path.end());
                    return final_path;
                }

                min_heap.push(*it);
            }
        }
    }

    return std::vector<std::uint8_t>(1, -1);
}