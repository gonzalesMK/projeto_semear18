#include <ros/ros.h>
#include <projeto_semear/Pose.h>
#include <projeto_semear/GetPose.h>
#include <projeto_semear/PathPlanning.h>
#include <vector>
#include <queue>
#include <forward_list>

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

const int N_VERTICES = 7;     // Number of vertices in the GRAPH                 
const std::uint8_t INFINITY_ = N_VERTICES + 10; // A value big enough to be considered INFINITY_

// Algoritmo para Gerar o caminho entre 2 pontos
std::vector<std::uint8_t> dijkstra(std::uint8_t vertice_alvo, std::uint8_t vertice_inicio);
// Código que inicializa as variáveis necessárias, evitando repetição de contas
void initialize_dijkstra();

// Class to represent the vertices in the Map Graph. Each vertice is a possible pose.
class Vertice
{
    std::uint8_t number;

  public:
  // Distance b
    int distance;
    Vertice(std::uint8_t vertice_number, std::uint8_t _distance)
    {
        number = vertice_number;
        distance = _distance;
    }
    Vertice(){};
    std::uint8_t getDistance() const { return distance; }
    std::uint8_t getNumber() const { return number; }
};
std::vector<Vertice> vertices;
std::vector<std::vector<Vertice *>> adjacent_list(N_VERTICES);//adjacent_list[v] = Vetor com as conexões de V

// Class used to compare two Vertices in inverted ordem in the PriorityQueue. So, is true for the vertice with the smallest distance.
class myComparator
{
  public:
    int operator()(const Vertice *p1, const Vertice *p2)
    {
        return p1->getDistance() < p2->getDistance();
    }
};

bool path_plannning(projeto_semear::PathPlanning::Request &req,
         projeto_semear::PathPlanning::Response &res)
{
    projeto_semear::Pose robot_pose = req.initial_pose;
    projeto_semear::Pose goal_pose =  req.goal_pose;

    // Get PATH
    if( robot_pose.location == goal_pose.location){
        res.path = std::vector<std::uint8_t>(0, -1);
        return true;
    }
    res.path = dijkstra(goal_pose.location, robot_pose.location);

    if( res.path.back() != 255) return true;    
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathPlanningServer");
    ros::NodeHandle node;

    initialize_dijkstra();

    // Cria o serviço
    ros::ServiceServer service = node.advertiseService("pathPlanning", path_plannning);

    ros::spin();

    return 0;
}

void initialize_dijkstra(){

    // Create vertices
    for (std::uint8_t i = 0; i < N_VERTICES; i++)
    {
        vertices.push_back(Vertice(i, -1));
    }

    // Convert connection Graph to Adjacent List
    std::uint8_t i = 0;
    for (auto links_list : GRAPH)
    {
        std::uint8_t j = 0;
        for (std::uint8_t &v : links_list)
        {
            if (v == 1)
                adjacent_list[i].push_back(&vertices[j]);
            j++;
        }

        i++;
    }
    
}
std::vector<std::uint8_t> dijkstra(std::uint8_t vertice_alvo, std::uint8_t vertice_inicio)
{
    
    std::vector<std::uint8_t> path(N_VERTICES, -1);               // path[v] = Vetor com o caminho de V até o alvo

    // Initialize vertices
    for (auto it = vertices.begin(); it != vertices.end() ; it++)
    {
        (*it).distance = INFINITY_;
    }
    vertices[vertice_inicio].distance = 0;

    /*  Dijkstra
    enquanto Q ≠ ø
         u ← extrair-mín(Q)                     //Q ← Q - {u}
         para cada v adjacente a u
              se d[v] > d[u] + w(u, v)          //relaxe (u, v)
                 então d[v] ← d[u] + w(u, v)
                       π[v] ← u

    */

    // The minimun distance value is the first in this qeue
    std::priority_queue<Vertice *, std::vector<Vertice *>, myComparator> min_heap;
    min_heap.push(&vertices[vertice_inicio]);
    
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