#include <limits>
#include <memory>
#include <iostream>

#include "aco.h"

aco::Node::Node(double x, double y)
{
    this->x = x;
    this->y = y;
}

bool aco::Node::operator==(const aco::Node &other) const
{
    return this->id == other.id;
}

double aco::Node::get_distance_from_neighbor(int node_id) const
{
    for(const auto& neighbor : this->neighbors)
    {
        if(neighbor.first == node_id)
        {
            return neighbor.second;
        }
    }
    return std::numeric_limits<double>::max();
}

int aco::Graph::size() const
{
    return n_nodes_;
}

int aco::Graph::add_node_to_graph(aco::Node& node)
{
    static int count = 0;
    node.id = count++;
    this->graph.emplace_back(node);
    n_nodes_ = count;
    return node.id;
}

/**
 * Add edge (onw way) from node_id_from to node_id_to in the graph
 * @param node_id_from
 * @param node_id_to
 */
void aco::Graph::add_edge(int node_id_from, int node_id_to)
{
    const auto node_from = this->get_node_from_graph(node_id_from);
    if(node_from == nullptr)
    {
        std::cout << "Cannot add edge. Node (from) not present in the graph. \n";
        return;
    }
    const auto node_to = this->get_node_from_graph(node_id_to);
    if(node_to == nullptr)
    {
        std::cout << "Cannot add edge. Node (to) not present in the graph. \n";
        return;
    }
    int neighbor_id = node_to->id;
    double distance = sqrt(pow((node_from->x - node_to->x), 2) + pow((node_from->y - node_to->y), 2));
    node_from->neighbors.emplace_back(std::make_pair(neighbor_id, distance));
}

/**
 * Get the node from the graph using node id
 * @param node_id - id of the node
 * @return Node
 */
aco::Node aco::Graph::get_node_from_graph(int node_id) const
{
    return graph.at(node_id);
}

/**
 * Get pointer to the node from the graph using node id
 * @param node_id - id of the node
 * @return Node*
 */
aco::Node* aco::Graph::get_node_from_graph(int node_id)
{
    return &graph.at(node_id);
}

/**
 * Get Cost Matrix for the graph required by Ant Colony Optimization
 * @details This function runs the Floyd-Warshall Algorithm for finding shortest path between each pair of the graph
 * @param graph
 * @return
 */
Eigen::MatrixXd get_cost_matrix(const aco::Graph& graph)
{
    const int n_nodes = graph.size();
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(n_nodes, n_nodes);

    for(int i=0; i<n_nodes; i++)
    {
        const auto node_i = graph.get_node_from_graph(i);
        for(int j=0; j<n_nodes; j++)
        {
            if(i != j)
            {
                cost_matrix(i, j) = node_i.get_distance_from_neighbor(j);
            }
        }
    }

    for(int k=0; k<n_nodes; k++)
    {
        for(int i=0; i<n_nodes; i++)
        {
            for(int j=0; j<n_nodes; j++)
            {
                if(i == j) continue;
                if(cost_matrix(i, j) > cost_matrix(i, k) + cost_matrix(k, j))
                {
                    cost_matrix(i, j) = cost_matrix(i, k) + cost_matrix(k, j);
                }
            }
        }
    }

    return cost_matrix;
}

/**
 * Function to solve the traveling salesman problem using ant colony optimization
 * @param graph
 * @param params
 * @return
 */
std::vector<aco::Node> aco::solve_tsp(const aco::Graph& graph, const aco::AcoParams& params)
{
    const auto cost_matrix = get_cost_matrix(graph);
    std::cout << cost_matrix;

    std::vector<aco::Node> best_route;
    return best_route;
}
