#include <limits>
#include <memory>
#include <iostream>
#include <random>

#include "aco.h"

aco::Node::Node()
{
}

aco::Node::Node(double x, double y, int id)
{
    this->id = id;
    this->x = x;
    this->y = y;
}

bool aco::Node::operator==(const aco::Node &other) const
{
    return this->id == other.id;
}

/**
 * Get distance between current node and node with node id as node_id
 * @param node_id
 * @return distance
 */
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

/**
 * Get the mean of all edge weights in the graph
 * @return
 */
double aco::Graph::mean_edge_weight() const
{
    double sum_edge_weights = 0;
    int n_edges = 0;
    for(const auto& node: graph_)
    {
        for(const auto& neighbor : node.neighbors)
        {
            sum_edge_weights += neighbor.second;
            n_edges++;
        }
    }
    return (sum_edge_weights)/(n_edges);
}

/**
 * Create a new node in Graph with co-ordinates x and y
 * @param x - x coordinate
 * @param y - y coordinate
 * @return
 */
int aco::Graph::create_node_in_graph(double x, double y)
{
    static int count = 0;
    const int id = count++;
    this->graph_.emplace_back(Node(x, y, id));
    n_nodes_ = count;
    return id;
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
    return graph_.at(node_id);
}

/**
 * Get pointer to the node from the graph using node id
 * @param node_id - id of the node
 * @return Node*
 */
aco::Node* aco::Graph::get_node_from_graph(int node_id)
{
    return &graph_.at(node_id);
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
 * Find a random index based on probabilities in the probability array
 * @param probability_array
 * @return
 */
int run_roulette_wheel(const Eigen::ArrayXd& probability_array)
{
    Eigen::ArrayXd cumulative_sum = Eigen::ArrayXd::Zero(probability_array.size());
    double current_sum = 0;
    for(int i=0; i<probability_array.size(); i++)
    {
        current_sum += probability_array(i);
        cumulative_sum(i) = current_sum;
    }

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> real_dist(0.0, current_sum);

    double rolled_value = real_dist(mt);

    int i=0;
    while(rolled_value > cumulative_sum(i))
    {
        if(i >= cumulative_sum.size())
        {
            std::cout << "invalid: logical error in roulette wheel. \n";
        }
        i++;
    }

    return i;
}

/**
 * Creates a colony of ants (set of improving sub optimal tsp paths)
 * @param colony - colony of ants
 * @param tau - pheromene (desirability obtained till now)
 * @param eta - desirabiliyu of path (inverse of the cost matrix)
 * @param params - ant colony optimization parameters
 */
void create_colony(const aco::Graph& graph,
                   std::vector<std::vector<aco::Node>>& colony,
                   const Eigen::MatrixXd& tau,
                   const Eigen::MatrixXd& eta,
                   const aco::AcoParams& params,
                   const int initial_node_id)
{
    // Find whether the user wants to set intial node for the ants to start or random
    bool use_random_start = true;
    if(initial_node_id >= 0 && initial_node_id < tau.rows())
    {
        use_random_start = false;
    }

    // Find the initial node for the ant to start
    auto get_intial_node = [&](){
        if(use_random_start)
        {
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_int_distribution<int> int_dist(0, graph.size());
            return graph.get_node_from_graph(int_dist(mt));
        }
        else
        {
            return graph.get_node_from_graph(initial_node_id);
        }
    };

    for(int i=0; i<params.n_ants; i++)
    {
        // Initialize vectors
        std::vector<aco::Node> current_ant_path;
        Eigen::ArrayXd visited = Eigen::ArrayXd::Zero(graph.size());

        // Initial Node
        const auto init_node = get_intial_node();
        visited(init_node.id) = 1;
        current_ant_path.emplace_back(init_node);

        // Move the ant through the n nodes based on the probabilities received in the previous iteration
        for(int j=0; j<params.n_ants-1; j++)
        {
            const auto current_node = current_ant_path.back();

            // Calculate Probabilities of next choice of path for the ant
            Eigen::ArrayXd probability_array = Eigen::ArrayXd::Zero(graph.size());
            for(int k=0; k<current_node.neighbors.size(); k++)
            {
                if(visited(k)) continue;
                probability_array(k) = pow(tau(current_node.id, k), params.alpha) * pow(eta(current_node.id, k), params.beta);
            }
            double probability_array_sum = probability_array.sum();
            Eigen::ArrayXd norm_prob_array = probability_array/probability_array_sum;

            // Call Roulette Wheel to get the next node
            int next_node_id = run_roulette_wheel(norm_prob_array);
            const aco::Node next_node = graph.get_node_from_graph(next_node_id);

            // Add next node to the current ant path
            current_ant_path.emplace_back(next_node);
        }

        // Complete the TSP loop
        current_ant_path.emplace_back(init_node);
    }
}

/**
 * Function to solve the traveling salesman problem using ant colony optimization
 * @param graph
 * @param params
 * @return
 */
std::vector<aco::Node> aco::solve_tsp(const aco::Graph& graph, const aco::AcoParams& params, int initial_node_id)
{
    // Get Initial Parameters

    // Cost/Distance Matrix
    const auto cost_matrix = get_cost_matrix(graph);

    // Get the initial pheromene matrix
    double tau0 = 10/(graph.size() * graph.mean_edge_weight());
    int n_nodes = graph.size();
    Eigen::MatrixXd tau = Eigen::MatrixXd::Constant(n_nodes, n_nodes, tau0);
    const Eigen::MatrixXd eta = cost_matrix.cwiseInverse();

    std::vector<aco::Node> best_route;
    std::vector<std::vector<aco::Node>> colony;

    // Main ACO Loop
    for(int i=0; i<params.max_iters; i++)
    {
        // Create Colony
        create_colony(graph, colony, tau, eta, params, initial_node_id);
    }
    
    return best_route;
}

namespace std
{
    template <>
    struct hash<aco::Node>
    {
        std::size_t operator()(const aco::Node& k) const
        {
            using std::size_t;
            using std::hash;
            using std::string;

            // Compute individual hash values for first,
            // second and third and combine them using XOR
            // and bit shifting:
            return ((hash<int>()(k.id) << 1) >> 1) ^ (hash<int>()(k.id) << 1);
        }
    };
}
