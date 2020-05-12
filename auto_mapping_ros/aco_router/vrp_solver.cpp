#include <limits>

#include "utils.h"
#include "vrp_solver.h"

/**
 * Creates a colony of ants (set of improving sub optimal tsp paths)
 * @param colony - colony of ants
 * @param tau - pheromene (desirability obtained till now)
 * @param eta - desirabiliyu of path (inverse of the cost matrix)
 * @param params - ant colony optimization parameters
 */
void create_colony(const aco::Graph& graph,
                   std::vector<std::vector<std::vector<aco::Node>>>& colony,
                   const Eigen::MatrixXd& cost_matrix,
                   const Eigen::MatrixXd& tau,
                   const Eigen::MatrixXd& eta,
                   const aco::IacoParamas& params,
                   const int initial_node_id)
{
    // TODO: Use previous elements from the colony
    colony.clear();

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
            std::uniform_int_distribution<int> int_dist(0, graph.size()-1);
            return graph.get_node_from_graph(int_dist(mt));
        }
        else
        {
            return graph.get_node_from_graph(initial_node_id);
        }
    };

    for(int i=0; i<params.n_ants; i++)
    {
        // Vector of all routes of the current ant. Different routes representing different vehicles
        std::vector<std::vector<aco::Node>> current_ant_routes{};

        // Array representing the visited nodes of the graph
        Eigen::ArrayXd visited = Eigen::ArrayXd::Zero(graph.size());

        // Initial Node
        std::vector<aco::Node> current_ant_route{};
        double current_capacity_left = params.max_route_per_vehicle;

        const auto init_node = get_intial_node();
        visited(init_node.id) = 1;
        current_ant_route.emplace_back(init_node);

        bool capacity_reached = true;

        // Move the ant through the entire graph
        for(int j=0; j<graph.size()-1; j++)
        {
            const auto current_node = current_ant_route.back();

            // Calculate Probabilities of next choice of path for the ant
            Eigen::ArrayXd probability_array = Eigen::ArrayXd::Zero(graph.size());
            capacity_reached = true;
            for(int k=0; k<graph.size(); k++)
            {
                // If the node is visited
                // or
                // If the node cannot be visited in the current capacity skip the current node
                if(visited(k) || (current_capacity_left - cost_matrix(current_node.id, k)) < 0) continue;
                capacity_reached = false;
                probability_array(k) = pow(tau(current_node.id, k), params.alpha) * pow(eta(current_node.id, k), params.beta);
            }

            // If capacity reached use a new vehicle/vector
            if(capacity_reached)
            {
                current_ant_route.emplace_back(init_node);
                current_ant_routes.emplace_back(current_ant_route);
                current_ant_route.clear();
                current_capacity_left = params.max_route_per_vehicle;
                j--;
                continue;
            }

            // Normalize Probabilities
            double probability_array_sum = probability_array.sum();
            Eigen::ArrayXd norm_prob_array = probability_array/probability_array_sum;

            // Call Roulette Wheel to get the next node
            int next_node_id = run_roulette_wheel(norm_prob_array);
            visited(next_node_id) = 1;
            const aco::Node next_node = graph.get_node_from_graph(next_node_id);

            // Add next node to the current ant path
            current_ant_route.emplace_back(next_node);
        }

        if(!capacity_reached)
        {
            current_ant_route.emplace_back(init_node);
            current_ant_routes.emplace_back(current_ant_route);
        }
    }
}

/**
 * TODO: Function to solve the traveling salesman problem for multiple salesman using ant colony optimization
 * @param graph
 * @param params
 * @return
 */
std::vector<std::vector<aco::Node>> aco::solve_vrp(const aco::Graph& graph, const aco::IacoParamas& params, int initial_node_id)
{
    // Initialize Parameters

    // Cost/Distance Matrix
    const Eigen::MatrixXd cost_matrix = get_cost_matrix(graph);

    // Get the initial pheromene matrix
    double tau0 = 10/(graph.size() * graph.mean_edge_weight());
    int n_nodes = graph.size();
    Eigen::MatrixXd tau = Eigen::MatrixXd::Constant(n_nodes, n_nodes, tau0);
    const Eigen::MatrixXd eta = cost_matrix.cwiseInverse();

    // Initialize best route and fitness value
    std::vector<std::vector<aco::Node>> best_routes{};
    double best_fitness_value = std::numeric_limits<double>::max();

    // Intialize Colony
    std::vector<std::vector<std::vector<aco::Node>>> colony;

    // Main ACO Loop
    for(int i=0; i<params.max_iters; i++)
    {
        // Create Colony
        create_colony(graph, colony, cost_matrix, tau, eta, params, initial_node_id);
    }

    return best_routes;
}

