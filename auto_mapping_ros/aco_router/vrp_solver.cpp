#include "vrp_solver.h"

#include <eigen3/Eigen/Dense>
#include <libconfig.h++>
#include <limits>
#include <random>

#include "opt2_solver.h"
#include "utils.h"

/**
 * Creates a colony of ants (set of improving sub optimal tsp paths)
 * @param colony - colony of ants
 * @param tau - pheromone (desirability obtained till now)
 * @param eta - desirability of path (inverse of the cost matrix)
 * @param params - ant colony optimization parameters
 */
void create_colony(const aco::Graph& graph,
                   std::vector<std::vector<std::vector<aco::Node>>>& colony,
                   const Eigen::MatrixXd& cost_matrix,
                   const Eigen::MatrixXd& tau, const Eigen::MatrixXd& eta,
                   const aco::IacoParamas& params, const int initial_node_id) {
    // TODO: Use previous elements from the colony
    colony.clear();

    // Find whether the user wants to set initial node for the ants to start or
    // random
    bool use_random_start = true;
    if (initial_node_id >= 0 && initial_node_id < tau.rows()) {
        use_random_start = false;
    }

    // Find the initial node for the ant to start
    auto get_initial_node = [&]() {
        if (use_random_start) {
            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_int_distribution<int> int_dist(0, graph.size() - 1);
            return graph.get_node_from_graph(int_dist(mt));
        } else {
            return graph.get_node_from_graph(initial_node_id);
        }
    };

    for (int i = 0; i < params.n_ants; i++) {
        // Vector of all routes of the current ant. Different routes
        // representing different vehicles
        std::vector<std::vector<aco::Node>> current_ant_routes{};

        // Array representing the visited nodes of the graph
        Eigen::ArrayXd visited = Eigen::ArrayXd::Zero(graph.size());

        // Initial Node
        std::vector<aco::Node> current_ant_route{};
        double current_capacity_left = params.max_route_per_vehicle;

        const auto init_node  = get_initial_node();
        visited(init_node.id) = 1;
        current_ant_route.emplace_back(init_node);

        bool capacity_reached = true;

        // Move the ant through the entire graph
        for (int j = 0; j < graph.size() - 1; j++) {
            const auto current_node = current_ant_route.back();

            // Calculate Probabilities of next choice of path for the ant
            Eigen::ArrayXd probability_array =
                Eigen::ArrayXd::Zero(graph.size());
            capacity_reached = true;
            for (int k = 0; k < graph.size(); k++) {
                // If the node is visited
                // or
                // If the node cannot be visited in the current capacity skip
                // the current node
                if (visited(k) || (current_capacity_left -
                                   cost_matrix(current_node.id, k)) < 0)
                    continue;
                capacity_reached = false;
                probability_array(k) =
                    pow(tau(current_node.id, k), params.alpha) *
                    pow(eta(current_node.id, k), params.beta);
            }

            // If capacity reached use a new vehicle/vector
            if (capacity_reached) {
                current_ant_route.emplace_back(init_node);
                current_ant_routes.emplace_back(current_ant_route);
                current_ant_route.clear();
                current_ant_route.emplace_back(init_node);
                current_capacity_left = params.max_route_per_vehicle;
                j--;
                continue;
            }

            // Normalize Probabilities
            double probability_array_sum = probability_array.sum();
            Eigen::ArrayXd norm_prob_array =
                probability_array / probability_array_sum;

            // Call Roulette Wheel to get the next node
            int next_node_id      = aco::run_roulette_wheel(norm_prob_array);
            visited(next_node_id) = 1;
            current_capacity_left -= cost_matrix(current_node.id, next_node_id);
            const aco::Node next_node = graph.get_node_from_graph(next_node_id);

            // Add next node to the current ant path
            current_ant_route.emplace_back(next_node);
        }

        if (!capacity_reached) {
            current_ant_route.emplace_back(init_node);
            current_ant_routes.emplace_back(current_ant_route);
        }

        colony.emplace_back(current_ant_routes);
    }
}

/**
 * Evaporate the pheromone matrix (tau)
 * @param params
 * @param tau
 */
void evaporate_pheromone_matrix(const aco::IacoParamas& params,
                                Eigen::MatrixXd& tau) {
    tau = (1 - params.rho) * tau;
}

/**
 * Updates the pheromone values (tau matrix) based on the paths that the ants
 * moved on and the quality of those paths
 * @param colony - collection of paths that the ants have moved on
 * @param fitness_values - fitness values of each of the paths the ants of the
 * colony moved on
 * @param tau - pheromone matrix
 */
void update_pheromone_matrix(
    const std::vector<std::vector<std::vector<aco::Node>>>& colony,
    const Eigen::MatrixXd& cost_matrix, const aco::IacoParamas& params,
    Eigen::MatrixXd& tau) {
    // For every ant (fresh solution)
    for (int ant_index = 0; ant_index < colony.size(); ant_index++) {
        double global_pheromone_update =
            params.max_route_per_vehicle /
            (colony.size() *
             find_fitness_values(cost_matrix, colony[ant_index]));

        // For every route cluster in the solution
        for (int route_index = 0; route_index < colony[ant_index].size();
             route_index++) {
            const double dk = find_fitness_values(
                cost_matrix, colony[ant_index][route_index]);
            const int mk = colony[ant_index].size();

            // For every customer/ node in the solution
            for (int node_index = 0;
                 node_index < colony[ant_index][route_index].size() - 1;
                 node_index++) {
                const int current_node_id =
                    colony[ant_index][route_index][node_index].id;
                const int next_node_id =
                    colony[ant_index][route_index][node_index + 1].id;
                const double dij = cost_matrix(current_node_id, next_node_id);

                double local_pheromone_update = (dk - dij) / (mk * dk);

                tau(current_node_id, next_node_id) =
                    global_pheromone_update * local_pheromone_update;
            }
        }
    }
}

/**
 * Function to solve the traveling salesman problem for multiple salesman using
 * ant colony optimization
 * @param graph
 * @param params
 * @return
 */
std::pair<std::vector<std::vector<aco::Node>>, double> aco::solve_vrp(
    const aco::Graph& graph, int initial_node_id) {
    // Initialize Parameters

    // Get ACO VRP Parameters
    aco::IacoParamas params = aco::get_vrp_params();

    // Cost/Distance Matrix
    const Eigen::MatrixXd cost_matrix = aco::get_cost_matrix(graph);

    // Get the initial pheromone matrix
    double tau0         = 10 / (graph.size() * graph.mean_edge_weight());
    int n_nodes         = graph.size();
    Eigen::MatrixXd tau = Eigen::MatrixXd::Constant(n_nodes, n_nodes, tau0);
    const Eigen::MatrixXd eta = cost_matrix.cwiseInverse();

    // Initialize Capacity
    if (params.max_route_per_vehicle < 0) {
        // TODO: Find if capacity is a tunable parameter or if this value works
        // for all problems
        double sum_edge_distances = cost_matrix.sum();
        if (params.vehicles_available == 1) {
            params.max_route_per_vehicle = sum_edge_distances;
        } else {
            double max_edge_distance = cost_matrix.maxCoeff();
            double capacity          = cost_matrix.mean() * graph.size() /
                              (params.vehicles_available * 2);
            params.max_route_per_vehicle =
                std::clamp(capacity, max_edge_distance, sum_edge_distances);
        }
    }

    // Initialize Number of ants
    if (params.n_ants < 0) {
        params.n_ants = graph.size();
    }

    // Initialize best route and fitness value
    std::vector<std::vector<aco::Node>> best_routes{};
    double best_fitness_value = std::numeric_limits<double>::max();

    // Initialize Colony
    std::vector<std::vector<std::vector<aco::Node>>> colony;

    // Main ACO Loop
    for (int i = 0; i < params.max_iters; i++) {
        // Create Colony
        create_colony(graph, colony, cost_matrix, tau, eta, params,
                      initial_node_id);

        // Find Best Fitness value
        for (int j = 0; j < params.n_ants; j++) {
            double fitness_value =
                find_fitness_values(cost_matrix, colony.at(j));
            if (fitness_value < best_fitness_value) {
                best_fitness_value = fitness_value;
                best_routes        = colony.at(j);
            }
        }

        // Evaporate Tau
        evaporate_pheromone_matrix(params, tau);

        // Update Pheromone Matrix
        update_pheromone_matrix(colony, cost_matrix, params, tau);
    }

    // Use Opt-2 Local Search to improve the routes
    for (auto& route : best_routes) {
        run_opt2(cost_matrix, route);
    }

    return {best_routes, find_fitness_values(cost_matrix, best_routes)};
}

/**
 * Load the VRP configuration parameters from the config file
 * @return TSP config parameters
 */
aco::IacoParamas aco::get_vrp_params() {
    IacoParamas params{};
    libconfig::Config cfg;
    try {
        const std::string package_name          = "aco_router";
        const std::string package_relative_path = "/config.cfg";
        const std::string filename =
            aco::get_directory_path(package_name, package_relative_path);
        char* tab2 = new char[filename.length() + 1];
        strcpy(tab2, filename.c_str());
        cfg.readFile(tab2);
    } catch (const libconfig::FileIOException& fioex) {
        std::__throw_invalid_argument("I/O error while reading file.");
    }

    try {
        params.n_ants                = cfg.lookup("n_ants");
        params.rho                   = cfg.lookup("rho");
        params.alpha                 = cfg.lookup("alpha");
        params.beta                  = cfg.lookup("beta");
        params.max_iters             = cfg.lookup("max_iters");
        params.vehicles_available    = cfg.lookup("vehicles_available");
        params.max_route_per_vehicle = cfg.lookup("max_route_per_vehicle");
    } catch (const libconfig::SettingNotFoundException& nfex) {
        std::cerr << "Missing setting in configuration file." << std::endl;
    }
    return params;
}