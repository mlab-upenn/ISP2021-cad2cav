#ifndef ACO_TSP_UTILS_H
#define ACO_TSP_UTILS_H

#include "tsp_solver.h"

namespace aco {

/**
 * Get Cost Matrix for the graph required by Ant Colony Optimization
 * @details This function runs the Floyd-Warshall Algorithm for finding shortest
 *      path between each pair of the graph
 * @param graph
 * @return
 */
Eigen::MatrixXd get_cost_matrix(const aco::Graph& graph);

/**
 * Finds fitness value for a single path of ant
 * @param cost_matrix
 * @param ant_path
 * @return
 */
double find_fitness_values(const Eigen::MatrixXd& cost_matrix,
                           const std::vector<aco::Node>& ant_path);

/**
 * Finds fitness value for a single path of ant
 * @param cost_matrix
 * @param ant_path
 * @return
 */
double find_fitness_values(
    const Eigen::MatrixXd& cost_matrix,
    const std::vector<std::vector<aco::Node>>& ant_paths);

/**
 * Find a random index based on probabilities in the probability array
 * @param probability_array
 * @return
 */
int run_roulette_wheel(const Eigen::ArrayXd& probability_array);

/**
 * Convert user defined graph type to aco graph type
 * @tparam GraphType - GraphType must be a sequential std container (Eg. Vector)
 * containing sequence of nodes
 * @param graph
 * @return
 */
template <typename GraphType>
aco::Graph convert_to_aco_graph(const GraphType& graph);

/**
 * Get the directory path string
 * @param package_name - name of the package/ project (eg. "Example Project)
 * @param package_relative_path - path relative to the project (eg.
 * "/config/abc.cfg"
 * @return
 */
std::string get_directory_path(const std::string& package_name,
                               const std::string& package_relative_path);

}  // namespace aco

#endif  // ACO_TSP_UTILS_H

#include "utils_impl.h"