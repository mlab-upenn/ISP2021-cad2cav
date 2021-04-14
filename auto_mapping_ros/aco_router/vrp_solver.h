#ifndef ACO_TSP_VRP_SOLVER_H
#define ACO_TSP_VRP_SOLVER_H

#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include "tsp_solver.h"

namespace aco {

/**
 * Parameters for solving the Improved Ant Colony Optimization for VRP
 */
struct IacoParamas : public AcoParams {
    int vehicles_available;
    double max_route_per_vehicle = -1.0;
};

/**
 * Function to solve the vehicle routing problem using the ant colony
 * optimization
 * @param graph
 * @param params
 * @return
 */
std::pair<std::vector<std::vector<aco::Node>>, double> solve_vrp(
    const Graph& graph, int initial_node_id = -1);

/**
 * Load the VRP configuration parameters from the config file
 * @return TSP config parameters
 */
aco::IacoParamas get_vrp_params();

}  // namespace aco

#endif  // ACO_TSP_VRP_SOLVER_H
