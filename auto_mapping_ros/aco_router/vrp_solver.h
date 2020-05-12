#ifndef ACO_TSP_VRP_SOLVER_H
#define ACO_TSP_VRP_SOLVER_H

#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include "tsp_solver.h"

namespace aco
{
    /**
     * Parameters for solving the Improved Ant Colony Optimization for VRP
     */
    struct IacoParamas : public AcoParams
    {
        int n_vehicles;
    };

    /**
     * Function to solve the vehicle routing problem using the ant colony optimization
     * @param graph
     * @param params
     * @return
     */
    std::vector<std::vector<aco::Node>> solve_vrp(const Graph& graph, const AcoParams& params, int initial_node_id = -1);
}

#endif //ACO_TSP_VRP_SOLVER_H
