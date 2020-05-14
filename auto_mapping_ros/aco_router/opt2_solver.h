#ifndef ACO_TSP_OPT2_SOLVER_H
#define ACO_TSP_OPT2_SOLVER_H

#include <eigen3/Eigen/Dense>

#include "types.h"

namespace aco
{
    struct Opt2Params
    {
        bool use_opt2;
        int max_iters;
    };

    /**
     * Runs the opt-2 algorithm for local search for improving the sequence
     * @param cost_matrix - distance matrix
     * @param sequence - sequence to update based on opt-2 algorithm
     */
    void run_opt2(const Eigen::MatrixXd& cost_matrix, std::vector<aco::Node>& sequence);

    /**
     * Load the VRP configuration parameters from the config file
     * @return TSP config parameters
     */
    aco::Opt2Params get_opt2_params();
}

#endif //ACO_TSP_OPT2_SOLVER_H
