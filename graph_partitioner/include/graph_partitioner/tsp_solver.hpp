#ifndef __GRAPH_PARTITIONER_TSP_SOLVER_HPP__
#define __GRAPH_PARTITIONER_TSP_SOLVER_HPP__

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

#include "cad2cav_types/graph.hpp"

namespace cad2cav {
namespace graph_partitioner {

/**
 * @brief Specifies which TSP solver to use on graphs
 *
 * 1. GOOGLE_ORTOOLS:   Google OR-Tools solver (default)
 * 2. BRANCH_AND_BOUND: branch-and-bound solver
 *
 */
enum TSPSolverType { GOOGLE_ORTOOLS = 0, BRANCH_AND_BOUND };

class TSPSolver {
public:
  typedef std::vector<std::array<double, 2>> Path;

  TSPSolver(const cad2cav::Graph& graph)
      : graph_(graph), tsp_min_cost_(std::numeric_limits<double>::infinity()) {}
  TSPSolver(const TSPSolver& other)
      : graph_(other.graph_),
        tsp_min_cost_(other.tsp_min_cost_),
        tsp_sequence_(other.tsp_sequence_) {}

  /**
   * @brief Regernerate the TSP sequence that covers all the nodes in the
   * graph. Sequence empty means failure.
   *
   * @param solver:   specify solver type. Default is Google OR Tools
   * @return double:  new TSP path cost
   */
  double updateTSPSequence(TSPSolverType solver = GOOGLE_ORTOOLS) noexcept;

  /**
   * @brief Gets the current TSP sequence
   *
   * @return const Path
   */
  const Path getTSPSequence() const noexcept;

private:
  const cad2cav::Graph graph_;
  double tsp_min_cost_;
  std::vector<int> tsp_sequence_;

  /**
   * @brief Solves TSP with Google OR tools
   *
   * Reference: https://developers.google.com/optimization/routing/tsp#c++
   *
   * @param start_node_id:        starting node of the TSP path
   * @return std::pair<double, std::vector<int>>: total cost and sequence
   */
  std::pair<double, std::vector<int>> tsp_ortools_solver(
      const int start_node_id);

  /**
   * @brief Applies branch-and-bound method for solving TSP.
   *      Assumes graph undirected and only works for n_nodes_< 10.
   *
   * Reference:
   * https://www.geeksforgeeks.org/traveling-salesman-problem-using-branch-and-bound-2/
   *
   */
  [[deprecated]] void tsp_branch_and_bound() noexcept;

  /**
   * @brief recursive helper function for TSP Branch and Bound solver.
   *
   * @param current_bound:    lower bound estimate
   * @param current_cost:     running path cost
   * @param level:            recursion level
   * @param visited:          visited nodes (in node ID)
   * @param current_path:     running TSP path
   */
  [[deprecated]] void tsp_explore(double current_bound, double current_cost,
                                  double level, std::set<int>& visited,
                                  std::vector<int>& current_path);
};

}  // namespace graph_partitioner
}  // namespace cad2cav

#endif /* __GRAPH_PARTITIONER_TSP_SOLVER_HPP__ */
