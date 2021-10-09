#include "graph_partitioner/tsp_solver.hpp"

namespace cad2cav {
namespace graph_partitioner {

const TSPSolver::Path TSPSolver::getTSPSequence() const noexcept {
  Path path;

  // convert solved node id sequence into coord sequence
  for (const auto& id : tsp_sequence_) {
    const auto& n = graph_.getNode(id);
    path.push_back({n.x, n.y});
  }

  return path;
}

double TSPSolver::updateTSPSequence(TSPSolverType solver) noexcept {
  if (solver == TSPSolverType::GOOGLE_ORTOOLS) {
    // clears previous TSP solutions
    tsp_min_cost_ = std::numeric_limits<double>::infinity();
    tsp_sequence_.clear();

    for (int i = 0; i < graph_.size(); ++i) {
      const auto [cost, sequence] = tsp_ortools_solver(i);
      if (static_cast<double>(cost) < tsp_min_cost_) {
        tsp_min_cost_ = static_cast<double>(cost);
        tsp_sequence_ = sequence;
      }
    }
  } else if (solver == TSPSolverType::BRANCH_AND_BOUND) {
    tsp_branch_and_bound();
  }

  return tsp_min_cost_;
}

std::pair<double, std::vector<int>> TSPSolver::tsp_ortools_solver(
    const int start_node_id) {
  // starting node of TSP
  operations_research::RoutingIndexManager::NodeIndex depot{start_node_id};
  // TSP solver manager
  operations_research::RoutingIndexManager manager(graph_.size(), 1, depot);
  // TSP solver model
  operations_research::RoutingModel routing(manager);

  // initialize a callback for retrieving distances between two nodes
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&](int64_t from_node, int64_t to_node) -> int64_t {
        // convert from OR Tools node id to Graph node id
        const auto start_node_id = manager.IndexToNode(from_node).value();
        const auto end_node_id   = manager.IndexToNode(to_node).value();

        // retrieve neighboring distance
        const auto node  = graph_.getNode(start_node_id);
        int64_t distance = node.neighbors.count(end_node_id) > 0
                               ? static_cast<int64_t>(
                                     std::ceil(node.neighbors.at(end_node_id)))
                               : std::numeric_limits<int>::max();
        return distance;
      });
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // define routing search parametes
  operations_research::RoutingSearchParameters param =
      operations_research::DefaultRoutingSearchParameters();
  param.set_first_solution_strategy(
      operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // solve the problem
  const operations_research::Assignment* solution =
      routing.SolveWithParameters(param);

  // get best sequence
  std::vector<int> sequence;
  // Get sequence.
  int64_t index = routing.Start(0);
  sequence.push_back(manager.IndexToNode(index).value());
  while (!routing.IsEnd(index)) {
    index = solution->Value(routing.NextVar(index));
    sequence.push_back(manager.IndexToNode(index).value());
  }

  return {solution->ObjectiveValue(), sequence};
}

void TSPSolver::tsp_branch_and_bound() noexcept {
  // Assumes the problem size is small; use branch and bound search
  // Also assumes the graph is undirected.
  //  Reference:
  //  https://www.geeksforgeeks.org/traveling-salesman-problem-using-branch-and-bound-2/

  // clears previous TSP solutions
  tsp_min_cost_ = std::numeric_limits<double>::infinity();
  tsp_sequence_.clear();

  // initialize running sequence
  std::vector<int> sequence;

  // initialize lower bound
  double current_bound = 0.0;
  for (int i = 0; i < graph_.size(); ++i) {
    const auto& node = graph_.getNode(i);
    // for a solvable TSP graph, all nodes should at least have 2 neighbors
    if (node.neighbors.size() < 2) {
      ROS_ERROR_STREAM("Singular node at ("
                       << node.x << ", " << node.y
                       << ") found. Cannot generate TSP path.");
      return;
    }
    // initial lower bound is
    //  0.5 * sum[foreach node](nearest_neighbor + second_nearest_neighbor)
    double nearest_dist        = node.distances.begin()->first;
    double second_nearest_dist = std::next(node.distances.begin())->first;
    current_bound += nearest_dist + second_nearest_dist;
  }
  current_bound /= 2.0;

  // initialize `visited` set
  std::set<int> visited_node_list;

  // iterate over all nodes as starting node
  for (int i = 0; i < graph_.size(); ++i) {
    sequence.push_back(i);
    visited_node_list.insert(i);
    tsp_explore(current_bound, 0.0, 1, visited_node_list, sequence);
    sequence.pop_back();
    visited_node_list.erase(i);
  }
}

void TSPSolver::tsp_explore(double current_bound, double current_cost,
                            double level, std::set<int>& visited,
                            std::vector<int>& current_path) {
  if (level == graph_.size()) {
    // all nodes have been visited
    const auto& node = graph_.getNode(current_path.back());
    // if the last node in path connects with the first node in path, one
    // solution is found
    if (node.neighbors.count(current_path.front()) > 0) {
      double total_cost =
          current_cost + node.neighbors.at(current_path.front());

      // update global minimum
      if (total_cost < tsp_min_cost_) {
        tsp_min_cost_ = total_cost;
        tsp_sequence_ = current_path;
        tsp_sequence_.push_back(current_path.front());
      }
    }
  } else {
    const auto& node = graph_.getNode(current_path.back());
    for (int i = 0; i < graph_.size(); ++i) {
      if (node.neighbors.count(i) == 0 || visited.count(i) > 0) continue;
      const auto& neighbor = graph_.getNode(i);

      // save current lower bound estimate & running cost
      double prev_bound = current_bound;
      double prev_cost  = current_cost;

      // update running cost
      current_cost += node.neighbors.at(i);

      // update lower bound estimate
      if (level == 1) {
        // shortest edge incident to current node
        double min_inc_edge = node.distances.begin()->first;
        // shortest edge incident to neighbor node
        double min_inc_edge_neighbor = neighbor.distances.begin()->first;
        current_bound -= (min_inc_edge + min_inc_edge_neighbor) / 2.0;
      } else {
        // 2nd shortest edge incident to current node
        double second_min_inc_edge = std::next(node.distances.begin())->first;
        // shortest edge incident to neighbor node
        double min_inc_edge_neighbor = neighbor.distances.begin()->first;
        current_bound -= (second_min_inc_edge + min_inc_edge_neighbor) / 2.0;
      }

      // pruning --- only proceed if current lower bound is lower than
      // min_cost
      if (current_bound + current_cost < tsp_min_cost_) {
        current_path.push_back(i);
        visited.insert(i);

        tsp_explore(current_bound, current_cost, level + 1, visited,
                    current_path);

        visited.erase(i);
        current_path.pop_back();
      }

      // restore lower bound, running cost, visited node status
      current_bound = prev_bound;
      current_cost  = prev_cost;
    }
  }
}

}  // namespace graph_partitioner
}  // namespace cad2cav