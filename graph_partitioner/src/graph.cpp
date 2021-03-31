#include "graph_partitioner/graph.hpp"

#include <algorithm>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/irange.hpp>

namespace graph_partitioner {

Graph::Graph()
    : edge_directions(edge_directions_),
      edge_weights(edge_weights_),
      n_nodes_(0),
      tsp_min_cost_(std::numeric_limits<double>::infinity()) {}

Graph::Graph(const Graph& other)
    : edge_directions(edge_directions_),
      edge_weights(edge_weights_),
      nodes_(other.nodes_),
      n_nodes_(other.n_nodes_),
      edge_directions_(other.edge_directions_),
      edge_weights_(other.edge_weights_),
      tsp_min_cost_(other.tsp_min_cost_),
      tsp_sequence_(other.tsp_sequence_) {}

const int Graph::size() const noexcept { return n_nodes_; }

int Graph::addNewNode(const double node_x, const double node_y) noexcept {
    int new_node_id = this->n_nodes_++;
    this->nodes_.emplace_back(node_x, node_y, new_node_id);
    return new_node_id;
}

Node& Graph::getNode(const int id) {
    try {
        return this->nodes_.at(id);
    } catch (const std::out_of_range& e) {
        throw ros::Exception("node is not in the graph");
    } catch (const std::exception& e) {
        throw ros::Exception(e.what());
    }
}

const Node& Graph::getNode(const int id) const {
    try {
        return this->nodes_.at(id);
    } catch (const std::out_of_range& e) {
        throw ros::Exception("node is not in the graph");
    } catch (const std::exception& e) {
        throw ros::Exception(e.what());
    }
}

double Graph::addEdge(const int node_from_id, const int node_to_id,
                      double edge_weight) {
    Node& start_node = getNode(node_from_id);
    Node& end_node   = getNode(node_to_id);

    // does not allow pre-existence of edge
    if (start_node.neighbors.find(node_to_id) != start_node.neighbors.end()) {
        throw ros::Exception("add edge failure. edge already exists.");
    }

    if (edge_weight < 0)
        edge_weight = std::sqrt(std::pow(end_node.x - start_node.x, 2) +
                                std::pow(end_node.y - start_node.y, 2));

    start_node.neighbors_.insert({node_to_id, edge_weight});
    start_node.distances_.insert({edge_weight, node_to_id});

    this->edge_directions_.emplace_back(node_from_id, node_to_id);
    this->edge_weights_.push_back(edge_weight);

    return edge_weight;
}

std::ostream& operator<<(std::ostream& o, const Graph& graph) {
    o << "{ n_nodes: " << graph.n_nodes_ << ", node_xy: [";
    for (const auto& node : graph.nodes_) {
        o << "(" << node.x << ", " << node.y << "), ";
    }
    o << "] }";

    return o;
}

const Graph::Path Graph::getTSPSequence() const noexcept {
    Graph::Path path;

    // convert solved node id sequence into coord sequence
    for (const auto& id : tsp_sequence_) {
        const auto& n = getNode(id);
        path.push_back({n.x, n.y});
    }

    return path;
}

double Graph::updateTSPSequence(TSPSolver solver) noexcept {
    if (solver == TSPSolver::GOOGLE_ORTOOLS) {
        // clears previous TSP solutions
        tsp_min_cost_ = std::numeric_limits<double>::infinity();
        tsp_sequence_.clear();

        for (int i = 0; i < n_nodes_; ++i) {
            const auto [cost, sequence] = tsp_ortools_solver(i);
            if (static_cast<double>(cost) < tsp_min_cost_) {
                tsp_min_cost_ = static_cast<double>(cost);
                tsp_sequence_ = sequence;
            }
        }
    } else if (solver == TSPSolver::BRANCH_AND_BOUND) {
        tsp_branch_and_bound();
    }

    return tsp_min_cost_;
}

std::pair<double, std::vector<int>> Graph::tsp_ortools_solver(
    const int start_node_id) {
    // starting node of TSP
    operations_research::RoutingIndexManager::NodeIndex depot{start_node_id};
    // TSP solver manager
    operations_research::RoutingIndexManager manager(n_nodes_, 1, depot);
    // TSP solver model
    operations_research::RoutingModel routing(manager);

    // initialize a callback for retrieving distances between two nodes
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&](int64_t from_node, int64_t to_node) -> int64_t {
            // convert from OR Tools node id to Graph node id
            const auto start_node_id = manager.IndexToNode(from_node).value();
            const auto end_node_id   = manager.IndexToNode(to_node).value();

            // retrieve neighboring distance
            const auto node  = getNode(start_node_id);
            int64_t distance = node.neighbors.count(end_node_id) > 0
                                   ? static_cast<int64_t>(std::ceil(
                                         node.neighbors.at(end_node_id)))
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

void Graph::tsp_branch_and_bound() noexcept {
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
    for (const auto& node : nodes_) {
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
    for (int i = 0; i < n_nodes_; ++i) {
        sequence.push_back(i);
        visited_node_list.insert(i);
        tsp_explore(current_bound, 0.0, 1, visited_node_list, sequence);
        sequence.pop_back();
        visited_node_list.erase(i);
    }
}

void Graph::tsp_explore(double current_bound, double current_cost, double level,
                        std::set<int>& visited,
                        std::vector<int>& current_path) {
    if (level == n_nodes_) {
        // all nodes have been visited
        const auto& node = getNode(current_path.back());
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
        const auto& node = getNode(current_path.back());
        for (int i = 0; i < n_nodes_; ++i) {
            if (node.neighbors.count(i) == 0 || visited.count(i) > 0) continue;
            const auto& neighbor = getNode(i);

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
                double min_inc_edge_neighbor =
                    neighbor.distances.begin()->first;
                current_bound -= (min_inc_edge + min_inc_edge_neighbor) / 2.0;
            } else {
                // 2nd shortest edge incident to current node
                double second_min_inc_edge =
                    std::next(node.distances.begin())->first;
                // shortest edge incident to neighbor node
                double min_inc_edge_neighbor =
                    neighbor.distances.begin()->first;
                current_bound -=
                    (second_min_inc_edge + min_inc_edge_neighbor) / 2.0;
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
