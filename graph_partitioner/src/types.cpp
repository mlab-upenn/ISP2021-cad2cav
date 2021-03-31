#include <algorithm>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/irange.hpp>
#include <graph_partitioner/types.hpp>

namespace graph_partitioner {

Graph::Graph()
    : edge_directions(edge_directions_),
      edge_weights(edge_weights_),
      n_nodes_(0) {}

Graph::Graph(const Graph& other)
    : edge_directions(edge_directions_),
      edge_weights(edge_weights_),
      nodes_(other.nodes_),
      n_nodes_(other.n_nodes_),
      edge_directions_(other.edge_directions_),
      edge_weights_(other.edge_weights_) {}

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

    start_node.neighbors[node_to_id] = edge_weight;

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

    // Assumes the problem size is small; use brute force search
    //  (generate all permutations)
    // initialize possible sequence
    std::vector<int> sequence;
    boost::push_back(sequence, boost::irange(n_nodes_));

    // initialize best sequence
    double min_cost = std::numeric_limits<double>::infinity();
    std::vector<int> min_cost_sequence;

    do {
        double total_cost = 0.0;
        bool valid_path   = true;

        for (unsigned int i = 0; i < sequence.size(); ++i) {
            int current_node_id = sequence[i];
            int next_node_id =
                (i == sequence.size() - 1) ? sequence.front() : sequence[i + 1];
            const auto& node = getNode(current_node_id);

            if (node.neighbors.count(next_node_id) > 0) {
                total_cost += node.neighbors.at(next_node_id);
            } else {
                valid_path = false;
                break;
            }
        }

        if (!valid_path)
            continue;
        else if (total_cost < min_cost) {
            min_cost          = total_cost;
            min_cost_sequence = sequence;
        }

    } while (std::next_permutation(sequence.begin(), sequence.end()));

    for (const auto& id : min_cost_sequence) {
        const auto& n = getNode(id);
        path.push_back({n.x, n.y});
    }
    path.push_back({getNode(min_cost_sequence.front()).x,
                    getNode(min_cost_sequence.front()).y});

    return path;
}

}  // namespace graph_partitioner
