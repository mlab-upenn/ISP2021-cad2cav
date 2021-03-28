#include <graph_partitioner/types.hpp>

namespace graph_partitioner {

Graph::Graph() : n_nodes_(0) {}

int Graph::addNewNode(const double node_x, const double node_y) noexcept {
    int new_node_id = this->n_nodes_++;
    this->nodes_.emplace_back(node_x, node_y, new_node_id);
    return new_node_id;
}

Node& Graph::getNode(const int id) {
    try {
        return this->nodes_.at(id);
    } catch (const std::exception& e) {
        throw ros::Exception(e.what());
    }
}

const Node& Graph::getNode(const int id) const {
    try {
        return this->nodes_.at(id);
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

}  // namespace graph_partitioner
