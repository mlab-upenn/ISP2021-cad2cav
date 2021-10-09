#include "cad2cav_types/graph.hpp"

#include <algorithm>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/irange.hpp>

namespace cad2cav {

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
  Node& end_node = getNode(node_to_id);

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

void Graph::getCSRFormat(std::vector<int>& out_xadj,
                         std::vector<int>& out_adjncy,
                         std::vector<int>& out_adjwgt,
                         CSR_Type edge_type) const {
  out_xadj.push_back(out_adjncy.size());

  if (edge_type == CSR_Type::LINEAR) {
    for (int i = 0; i < n_nodes_; ++i) {
      const Node& node = getNode(i);
      for (const auto& p : node.neighbors) {
        out_adjncy.push_back(p.first);
        out_adjwgt.push_back(static_cast<int>(std::round(p.second)));
      }
      out_xadj.push_back(out_adjncy.size());
    }
  } else if (edge_type == CSR_Type::GAUSSIAN) {
    // get max weight among all edges of graph for further
    //  `Gaussian similarity metric` normalization
    const double max_distance =
        *std::max_element(edge_weights.begin(), edge_weights.end());

    for (int i = 0; i < n_nodes_; ++i) {
      const Node& node = getNode(i);
      for (const auto& p : node.neighbors) {
        out_adjncy.push_back(p.first);

        // compute Gaussian similarity
        //  set sigma to be 0.5*max_distance
        double similarity =
            std::exp(-1 * std::pow(p.second / (max_distance / 2), 2) / 2);
        // add a scaling factor of 100 to ensure sufficient precision of
        // edge weight after casting to int
        double scale_factor = 100.0;

        out_adjwgt.push_back(
            static_cast<int>(std::round(scale_factor * similarity)));
      }
      out_xadj.push_back(out_adjncy.size());
    }
  } else {
    ROS_FATAL("Unrecognized CSR edge calculation mode");
    throw ros::Exception("Unrecognized CSR edge calculation mode");
  }
}

}  // namespace cad2cav
