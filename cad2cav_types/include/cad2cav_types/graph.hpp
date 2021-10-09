#ifndef __CAD2CAV_GRAPH_HPP__
#define __CAD2CAV_GRAPH_HPP__
#include <ros/ros.h>

#include <array>
#include <iostream>
#include <utility>
#include <vector>

#include "cad2cav_types/node.hpp"

namespace cad2cav {

class Graph {
public:
  enum CSR_Type {
    LINEAR = 0,  // linear mapping from output CSR edge weight to graph edge
                 // weight
    GAUSSIAN     // Gaussian similarity of graph edge weight as CSR output edge
                 // weight
  };

  // Read-only const references of private data
  const std::vector<std::pair<int, int>>& edge_directions;
  const std::vector<double>& edge_weights;

  /**
   * @brief Construct a new Graph object
   *
   */
  Graph();

  /**
   * @brief Copy constructor.
   *
   * @param other
   */
  Graph(const Graph& other);

  /**
   * @brief Adds a new node into graph
   *
   * @param node_x:   x coordinate of node
   * @param node_y:   y coordinate of node
   * @return int:     unique id of the new node in graph
   */
  int addNewNode(const double node_x, const double node_y) noexcept;

  /**
   * @brief Gets a reference to node from graph by node unique id
   *
   * @param id:       unique id of the node
   * @return Node&
   */
  const Node& getNode(const int id) const;

  /**
   * @brief Gets the size of the graph (no. of nodes)
   *
   * @return const int
   */
  const int size() const noexcept;

  /**
   * @brief Adds an edge between two nodes.
   *
   * NOTE. Upon adding the edge, node_from_id and node_to_id must be present
   * in the graph.
   *
   * @param node_from_id:     unique id of the start of edge.
   * @param node_to_id:       unique id of the end of edge.
   * @param edge_weight:      customized edge weight.
   *                          Default is the Euclidean distance between nodes.
   *                          Edge weight must be non-negative.
   * @return double:          length (weight) of edge
   */
  double addEdge(const int node_from_id, const int node_to_id,
                 double edge_weight = -1);

  /**
   * @brief Returns the CSR (Compressed Storage) format of the graph.
   *      Variable name is consistent with METIS manual 5.1.0 Section 5.5,
   *      which can be retrieved here:
   *      http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf
   *
   * @param out_xadj:     adjacency list index array. Length: n_nodes_ + 1
   * @param out_adjncy:   adjacency list. Length: 2*num_edges
   * @param out_adjwgt:   edge weight. Length: 2*num_edges
   * @param edge_type:    calculation mode for elements of out_adjwgt.
   *                      Can be LINEAR or GAUSSIAN
   */
  void getCSRFormat(std::vector<int>& out_xadj, std::vector<int>& out_adjncy,
                    std::vector<int>& out_adjwgt, CSR_Type edge_type) const;

  /**
   * @brief Overloaded operator<< for graph debug purposes
   *
   * @param o
   * @param graph
   * @return std::ostream&
   */
  friend std::ostream& operator<<(std::ostream& o, const Graph& graph);

protected:
  std::vector<Node> nodes_;
  int n_nodes_;

  /****************************************/
  // Edge information (for quick lookup)
  // {start_node_id, end_node_id}
  std::vector<std::pair<int, int>> edge_directions_;
  // edge weights corresponding to {start_node_id, end_node_id}
  // in edge_directions_
  std::vector<double> edge_weights_;

  Node& getNode(const int id);
};

std::ostream& operator<<(std::ostream& o, const Graph& graph);

}  // namespace cad2cav

#endif /* __CAD2CAV_GRAPH_HPP__ */
