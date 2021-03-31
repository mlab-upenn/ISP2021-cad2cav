#ifndef __GRAPH_PARTITIONER_GRAPH_HPP__
#define __GRAPH_PARTITIONER_GRAPH_HPP__

#include <ros/ros.h>

#include <array>
#include <iostream>
#include <utility>
#include <vector>

#include "graph_partitioner/node.hpp"

namespace graph_partitioner {

class Graph {
public:
    // Read-only const references of private data
    const std::vector<std::pair<int, int>>& edge_directions;
    const std::vector<double>& edge_weights;

    typedef std::vector<std::array<double, 2>> Path;

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
     * @brief Regernerate the TSP sequence that covers all the nodes in the
     * graph. Sequence empty means failure.
     *
     * @return const Path
     */
    void updateTSPSequence() noexcept;

    /**
     * @brief Gets the current TSP sequence
     *
     * @return const Path
     */
    const Path getTSPSequence() const noexcept;

    /**
     * @brief Overloaded operator<< for graph debug purposes
     *
     * @param o
     * @param graph
     * @return std::ostream&
     */
    friend std::ostream& operator<<(std::ostream& o, const Graph& graph);

protected:
    Node& getNode(const int id);

private:
    std::vector<Node> nodes_;
    int n_nodes_;

    /****************************************/
    // Edge information (for quick lookup)
    // {start_node_id, end_node_id}
    std::vector<std::pair<int, int>> edge_directions_;
    // edge weights corresponding to {start_node_id, end_node_id}
    // in edge_directions_
    std::vector<double> edge_weights_;

    // TSP path minimal cost
    double tsp_min_cost_;
    // TSP sequence in node IDs
    std::vector<int> tsp_sequence_;

    /**
     * @brief recursive helper function for TSP Branch and Bound solver.
     *
     * @param current_bound:    lower bound estimate
     * @param current_cost:     running path cost
     * @param level:            recursion level
     * @param visited:          visited nodes (in node ID)
     * @param current_path:     running TSP path
     */
    void tsp_explore(double current_bound, double current_cost, double level,
                     std::set<int>& visited, std::vector<int>& current_path);
};

std::ostream& operator<<(std::ostream& o, const Graph& graph);

}  // namespace graph_partitioner

#endif /* __GRAPH_PARTITIONER_GRAPH_HPP__ */
