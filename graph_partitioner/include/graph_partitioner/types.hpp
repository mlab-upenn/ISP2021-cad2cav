#ifndef __GRAPH_PARTITIONER_TYPES_HPP__
#define __GRAPH_PARTITIONER_TYPES_HPP__

#include <ros/ros.h>

#include <array>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace graph_partitioner {

/**
 * @brief Node of the graph.
 *      node can be in any real coordinates.
 */
struct Node {
    // unique id
    int id;
    // location
    double x, y;
    // neighbors --- node id with associated distances
    std::unordered_map<int, double> neighbors;

    /**
     * @brief Construct a new Node object
     *
     * @param in_x
     * @param in_y
     * @param in_id
     */
    explicit Node(double in_x, double in_y, int in_id)
        : id(in_id), x(in_x), y(in_y) {}

    /**
     * @brief Construct a new Node object
     *
     * @param coord
     * @param in_id
     */
    explicit Node(std::array<double, 2> coord, int in_id)
        : id(in_id), x(coord[0]), y(coord[1]) {}

    /**
     * @brief Construct a new Node object
     *
     * @param other
     */
    Node(const Node& other)
        : id(other.id), x(other.x), y(other.y), neighbors(other.neighbors) {}

    /**
     * @brief Copy id, x, y, neighbors
     *
     * @param other_node
     * @return Node&
     */
    Node& operator=(const Node& other) {
        id        = other.id;
        x         = other.x;
        y         = other.y;
        neighbors = other.neighbors;
        return *this;
    }
};

class Graph {
public:
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
};

std::ostream& operator<<(std::ostream& o, const Graph& graph);

}  // namespace graph_partitioner

#endif /* __GRAPH_PARTITIONER_TYPES_HPP__ */
