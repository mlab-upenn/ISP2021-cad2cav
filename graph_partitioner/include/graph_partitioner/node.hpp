#ifndef __GRAPH_PARTITIONER_NODE_HPP__
#define __GRAPH_PARTITIONER_NODE_HPP__

#include <map>
#include <unordered_map>
#include <utility>

namespace graph_partitioner {

/**
 * @brief Node of the graph.
 *      node can be in any real coordinates.
 */
class Node {
public:
    // unique id
    const int& id;
    // location
    const double &x, &y;
    // neighbors --- node id with associated distances
    const std::unordered_map<int, double>& neighbors;
    // distances --- distance to neighboring nodes sort in ascending order
    const std::multimap<double, int>& distances;

    /**
     * @brief Construct a new Node object
     *
     * @param in_x
     * @param in_y
     * @param in_id
     */
    explicit Node(double in_x, double in_y, int in_id);

    /**
     * @brief Construct a new Node object
     *
     * @param coord
     * @param in_id
     */
    explicit Node(std::array<double, 2> coord, int in_id);

    /**
     * @brief Construct a new Node object
     *
     * @param other
     */
    Node(const Node& other);

    /**
     * @brief Copy id, x, y, neighbors
     *
     * @param other_node
     * @return Node&
     */
    Node& operator=(const Node& other);

private:
    int id_;
    double x_, y_;
    std::unordered_map<int, double> neighbors_;
    std::multimap<double, int> distances_;

    friend class Graph;
};

}  // namespace graph_partitioner

#endif /* __GRAPH_PARTITIONER_NODE_HPP__ */
