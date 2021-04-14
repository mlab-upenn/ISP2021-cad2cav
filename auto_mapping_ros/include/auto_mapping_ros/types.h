#ifndef AUTO_MAPPING_ROS_TYPES_H
#define AUTO_MAPPING_ROS_TYPES_H

#include <array>
#include <functional>
#include <vector>

/// Basic Cell of the Graph
struct Node {
    explicit Node(const std::array<int, 2> &node, int id);
    explicit Node(int x, int y, int id);

    int id;
    int x;
    int y;

    // Graph Node Neighbor Variables
    std::vector<Node *> neighbors;
    std::vector<double> neighbors_cost;

    // Landmark Information
    int n_lane_intersections;

    bool operator==(const Node &rhs) const;
    std::array<int, 2> get_location() const;
};

// Overload of C++ std template hash for customized class `Node`
//  Reference: https://stackoverflow.com/a/17017281/9960905
template <>
struct std::hash<Node> {
    std::size_t operator()(const Node &n) const {
        return std::hash<int>()(n.id);
    }
};

using Graph = std::vector<Node>;
Graph deep_copy_graph(const Graph &graph);

struct Plan {
    std::array<double, 2> start;
    std::array<double, 2> end;
    std::vector<std::array<double, 2>> plan;
};

#endif  //AUTO_MAPPING_ROS_TYPES_H
