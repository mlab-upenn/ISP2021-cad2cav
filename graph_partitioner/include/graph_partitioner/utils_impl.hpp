#ifndef __GRAPH_PARTITIONER_UTILS_IMPL_HPP__
#define __GRAPH_PARTITIONER_UTILS_IMPL_HPP__

#include <boost/foreach.hpp>
#include <boost/range/combine.hpp>
#include <unordered_map>

#include "graph_partitioner/types.hpp"

namespace graph_partitioner {

/**
 * @brief Converts a user-defined graph into graph_partitioner::Graph
 *
 * @tparam GraphType:   user-defined graph.
 *                      Requires GraphType to be some iterable container of
 *                      nodes, (i.e., std::vector<Node>), in which Node must
 *                      have the following properties:
 *                      (1) an overloaded hash function for std::unordered_map;
 *                      (2) a member `x`;
 *                      (3) a member `y`;
 *                      (4) a member `neighbors` of container of Node*.
 *                          Container must be sequential container.
 *                      (5) a member `neighbors_cost` of sequential container of
 *                          double;
 *
 * @param user_graph:   input user graph
 * @param switch_xy:    whether node x/y in user graph is switched (used for
 *                          auto_mapping_ros graph)
 * @return Graph
 */
template <typename GraphType>
Graph fromUserGraph(const GraphType &user_graph, bool switch_xy) {
    Graph graph;
    std::unordered_map<typename GraphType::value_type, int> node_to_id_dict;

    for (const auto &node : user_graph) {
        int id = (switch_xy) ? graph.addNewNode(node.y, node.x)
                             : graph.addNewNode(node.x, node.y);
        node_to_id_dict.emplace(node, id);
    }

    for (const auto &node : user_graph) {
        int start_node_id = node_to_id_dict.at(node);

        typename GraphType::value_type *end_node;
        double distance;
        // works as a Python zip() function for C++ containers
        // https://www.boost.org/doc/libs/1_75_0/libs/range/doc/html/range/reference/utilities/combine.html
        BOOST_FOREACH (boost::tie(end_node, distance),
                       boost::combine(node.neighbors, node.neighbors_cost)) {
            int end_node_id = node_to_id_dict.at(*end_node);
            graph.addEdge(start_node_id, end_node_id, distance);
        }
    }

    return graph;
}

}  // namespace graph_partitioner

#endif /* __GRAPH_PARTITIONER_UTILS_IMPL_HPP__ */
