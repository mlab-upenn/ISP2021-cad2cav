#ifndef __GRAPH_PARTITIONER_UTILS_HPP__
#define __GRAPH_PARTITIONER_UTILS_HPP__

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
Graph fromUserGraph(const GraphType& user_graph, bool switch_xy = false);

}  // namespace graph_partitioner

#include "utils_impl.hpp"

#endif /* __GRAPH_PARTITIONER_UTILS_HPP__ */
