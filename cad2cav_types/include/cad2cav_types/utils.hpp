#ifndef __CAD2CAD_TYPES_UTILS_HPP__
#define __CAD2CAD_TYPES_UTILS_HPP__

#include "cad2cav_types/graph.hpp"

namespace cad2cav {

/**
 * @brief Converts a user-defined graph into cad2cav::Graph
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
cad2cav::Graph fromUserGraph(const GraphType& user_graph,
                             bool switch_xy = false);

}  // namespace cad2cav

#include "utils_impl.hpp"

#endif /* __CAD2CAD_TYPES_UTILS_HPP__ */
