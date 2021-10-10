#ifndef __CAD2CAD_TYPES_UTILS_HPP__
#define __CAD2CAD_TYPES_UTILS_HPP__

#include <limits>
#include <type_traits>

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

/**
 * @brief Referenced at
 * https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 *
 * @tparam T
 * @param x
 * @param y
 * @param ulp
 * @return std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
 */
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp = 2) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <=
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

}  // namespace cad2cav

#include "utils_impl.hpp"

#endif /* __CAD2CAD_TYPES_UTILS_HPP__ */
