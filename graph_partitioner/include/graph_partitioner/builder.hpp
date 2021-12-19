#ifndef __CAD2CAV_GRAPH_PARTITIONER_GRAPH_BUILDER_HPP__
#define __CAD2CAV_GRAPH_PARTITIONER_GRAPH_BUILDER_HPP__

#include <Eigen/Dense>

#include "cad2cav_types/types.hpp"

namespace cad2cav {
namespace graph_partitioner {

class GraphBuilder {
public:
  GraphBuilder() : is_graph_constructed_(false) {}

  cad2cav::Graph getGraph() const { return graph_; }
  nav_msgs::OccupancyGrid getMap() const { return map_; }
  void updateMap(const nav_msgs::OccupancyGrid& new_map) { map_ = new_map; }

  bool buildGraph(const std::vector<Eigen::Vector2d>& waypoints);

private:
  cad2cav::Graph graph_;
  nav_msgs::OccupancyGrid map_;  // stores the current copy of map

  bool is_graph_constructed_;
};

}  // namespace graph_partitioner
}  // namespace cad2cav

#endif /* __CAD2CAV_GRAPH_PARTITIONER_GRAPH_BUILDER_HPP__ */
