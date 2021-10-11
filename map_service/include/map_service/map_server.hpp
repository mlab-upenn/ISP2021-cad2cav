#ifndef __MAP_SERVICE_MAP_SERVER_HPP__
#define __MAP_SERVICE_MAP_SERVER_HPP__

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "cad2cav_types/types.hpp"
#include "map_service/revit_info.hpp"

namespace cad2cav {
namespace map_service {

class MapServer {
public:
  // Check nav_msgs/OccupancyGrid documentation for these values
  static constexpr int8_t CELL_FREE     = 0;
  static constexpr int8_t CELL_OCCUPIED = 100;
  static constexpr int8_t CELL_UNKNOWN  = -1;

  /**
   * @brief Construct a new MapServer
   *
   * @param resolution: map resolution. Unit in meter/cell
   * @param map_topic:  map publisher topic
   */
  MapServer(const double resolution     = 0.01,
            const std::string map_topic = "/cad2cav/map")
      : n_(ros::NodeHandle()), map_topic_(map_topic), resolution_(resolution) {
    map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);
  }
  /**
   * @brief Sets the timestamp for map_ and publishes the current map
   *
   * @param timestamp:  specified timestamp for map msg
   */
  void publishMap(ros::Time timestamp = ros::Time::now()) {
    map_.header.stamp = timestamp;
    map_pub_.publish(map_);
  }

  // Disable copy operations
  MapServer(const MapServer& other) = delete;
  MapServer& operator=(const MapServer& other) = delete;

  /**
   * @brief Reads in a RevitInfo struct and builds new map.
   *
   * @param revit_info
   */
  void buildFromRevitInfo(const RevitInfo& revit_info);
  /**
   * @brief Converts world coordinates to grid coordinates
   *
   * @param world_xy
   * @return Eigen::Vector2i
   */
  Eigen::Vector2i worldToGridCoordinates(const Eigen::Vector2d& world_xy);
  /**
   * @brief Converts grid coordinates (int-valued coordinate) to world
   * coordinates
   *
   * @param grid_xy
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d gridToWorldCoordinates(const Eigen::Vector2d& grid_xy);

private:
  ros::NodeHandle n_;
  ros::Publisher map_pub_;

  std::string map_topic_;
  double resolution_;  // m/cell
  nav_msgs::OccupancyGrid map_;

  Eigen::Vector2d world_min_xy_;
  Eigen::Vector2d world_max_xy_;

  int gridCoordToIndex(const Eigen::Vector2i& grid_xy) {
    return grid_xy.y() * map_.info.width + grid_xy.x();
  }

  /**
   * @brief Apply Bresenham's line algorithm to rasterize line segments in map.
   *  Bresenham's algorithm is preferred as it avoids floating point operations.
   *
   * Reference:
   * https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf
   *
   * @param startpoint: start point grid coordinates
   * @param endpoint:   end point grid coordinates
   */
  void setLineSegmentOnMap(const Eigen::Vector2i& startpoint_grid,
                           const Eigen::Vector2i& endpoint_grid);
};

}  // namespace map_service
}  // namespace cad2cav

#endif /* __MAP_SERVICE_MAP_SERVER_HPP__ */
