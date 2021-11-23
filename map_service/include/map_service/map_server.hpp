#ifndef __MAP_SERVICE_MAP_SERVER_HPP__
#define __MAP_SERVICE_MAP_SERVER_HPP__

#include <ros/ros.h>

#include <Eigen/Dense>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string>

#include "cad2cav_msgs/LandmarkDetectionList.h"
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

  // read-only member variables
  const ros::Publisher& map_pub;

  /**
   * @brief Construct a new MapServer
   *
   * @param resolution: map resolution. Unit in meter/cell
   * @param map_topic:  map publisher topic
   */
  MapServer(const double resolution        = 0.01,
            const std::string map_topic    = "/map",
            const std::string door_topic   = "/revit/doors",
            const std::string window_topic = "/revit/windows")
      : map_pub(map_pub_),
        n_(ros::NodeHandle()),
        map_topic_(map_topic),
        door_topic_(door_topic),
        window_topic_(window_topic),
        resolution_(resolution) {
    map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);
    door_pub_ =
        n_.advertise<cad2cav_msgs::LandmarkDetectionList>(door_topic_, 1);
    window_pub_ =
        n_.advertise<cad2cav_msgs::LandmarkDetectionList>(window_topic_, 1);
  }
  /**
   * @brief Sets the timestamp for map_ and publishes the current map
   *
   * @param timestamp:  specified timestamp for map msg
   */
  void publish(ros::Time timestamp = ros::Time::now()) {
    map_.header.stamp = timestamp;
    map_pub_.publish(map_);
    door_pub_.publish(composeDoorLandmarkMsg(timestamp));
    window_pub_.publish(composeWindowLandmarkMsg(timestamp));
  }
  /**
   * @brief Get the current Map Info
   *
   * @return nav_msgs::MapMetaData
   */
  nav_msgs::MapMetaData getMapInfo() const { return map_.info; }
  /**
   * @brief Get all the door objects
   *
   * @return const std::vector<cad2cav::revit::Door>&
   */
  const std::vector<cad2cav::revit::Door>& getDoors() const { return doors_; }
  /**
   * @brief Get all the window objects
   *
   * @return const std::vector<cad2cav::revit::Window>&
   */
  const std::vector<cad2cav::revit::Window>& getWindows() const {
    return windows_;
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
  /**
   * @brief Converts occupancy grid data to GridMap.
   *
   * @return grid_map::GridMap
   */
  cv::Mat toCvImage() const;

private:
  ros::NodeHandle n_;
  ros::Publisher map_pub_;
  ros::Publisher door_pub_;
  ros::Publisher window_pub_;

  std::string map_topic_;
  std::string door_topic_;
  std::string window_topic_;

  double resolution_;  // m/cell
  nav_msgs::OccupancyGrid map_;

  Eigen::Vector2d world_min_xy_;
  Eigen::Vector2d world_max_xy_;

  std::vector<cad2cav::revit::Door> doors_;
  std::vector<cad2cav::revit::Window> windows_;

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

  /**
   * @brief Composes door msg for landmark detection ground truth reference
   *
   * @param timestamp
   * @return cad2cav_msgs::LandmarkDetectionList
   */
  cad2cav_msgs::LandmarkDetectionList composeDoorLandmarkMsg(
      ros::Time timestamp) const;

  /**
   * @brief Composes window msg for landmark detection ground truth reference
   *
   * @param timestamp
   * @return cad2cav_msgs::LandmarkDetectionList
   */
  cad2cav_msgs::LandmarkDetectionList composeWindowLandmarkMsg(
      ros::Time timestamp) const;
};

}  // namespace map_service
}  // namespace cad2cav

#endif /* __MAP_SERVICE_MAP_SERVER_HPP__ */
