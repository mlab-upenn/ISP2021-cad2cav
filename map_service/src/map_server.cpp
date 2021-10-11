#include "map_service/map_server.hpp"

namespace cad2cav {
namespace map_service {

void MapServer::buildFromRevitInfo(const RevitInfo& revit_info) {
  // 1. Allocate empty map. Make 1m extra room for the recorded min/max
  //  boundary.
  world_min_xy_ = revit_info.world_min_xy_ - Eigen::Vector2d(1.0, 1.0);
  world_max_xy_ = revit_info.world_max_xy_ + Eigen::Vector2d(1.0, 1.0);
  Eigen::Vector2d world_size = world_max_xy_ - world_min_xy_;

  map_.info.width =
      static_cast<uint32_t>(std::ceil(world_size.x() / resolution_));
  map_.info.height =
      static_cast<uint32_t>(std::ceil(world_size.y() / resolution_));
  map_.info.resolution        = static_cast<float>(resolution_);
  map_.info.origin.position.x = world_min_xy_.x();
  map_.info.origin.position.y = world_min_xy_.y();
  map_.data.resize(map_.info.width * map_.info.height);
  std::fill(map_.data.begin(), map_.data.end(), CELL_FREE);
  ROS_INFO_STREAM("Allocated occupancy map with size (height, width): "
                  << map_.info.height << "x" << map_.info.width << " = "
                  << map_.data.size());

  // 2. Loop over all walls, mark all cells along the wall to OCCUPIED
  for (const auto& wall : revit_info.walls_) {
    auto endpoints       = wall.shape_.getEndpoints();
    auto startpoint_grid = worldToGridCoordinates(endpoints[0]);
    auto endpoint_grid   = worldToGridCoordinates(endpoints[1]);
    if (startpoint_grid.x() >= 0 && startpoint_grid.y() >= 0 &&
        endpoint_grid.x() >= 0 && endpoint_grid.y() >= 0) {
      setLineSegmentOnMap(startpoint_grid, endpoint_grid);
    }
  }
  ROS_INFO_STREAM("Wall shape set in map successful");
}

void MapServer::setLineSegmentOnMap(const Eigen::Vector2i& startpoint_grid,
                                    const Eigen::Vector2i& endpoint_grid) {
  // Error checking
  if (startpoint_grid.x() < 0 ||
      startpoint_grid.x() >= static_cast<int>(map_.info.width) ||
      startpoint_grid.y() < 0 ||
      startpoint_grid.y() >= static_cast<int>(map_.info.height)) {
    ROS_ERROR("startpoint out of grid!");
    return;
  }
  if (endpoint_grid.x() < 0 ||
      endpoint_grid.x() >= static_cast<int>(map_.info.width) ||
      endpoint_grid.y() < 0 ||
      endpoint_grid.y() >= static_cast<int>(map_.info.height)) {
    ROS_ERROR("endpoint out of grid!");
    return;
  }

  // Start Bresenham
  Eigen::Vector2i pos_diff = endpoint_grid - startpoint_grid;
  int step_x = 0, step_y = 0;  // grid marching unit step (1 or -1)
  int dx = pos_diff.x(), dy = pos_diff.y();

  if (dy < 0) {
    dy     = -dy;
    step_y = -1;
  } else {
    step_y = 1;
  }
  if (dx < 0) {
    dx     = -dx;
    step_x = -1;
  } else {
    step_x = 1;
  }

  dy <<= 1;
  dx <<= 1;

  map_.data[gridCoordToIndex(startpoint_grid)] = CELL_OCCUPIED;
  map_.data[gridCoordToIndex(endpoint_grid)]   = CELL_OCCUPIED;
  Eigen::Vector2i current_point                = startpoint_grid;

  if (dx > dy) {
    int fraction = dy - (dx >> 1);
    while (current_point.x() != endpoint_grid.x()) {
      current_point.x() += step_x;
      if (fraction >= 0) {
        current_point.y() += step_y;
        fraction -= dx;
      }
      fraction += dy;
      map_.data[gridCoordToIndex(current_point)] = CELL_OCCUPIED;
    }
  } else {
    int fraction = dx - (dy >> 1);
    while (current_point.y() != endpoint_grid.y()) {
      if (fraction >= 0) {
        current_point.x() += step_x;
        fraction -= dy;
      }
      current_point.y() += step_y;
      fraction += dx;
      map_.data[gridCoordToIndex(current_point)] = CELL_OCCUPIED;
    }
  }
}

Eigen::Vector2i MapServer::worldToGridCoordinates(
    const Eigen::Vector2d& world_xy) {
  Eigen::Vector2i grid_coords =
      Eigen::round(((world_xy - world_min_xy_) / resolution_).array())
          .cast<int>();
  if (grid_coords.x() < 0 || grid_coords.y() < 0 ||
      grid_coords.x() >= static_cast<int>(map_.info.width) ||
      grid_coords.y() > static_cast<int>(map_.info.height)) {
    ROS_ERROR_STREAM("world coordinates out of map! ["
                     << grid_coords.transpose() << "]");
    return Eigen::Vector2i(-1, -1);
  }
  return grid_coords;
}

Eigen::Vector2d MapServer::gridToWorldCoordinates(
    const Eigen::Vector2d& grid_xy) {
  return world_min_xy_ + resolution_ * grid_xy;
}

}  // namespace map_service
}  // namespace cad2cav
