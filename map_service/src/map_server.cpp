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

  // 3. Save info of all doors
  doors_ = revit_info.doors_;
  ROS_INFO_STREAM("Door info received with size " << doors_.size());

  // 4. Save info of all windows
  windows_ = revit_info.windows_;
  ROS_INFO_STREAM("Window info received with size " << windows_.size());
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

cad2cav_msgs::LandmarkDetectionList MapServer::composeDoorLandmarkMsg(
    ros::Time timestamp) const {
  cad2cav_msgs::LandmarkDetectionList door_msg;
  door_msg.header.stamp = timestamp;

  cad2cav_msgs::LandmarkDetection door_detection;

  for (size_t i = 0; i < doors_.size(); ++i) {
    cad2cav_msgs::LandmarkEntry door_entry;
    door_entry.id                            = std::to_string(i);
    door_entry.pose_in_body_frame.position.x = doors_[i].pos_.x();
    door_entry.pose_in_body_frame.position.y = doors_[i].pos_.y();
    door_entry.pose_in_body_frame.position.z = 0.0;

    Eigen::AngleAxisd rot_aa{doors_[i].orientation_, Eigen::Vector3d::UnitZ()};
    Eigen::Quaterniond rot_quat(rot_aa);
    door_entry.pose_in_body_frame.orientation.w = rot_quat.w();
    door_entry.pose_in_body_frame.orientation.x = rot_quat.x();
    door_entry.pose_in_body_frame.orientation.y = rot_quat.y();
    door_entry.pose_in_body_frame.orientation.z = rot_quat.z();

    door_detection.landmark_list.push_back(door_entry);
  }

  door_msg.landmark_detections.push_back(door_detection);
  return door_msg;
}

cad2cav_msgs::LandmarkDetectionList MapServer::composeWindowLandmarkMsg(
    ros::Time timestamp) const {
  cad2cav_msgs::LandmarkDetectionList window_msg;
  window_msg.header.stamp = timestamp;

  cad2cav_msgs::LandmarkDetection window_detection;

  for (size_t i = 0; i < windows_.size(); ++i) {
    cad2cav_msgs::LandmarkEntry window_entry;
    window_entry.id                            = std::to_string(i);
    window_entry.pose_in_body_frame.position.x = windows_[i].pos_.x();
    window_entry.pose_in_body_frame.position.y = windows_[i].pos_.y();
    window_entry.pose_in_body_frame.position.z = 0.0;

    Eigen::AngleAxisd rot_aa{windows_[i].orientation_,
                             Eigen::Vector3d::UnitZ()};
    Eigen::Quaterniond rot_quat(rot_aa);
    window_entry.pose_in_body_frame.orientation.w = rot_quat.w();
    window_entry.pose_in_body_frame.orientation.x = rot_quat.x();
    window_entry.pose_in_body_frame.orientation.y = rot_quat.y();
    window_entry.pose_in_body_frame.orientation.z = rot_quat.z();

    window_detection.landmark_list.push_back(window_entry);
  }

  window_msg.landmark_detections.push_back(window_detection);
  return window_msg;
}

namespace {
cv::Mat transposeMapImage(const cv::Mat& map_image_original) {
  cv::Mat map_image{map_image_original.cols, map_image_original.rows, CV_8UC1},
      temp1{map_image_original.rows, map_image_original.cols, CV_8UC1},
      temp2{map_image_original.rows, map_image_original.cols, CV_8UC1};

  cv::flip(map_image_original, temp1, 0);
  cv::flip(temp1, temp2, 1);
  cv::transpose(temp2, map_image);

  cv::Mat map_image_colored;
  cv::cvtColor(map_image, map_image_colored, cv::COLOR_GRAY2BGR);

  return map_image_colored;
}
}  // namespace

cv::Mat MapServer::toCvImage() const {
  static const std::string map_layer = "base";

  grid_map::GridMap grid_map;
  if (!grid_map::GridMapRosConverter::fromOccupancyGrid(map_, map_layer,
                                                        grid_map)) {
    ROS_ERROR("Conversion to GridMap failed");
  }

  cv::Mat map_img;
  if (!grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
          grid_map, map_layer, CV_8UC1, 0, 100, map_img)) {
    ROS_ERROR("Conversion to cv::Mat failed");
  }
  return transposeMapImage(map_img);
}

}  // namespace map_service
}  // namespace cad2cav
