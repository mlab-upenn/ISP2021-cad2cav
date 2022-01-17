#include "map_service/utils.hpp"

#include <fast-csv-reader/csv.h>
#include <ros/console.h>
#include <ros/package.h>

#include <filesystem>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace fs = std::filesystem;

namespace cad2cav {
namespace map_service {
namespace utils {

RevitInfo readRevitStructure(const std::string file_name,
                             const std::string file_dir) {
  fs::path package_path = ros::package::getPath("map_service");
  fs::path file_path    = package_path / file_dir / file_name;

  RevitInfo revit_info;
  revit_info.filename_ = file_name;
  revit_info.world_min_xy_ << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity();
  revit_info.world_max_xy_ << -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity();

  try {
    io::CSVReader<8> revit_reader(file_path);
    revit_reader.read_header(io::ignore_extra_column, "Type", "x_1", "y_1",
                             "z_1", "x_2", "y_2", "z_2", "Orientation");
    std::string object_type;
    double endpoint1_x = 0.0, endpoint1_y = 0.0, endpoint1_z = 0.0,
           endpoint2_x = 0.0, endpoint2_y = 0.0, endpoint2_z = 0.0,
           orientation = 0.0;
    while (revit_reader.read_row(object_type, endpoint1_x, endpoint1_y,
                                 endpoint1_z, endpoint2_x, endpoint2_y,
                                 endpoint2_z, orientation)) {
      // record world boundary
      if (std::min(endpoint1_x, endpoint2_x) < revit_info.world_min_xy_.x()) {
        revit_info.world_min_xy_.x() = std::min(endpoint1_x, endpoint2_x);
      }
      if (std::min(endpoint1_y, endpoint2_y) < revit_info.world_min_xy_.y()) {
        revit_info.world_min_xy_.y() = std::min(endpoint1_y, endpoint2_y);
      }
      if (std::max(endpoint1_x, endpoint2_x) > revit_info.world_max_xy_.x()) {
        revit_info.world_max_xy_.x() = std::max(endpoint1_x, endpoint2_x);
      }
      if (std::max(endpoint1_y, endpoint2_y) > revit_info.world_max_xy_.y()) {
        revit_info.world_max_xy_.y() = std::max(endpoint1_y, endpoint2_y);
      }

      // save Revit objects
      if (cad2cav::RevitObjectTypeFromString(object_type) ==
          cad2cav::RevitObjectType::WALL) {
        cad2cav::revit::Wall wall{endpoint1_x, endpoint1_y, endpoint2_x,
                                  endpoint2_y};
        revit_info.walls_.push_back(std::move(wall));
      } else if (cad2cav::RevitObjectTypeFromString(object_type) ==
                 cad2cav::RevitObjectType::DOOR) {
        cad2cav::revit::Door door{Eigen::Vector2d(endpoint1_x, endpoint1_y),
                                  0.0, 0.0, orientation};
        revit_info.doors_.push_back(std::move(door));
      } else if (cad2cav::RevitObjectTypeFromString(object_type) ==
                 cad2cav::RevitObjectType::WINDOW) {
        cad2cav::revit::Window window{Eigen::Vector2d(endpoint1_x, endpoint1_y),
                                      0.0, 0.0, orientation};
        revit_info.windows_.push_back(std::move(window));
      }
    }
  } catch (const io::error::can_not_open_file& e) {
    ROS_FATAL("%s", e.what());
    throw e;
  }

  return revit_info;
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

cv::Mat occupancyGridToCvImage(const nav_msgs::OccupancyGrid& occ_grid) {
  static const std::string map_layer = "base";

  grid_map::GridMap grid_map;
  if (!grid_map::GridMapRosConverter::fromOccupancyGrid(occ_grid, map_layer,
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

}  // namespace utils
}  // namespace map_service
}  // namespace cad2cav