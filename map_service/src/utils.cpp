#include "map_service/utils.hpp"
#include <fast-csv-reader/csv.h>
#include <ros/package.h>
#include <ros/console.h>
#include <filesystem>
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
    io::CSVReader<7> revit_reader(file_path);
    std::string object_type;
    double endpoint1_x = 0.0, endpoint1_y = 0.0, endpoint1_z = 0.0,
           endpoint2_x = 0.0, endpoint2_y = 0.0, endpoint2_z = 0.0;
    while (revit_reader.read_row(object_type, endpoint1_x, endpoint1_y,
                                 endpoint1_z, endpoint2_x, endpoint2_y,
                                 endpoint2_z)) {
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
      }
    }
  } catch (const io::error::can_not_open_file& e) {
    ROS_FATAL("%s", e.what());
    throw e;
  }

  return revit_info;
}

}  // namespace utils
}  // namespace map_service
}  // namespace cad2cav