#ifndef __MAP_SERVICE_REVIT_INFO_HPP__
#define __MAP_SERVICE_REVIT_INFO_HPP__

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "cad2cav_types/revit_types.hpp"

namespace cad2cav {
namespace map_service {

struct RevitInfo {
  // identifier
  std::string filename_;
  // bottom-left boundary of the world (x_min, y_min)
  Eigen::Vector2d world_min_xy_;
  // top-right boundary of the world (x_max, y_max)
  Eigen::Vector2d world_max_xy_;

  std::vector<cad2cav::revit::Wall> walls_;
  std::vector<cad2cav::revit::Door> doors_;
  std::vector<cad2cav::revit::Window> windows_;
};

}  // namespace map_service
}  // namespace cad2cav

#endif /* __MAP_SERVICE_REVIT_INFO_HPP__ */
