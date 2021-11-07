#ifndef __CAD2CAV_TYPES_REVIT_TYPES_HPP__
#define __CAD2CAV_TYPES_REVIT_TYPES_HPP__

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <string>

#include "cad2cav_types/line_segment2d.hpp"

namespace cad2cav {

enum class RevitObjectType { UNKNOWN = 0, WALL, DOOR, WINDOW };

RevitObjectType RevitObjectTypeFromString(const std::string& data) {
  std::string name = data;
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (name == "wall")
    return RevitObjectType::WALL;
  else if (name == "door")
    return RevitObjectType::DOOR;
  else if (name == "window")
    return RevitObjectType::WINDOW;

  return RevitObjectType::UNKNOWN;
}

namespace revit {

struct Wall {
  LineSegment2D shape_;
  double height_;  // unused
  Wall() : shape_(), height_(-1.0) {}
  Wall(double endpoint1_x, double endpoint1_y, double endpoint2_x,
       double endpoint2_y, double height = -1.0)
      : shape_(endpoint1_x, endpoint1_y, endpoint2_x, endpoint2_y),
        height_(height) {}
  Wall(const Eigen::Vector2d& endpoint1, const Eigen::Vector2d& endpoint2,
       double height = -1.0)
      : shape_(endpoint1, endpoint2), height_(height) {}
};

struct Door {
  Eigen::Vector2d pos_;
  double width_;
  double height_;
  double orientation_;
};

struct Window {
  Eigen::Vector2d pos_;
  double width_;
  double height_;
  double orientation_;
};

}  // namespace revit
}  // namespace cad2cav

#endif /* __CAD2CAV_TYPES_REVIT_TYPES_HPP__ */
