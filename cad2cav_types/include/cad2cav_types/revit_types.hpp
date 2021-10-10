#ifndef __CAD2CAV_TYPES_REVIT_TYPES_HPP__
#define __CAD2CAV_TYPES_REVIT_TYPES_HPP__

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <string>

#include "cad2cav_types/line_segment2d.hpp"

namespace cad2cav {

enum class RevitObjectType { WALL = 0, DOOR, WINDOW };

RevitObjectType RevitObjectTypeFromString(const std::string& data) {
  std::transform(data.begin(), data.end(), data.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (data == "wall")
    return RevitObjectType::WALL;
  else if (data == "door")
    return RevitObjectType::DOOR;
  else if (data == "window")
    return RevitObjectType::WINDOW;
}

namespace revit {

struct Wall {
  LineSegment2D shape_;
  double height_;  // unused
  Wall() : shape_(), height_(-1.0) {}
  Wall(double endpoint1_x, double endpoint1_y, double endpoint2_x,
       double endpoint2_y, double height)
      : shape_(endpoint1_x, endpoint1_y, endpoint2_x, endpoint2_y),
        height_(height) {}
  Wall(const Eigen::Vector2d& endpoint1, const Eigen::Vector2d& endpoint2,
       double height)
      : shape_(endpoint1, endpoint2), height_(height) {}
};

struct Door {
  Eigen::Vector2d pos_;
  double width_;
  double height_;
};

struct Window {
  Eigen::Vector2d pos_;
  double width_;
  double height_;
};

}  // namespace revit
}  // namespace cad2cav

#endif /* __CAD2CAV_TYPES_REVIT_TYPES_HPP__ */
