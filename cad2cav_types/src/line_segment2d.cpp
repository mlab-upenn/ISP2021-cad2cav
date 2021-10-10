#include "cad2cav_types/line_segment2d.hpp"

namespace cad2cav {

LineSegment2D::LineSegment2D(double endpoint1_x, double endpoint1_y,
                             double endpoint2_x, double endpoint2_y)
    : initialized_(true) {
  origin_   = Eigen::Vector2d(endpoint1_x, endpoint1_y);
  endpoint_ = Eigen::Vector2d(endpoint2_x, endpoint2_y);
  dir_      = (endpoint_ - origin_).normalized();
  length_   = (endpoint_ - origin_).norm();
}

LineSegment2D::LineSegment2D(const Eigen::Vector2d endpoint1,
                             const Eigen::Vector2d endpoint2)
    : initialized_(true) {
  origin_   = endpoint1;
  endpoint_ = endpoint2;
  dir_      = (endpoint2 - endpoint1).normalized();
  length_   = (endpoint2 - endpoint1).norm();
}

}  // namespace cad2cav