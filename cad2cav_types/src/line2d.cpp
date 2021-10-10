#include "cad2cav_types/line2d.hpp"

#include <Eigen/Geometry>

#include "cad2cav_types/utils.hpp"

namespace cad2cav {

Line2D::Line2D(double origin_x, double origin_y, double dir_x, double dir_y)
    : initialized_(true) {
  origin_ << origin_x, origin_y;
  dir_ << dir_x, dir_y;
  dir_.normalize();
}

Line2D::Line2D(const Eigen::Vector2d origin, const Eigen::Vector2d ref_pt)
    : initialized_(true) {
  origin_ = origin;
  dir_    = (ref_pt - origin).normalized();
}

bool Line2D::hasIntersection(const Line2D& other,
                             Eigen::Vector2d& intersect) const {
  using EigenLine2D = Eigen::Hyperplane<double, 2>;

  // if the two lines are parallel, no intersection
  Eigen::Vector3d dir1{dir_.x(), dir_.y(), 0.0};
  Eigen::Vector3d dir2{other.dir_.x(), other.dir_.y(), 0.0};
  if (almost_equal(dir1.cross(dir2).norm(), 0.0)) return false;

  // Use Eigen::Hyperplane to calculate intersection
  EigenLine2D line1 = EigenLine2D::Through(origin_, origin_ + 1.0 * dir_);
  EigenLine2D line2 =
      EigenLine2D::Through(other.origin_, other.origin_ + 1.0 * other.dir_);
  intersect = line1.intersection(line2);
  return true;
}

}  // namespace cad2cav
