#ifndef __CAD2CAV_LINE_HPP__
#define __CAD2CAV_LINE_HPP__

#include <Eigen/Dense>

namespace cad2cav {

class Line2D {
public:
  Line2D()
      : initialized_(false),
        origin_(Eigen::Vector2d::Zero()),
        dir_(Eigen::Vector2d::Zero()) {}
  Line2D(const Eigen::Vector2d& origin, const Eigen::Vector2d& dir)
      : initialized_(true), origin_(origin), dir_(dir) {}
  /**
   * @brief Initializes a line with origin and direction. Direction vector need
   * not be normalized.
   *
   * @param origin_x
   * @param origin_y
   * @param dir_x
   * @param dir_y
   */
  Line2D(double origin_x, double origin_y, double dir_x, double dir_y);
  /**
   * @brief Initializes a line with origin and a reference point. The line is
   * represented as origin + t * normalize(ref_pt - origin).
   *
   * @param origin
   * @param ref_pt
   */
  Line2D(const Eigen::Vector2d origin, const Eigen::Vector2d ref_pt);

  /********** GETTERS *********/
  bool isInitialized() const noexcept { return initialized_; }
  Eigen::Vector2d getOrigin() const noexcept { return origin_; }
  Eigen::Vector2d getDir() const noexcept { return dir_; }
  /****************************/

  /**
   * @brief Returns a point position following line's equations at t.
   *
   * @param t
   * @return Eigen::Vector3d: origin + t * dir
   */
  Eigen::Vector2d operator()(double t) const { return origin_ + t * dir_; }
  /**
   * @brief Computes if the line intersects with the other line.
   *
   * @param other:      the other line that needs to compute intersection
   * @param intersect:  output variable which stores the intersection point
   * @return bool, intersect
   */
  bool hasIntersection(const Line2D& other, Eigen::Vector2d& intersect) const;

protected:
  bool initialized_;
  Eigen::Vector2d origin_;
  Eigen::Vector2d dir_;
};

}  // namespace cad2cav

#endif /* __CAD2CAV_LINE_HPP__ */
