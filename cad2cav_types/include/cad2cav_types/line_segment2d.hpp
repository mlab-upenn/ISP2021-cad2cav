#ifndef __CAD2CAV_SEGMENT_2D_HPP__
#define __CAD2CAV_SEGMENT_2D_HPP__

#include <Eigen/Dense>
#include <vector>

namespace cad2cav {

class LineSegment2D {
  LineSegment2D()
      : initialized_(false),
        origin_(Eigen::Vector2d::Zero()),
        endpoint_(Eigen::Vector2d::Zero()),
        dir_(Eigen::Vector2d::Zero()),
        length_(0.0) {}
  LineSegment2D(const Eigen::Vector2d& origin, const Eigen::Vector2d& dir,
                const Eigen::Vector2d& endpoint)
      : initialized_(true), origin_(origin), dir_(dir), endpoint_(endpoint) {
    length_ = (endpoint_ - origin_).norm();
  }
  /**
   * @brief Initializes a line segment with endpoint1 and endpoint2.
   *
   * @param endpoint1_x
   * @param endpoint1_y
   * @param endpoint2_x
   * @param endpoint2_y
   */
  LineSegment2D(double endpoint1_x, double endpoint1_y, double endpoint2_x,
                double endpoint2_y);
  /**
   * @brief Initializes a line segment with endpoint1 and endpoint2.
   *
   * @param endpoint1
   * @param endpoint2
   */
  LineSegment2D(const Eigen::Vector2d endpoint1,
                const Eigen::Vector2d endpoint2);

  /********** GETTERS *********/
  bool isInitialized() const noexcept { return initialized_; }
  std::vector<Eigen::Vector2d> getEndpoints() const noexcept {
    return {origin_, endpoint_};
  }
  Eigen::Vector2d getDir() const noexcept { return dir_; }
  double getLength() const noexcept { return length_; }
  /****************************/

  /**
   * @brief Returns a point position on the line segment.
   *
   * @param lambda
   * @return Eigen::Vector3d: lambda * origin + (1 - lambda) * endpoint
   */
  Eigen::Vector2d operator()(double lambda) const {
    return lambda * origin_ + (1 - lambda) * endpoint_;
  }

private:
  bool initialized_;
  Eigen::Vector2d origin_;
  Eigen::Vector2d endpoint_;
  Eigen::Vector2d dir_;
  double length_;
};

}  // namespace cad2cav

#endif /* __CAD2CAV_SEGMENT_2D_HPP__ */
