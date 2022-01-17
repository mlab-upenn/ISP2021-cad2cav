#ifndef __MAP_SKELETONIZER_DISTANCE_MAP_HPP__
#define __MAP_SKELETONIZER_DISTANCE_MAP_HPP__

/**
 * @file distance_map.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 *
 * @brief Implements distance map update strategy in:
 *    B. Lau, C. Sprunk and W. Burgard, "Improved updating of Euclidean distance
 *  maps and Voronoi diagrams," IROS 2010.
 *
 * @date 2021-11-14
 */

#include <Eigen/Dense>
#include <limits>
#include <queue>
#include <vector>

#include "cad2cav_types/types.hpp"

namespace cad2cav {
namespace skeletonizer {

struct CellAttribute {
  Eigen::Vector2i obs_pos;  // position of the closest obstacle in map
  bool to_raise;
  bool on_voronoi;

  CellAttribute() : obs_pos(-1, -1), to_raise(false), on_voronoi(false) {}
};

struct Cell {
  // cell position in map
  Eigen::Vector2i pos;
  // distance to closest obstacle in map
  double dist;
  // may be the same attribute as in attrib_, or value-to-set
  CellAttribute attrib;

  Cell() : pos(-1, -1), dist(std::numeric_limits<double>::infinity()) {}
  Cell(Eigen::Vector2i in_pos, double in_dist) : pos(in_pos), dist(in_dist) {}
};

bool operator>(const Cell& a, const Cell& b) { return a.dist > b.dist; }

class DistanceMapBuilder {
public:
  DistanceMapBuilder() : map_width_(-1), map_height_(-1) {}
  DistanceMapBuilder(const nav_msgs::OccupancyGrid& occ_grid,
                     int8_t cell_occupied_threshold = 50);
  /**
   * @brief Gets the current distance map. Stored in row-major order.
   *
   * @return const std::vector<double>&
   */
  const std::vector<double>& getDistanceMap() const noexcept { return dist_; }
  /**
   * @brief Gets the distance to closest obstacle of cell (x,y).
   *
   * @param x
   * @param y
   * @return double&
   */
  double& distAt(int x, int y);
  double& distAt(Eigen::Vector2i xy);
  const double& distAt(int x, int y) const;
  const double& distAt(Eigen::Vector2i xy) const;
  /**
   * @brief Gets the attribute of cell (x,y).
   *
   * @param x
   * @param y
   * @return CellAttribute&
   */
  CellAttribute& attributeAt(int x, int y);
  CellAttribute& attributeAt(Eigen::Vector2i xy);
  const CellAttribute& attributeAt(int x, int y) const;
  const CellAttribute& attributeAt(Eigen::Vector2i xy) const;
  /**
   * @brief Check if (x,y) is an obstacle.
   *  Only depends on whether attrib_[x,y].obs_pos == (x,y).
   *
   * @param x
   * @param y
   * @return true
   * @return false
   */
  bool isOccupied(int x, int y) const;
  bool isOccupied(Eigen::Vector2i xy) const;
  /**
   * @brief Makes (x,y) as a new obstacle, and starts a new wave propagation.
   *
   * @param x
   * @param y
   * @return Cell
   */
  Cell makeObstacle(int x, int y) const;
  /**
   * @brief Makes (x,y) as a free cell (unoccupied), and starts a new wave
   * propagation.
   *  Sets: cell.dist = inf, cell.attrib.obs_pos = (-1,-1).
   *
   * @param x
   * @param y
   * @return Cell
   */
  Cell makeFree(int x, int y) const;
  /**
   * @brief Updates distance map with `open` queue and brushfire algorithm.
   *
   * Reference:
   * https://cs.gmu.edu/~kosecka/cs685/cs685-motion-planning.pdf
   *
   */
  void updateDistanceMap();

private:
  int map_width_;
  int map_height_;

  // distance map
  std::vector<double> dist_;
  // attributes for each cell in distance map
  std::vector<CellAttribute> attrib_;
  // `open` priority queue
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open_;

  /**
   * @brief Boundary check for map element/attribute access.
   *
   * @param x
   * @param y
   */
  bool isOutOfBound(int x, int y) const;
  bool isOutOfBound(Eigen::Vector2i xy) const;
  bool isOutOfBound(int x, int y, std::string& err_msg) const;
  /**
   * @brief Resize dist_ and attrib_ to default empty values of size
   *   (map_height, map_width).
   *
   * @param map_width
   * @param map_height
   */
  void allocateEmptyDistMap(int map_width, int map_height);
};

}  // namespace skeletonizer
}  // namespace cad2cav

#endif /* __MAP_SKELETONIZER_DISTANCE_MAP_HPP__ */
