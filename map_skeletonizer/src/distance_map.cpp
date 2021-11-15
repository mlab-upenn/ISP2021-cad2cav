#include "map_skeletonizer/distance_map.hpp"

#include <ros/exception.h>

#include <boost/format.hpp>

namespace cad2cav {
namespace skeletonizer {

DistanceMapBuilder::DistanceMapBuilder(const nav_msgs::OccupancyGrid& occ_grid,
                                       int8_t cell_occupied_threshold)
    : map_width_(occ_grid.info.width), map_height_(occ_grid.info.height) {
  ROS_INFO("DistanceMap initialized with occupancy map: (%d, %d)", map_height_,
           map_width_);

  allocateEmptyDistMap(map_width_, map_height_);

  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      // if the cell is occupied in occupancy map, push it into `open` queue
      if (occ_grid.data[y * map_width_ + x] > cell_occupied_threshold &&
          !isOccupied(x, y)) {
        open_.push(makeObstacle(x, y));
      }
    }
  }
}

void DistanceMapBuilder::allocateEmptyDistMap(int map_width, int map_height) {
  dist_.resize(map_height * map_width, std::numeric_limits<double>::infinity());
  attrib_.resize(map_height * map_width, CellAttribute());
}

bool DistanceMapBuilder::isOutOfBound(int x, int y) const {
  return (x >= map_width_ || x < 0 || y >= map_height_ || y < 0);
}

bool DistanceMapBuilder::isOutOfBound(Eigen::Vector2i xy) const {
  return isOutOfBound(xy.x(), xy.y());
}

bool DistanceMapBuilder::isOutOfBound(int x, int y,
                                      std::string& err_msg) const {
  err_msg = (boost::format(
                 "DistanceMapBuilder: %1% (%2%, %3%) out of bound (%4%, %5%)") %
             err_msg % x % y % map_width_ % map_height_)
                .str();
  return (x >= map_width_ || x < 0 || y >= map_height_ || y < 0);
}

double& DistanceMapBuilder::distAt(int x, int y) {
  std::string err_msg = "distance map access";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }
  return dist_[y * map_width_ + x];
}

const double& DistanceMapBuilder::distAt(int x, int y) const {
  std::string err_msg = "distance map access";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }
  return dist_[y * map_width_ + x];
}

double& DistanceMapBuilder::distAt(Eigen::Vector2i xy) {
  return distAt(xy.x(), xy.y());
}

const double& DistanceMapBuilder::distAt(Eigen::Vector2i xy) const {
  return distAt(xy.x(), xy.y());
}

CellAttribute& DistanceMapBuilder::attributeAt(int x, int y) {
  std::string err_msg = "cell attribute access";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }
  return attrib_[y * map_width_ + x];
}

const CellAttribute& DistanceMapBuilder::attributeAt(int x, int y) const {
  std::string err_msg = "cell attribute access";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }
  return attrib_[y * map_width_ + x];
}

CellAttribute& DistanceMapBuilder::attributeAt(Eigen::Vector2i xy) {
  return attributeAt(xy.x(), xy.y());
}

const CellAttribute& DistanceMapBuilder::attributeAt(Eigen::Vector2i xy) const {
  return attributeAt(xy.x(), xy.y());
}

bool DistanceMapBuilder::isOccupied(int x, int y) const {
  std::string err_msg = "isOccupied";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }
  auto obs_pos = attributeAt(x, y).obs_pos;
  return (obs_pos.x() == x && obs_pos.y() == y);
}

bool DistanceMapBuilder::isOccupied(Eigen::Vector2i xy) const {
  return isOccupied(xy.x(), xy.y());
}

Cell DistanceMapBuilder::makeObstacle(int x, int y) const {
  std::string err_msg = "makeObstacle";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }

  Cell cell{Eigen::Vector2i(x, y), 0.0};
  cell.attrib.obs_pos = cell.pos;
  return cell;
}

Cell DistanceMapBuilder::makeFree(int x, int y) const {
  std::string err_msg = "makeFree";
  if (isOutOfBound(x, y, err_msg)) {
    throw ros::Exception(err_msg);
  }

  Cell cell{Eigen::Vector2i(x, y), std::numeric_limits<double>::infinity()};
  cell.attrib.obs_pos  = Eigen::Vector2i(-1, -1);
  cell.attrib.to_raise = true;
  return cell;
}

void DistanceMapBuilder::updateDistanceMap() {
  static const std::vector<Eigen::Vector2i> neighbor_cells{
      {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

  while (!open_.empty()) {
    Cell cell = open_.top();
    open_.pop();

    if (cell.attrib.to_raise) {
      // Raise
    } else if (isOccupied(cell.attrib.obs_pos)) {
      // Lower
      cell.attrib.on_voronoi = false;
      for (const auto& dxy : neighbor_cells) {
        Eigen::Vector2i neighbor_pos = cell.pos + dxy;
        if (isOutOfBound(neighbor_pos)) continue;
        if (!attributeAt(neighbor_pos).to_raise) {
          double new_distance =
              (cell.attrib.obs_pos - neighbor_pos).squaredNorm();
          if (new_distance < distAt(neighbor_pos)) {
            Cell neighbor_cell{neighbor_pos, new_distance};
            neighbor_cell.attrib.obs_pos = cell.attrib.obs_pos;
            open_.push(neighbor_cell);
          } else {
            // chkVoro(s, n);
          }
        }
      }
    }

    distAt(cell.pos)      = cell.dist;
    attributeAt(cell.pos) = cell.attrib;
  }
}

}  // namespace skeletonizer
}  // namespace cad2cav
