#include "cad2cav_types/node.hpp"

namespace cad2cav {

Node::Node(double in_x, double in_y, int in_id)
    : id(id_),
      x(x_),
      y(y_),
      neighbors(neighbors_),
      distances(distances_),
      id_(in_id),
      x_(in_x),
      y_(in_y) {}

Node::Node(std::array<double, 2> coord, int in_id)
    : id(id_),
      x(x_),
      y(y_),
      neighbors(neighbors_),
      distances(distances_),
      id_(in_id),
      x_(coord[0]),
      y_(coord[1]) {}

Node::Node(const Node& other)
    : id(id_),
      x(x_),
      y(y_),
      neighbors(neighbors_),
      distances(distances_),
      id_(other.id),
      x_(other.x_),
      y_(other.y_),
      neighbors_(other.neighbors_),
      distances_(other.distances_) {}

Node& Node::operator=(const Node& other) {
  id_ = other.id_;
  x_ = other.x_;
  y_ = other.y_;
  neighbors_ = other.neighbors_;
  distances_ = other.distances_;
  return *this;
}

}  // namespace cad2cav
