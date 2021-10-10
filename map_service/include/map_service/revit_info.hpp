#ifndef __MAP_SERVICE_REVIT_INFO_HPP__
#define __MAP_SERVICE_REVIT_INFO_HPP__

#include "cad2cav_types/revit_types.hpp"
#include <vector>
#include <string>

namespace cad2cav {
namespace map_service {

struct RevitInfo {
  std::string filename_;
  std::vector<cad2cav::revit::Wall> walls_;
  std::vector<cad2cav::revit::Door> doors_;
  std::vector<cad2cav::revit::Window> windows_;
};

}  // namespace map_service
}  // namespace cad2cav

#endif /* __MAP_SERVICE_REVIT_INFO_HPP__ */
