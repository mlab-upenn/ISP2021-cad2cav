#ifndef __CAD2CAV_TYPES_REVIT_TYPES_HPP__
#define __CAD2CAV_TYPES_REVIT_TYPES_HPP__

#include <algorithm>
#include <cctype>
#include <string>

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

}  // namespace cad2cav

#endif /* __CAD2CAV_TYPES_REVIT_TYPES_HPP__ */
