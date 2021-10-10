#ifndef __MAP_SERVER_UTILS_HPP__
#define __MAP_SERVER_UTILS_HPP__

#include "cad2cav_types/line_segment2d.hpp"
#include "cad2cav_types/revit_types.hpp"
#include <vector>
#include <string>

namespace cad2cav {
namespace map_service {
/**
 * @brief Reads revit exported building structure from CSV file.
 * File constains the following attributes:
 *
 * object type, endpoint1_x, endpoint1_y, endpoint1_z, endpoint2_x, endpoint2_y,
 * endpoint2_z
 *
 * This function only supports reading wall locations so far.
 *
 * @param file_name:    name of the csv file
 * @param file_dir:     dir of the csv file
 * @return std::vector<cad2cav::LineSegment2D>: list of walls
 */
std::vector<cad2cav::LineSegment2D> readRevitStructure(
    const std::string file_name, const std::string file_dir = "revit_export");

}  // namespace map_service
}  // namespace cad2cav

#endif /* __MAP_SERVER_UTILS_HPP__ */
