#ifndef __GRAPH_BUILDER_BP_HPP__
#define __GRAPH_BUILDER_BP_HPP__

/**
 * @file graph_builder_bp.hpp
 * @author Zhihao Ruan (ruanzh@seas.upenn.edu)
 * @brief A wrapper for graph builder which reads from CAD blueprint screenshot & unreal waypoints  
 * @date 2021-03-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>

#include <array>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "auto_mapping_ros/graph_builder.h"
#include "auto_mapping_ros/utils.h"
#include "fast_csv_parser/csv.h"

namespace amr {

/**
 * @brief GraphBuilderBP --- Graph Builder "Blueprint"
 *  This class reads map from Autodesk Viewer screenshots & waypoints from building UE4 models.
 *  For details of UE4 building model construction see https://github.com/shineyruan/unreal_levine_4
 */
class GraphBuilderBP : public GraphBuilder {
public:
    /**
     * @brief Construct a new GraphBuilderBP object
     * 
     * @param map 
     */
    explicit GraphBuilderBP(const cv::Mat& map);

    /**
     * @brief builds the graph 
     * 
     * @param path_to_csv:  path to the pre-defined UE4 waypoints
     */
    void build_graph(const std::string& path_to_csv);

    /**
     * @brief builds the graph
     * 
     * @param waypoints:    list of waypoint (landmark) positions
     */
    void build_graph(const std::vector<cv::Point2f>& waypoints);

protected:
    /**
     * @brief Reads pre-defined waypoints from a CSV file generated from UE4 model
     * 
     * @param path_to_csv 
     */
    std::vector<cv::Point2f> readUnrealWaypoints(const std::string& path_to_csv);

    /**
     * @brief Computes the transformation from UE4 coordinates to map coordinates
     * 
     * @param unreal_waypoints:             UE4 waypoints in UE4 coordinates
     * @return std::vector<cv::Point2f>:    UE4 waypoints in map pixel coordinates
     */
    std::vector<cv::Point2f> calibrateMap(const std::vector<cv::Point2f>& unreal_waypoints);

private:
    /**
     * @brief OpenCV callback function for mouse left-click on waypoints matching
     * 
     * @param event 
     * @param x 
     * @param y 
     * @param flags 
     * @param userData 
     */
    static void calibrateMapMouseCallback(int event, int x, int y, int flags, void* userData);

    static std::array<cv::Point2f, 3> user_clicked_waypoints;
};

}  // namespace amr

#endif /* __GRAPH_BUILDER_BP_HPP__ */
