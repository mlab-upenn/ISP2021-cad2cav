#include "auto_mapping_ros/graph_builder_bp.hpp"

namespace amr {

std::array<cv::Point2f, 3> GraphBuilderBP::user_clicked_waypoints = {};

GraphBuilderBP::GraphBuilderBP(const cv::Mat& map)
    : GraphBuilder(cv::Mat(), map) {}

std::vector<cv::Point2f> GraphBuilderBP::readUnrealWaypoints(const std::string& path_to_csv) {
    std::vector<cv::Point2f> ret;

    try {
        io::CSVReader<3> waypoint_reader(path_to_csv);
        float x = 0.f, y = 0.f, z = 0.f;
        while (waypoint_reader.read_row(x, y, z)) {
            // simply discard z position & store into array
            ret.emplace_back(x, y);
        }

    } catch (const io::error::can_not_open_file& e) {
        ROS_FATAL("%s", e.what());
        throw e;
    }

    return ret;
}

void GraphBuilderBP::calibrateMapMouseCallback(int event, int x, int y, int flags, void* idx_ptr) {
    // adds current pixel coordinate to list upon user left click
    if (event == cv::EVENT_LBUTTONDOWN) {
        ROS_INFO_STREAM("Waypoint selected: (" << x << ", " << y << ")");
        unsigned int* idx = static_cast<unsigned int*>(idx_ptr);
        user_clicked_waypoints[(*idx)++] = cv::Point2f(x, y);
    }
}

std::vector<cv::Point2f> GraphBuilderBP::calibrateMap(const std::vector<cv::Point2f>& unreal_waypoints) {
    // pop up an openCV window & let user left-click on waypoints in the map
    const std::string window_name = "Waypoint selection --- press Space for finish";
    ROS_INFO(
        "Please left-click on the window for the first 3 consecutive waypoints "
        "that matches with the list from UE4 model.\n"
        "\t\tPress Space for finish.");
    unsigned int len = 0;
    cv::namedWindow(window_name, cv::WindowFlags::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, this->calibrateMapMouseCallback, static_cast<void*>(&len));
    do {
        cv::imshow(window_name, map_);
    } while (!(cv::waitKey(1) && len >= 3));
    cv::destroyWindow(window_name);

    // compute affine transformation matrix (from UE4 coords to pixel coords)
    if (unreal_waypoints.size() < user_clicked_waypoints.size())
        throw ros::Exception("user clicks more waypoints than UE4 waypoints");
    const std::vector<cv::Point2f> unreal_waypoints_in(unreal_waypoints.begin(), unreal_waypoints.begin() + 3);
    cv::Mat affineMatrix = cv::getAffineTransform(unreal_waypoints_in.data(), user_clicked_waypoints.data());
    // safety check
    //  if affineMatrix is all zeros, something went wrong with the affine transform fitting
    //  it is probably because the first 3 unreal waypoints does not define a valid triangle
    //  (hits the singularity of lease squares affine transformation fitting)
    if (cv::countNonZero(affineMatrix) < 1) {
        ROS_ERROR("Both first 3 UE4 waypoints & user defined waypoints should form a valid 2D triangle");
        throw ros::Exception("affine transformation hits singularity");
    }

    // transform every UE4 waypoint into pixel coordinates
    std::vector<cv::Point2f> map_waypoints(unreal_waypoints.size());
    cv::transform(unreal_waypoints, map_waypoints, affineMatrix);

    // return the tranformed UE4 waypoints
    return map_waypoints;
}

void GraphBuilderBP::build_graph(const std::string& path_to_csv) {
    // reads and transforms UE4 waypoint into map coordinates
    const auto unreal_waypoints = readUnrealWaypoints(path_to_csv);
    const auto map_waypoints = calibrateMap(unreal_waypoints);

    // constructs waypoints in graph format
    std::vector<std::array<int, 2>> map_waypoint_nodes;
    for (const auto& p : map_waypoints) {
        int intX = static_cast<int>(p.x), intY = static_cast<int>(p.y);
        map_waypoint_nodes.emplace_back(std::array<int, 2>{intY, intX});
    }

    // For debug
    ROS_WARN("The tranformed UE4 waypoints in map coordinates:");
    for (const auto& p : map_waypoint_nodes) {
        std::cout << "(" << p[1] << ", " << p[0] << ")\n";
    }

    // constructs graph
    construct_graph(map_waypoint_nodes);

    // visualizes graph
    visualize_graph(map_, graph_);
}

}  // namespace amr
