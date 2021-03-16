#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>
namespace fs = std::filesystem;

#include "../aco_router/utils.h"
#include "../aco_router/vrp_solver.h"
#include "auto_mapping_ros/graph_builder_bp.hpp"
#include "auto_mapping_ros/utils.h"
#include "fast_csv_parser/csv.h"

int main(int argc, char const* argv[]) {
    const fs::path amr_package_path = ros::package::getPath("auto_mapping_ros");
    const fs::path simulator_package_path = ros::package::getPath("f110_simulator");
    const fs::path cad_map_filepath = simulator_package_path / "maps/levine_4.dwg.png";
    const fs::path unreal_waypoints_path = amr_package_path / "csv/actorLocation.csv";

    cv::Mat map = cv::imread(cad_map_filepath, cv::IMREAD_GRAYSCALE);

    amr::GraphBuilderBP graph_builder(map);
    graph_builder.build_graph(unreal_waypoints_path);
    auto graph = graph_builder.get_graph();
    auto aco_graph = aco::convert_to_aco_graph(graph);

    // Get Initial Point
    int init_id = amr::get_closest_clicked_node_on_map(map, aco_graph);

    // Find Best Sequence
    const std::string csv_filepath = amr_package_path / "csv/sequence";
    const auto sequences_node = aco::solve_vrp(aco_graph, init_id);

    for (int i = 0; i < sequences_node.first.size(); i++) {
        std::vector<std::array<int, 2>> current_sequence{};
        for (const auto& node : sequences_node.first[i]) {
            current_sequence.emplace_back(std::array<int, 2>{static_cast<int>(node.x), static_cast<int>(node.y)});
        }
        const auto current_csv_filepath = csv_filepath + "_" + std::to_string(i + 1) + ".csv";
        amr::write_sequence_to_csv(current_sequence, current_csv_filepath);
        amr::visualize_sequence_on_graph(map, graph, current_sequence);
    }

    return 0;
}
