#include <ros/package.h>
#include <ros/ros.h>

#include <filesystem>
namespace fs = std::filesystem;

#include "../aco_router/utils.h"
#include "../aco_router/vrp_solver.h"
#include "auto_mapping_ros/graph_builder_bp.hpp"
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros/utils.h"
#include "fast_csv_parser/csv.h"
#include "graph_partitioner/partitioner.hpp"
#include "graph_partitioner/utils.hpp"

std::pair<Graph, cv::Mat> get_cad_graph() {
    const fs::path amr_package_path = ros::package::getPath("auto_mapping_ros");
    const fs::path simulator_package_path = ros::package::getPath("f110_simulator");
    const fs::path cad_map_filepath = simulator_package_path / "maps/levine_4.dwg.png";
    const fs::path unreal_waypoints_path = amr_package_path / "csv/actorLocation.csv";

    cv::Mat map = cv::imread(cad_map_filepath, cv::IMREAD_GRAYSCALE);
    ROS_INFO_STREAM("Read in map of size (col*row): " << map.cols << "x" << map.rows);

    amr::GraphBuilderBP graph_builder(map);
    graph_builder.build_graph(unreal_waypoints_path);
    return {graph_builder.get_graph(), map};
}

std::pair<Graph, cv::Mat> get_f110_map_graph() {
    const auto filepath = ros::package::getPath("auto_mapping_ros") + "/maps/levine_4.jpg";
    const auto csv_filepath = ros::package::getPath("auto_mapping_ros") + "/csv/sequence";

    amr::Skeletonizer processor;
    processor.read_map(filepath);
    cv::Mat skeleton = processor.skeletonize();

    cv::Mat map = cv::imread(filepath, 0);

    assert(skeleton.rows == map.rows);
    assert(skeleton.cols == map.cols);

    amr::GraphBuilder builder(skeleton, map);
    builder.build_graph();
    return {builder.get_graph(), map};
}

int main(int argc, char const* argv[]) {
    // const auto [graph, map] = get_cad_graph();
    const auto [graph, map] = get_f110_map_graph();

    const graph_partitioner::Graph gp_graph = graph_partitioner::fromUserGraph(graph, true);
    graph_partitioner::GraphPartitioner gp(gp_graph);

    // visualize graph nodes
    ROS_WARN("Visualizing graph node IDs. Sequence 0 --> 1 goes from light to dark.");
    std::vector<std::array<int, 2>> sequence;
    for (int i = 0; i < gp_graph.size(); ++i) {
        sequence.emplace_back(std::array<int, 2>{static_cast<int>(gp_graph.getNode(i).y),
                                                 static_cast<int>(gp_graph.getNode(i).x)});
    }
    amr::visualize_sequence_on_graph(map, graph, sequence);

    // compute graph partition
    auto subgraphs = gp.getPartition(2, graph_partitioner::PartitionType::SPECTRAL);

    // visualize subgraphs
    for (auto& subgraph : subgraphs) {
        subgraph.updateTSPSequence();
        const auto current_sequence = subgraph.getTSPSequence();
        amr::visualize_sequence_on_graph(map, graph, current_sequence, true);
    }

    return 0;
}
