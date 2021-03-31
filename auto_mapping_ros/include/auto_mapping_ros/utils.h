#ifndef AUTO_MAPPING_ROS_UTILS_H
#define AUTO_MAPPING_ROS_UTILS_H

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "../aco_router/types.h"
#include "auto_mapping_ros/types.h"

namespace amr {

void print_vector_of_nodes(const std::vector<std::array<double, 2>>& vector_of_nodes);

void print_graph(const Graph& graph);

void print_graph_with_new_ids(Graph& graph);

void visualize_graph(const cv::Mat& map, const Graph& graph);

int get_closest_clicked_node_on_map(const cv::Mat& map, aco::Graph& graph);

void visualize_sequence_on_graph(const cv::Mat& map, const Graph& graph,
                                 const std::vector<std::array<int, 2>>& sequence);

void visualize_sequence_on_graph(const cv::Mat& map, const Graph& graph,
                                 const std::vector<std::array<double, 2>>& sequence,
                                 bool switch_xy = false);

void write_plans_to_csv(const std::vector<Plan>& plans, std::string filename);

void read_sequence_from_csv(std::vector<std::array<int, 2>>* sequence, const std::string& filename = "sequence.csv");

std::array<double, 2> translate_indices_to_xy(const std::array<int, 2>& node_on_previous_map, double resolution);

std::vector<std::array<double, 2>>
translate_vector_of_indices_to_xy(const std::vector<std::array<int, 2>>& vector_of_nodes, double resolution);

template <typename Arithmetic>
double distance(const std::array<Arithmetic, 2>& node1, const std::array<Arithmetic, 2>& node2);

std::string get_package_directory();

}  // namespace amr

/***********************/
/* TEMPLATED FUNCTIONS */
/***********************/
namespace amr {

template <typename T>
void write_sequence_to_csv(const std::vector<std::array<T, 2>>& sequence, std::string filename) {
    std::ofstream file_to_write;
    file_to_write.open(filename);
    if (!file_to_write) {
        throw std::runtime_error("Invalid Path for csv file.");
    }
    for (const auto& node : sequence) {
        file_to_write << node[0] << ", " << node[1] << "\n";
    }
    file_to_write.close();
}

template <typename Arithmetic>
double distance(const std::array<Arithmetic, 2>& node1, const std::array<Arithmetic, 2>& node2) {
    return sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2));
}

}  // namespace amr

#endif  //AUTO_MAPPING_ROS_UTILS_H
