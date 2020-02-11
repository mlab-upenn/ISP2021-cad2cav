#ifndef AUTO_MAPPING_ROS_GRAPH_BUILDER_H
#define AUTO_MAPPING_ROS_GRAPH_BUILDER_H

#include <array>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <utility>

struct Node
{
    explicit Node(const std::array<int, 2>& node): x(node[0]), y(node[1]){}
    int x;
    int y;
    std::vector<Node*> neighbors;
};

bool operator==(const Node& lhs, const Node& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

class GraphBuilder
{
public:
    explicit GraphBuilder(cv::Mat skeletonized_image, cv::Mat map):
    skeletonized_image_(std::move(skeletonized_image)), map_(std::move(map))
    {}

    void build_graph()
    {
        const auto corners = find_corners();
        construct_graph(corners);
    }

    std::vector<Node> get_graph()
    {
        if(graph_.empty())
        {
            std::__throw_invalid_argument("Graph needs to be built before using it.");
        }
        return graph_;
    }

private:
    cv::Mat skeletonized_image_;
    cv::Mat map_;
    std::vector<Node> graph_;

    std::vector<std::array<int, 2>> find_corners()
    {
        int blockSize = 3;
        int apertureSize = 5;
        double k = 0.02;

        cv::Mat dst = cv::Mat::zeros(skeletonized_image_.size(), CV_32FC1);
        cv::cornerHarris(skeletonized_image_, dst, blockSize, apertureSize, k);

        cv::Mat dst_norm, dst_norm_scaled;
        normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        convertScaleAbs( dst_norm, dst_norm_scaled );

        std::vector<std::array<int, 2>> corner_points;

        for(int i = 0; i < dst_norm.rows ; i++)
        {
            for( int j = 0; j < dst_norm.cols; j++)
            {
                if( (int) dst_norm.at<float>(i,j) > 250)
                {
                    corner_points.emplace_back(std::array<int, 2>{i, j});
                    std::cout << 1;
                }
                else
                {
                    std::cout << 0;
                }
                std::cout << " ";
            }
            std::cout << "\n";
        }

        return corner_points;
    }

    bool check_collision(const Node& current_node, const Node& neighbor_node)
    {
        //TODO: Bresenhams Algorithm 
    }

    void construct_graph(const std::vector<std::array<int, 2>>& corners)
    {
        for(const auto& corner: corners)
        {
            Node node(corner);
            graph_.push_back(node);
        }
        for(auto& node: graph_)
        {
            std::vector<Node*> neighbor_nodes;
            for(auto& candidate_neighbor_node: graph_)
            {
                if(check_collision(node, candidate_neighbor_node))
                {
                    neighbor_nodes.emplace_back(&candidate_neighbor_node);
                }
            }
            node.neighbors = neighbor_nodes;
        }
    }
};

#endif //AUTO_MAPPING_ROS_GRAPH_BUILDER_H
