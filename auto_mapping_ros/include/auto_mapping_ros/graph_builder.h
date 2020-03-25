#ifndef AUTO_MAPPING_ROS_GRAPH_BUILDER_H
#define AUTO_MAPPING_ROS_GRAPH_BUILDER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <array>
#include <iostream>
#include <libconfig.h++>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <set>
#include <utility>

#include "auto_mapping_ros/landmarks.h"
#include "auto_mapping_ros/types.h"
#include "auto_mapping_ros/utils.h"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

void visualize_graph(const cv::Mat& map, const Graph& graph);

/// Class for constructing the graph from skeleton image and blueprint
class GraphBuilder
{
public:
    /// Constructs GraphBuilder class
    /// @param skeletonized_image - image obtained from skeletonization and subtracting the blueprint
    /// @param map - Original blueprint of the map
    GraphBuilder(cv::Mat skeletonized_image, cv::Mat map) :
            skeletonized_image_(std::move(skeletonized_image)), map_(std::move(map))
    {
        init_config();
    }

    void init_config()
    {
        libconfig::Config cfg;
        try
        {
            const std::string filename = ros::package::getPath("auto_mapping_ros") + "/config/graph_builder.cfg";
            char *tab2 = new char [filename.length()+1];
            strcpy (tab2, filename.c_str());
            cfg.readFile(tab2);
        }
        catch(const libconfig::FileIOException &fioex)
        {
            std::__throw_invalid_argument("I/O error while reading file.");
        }

        try
        {
            dilation_size_ = cfg.lookup("dilation_size");
            block_size_ = cfg.lookup("blockSize");
            aperture_size_ = cfg.lookup("apertureSize");
            k_ = cfg.lookup("k");
            distance_threshold_ = cfg.lookup("distance_threshold");
            obstacle_threshold_ = cfg.lookup("obstacle_threshold");
        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            std::cerr << "Missing setting in configuration file." << std::endl;
        }
    }


    /// Builds the graph
    void build_graph()
    {
        register_boundary();
        const auto corners = find_corners();
        construct_graph(corners);
        if(DEBUG)
        {
            visualize_graph(map_, graph_);
        }
    }

    /// Get the graph if it is constructed
    /// @return Built graph
    Graph get_graph()
    {
        if (graph_.empty())
        {
            std::__throw_invalid_argument("Graph needs to be built before using it.");
        }
        return std::move(graph_);
    }

    static std::vector<std::array<int, 2>> boundary_corners_;
    static void CallBackFunc(int event, int x, int y, int flags, void* userdata)
    {
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            std::cout << "Boundary corner selected: (" << x << ", " << y << ") \n";
            boundary_corners_.emplace_back(std::array<int, 2>{x, y});
        }
    }

private:
    cv::Mat skeletonized_image_;
    cv::Mat map_;
    std::vector<Node> graph_;

    int dilation_size_;
    int block_size_;
    int aperture_size_;
    double k_;
    double distance_threshold_;
    int obstacle_threshold_;

    void register_boundary()
    {
        const std::string window_name = "Functional Area Registration";
        cv::namedWindow(window_name, 1);
        cv::setMouseCallback(window_name, this->CallBackFunc, NULL);
        while(true)
        {
            cv::imshow(window_name, map_);
            if (cv::waitKey(1) & boundary_corners_.size() == 2) break;
        }
        cv::destroyWindow(window_name);
        return;
    }

    /// Performs morphological operation of dilation on the input image
    /// @param img - input image to be diluted
    /// @return
    cv::Mat dilate(const cv::Mat& img) const
    {
        const int dilation_size = dilation_size_;
        const int dilation_type = cv::MORPH_RECT;
        cv::Mat element = getStructuringElement( dilation_type,
                                             cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                             cv::Point( dilation_size, dilation_size));

        cv::Mat dilated_dilated_img;
        cv::dilate(img, dilated_dilated_img, element);
        return dilated_dilated_img;
    }

    /// Computes the centers of multiple blobs in a binary image
    /// @param binary_image Input Image (Binary/GrayScale)
    /// @return vector of centroids of the white blobs in the input image
    std::vector<std::array<int, 2>> compute_blob_centers(const cv::Mat& binary_image) const
    {
        cv::Mat canny_output;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        // detect edges using canny
        cv::Canny(binary_image, canny_output, 50, 150, 3 );

        // find contours
        cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        // get the moments
        std::vector<cv::Moments> mu(contours.size());
        for(size_t i = 0; i<contours.size(); i++ )
        {
            mu[i] = moments( contours[i], false );
        }

        // get the centroid of figures.
        std::vector<cv::Point2f> mc(contours.size());
        for(auto i = 0; i<contours.size(); i++)
        {
            mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        }

        if(DEBUG)
        {
            // draw contours
            cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
            for(size_t i = 0; i<contours.size(); i++ )
            {
                cv::Scalar color = cv::Scalar(167,151,0); // B G R values
                drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                circle( drawing, mc[i], 4, color, -1, 8, 0 );
            }

            // show the resultant image
            namedWindow( "Contours", cv::WINDOW_AUTOSIZE);
            imshow( "Contours", drawing );
            cv::waitKey(0);
        }

        std::vector<std::array<int, 2>> centroids;
        for(const auto& mc_points: mc)
        {
            centroids.emplace_back(std::array<int, 2>(
                    {static_cast<int>(mc_points.y), static_cast<int>(mc_points.x)}));
        }

        return centroids;
    }

    /// Filters the corners vector by finding sparse matrices using morphological techniques
    /// @param corners - vector of features detected
    /// @return sparse vector of features
    std::vector<std::array<int, 2>> find_sparse_centroid(
            const std::vector<std::array<int, 2>>& corners) const
    {
        cv::Mat dense_corner_img(map_.size(), CV_8UC1, cv::Scalar(0));
        for(const auto& corner: corners)
        {
            dense_corner_img.at<uchar>(corner[0], corner[1]) = 255;
        }

        const auto dilated_dense_corner_img = dilate(dense_corner_img);
        return compute_blob_centers(dilated_dense_corner_img);
    }

    static bool is_within_specified_region(int x, int y)
    {
        return x > boundary_corners_[0][0] && x < boundary_corners_[1][0]
               && y > boundary_corners_[0][1] && y < boundary_corners_[1][1];
    }

    /// Finds all the important features/corners in the blueprint which can be used as nodes
    /// @return vector of corners/features
    std::vector<std::array<int, 2>> find_corners() const
    {
        cv::Mat dst = cv::Mat::zeros(skeletonized_image_.size(), CV_32FC1);
        cv::cornerHarris(skeletonized_image_, dst, block_size_, aperture_size_, k_);

        cv::Mat dst_norm, dst_norm_scaled;
        normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        std::vector<std::array<int, 2>> corner_points;

        for (int i = 0; i < dst_norm.rows; i++)
        {
            for (int j = 0; j < dst_norm.cols; j++)
            {
                if (static_cast<int>(dst_norm.at<float>(i, j)) > 100 && is_within_specified_region(j, i))
                {
                    corner_points.emplace_back(std::array<int, 2>{i, j});
                }
            }
        }

        const auto dense_corner_centroids = find_sparse_centroid(corner_points);

        std::vector<std::array<int, 2>> unique_sparse_centroids;
        std::set<std::array<int, 2>> sparse_centroid_set(dense_corner_centroids.begin(), dense_corner_centroids.end());
        unique_sparse_centroids.assign(sparse_centroid_set.begin(), sparse_centroid_set.end());

        std::cout << "There are " << unique_sparse_centroids.size() << " features detected in this image.";
        return unique_sparse_centroids;
    }

    /// Finds the distance between a point and line segment
    /// @param x_point - x co-ordinate of point
    /// @param y_point - y co-ordinate of point
    /// @param x_line_segment_start - x co-ordinate of start of line segment
    /// @param y_line_segment_start - y co-ordinate of start of line segment
    /// @param x_line_segment_end - x co-ordinate of end of line segment
    /// @param y_line_segment_end - y co-ordinate of end of line segment
    /// @return distance between point and line segment
    double pDistance(double x_point, double y_point,
                     double x_line_segment_start, double y_line_segment_start,
                     double x_line_segment_end, double y_line_segment_end) const
    {

        double A = x_point - x_line_segment_start;
        double B = y_point - y_line_segment_start;
        double C = x_line_segment_end - x_line_segment_start;
        double D = y_line_segment_end - y_line_segment_start;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;

        if (len_sq != 0)
            param = dot/len_sq;

        double xx, yy;

        if (param < 0) {
            xx = x_line_segment_start;
            yy = y_line_segment_start;
        }
        else if (param > 1) {
            xx = x_line_segment_end;
            yy = y_line_segment_end;
        }
        else {
            xx = x_line_segment_start + param * C;
            yy = y_line_segment_start + param * D;
        }

        auto dx = x_point - xx;
        auto dy = y_point - yy;
        return sqrt(dx * dx + dy * dy);
    }

    /// Checks if any neighbors of the current node lie between the current node and the neighbor node
    /// @param current_node - current node of the graph
    /// @param neighbor_node - neighbor node of the current node of the graph
    /// @return true if a neighbor of current node lies within a threshold distance of the two input nodes
    bool is_another_node_in_between(Node* current_node, Node* neighbor_node) const
    {
        double threshold = distance_threshold_;
        for(const auto& current_node_neighbor: current_node->neighbors)
        {
            if(current_node_neighbor==neighbor_node) continue;
            const auto dist = pDistance(current_node_neighbor->x, current_node_neighbor->y,
                    current_node->x, current_node->y, neighbor_node->x, neighbor_node->y);
            if(dist < threshold)
            {
                return true;
            }
        }
        return false;
    }

    /// Trims away edges to reduce overlapping edges
    void trim_edges()
    {
        for(auto &current_node: graph_)
        {
            const int current_node_neighbors_size = current_node.neighbors.size();

            std::vector<bool> is_valid(current_node_neighbors_size, true);
            std::vector<Node*> new_current_node_neighbors;

            for(const auto& neighbor_node: current_node.neighbors)
            {
                if(is_another_node_in_between(&current_node, neighbor_node))
                {
                    continue;
                }
                new_current_node_neighbors.emplace_back(neighbor_node);
            }
            current_node.neighbors = new_current_node_neighbors;
        }
    }

    /// Checks collision between two nodes using the DDA Line algorithm
    /// @param current_node
    /// @param neighbor_node
    /// @return true if there is a collision else false
    bool check_collision(const Node &current_node, const Node &neighbor_node) const
    {
        const int dx = neighbor_node.x - current_node.x;
        const int dy = neighbor_node.y - current_node.y;

        int steps = 0;
        if (std::abs(dx) > std::abs(dy))
        {
            steps = std::abs(dx);
        }
        else
        {
            steps = std::abs(dy);
        }

        double x_increment = dx / static_cast<double>(steps);
        double y_increment = dy / static_cast<double>(steps);

        double intermediate_x = current_node.x;
        double intermediate_y = current_node.y;
        for (size_t v = 0; v < steps; v++)
        {
            intermediate_x = intermediate_x + x_increment;
            intermediate_y = intermediate_y + y_increment;
            const auto pixel_value = static_cast<int>(
                    map_.at<uchar>(static_cast<int>(intermediate_x), static_cast<int>(intermediate_y)));
            if (pixel_value < obstacle_threshold_)
            {
                return true;
            }
        }
        return false;
    }

    /// Constructs the graph using the input feature cells (corners)
    /// @param corners
    void construct_graph(const std::vector<std::array<int, 2>> &corners)
    {
        for (const auto &corner: corners)
        {
            Node node(corner);
            graph_.push_back(std::move(node));
        }
        for (auto &node: graph_)
        {
            std::vector<Node *> neighbor_nodes;
            std::vector<double> neighbor_nodes_cost;
            for (auto& candidate_neighbor_node: graph_)
            {
                if (node == candidate_neighbor_node || check_collision(node, candidate_neighbor_node))
                {
                    continue;
                }
                neighbor_nodes.emplace_back(&candidate_neighbor_node);
            }

            node.neighbors = neighbor_nodes;
            node.neighbors_cost = neighbor_nodes_cost;
        }

        trim_edges();

        for(auto& node: graph_)
        {
            std::vector<double> neighbor_nodes_cost;
            for(const auto& neighbor_node: node.neighbors)
            {
                const auto distance = static_cast<double>(
                        sqrt(pow(node.y-neighbor_node->y, 2)+pow(node.x-neighbor_node->x, 2)));
                neighbor_nodes_cost.emplace_back(distance);
            }
            node.neighbors_cost = neighbor_nodes_cost;
        }
    }
};

std::vector<std::array<int, 2>> GraphBuilder::boundary_corners_ = {};

}

#endif //AUTO_MAPPING_ROS_GRAPH_BUILDER_H
