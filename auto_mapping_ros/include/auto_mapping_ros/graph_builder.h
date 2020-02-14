#ifndef AUTO_MAPPING_ROS_GRAPH_BUILDER_H
#define AUTO_MAPPING_ROS_GRAPH_BUILDER_H

#include <array>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <utility>
#include <set>

#ifndef DEBUG
#define DEBUG 1
#endif

namespace amr
{

/// Basic Cell of the Graph
struct Node
{
    explicit Node(const std::array<int, 2> &node) : x(node[0]), y(node[1])
    {}

    int x;
    int y;
    std::vector<Node *> neighbors;
    std::vector<double> neighbors_cost;
};

bool operator==(const Node &lhs, const Node &rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

using Graph = std::vector<Node>;

/// Class for constructing the graph from skeleton image and blueprint
class GraphBuilder
{
public:
    /// Constructs GraphBuilder class
    /// @param skeletonized_image - image obtained from skeletonization and subtracting the blueprint
    /// @param map - Original blueprint of the map
    GraphBuilder(cv::Mat skeletonized_image, cv::Mat map) :
            skeletonized_image_(std::move(skeletonized_image)), map_(std::move(map))
    {}

    /// Builds the graph
    void build_graph()
    {
        const auto corners = find_corners();
        construct_graph(corners);
    }

    /// Get the graph if it is constructed
    /// @return Built graph
    Graph get_graph()
    {
        if (graph_.empty())
        {
            std::__throw_invalid_argument("Graph needs to be built before using it.");
        }
        return graph_;
    }

private:
    cv::Mat skeletonized_image_;
    cv::Mat map_;
    std::vector<Node> graph_;

    /// Debug Function- Displays the corners overlayed on the image
    /// @param img - Input Image
    /// @param corners - vector of all corner features
    /// @param name - Display name
    void display_corners(const cv::Mat& img,
            const std::vector<std::array<int, 2>>& corners,
            const std::string& name = "Skeletonized Image")
    {
        for(const auto& corner: corners)
        {
            cv::circle(img, cv::Point(corner[1], corner[0]), 2, cv::Scalar(0));
        }

        namedWindow(name, cv::WINDOW_AUTOSIZE);
        cv::imshow(name, img);
        cv::waitKey(0);
    }

    /// Performs morphological operation of dilation on the input image
    /// @param img - input image to be diluted
    /// @return
    cv::Mat dilate(const cv::Mat& img)
    {
        const int dilation_size = 6;
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
    std::vector<std::array<int, 2>> compute_blob_centers(const cv::Mat& binary_image)
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
        for(size_t i = 0; i<contours.size(); i++)
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
            const std::vector<std::array<int, 2>>& corners)
    {
        cv::Mat dense_corner_img(map_.size(), CV_8UC1, cv::Scalar(0));
        for(const auto& corner: corners)
        {
            dense_corner_img.at<uchar>(corner[0], corner[1]) = 255;
        }

        const auto dilated_dense_corner_img = dilate(dense_corner_img);
        return compute_blob_centers(dilated_dense_corner_img);
    }

    /// Finds all the important features/corners in the blueprint which can be used as nodes
    /// @return vector of corners/features
    std::vector<std::array<int, 2>> find_corners()
    {
        int blockSize = 12;
        int apertureSize = 7;
        double k = 0.04;

        cv::Mat dst = cv::Mat::zeros(skeletonized_image_.size(), CV_32FC1);
        cv::cornerHarris(skeletonized_image_, dst, blockSize, apertureSize, k);

        cv::Mat dst_norm, dst_norm_scaled;
        normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        std::vector<std::array<int, 2>> corner_points;

        for (int i = 0; i < dst_norm.rows; i++)
        {
            for (int j = 0; j < dst_norm.cols; j++)
            {
                if (static_cast<int>(dst_norm.at<float>(i, j)) > 100)
                {
                    corner_points.emplace_back(std::array<int, 2>{i, j});
                }
            }
        }

        const auto dense_corner_centroids = find_sparse_centroid(corner_points);

        if(DEBUG)
        {
            cv::Mat display_img = map_;
            display_corners(display_img, dense_corner_centroids);
        }

        std::vector<std::array<int, 2>> unique_sparse_centroids;
        std::set<std::array<int, 2>> sparse_centroid_set(dense_corner_centroids.begin(), dense_corner_centroids.end());
        unique_sparse_centroids.assign(sparse_centroid_set.begin(), sparse_centroid_set.end());

        std::cout << "There are " << unique_sparse_centroids.size() << " features detected in this image.";
        return unique_sparse_centroids;
    }

    /// Checks collision between two nodes using the DDA Line algorithm
    /// @param current_node
    /// @param neighbor_node
    /// @return true if there is a collision else false
    bool check_collision(const Node &current_node, const Node &neighbor_node)
    {
        const int dx = neighbor_node.x - current_node.x;
        const int dy = neighbor_node.y - current_node.y;

        int steps = 0;
        if (std::abs(dx) > std::abs(dy))
        {
            steps = std::abs(dx);
        } else
        {
            steps = std::abs(dy);
        }

        double x_increment = dx / static_cast<double>(steps);
        double y_increment = dy / static_cast<double>(steps);

        double intermediate_x = current_node.x;
        double intermediate_y = current_node.y;
        for (int v = 0; v < steps; v++)
        {
            intermediate_x = intermediate_x + x_increment;
            intermediate_y = intermediate_y + y_increment;
            if (map_.at<uchar>(static_cast<int>(intermediate_y), static_cast<int>(intermediate_x)) == 255)
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
            graph_.push_back(node);
        }
        for (auto &node: graph_)
        {
            std::vector<Node *> neighbor_nodes;
            std::vector<double> neighbor_nodes_cost;
            for (auto &candidate_neighbor_node: graph_)
            {
                if (node == candidate_neighbor_node || check_collision(node, candidate_neighbor_node))
                {
                    continue;
                }
                neighbor_nodes.emplace_back(&candidate_neighbor_node);
                neighbor_nodes_cost.emplace_back();
            }
            node.neighbors = neighbor_nodes;
        }
    }
};

}

#endif //AUTO_MAPPING_ROS_GRAPH_BUILDER_H
