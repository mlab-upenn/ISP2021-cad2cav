#ifndef __GRAPH_PARTITIONER_PARTITIONER_HPP__
#define __GRAPH_PARTITIONER_PARTITIONER_HPP__

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <vector>

#include "graph_partitioner/types.hpp"

namespace graph_partitioner {

/**
 * @brief Specifies the metric for setting edge weights between two graph nodes
 * in adjacency matrix.
 *
 *  1. DISTANCE: edge weights would be plain distance between two nodes
 *  2. SIMILARITY: edge weights would be similarity (1/distance) between nodes
 */
enum AdjMatrixMetricType { DISTANCE = 0, SIMILARITY };

class GraphPartitioner {
public:
    GraphPartitioner() = default;

    /**
     * @brief Construct a new Graph Partitioner object
     *
     * @param graph:    input graph for initialization
     */
    GraphPartitioner(const Graph& graph);

    /**
     * @brief Get the k-way partition of the graph
     *
     * @return std::vector<Graph>:  k subgraphs of the original graph
     */
    std::vector<Graph> getPartition(const int k) const;

    /**
     * @brief Constructs an adjacency matrix for spectral clustering
     *       based on current graph
     *
     * @param metric:           metric for setting edge weights.
     *                          DISTANCE or SIMILARITY.
     * @return Eigen::MatrixXd
     */
    Eigen::MatrixXd adjacencyMatrix(const AdjMatrixMetricType metric) const;

    /**
     * @brief Runs k-means clustering on a matrix of feature vectors
     *
     * @param fv:                   feature vectors. Each column is a data point
     * @param k:                    number of clusters
     * @return std::vector<int>:    assignments of each feature vector
     *                              each assignment is in [0,len(fv)-1]
     */
    static std::vector<int> kMeans(const Eigen::MatrixXd& fv, const int k);

private:
    Graph graph_;
};

}  // namespace graph_partitioner

#endif /* __GRAPH_PARTITIONER_PARTITIONER_HPP__ */
