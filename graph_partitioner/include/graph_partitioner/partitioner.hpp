#ifndef __GRAPH_PARTITIONER_PARTITIONER_HPP__
#define __GRAPH_PARTITIONER_PARTITIONER_HPP__

#include <metis.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <vector>

#include "cad2cav_types/graph.hpp"

namespace cad2cav {
namespace graph_partitioner {

/**
 * @brief Specifies the metric for setting edge weights between two graph nodes
 * in adjacency matrix.
 *
 *  1. DISTANCE: edge weights would be plain distance between two nodes
 *  2. SIMILARITY: edge weights would be similarity (1/distance) between nodes
 */
enum AdjMatrixMetricType { DISTANCE = 0, SIMILARITY };

/**
 * @brief Specifies the graph partitioning algorithms to generate subgraphs.
 *
 *  1. SPECTRAL:        spectral clustering
 *  2. KERNIGHAN_LIN:   Kernighan-Lin algorithm
 *  3. METIS:           METIS partitioner software. Uses multilevel k-way
 *                      partitioning
 */
enum PartitionType { SPECTRAL = 0, KERNIGHAN_LIN, METIS };

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
   * @param type:         partition algorithms to use
   * @param k:            no. of partitions to generate
   * @return std::vector<Graph>:  k subgraphs of the original graph
   */
  std::vector<cad2cav::Graph> getPartition(
      const int k, PartitionType type = PartitionType::SPECTRAL) const;

  /**
   * @brief Constructs an adjacency matrix for spectral clustering
   *       based on current graph
   *
   * @param metric:           metric for setting edge weights.
   *                          DISTANCE or SIMILARITY.
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd adjacencyMatrix(const AdjMatrixMetricType metric) const;

private:
  cad2cav::Graph graph_;

  /**
   * @brief Runs spectral clustering on current graph and generate subgraphs.
   *
   * @param k:                    No. of subgraphs (clusters)
   * @return std::vector<Graph>:  List of subgraphs
   */
  std::vector<cad2cav::Graph> spectralClustering(const int k) const;

  /**
   * @brief Calls METIS graph partitioner software on current graph.
   *
   * @param k:                    No. of subgraphs
   * @return std::vector<Graph>:  List of subgraphs
   */
  std::vector<cad2cav::Graph> use_metis(const int k) const;
};

namespace utils {
/**
 * @brief Runs k-means clustering on a matrix of feature vectors
 *
 * @param fv:                   feature vectors. Each column is a data point
 * @param k:                    number of clusters
 * @return std::vector<int>:    assignments of each feature vector.
 *                              Each assignment is in [0,k-1]
 */
std::vector<int> kMeans(const Eigen::MatrixXd& fv, const int k);

/**
 * @brief Returns an initialization of centroids using k-means++ scheme
 *
 * @param fv:               feature vectors. Each column is a data point
 * @param k:                number of clusters (centroids)
 * @return Eigen::MatrixXd: coordinates of centroids. Size = (fv.size(0), k)
 */
Eigen::MatrixXd kmeanspp_init(const Eigen::MatrixXd& fv, const int k);

/**
 * @brief Construct and return Subgraph from node assignments.
 *
 * @param orig_graph:           original graph before partition
 * @param k:                    number of partitions
 * @param assignments:          vector of length n_nodes that stores the
 *                              assignment [0, k-1] of subgraph for each vertex
 * @return std::vector<Graph>:  list of subgraphs
 */
std::vector<Graph> getSubgraphFromAssignments(
    const Graph& orig_graph, const int k, const std::vector<int>& assignments);
}  // namespace utils

}  // namespace graph_partitioner
}  // namespace cad2cav

#endif /* __GRAPH_PARTITIONER_PARTITIONER_HPP__ */
