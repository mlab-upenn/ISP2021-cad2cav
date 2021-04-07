#include "graph_partitioner/partitioner.hpp"

#include <algorithm>
#include <boost/range/counting_range.hpp>
#include <set>
#include <unordered_map>

namespace graph_partitioner {

GraphPartitioner::GraphPartitioner(const Graph& graph) : graph_(graph) {}

Eigen::MatrixXd GraphPartitioner::adjacencyMatrix(
    const AdjMatrixMetricType metric) const {
    Eigen::MatrixXd adj_matrix =
        Eigen::MatrixXd::Identity(graph_.size(), graph_.size());

    if (metric == AdjMatrixMetricType::DISTANCE) {
        for (int i = 0; i < graph_.size(); ++i) {
            for (const auto edge : graph_.getNode(i).neighbors) {
                adj_matrix(i, edge.first) = edge.second;
            }
        }
    } else if (metric == AdjMatrixMetricType::SIMILARITY) {
        const double max_distance = *std::max_element(
            graph_.edge_weights.begin(), graph_.edge_weights.end());
        for (int i = 0; i < graph_.size(); ++i) {
            for (const auto edge : graph_.getNode(i).neighbors) {
                // uses Gaussian similarity function,
                //  sets std dev to max_distance/2
                adj_matrix(i, edge.first) = std::exp(
                    -1 * std::pow(edge.second / (max_distance / 2), 2) / 2);
            }
        }
    }

    return adj_matrix;
}

std::vector<Graph> GraphPartitioner::getPartition(const int k,
                                                  PartitionType type) const {
    std::vector<Graph> list_subgraphs;

    switch (type) {
        case PartitionType::SPECTRAL:
            ROS_INFO("Using spectral clustering as graph partitioning method");
            // Performs spectral clustering on graph partitioning task
            list_subgraphs = spectralClustering(k);
            break;
        case PartitionType::METIS:
            ROS_INFO("Using METIS as graph partitioning method");
            // Calls METIS software on graph partitioning task
            list_subgraphs = use_metis(k);
            break;

        default:
            break;
    }

    return list_subgraphs;
}

std::vector<Graph> GraphPartitioner::use_metis(const int k) const {
    // prepare CSR graph format
    //  Variable name is consistent with METIS manual 5.1.0 Section 5.5, which
    //  can be retrieved here:
    //  http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf
    int n_nodes = graph_.size();
    std::vector<int> xadj, adjncy, adjwgt;
    graph_.getCSRFormat(xadj, adjncy, adjwgt);

    // assignment vector for all graph nodes
    std::vector<int> assignments(n_nodes, 0);
    int partition_cost = 0;

    // set METIS partitioning option code
    //  see METIS manual 5.1.0 section 5.4
    std::vector<int> metis_option_codes(METIS_NOPTIONS, 0);
    METIS_SetDefaultOptions(metis_option_codes.data());
    metis_option_codes[METIS_OPTION_DBGLVL] = METIS_DBG_INFO | METIS_DBG_TIME;
    metis_option_codes[METIS_OPTION_NITER]  = 20;  // increate no. of iterations
    metis_option_codes[METIS_OPTION_CONTIG] = 1;   // force contiguous subgraphs

    // run METIS graph partitioner
    //  default: multilevel k-way graph partitioning
    //  reference paper:
    //  https://www-sciencedirect-com.proxy.library.upenn.edu/science/article/pii/S0743731597914040
    int n_constraints    = 1;
    int num_parts        = k;
    int partition_result = METIS_PartGraphKway(
        &n_nodes, &n_constraints, xadj.data(), adjncy.data(), NULL, NULL,
        adjwgt.data(), &num_parts, NULL, NULL, metis_option_codes.data(),
        &partition_cost, assignments.data());

    switch (partition_result) {
        case METIS_OK:
            ROS_INFO_STREAM("METIS partitioning finished with no errors."
                            << " Partition cost: " << partition_cost);
            break;
        case METIS_ERROR_INPUT:
            ROS_ERROR("METIS partitioning failed: input error");
            break;
        case METIS_ERROR_MEMORY:
            ROS_ERROR(
                "METIS partitioning failed: could not allocate enough memory");
            break;
        case METIS_ERROR:
            ROS_ERROR("METIS partitioning failed: other general errors");
            break;

        default:
            break;
    }

    return utils::getSubgraphFromAssignments(this->graph_, k, assignments);
}

std::vector<Graph> GraphPartitioner::spectralClustering(const int k) const {
    // adjacency matrix of the graph
    //  for spectral clustering, we should use similarity in adj matrix
    //  (closer nodes have higher edge weights)
    const auto adj_matrix = adjacencyMatrix(AdjMatrixMetricType::SIMILARITY);
    // degree matrix of the graph
    const Eigen::MatrixXd deg_matrix = adj_matrix.rowwise().sum().asDiagonal();
    // compute Laplacian matrix
    const auto laplacian = deg_matrix - adj_matrix;
    // solve the eigenvalues & eigenvectors of Laplacian
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_laplacian(laplacian);
    // transpose of k-th eigenvectors corr. to smallest k eigenvalues
    //  denotes the feature vector for each graph node
    const auto node_feature_vectors =
        es_laplacian.eigenvectors().leftCols(k).transpose();
    ROS_INFO_STREAM("Feature vectors from graph Laplacian:\n"
                    << node_feature_vectors.transpose());
    ROS_INFO_STREAM("Corresponding eigenvalues:\n"
                    << es_laplacian.eigenvalues().head(k).transpose());
    // run k-means clustering on feature vectors
    const auto assignments = utils::kMeans(node_feature_vectors, k);

    // construct subgraphs based on assignments
    return utils::getSubgraphFromAssignments(this->graph_, k, assignments);
}

namespace utils {

std::vector<int> kMeans(const Eigen::MatrixXd& fv, const int k) {
    int num_points = fv.cols();
    int dim        = fv.rows();  // dimension of feature vector
    std::vector<int> assignments_vec(num_points, 0);

    // compute feature points statictics
    const Eigen::VectorXd half_spread =
        (fv.rowwise().maxCoeff() - fv.rowwise().minCoeff()) / 2;
    const Eigen::VectorXd center = fv.rowwise().mean();

    // initialize centroids
    //  bias the initialization to be around the center of data,
    //  with no more than the L2 norm of half_spread of data
    Eigen::MatrixXd centroids = Eigen::MatrixXd::Random(dim, k);
    centroids.array().colwise() *= half_spread.array();
    centroids.colwise() += center;

    // initialize assignment results
    // Eigen::Map ensures that `assignments` and `assignments_vec` share the
    //  same memory data
    // https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html
    Eigen::Map<Eigen::VectorXi> assignments(assignments_vec.data(), num_points);
    Eigen::VectorXi prev_assignments = Eigen::VectorXi::Ones(num_points);

    // debug output
    ROS_INFO_STREAM("centroids before clustering:\n" << centroids);
    ROS_INFO_STREAM("assignments of nodes before clustering:\n"
                    << assignments.transpose());

    // main loop
    while (assignments != prev_assignments) {
        prev_assignments = assignments;

        // reassign each feature vector to the closest centroid
        for (int i = 0; i < num_points; ++i) {
            const Eigen::VectorXd feature = fv.col(i);
            double min_dist = std::numeric_limits<double>::infinity();
            for (int j = 0; j < k; ++j) {
                Eigen::VectorXd centroid = centroids.col(j);
                if ((feature - centroid).squaredNorm() < min_dist) {
                    min_dist       = (feature - centroid).squaredNorm();
                    assignments(i) = j;
                }
            }
        }

        // recompute each centroid as the mean of each cluster
        for (int j = 0; j < k; ++j) {
            Eigen::VectorXd sum_centroid = Eigen::VectorXd::Zero(dim);
            int count                    = 0;
            for (int i = 0; i < num_points; ++i) {
                if (assignments(i) != j) continue;
                sum_centroid += fv.col(i);
                ++count;
            }
            centroids.col(j) = (sum_centroid == Eigen::VectorXd::Zero(dim))
                                   ? sum_centroid
                                   : sum_centroid / count;
        }
    }

    ROS_INFO_STREAM("centroids after clustering:\n" << centroids);
    ROS_INFO_STREAM("assignments of nodes after clustering:\n"
                    << assignments.transpose());

    return assignments_vec;
}

std::vector<Graph> getSubgraphFromAssignments(
    const Graph& orig_graph, const int k, const std::vector<int>& assignments) {
    std::vector<Graph> list_subgraphs;
    for (int j = 0; j < k; ++j) {
        Graph subgraph;
        // a set of node ids that remembers the node unvisited
        // (not added to node_id_map)
        std::set<int> unvisited_nodes(
            boost::counting_iterator<int>(0),
            boost::counting_iterator<int>(assignments.size()));
        // a mapping from old node id to new node id
        std::unordered_map<int, int> node_id_map;

        while (!unvisited_nodes.empty()) {
            int i = *unvisited_nodes.begin();
            unvisited_nodes.erase(i);
            if (assignments[i] != j) continue;

            const Node node = orig_graph.getNode(i);
            int node_new_id;
            if (node_id_map.count(i) > 0)
                node_new_id = node_id_map.at(i);
            else {
                node_new_id = subgraph.addNewNode(node.x, node.y);
                node_id_map.insert({i, node_new_id});
            }

            for (const auto& [neighbor_id, distance] : node.neighbors) {
                // if node's neighbor is also in the same cluster, add it to
                // current subgraph. Otherwise, discard it
                if (assignments[neighbor_id] == j) {
                    const Node neighbor = orig_graph.getNode(neighbor_id);
                    int neighbor_new_id;
                    if (node_id_map.count(neighbor_id) > 0)
                        neighbor_new_id = node_id_map.at(neighbor_id);
                    else {
                        neighbor_new_id =
                            subgraph.addNewNode(neighbor.x, neighbor.y);
                        node_id_map.insert({neighbor_id, neighbor_new_id});
                    }

                    // add an edge between node and its current neighbor in
                    // new graph
                    subgraph.addEdge(node_new_id, neighbor_new_id, distance);
                }
            }
        }

        list_subgraphs.push_back(subgraph);
    }
    return list_subgraphs;
}

}  // namespace utils

}  // namespace graph_partitioner
