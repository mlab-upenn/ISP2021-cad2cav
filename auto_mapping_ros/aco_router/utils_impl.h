#ifndef ACO_TSP_UTILS_IMPL_H
#define ACO_TSP_UTILS_IMPL_H

#include <iostream>
#include <random>

#include "utils.h"

/**
 * Get Cost Matrix for the graph required by Ant Colony Optimization
 * @details This function runs the Floyd-Warshall Algorithm for finding shortest path between each pair of the graph
 * @param graph
 * @return
 */
Eigen::MatrixXd get_cost_matrix(const aco::Graph& graph)
{
    const int n_nodes = graph.size();
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(n_nodes, n_nodes);

    for(int i=0; i<n_nodes; i++)
    {
        const auto node_i = graph.get_node_from_graph(i);
        for(int j=0; j<n_nodes; j++)
        {
            if(i != j)
            {
                cost_matrix(i, j) = node_i.get_distance_from_neighbor(j);
            }
        }
    }

    for(int k=0; k<n_nodes; k++)
    {
        for(int i=0; i<n_nodes; i++)
        {
            for(int j=0; j<n_nodes; j++)
            {
                if(i == j) continue;
                if(cost_matrix(i, j) > cost_matrix(i, k) + cost_matrix(k, j))
                {
                    cost_matrix(i, j) = cost_matrix(i, k) + cost_matrix(k, j);
                }
            }
        }
    }

    return cost_matrix;
}

/**
 * Find a random index based on probabilities in the probability array
 * @param probability_array
 * @return
 */
int run_roulette_wheel(const Eigen::ArrayXd& probability_array)
{
    Eigen::ArrayXd cumulative_sum = Eigen::ArrayXd::Zero(probability_array.size());
    double current_sum = 0;
    for(int i=0; i<probability_array.size(); i++)
    {
        current_sum += probability_array(i);
        cumulative_sum(i) = current_sum;
    }

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> real_dist(0.0, current_sum);

    double rolled_value = real_dist(mt);

    int i=0;
    while(rolled_value > cumulative_sum(i))
    {
        if(i >= cumulative_sum.size())
        {
            std::cout << "invalid: logical error in roulette wheel. \n";
        }
        i++;
    }

    return i;
}

/**
 * Convert user defined graph type to aco graph type
 * @tparam GraphType - GraphType must be a sequential std container (Eg. Vector) containing sequence of nodes
 * @param graph - user defined graph
 * @return aco::Graph
 */
template <typename GraphType>
aco::Graph convert_to_aco_graph(const GraphType& graph)
{
    aco::Graph aco_graph;
    std::vector<std::pair<decltype (GraphType::value_type), int>> original_node_to_aco_id{};

    for(const auto& node: graph)
    {
        int id = aco_graph.create_node_in_graph(node.x, node.y);
        original_node_to_aco_id.emplace_back(std::pair{node, id});
    }

    for(const auto& node: graph)
    {
        int aco_cur_id = -1;
        const auto aco_cur_id_found = original_node_to_aco_id.find(node);
        if(aco_cur_id_found != original_node_to_aco_id.end())
        {
            aco_cur_id = aco_cur_id_found->second;
        }

        for(const auto& neighbor: node.neighbors)
        {
            int aco_nbr_id = -1;
            const auto aco_nbr_id_found = original_node_to_aco_id.find(neighbor);
            if(aco_nbr_id_found != original_node_to_aco_id.end())
            {
                aco_nbr_id = aco_nbr_id_found->second;
            }
            aco_graph.add_edge(aco_cur_id, aco_nbr_id);
        }
    }

    return aco_graph;
}

#endif //ACO_TSP_UTILS_IMPL_H
