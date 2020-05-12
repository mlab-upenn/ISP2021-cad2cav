#ifndef ACO_TSP_UTILS_IMPL_H
#define ACO_TSP_UTILS_IMPL_H

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
