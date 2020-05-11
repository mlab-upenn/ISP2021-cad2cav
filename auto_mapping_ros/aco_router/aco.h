#ifndef ACO_TSP_ACO_H
#define ACO_TSP_ACO_H

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace aco
{
    struct Node
    {
        Node(double x, double y);
        int id;
        double x{};
        double y{};
        std::vector<std::pair<int, double>> neighbors;

        bool operator==(const Node&other) const;

        double get_distance_from_neighbor(int node_id) const;
    };

    struct Graph
    {
    private:
        std::vector<Node> graph;
        int n_nodes_;
        aco::Node* get_node_from_graph(int node_id);

    public:
        int size() const;
        int add_node_to_graph(Node& node);
        void add_edge(int node_id_from, int node_id_to);
        aco::Node get_node_from_graph(int node_id) const;
    };

    /**
     * Parameters for solving the Ant Colony Optimzation Problem
     */
    struct AcoParams
    {
        int n_ants;
        int max_iters;
    };

    /**
     * Function to solve the
     * @param graph
     * @param params
     * @return
     */
    std::vector<Node> solve_tsp(const Graph& graph, const AcoParams& params);
}

namespace std {

    template <>
    struct hash<aco::Node>
    {
        std::size_t operator()(const aco::Node& k) const
        {
            using std::size_t;
            using std::hash;
            using std::string;

            // Compute individual hash values for first,
            // second and third and combine them using XOR
            // and bit shifting:
            return ((hash<int>()(k.id) << 1) >> 1) ^ (hash<int>()(k.id) << 1);
        }
    };
}

#endif //ACO_TSP_ACO_H
