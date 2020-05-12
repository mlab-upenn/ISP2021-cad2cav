#ifndef ACO_TSP_ACO_H
#define ACO_TSP_ACO_H

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace aco
{
    /**
     * Node used by the graph
     */
    struct Node
    {
        Node(double x, double y, int id);
        int id;
        double x{};
        double y{};
        std::vector<std::pair<int, double>> neighbors;

        bool operator==(const Node&other) const;

        /**
         * Get distance between current node and node with node id as node_id
         * @param node_id
         * @return distance
         */
        double get_distance_from_neighbor(int node_id) const;
    };

    struct Graph
    {
    private:
        /**
         * The main graph container (The graph is immutable)
         */
        std::vector<Node> graph_;

        /**
         * Number of nodes in the graph
         */
        int n_nodes_;

        /**
         * Get node pointer from graph corresponding to the node id
         * @param node_id
         * @return
         */
        aco::Node* get_node_from_graph(int node_id);

    public:
        /**
         * Get the number of nodes in a graph
         * @return
         */
        int size() const;

        /**
         * Get the mean of all edge weights in the graph
         * @return
         */
        double mean_edge_weight() const;

        /**
         * Create a new node in Graph with co-ordinates x and y
         * @param x - x coordinate
         * @param y - y coordinate
         * @return
         */
        int create_node_in_graph(double x, double y);

        /**
         * Add edge (onw way) from node_id_from to node_id_to in the graph
         * @param node_id_from
         * @param node_id_to
         */
        void add_edge(int node_id_from, int node_id_to);

        /**
         * Get node from graph corresponding to the node id
         * @param node_id
         * @return
         */
        aco::Node get_node_from_graph(int node_id) const;
    };

    /**
     * Parameters for solving the Ant Colony Optimzation Problem for TSP
     */
    struct AcoParams
    {
        int n_ants;
        int max_iters;
        double alpha;
        double beta;
        double rho;
    };

    /**
     * Parameters for solving the Improved Ant Colony Optimization for VRP
     */
    struct IacoParamas : public AcoParams
    {
        int n_vehicles;
    };

    /**
     * Function to solve the traveling salesman problem using the ant colony optimization
     * @param graph
     * @param params
     * @return
     */
    std::vector<aco::Node> solve_tsp(const Graph& graph, const AcoParams& params, int initial_node_id = -1);

    /**
     * Function to solve the vehicle routing problem using the ant colony optimization
     * @param graph
     * @param params
     * @return
     */
    std::vector<std::vector<aco::Node>> solve_vrp(const Graph& graph, const AcoParams& params, int initial_node_id = -1);
}

#endif //ACO_TSP_ACO_H
