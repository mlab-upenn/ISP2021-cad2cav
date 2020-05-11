#include "../aco.h"

int main()
{
    aco::Node A(0, 3);
    aco::Node B(0, 1);
    aco::Node C(0, 0);
    aco::Node D(2, 1);
    aco::Node E(2, 0);

    aco::Graph graph;

    // Add nodes to graph
    int id_A = graph.add_node_to_graph(A);
    int id_B = graph.add_node_to_graph(B);
    int id_C = graph.add_node_to_graph(C);
    int id_D = graph.add_node_to_graph(D);
    int id_E = graph.add_node_to_graph(E);

    // Add edges to graph
    graph.add_edge(id_A, id_B);
    graph.add_edge(id_B, id_A);
    graph.add_edge(id_B, id_C);
    graph.add_edge(id_B, id_D);
    graph.add_edge(id_C, id_B);
    graph.add_edge(id_C, id_E);
    graph.add_edge(id_D, id_B);
    graph.add_edge(id_D, id_E);
    graph.add_edge(id_E, id_C);
    graph.add_edge(id_E, id_D);

    aco::AcoParams params{.n_ants = 10, .max_iters = 100};

    aco::solve_tsp(graph, params);

    return 0;
}
