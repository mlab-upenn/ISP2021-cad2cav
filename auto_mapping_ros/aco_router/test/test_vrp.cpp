#include "../aco.h"

int main()
{
    aco::Graph graph;

    // Create nodes in graph
    int id_A = graph.create_node_in_graph(0, 1);
    int id_B = graph.create_node_in_graph(1, 0);
    int id_C = graph.create_node_in_graph(7, 0);
    int id_D = graph.create_node_in_graph(8, 1);
    int id_E = graph.create_node_in_graph(8, 7);
    int id_F = graph.create_node_in_graph(7, 8);
    int id_G = graph.create_node_in_graph(1, 8);
    int id_H = graph.create_node_in_graph(0, 7);
    int id_I = graph.create_node_in_graph(4, 4);

    // Add edges to graph
    graph.add_edge(id_A, id_B);
    graph.add_edge(id_A, id_I);

    graph.add_edge(id_B, id_A);
    graph.add_edge(id_B, id_I);

    graph.add_edge(id_C, id_D);
    graph.add_edge(id_C, id_I);

    graph.add_edge(id_D, id_C);
    graph.add_edge(id_D, id_I);

    graph.add_edge(id_E, id_F);
    graph.add_edge(id_E, id_I);

    graph.add_edge(id_F, id_E);
    graph.add_edge(id_F, id_I);

    graph.add_edge(id_G, id_H);
    graph.add_edge(id_G, id_I);

    graph.add_edge(id_H, id_G);
    graph.add_edge(id_H, id_I);

    graph.add_edge(id_I, id_A);
    graph.add_edge(id_I, id_B);
    graph.add_edge(id_I, id_C);
    graph.add_edge(id_I, id_D);
    graph.add_edge(id_I, id_E);
    graph.add_edge(id_I, id_F);
    graph.add_edge(id_I, id_G);
    graph.add_edge(id_I, id_H);

    // Set Parameters of the Ant Colony Optimization Problem
    aco::IacoParamas params{};
    params.max_iters = 10;
    params.n_ants = 10;
    params.alpha = 1;
    params.beta = 1;
    params.rho = 0.05;
    params.n_vehicles = 4;

    // Solve the TSP using Ant Colony Optimization
    const auto best_route = aco::solve_tsp(graph, params, id_A);

    return 0;
}
