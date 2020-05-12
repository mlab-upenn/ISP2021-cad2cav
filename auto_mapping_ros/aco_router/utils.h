#ifndef ACO_TSP_UTILS_H
#define ACO_TSP_UTILS_H

#include "tsp_solver.h"

/**
 * Convert user defined graph type to aco graph type
 * @tparam GraphType - GraphType must be a sequential std container (Eg. Vector) containing sequence of nodes
 * @param graph
 * @return
 */
template <typename GraphType>
aco::Graph convert_to_aco_graph(const GraphType& graph);

#endif //ACO_TSP_UTILS_H

#include "utils_impl.h"