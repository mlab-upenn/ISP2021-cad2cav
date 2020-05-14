# aco_router
Solves routing problems for coverage planning using Ant Colony Optimization based algorithms (tested on Google OR Tools dataset):
1. Traveling Salesman Problem
2. Vehicle Routing Problem

The input graph needs to be in this format: 

```
struct Graph
    {
    private:
        std::vector<Node> graph_;
        .
        .

```

You must use the two public helper functions to create the graph.
```
    public:
        /**
         * Create a new node in Graph with co-ordinates x and y
         * @param x - x coordinate
         * @param y - y coordinate
         * @return node id created
         */
        int create_node_in_graph(double x, double y);
        
        /**
         * Add edge (onw way) from node_id_from to node_id_to in the graph
         * @param node_id_from
         * @param node_id_to
         */
        void add_edge(int node_id_from, int node_id_to);
        .
        .
    };
```
The test examples shows examples of how graph is created.

Alternatively, you can also use the converter that we provide to convert your graph format to our graph format if you have a similar data structure (std::vector<NodeType> where NodeType has a neighbors field which is a vector of pointers).
```
    const auto aco_graph = aco::convert_to_aco_graph(user_graph);
```

After creating the graph, you need to set the parameters of the algorithm and then call tsp_solver or the vsp_solver.
```
    // Solve the TSP using Ant Colony Optimization
    std::pair< std::vector<aco::Node>, double> best_route_and_fitness = aco::solve_tsp(graph);
    
    // Solve the VRP using Ant Colony Optimization
    std::pair< std::vector<std::vector<aco::Node>>> best_routes_and_total_fitness = aco::solve_vrp(graph);
```
If the user wants to set an initial depot for the start point, they can do so by passing the node id to both the solvers for example:
```
    // Solve the VRP using Ant Colony Optimization
    std::pair< std::vector<std::vector<aco::Node>>> best_routes_and_total_fitness = aco::solve_vrp(graph, init_node_id);
```

The [Config File](https://github.com/YashTrikannad/aco_router/blob/master/config.cfg) contains the Parameters of Ant Colony Optimization and Opt2 Optimization that the user needs to set.

Here you can check out an example usage of this library usage on an indoor office floor-map in my [auto mapping](https://github.com/YashTrikannad/auto_mapping_ros) project. 

Note: In this case, the edges just are just for illustrative purpose to show the order of sequences (Light to Dark) and they are not equal to the actual distances between the two points. 

Vehicle Route for Car1
![Vehicle Route for Car 1](https://github.com/YashTrikannad/aco_router/blob/master/media/v1.png)

Vehicle Route for Car2
![Vehicle Route for Car 2](https://github.com/YashTrikannad/aco_router/blob/master/media/v2.png)
