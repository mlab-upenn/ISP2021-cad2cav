# Graph Partitioner
Graph partitioner module for the project. Offers graph partitioning & TSP loop solving functionalities regarding the Automapper.

The graph partitioner offers the following modes:
1. **Graph partitioning:** Spectral clustering, Multilevel k-way graph partitioning (METIS solver)
2. **TSP solving:** branch-and-bound algorithm, Google OR-Tools solver

# Software Requirements
- [METIS 5.1.0](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)
- [Google OR-Tools 9.0](https://github.com/google/or-tools/releases/tag/v9.0)

METIS can be installed by:
```bash
sudo apt install libmetis-dev
```

**Google OR-Tools can either be pre-installed onto the system or installed upon building the project.** By default, the software assumes that users have a working version of Google OR-Tools 9.0 installed in the system (`-DBUILD_ORTOOLS=OFF`). However, users can pass `-DBUILD_ORTOOLS=ON` as CMake arguments to switch on the functionality of compiling Google OR-Tools on the fly. 

In order to integrate OR-Tools libraries into the CMake build system, OR-Tools itself needs to be **built with CMake** (by default it's through Makefile). You can find instructions on how to build OR-Tools with CMake [here.](https://github.com/google/or-tools/tree/v9.0/cmake)

***NOTE. Compiling the minimum Google OR-Tools library takes about 5 minutes.***

# Usage
See general Automapper usage in [main README.](https://github.com/mlab-upenn/ISP2021-cad2cav/blob/main/README.md)
