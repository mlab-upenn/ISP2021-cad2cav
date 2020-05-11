cmake_minimum_required(VERSION 3.10)
project(aco_tsp)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_library(acolib aco.h aco.cpp)

include_directories(test)
add_executable(test_tsp test/test_tsp.cpp)
target_link_libraries(test_tsp acolib)