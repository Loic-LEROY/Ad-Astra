cmake_minimum_required(VERSION 3.10)
project(GraphAlgorithms)

set(CMAKE_CXX_STANDARD 17)

add_executable(GraphAlgorithms main.cpp graph.cpp commify.cpp)

find_package(Threads REQUIRED)
target_link_libraries(GraphAlgorithms PRIVATE Threads::Threads)