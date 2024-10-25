cmake_minimum_required(VERSION 3.25)
project(Graph)

set(CMAKE_CXX_STANDARD 23)

add_executable(Graph main.cpp Graph.cpp Graph.h AntColony.cpp AntColony.h)
