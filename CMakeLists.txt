cmake_minimum_required(VERSION 3.12)
project(TSP_GA)

if(UNIX AND NOT APPLE)
    set(CMAKE_C_COMPILER "gcc")
    set(CMAKE_CXX_COMPILER "g++")
elseif(APPLE)
    set(CMAKE_C_COMPILER "clang")
    set(CMAKE_CXX_COMPILER "clang++")
else()
    message( WARNING "Only support LINUX and APPLE for now." )
endif()

# For python binding
find_package(pybind11 REQUIRED)
find_package(Python COMPONENTS Interpreter Development)

# For multi threading
find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include header directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include)

# find all the header files
file(GLOB THETA_HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/*.hpp)
file(GLOB THETA_HEADER_FILES_H ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/*.h)
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
file(GLOB HEADER_FILES_H ${CMAKE_SOURCE_DIR}/include/*.h)

# add GA Solver test file as executable
add_executable(test_ga_solver ${CMAKE_SOURCE_DIR}/test/test_ga_solver.cpp ${HEADER_FILES_HPP})

# Build C++ library for Python, GA Solver
pybind11_add_module(TSP_GA_Solver src/TSP_GA_Solver.cpp)
target_link_libraries(TSP_GA_Solver PRIVATE)
