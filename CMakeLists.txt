cmake_minimum_required(VERSION 3.12)
project(TSP_GA)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find all the header files
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)

# include header directories
include_directories(
   ${CMAKE_SOURCE_DIR}/include)

# add the main file as executable
add_executable(run_solver ${CMAKE_SOURCE_DIR}/src/run_solver.cpp ${HEADER_FILES_HPP})
