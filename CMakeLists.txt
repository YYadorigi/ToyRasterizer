cmake_minimum_required(VERSION 3.10)

# Set the project name
project(ToyRasterizer)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(ToyRasterizer main.cpp tgaimage.hpp tgaimage.cpp geometry.hpp model.hpp model.cpp)