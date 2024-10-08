cmake_minimum_required(VERSION 3.10)

# Set the project name
project(ToyRasterizer)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add the executable
add_executable(ToyRasterizer main.cpp tgaimage.hpp tgaimage.cpp model.hpp model.cpp triangle.hpp shader.hpp)

target_link_libraries(ToyRasterizer Eigen3::Eigen)