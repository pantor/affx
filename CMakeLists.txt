cmake_minimum_required(VERSION 3.12)
project(affx)


find_package(Eigen3 REQUIRED)
find_package(pybind11 2.6 REQUIRED)


add_library(affx src/affine.cpp)
target_include_directories(affx PUBLIC include ${EIGEN3_INCLUDE_DIR})


pybind11_add_module(pyaffx src/python.cpp)
target_link_libraries(pyaffx PUBLIC affx)
