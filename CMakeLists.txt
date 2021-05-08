cmake_minimum_required(VERSION 3.12)
project(affx)


option(BUILD_PYTHON "Build python module" ON)


find_package(Eigen3 REQUIRED)


add_library(affx src/affine.cpp)
target_include_directories(affx PUBLIC include ${EIGEN3_INCLUDE_DIR})


if(BUILD_PYTHON)
    find_package(pybind11 2.6 REQUIRED)

    pybind11_add_module(pyaffx src/python.cpp)
    target_link_libraries(pyaffx PUBLIC affx)
endif()
