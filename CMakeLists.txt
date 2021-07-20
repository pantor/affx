cmake_minimum_required(VERSION 3.14)
project(affx)


option(BUILD_PYTHON "Build python module" ON)


find_package(Eigen3 REQUIRED)


add_library(affx src/affine.cpp)
target_compile_features(affx PUBLIC cxx_std_17)
target_include_directories(affx PUBLIC include)
target_link_libraries(affx PUBLIC Eigen3::Eigen)


if(BUILD_PYTHON)
    find_package(pybind11 2.6 REQUIRED)

    pybind11_add_module(pyaffx src/python.cpp)
    target_link_libraries(pyaffx PUBLIC affx)
endif()
