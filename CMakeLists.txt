cmake_minimum_required(VERSION 3.12)
project(affx)


option(BUILD_PYTHON "Build python module" ON)


find_package(Eigen3 REQUIRED)


add_library(affx src/affine.cpp)
target_include_directories(affx PUBLIC include)
target_link_libraries(affx PUBLIC Eigen3::Eigen)


if(BUILD_PYTHON)
    find_package(pybind11 2.6 QUIET)
    if(NOT pybind11_FOUND)
        FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.6.2)
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(pyaffx src/python.cpp)
    target_link_libraries(pyaffx PUBLIC affx)
endif()
