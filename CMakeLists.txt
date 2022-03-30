cmake_minimum_required(VERSION 3.14)
project(affx)


option(BUILD_PYTHON "Build python module" ON)
option(USE_INTERNAL_PYBIND11 "Build python module" OFF)


find_package(Eigen3 REQUIRED)


add_library(affx src/affine.cpp)
target_compile_features(affx PUBLIC cxx_std_17)
target_include_directories(affx PUBLIC include)
target_link_libraries(affx PUBLIC Eigen3::Eigen)
add_library(affx::affx ALIAS affx)


if(BUILD_PYTHON)
    if(NOT pybind11_FOUND)
        if(USE_INTERNAL_PYBIND11)
            FetchContent_Declare(pybind11_fetch GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.6.2)
            FetchContent_MakeAvailable(pybind11_fetch)
        else()
            find_package(pybind11 REQUIRED)
        endif()
    endif()

    pybind11_add_module(pyaffx src/python.cpp)
    target_link_libraries(pyaffx PUBLIC affx)
endif()
