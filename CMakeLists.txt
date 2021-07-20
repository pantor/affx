cmake_minimum_required(VERSION 3.12)
project(affx)


option(BUILD_PYTHON "Build python module" ON)


find_package(Eigen3)


add_library(affx src/affine.cpp)
target_include_directories(affx PUBLIC include)


if(NOT Eigen3_FOUND)
    FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.3.9)
    FetchContent_MakeAvailable(eigen)
    target_link_libraries(affx PUBLIC eigen)
else()
    target_include_directories(affx PUBLIC ${EIGEN3_INCLUDE_DIRS})
endif()


if(BUILD_PYTHON)
    find_package(pybind11 2.6)
    # if(NOT pybind11_FOUND)
    #     FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.6.2)
    #     FetchContent_MakeAvailable(pybind11)
    # endif()

    pybind11_add_module(pyaffx src/python.cpp)
    target_link_libraries(pyaffx PUBLIC affx)
endif()
