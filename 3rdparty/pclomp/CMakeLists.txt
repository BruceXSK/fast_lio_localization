CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(pclomp)

SET(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(PCL REQUIRED)

FIND_PACKAGE(OpenMP)
IF (OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF ()

INCLUDE_DIRECTORIES(
        include
)

ADD_LIBRARY(
        ndt_omp
        src/voxel_grid_covariance_omp.cpp
        src/ndt_omp.cpp
        src/gicp_omp.cpp
)
