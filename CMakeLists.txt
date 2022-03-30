CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(fast_lio_localization)

SET(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenMP)
FIND_PACKAGE(
        catkin REQUIRED
        roscpp
        std_msgs
        nav_msgs
        tf
        tf2_ros
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_filters
        tf_conversions
)

IF (OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF ()

CATKIN_PACKAGE(
        #  INCLUDE_DIRS include
        #  LIBRARIES fast_lio_localization
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

INCLUDE_DIRECTORIES(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

ADD_LIBRARY(
        ndt_omp
        src/pclomp/voxel_grid_covariance_omp.cpp
        src/pclomp/ndt_omp.cpp
        src/pclomp/gicp_omp.cpp
)

ADD_EXECUTABLE(fast_lio_localization src/fast_lio_localization.cpp)
ADD_DEPENDENCIES(fast_lio_localization ndt_omp)
TARGET_LINK_LIBRARIES(fast_lio_localization ${PCL_LIBRARIES} ${catkin_LIBRARIES} ndt_omp)

ADD_EXECUTABLE(map_loader src/map_loader.cpp)
TARGET_LINK_LIBRARIES(map_loader ${PCL_LIBRARIES} ${catkin_LIBRARIES})
