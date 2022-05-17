CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(ndt_gpu)

SET(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(CUDA)

SET_DIRECTORY_PROPERTIES(PROPERTIES COMPILE_DEFINITIONS "")

IF (CMAKE_CROSSCOMPILING)
    IF (NOT CUDA_ARCH)
        MESSAGE(FATAL_ERROR "Please define the CUDA_ARCH CMake variable")
    ENDIF ()
ELSE ()
    IF (NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
        SET(CUDA_CAPABILITY_VERSION_CHECKER
            "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker"
            )
    ENDIF ()

    EXECUTE_PROCESS(COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
                    OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    )

    IF ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
        SET(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
    ELSE ()
        SET(CUDA_ARCH "sm_52")
    ENDIF ()
    MESSAGE(STATUS CUDA_ARCH: ${CUDA_ARCH})
ENDIF ()

SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++14;--ptxas-options=-v)

INCLUDE_DIRECTORIES(
        include
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

SET(srcs
    src/MatrixDevice.cu
    src/MatrixHost.cu
    src/NormalDistributionsTransform.cu
    src/Registration.cu
    src/VoxelGrid.cu
    src/SymmetricEigenSolver.cu
    )
SET(incs
    include/ndt_gpu/common.h
    include/ndt_gpu/debug.h
    include/ndt_gpu/Matrix.h
    include/ndt_gpu/MatrixDevice.h
    include/ndt_gpu/MatrixHost.h
    include/ndt_gpu/NormalDistributionsTransform.h
    include/ndt_gpu/Registration.h
    include/ndt_gpu/SymmetricEigenSolver.h
    include/ndt_gpu/VoxelGrid.h
    )

CUDA_ADD_LIBRARY(ndt_gpu ${srcs} ${incs})
TARGET_LINK_LIBRARIES(
        ndt_gpu
        ${CUDA_LIBRARIES}
        ${CUDA_CUBLAS_LIBRARIES}
        ${CUDA_curand_LIBRARY}
        ${PCL_LIBRARIES}
)
