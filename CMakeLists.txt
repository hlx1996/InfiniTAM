cmake_minimum_required(VERSION 2.8.3)
project(infinitam_ros)
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
ADD_SUBDIRECTORY(libs/ITMLib)
ADD_SUBDIRECTORY(libs/ORUtils)
ADD_SUBDIRECTORY(libs/MiniSlamGraphLib)
ADD_SUBDIRECTORY(libs/FernRelocLib)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        tf
        cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    SYSTEM
    include
    libs
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11  -O3 -Wall -pthread")
list(APPEND CUDA_NVCC_FLAGS -arch=sm_61)
list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_61,code=sm_61)
list(APPEND CUDA_NVCC_FLAGS "-std=c++11 -O3 -use_fast_math")

set(CMAKE_BUILD_TYPE "RELEASE")
set(CUDA_PROPAGATE_HOST_FLAGS OFF)

cuda_add_executable(infinitam_ros_node
        src/CLIEngine.cpp
        src/RosWrappedMain.cpp
        src/InfiniTAMCheck.cu
)

target_link_libraries(infinitam_ros_node
        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ITMLib
        ORUtils
        MiniSlamGraphLib
        FernRelocLib
)