cmake_minimum_required(VERSION 2.8.3)
project(infinitam)
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
ADD_SUBDIRECTORY(libs/ITMLib)
ADD_SUBDIRECTORY(libs/ORUtils)
ADD_SUBDIRECTORY(libs/MiniSlamGraphLib)
ADD_SUBDIRECTORY(libs/FernRelocLib)

SET(targetname infinitam)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        tf
        cv_bridge
        )

find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)


include_directories(${PROJECT_SOURCE_DIR}/include)
find_package( OpenCV REQUIRED )
include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense2.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)
#link_directories(${PCL_LIBRARY_DIRS})

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
#set(CMAKE_BUILD_TYPE "RELEASE")
SET(sources
        src/CLIEngine.cpp
        src/RosWrappedMain.cpp
        src/InfiniTAMCheck.cu
        )

SET(headers
        include/CLIEngine.h
        include/InfiniTAMCheck.h

        )
SOURCE_GROUP("" FILES ${sources} ${headers})

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)



#add_executable(infinitam
#        include/CLIEngine.h
#        src/CLIEngine.cpp
#        src/InfiniTAM_cli.cc
#        )
target_link_libraries(infinitam
        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ITMLib
        ORUtils
        MiniSlamGraphLib
        FernRelocLib
        )
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense2.cmake)
#INCLUDE(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
