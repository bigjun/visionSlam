cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(slam)
# -DCMAKE_BUILD_TYPE=Debug/Release

set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")
#add_definitions( -std=c++0x )
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH $${PROJECT_SOURCE_DIR}/cmake)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)


#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)


#######################################################################################################################
## packages
#######################################################################################################################
# PCL required
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OctreeLib required
find_package(OctreeLib REQUIRED)
include_directories(${OCTREELIB_INCLUDE_DIRS})

# GSL required
#find_package(GSL REQUIRED)
#include_directories(${GSL_INCLUDE_DIRS})
#link_directories(${GSL_LIBRARY_DIRS})

# TBB required
#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIRS})

# OpenCV required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("${OpenCV_INCLUDE_DIRS}")

# G2O required
find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIR})
include_directories("/opt/ros/indigo/include")

# For building the CHOLMOD / CSPARSE solvers
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)
find_package(Qt4 REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
#include_directories("/usr/include/suitesparse")

#Boost
#find_package( Boost 1.40 COMPONENTS program_options REQUIRED )
find_package( Boost 1.40  REQUIRED)
include_directories( ${Boost_INCLUDE_DIR} )

#catkin
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole pcl_ros pcl_msgs pcl_conversions  cv_bridge sensor_msgs genmsg tf_conversions eigen_conversions image_geometry)
#catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES rosmrsmap
#  CATKIN_DEPENDS cv_bridge pcl_ros pcl_conversions roscpp sensor_msgs rospy tf_conversions eigen_conversions
#  DEPENDS system_lib std_msgs
#)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================
#include_directories("/usr/include/openni")
#include_directories("/usr/include")
include_directories(${PROJECT_SOURCE_DIR}/src/slam)
#==================
# libraries
#==================


set(SRC_ADDLIB  tool/pcl_utils.cpp
                tool/ParameterReader.cpp
                tool/ransac_transformation.cpp
                registration/registration.cpp
                feature/feature_detection.cpp
                feature/feature_matcher.cpp
                feature/planar.cpp
                slam/slamEnd.cpp)



add_library( slamlibs SHARED ${SRC_ADDLIB}  )




#==================
# executables
#==================
add_executable( slam_test slam/main.cpp )
add_executable( planeRegister slam/planeRegister.cpp )
add_executable( extractPlane  slam/extractPlane.cpp )
#add_executable( imgToPointCloud src/app/imgTopointCloud.cpp )
#add_executable( feature src/app/feature.cpp )
#==================
# dependencies
#==================


#==================
# library link
#==================
link_directories("/opt/ros/indigo/lib")
target_link_libraries(slamlibs
                    ${Boost_PROGRAM_OPTIONS_LIBRARY}
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    ${catkin_LIBRARIES}
                    ${G2O_CORE_LIBRARY}
                    ${G2O_STUFF_LIBRARY}
                    ${G2O_SOLVER_CSPARSE}
                    ${G2O_SOLVER_CSPARSE_EXTENSION}
                    ${G2O_TYPES_SLAM3D})


target_link_libraries(slam_test
                      slamlibs )
target_link_libraries(planeRegister
                      slamlibs )
target_link_libraries(extractPlane
                        slamlibs )

