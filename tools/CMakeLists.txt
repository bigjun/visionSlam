cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(catinWorkshop)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
# Debug Settings
set( CMAKE_BUILD_TYPE Debug )



#######################################################################################################################
## packages
#######################################################################################################################
# PCL required
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OctreeLib required
#find_package(OctreeLib REQUIRED)
#include_directories(${OCTREELIB_INCLUDE_DIRS})

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
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIR})

find_package( Boost 1.40 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================
include_directories("/usr/include/openni/")
include_directories(${PROJECT_SOURCE_DIR}/include)
#==================
# libraries
#==================
set(SRC_UTILITIESLIB src/utilities/utilities.cpp)
add_library( utilities SHARED ${SRC_UTILITIESLIB} )

#==================
# executables
#==================
add_executable( imgToPointCloud src/app/imgTopointCloud.cpp )
add_executable( feature src/app/feature.cpp )
#==================
# dependencies
#==================


#==================
# library link
#==================
target_link_libraries( imgToPointCloud utilities ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( feature utilities ${Boost_PROGRAM_OPTIONS_LIBRARY} ${PCL_LIBRARIES} ${OpenCV_LIBS} )
