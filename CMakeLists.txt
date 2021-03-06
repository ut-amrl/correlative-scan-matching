PROJECT(correlative_scan_matcher)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Correlative Scan Matching Implementation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror ")

IF(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")
ENDIF()

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
    option(TESTING "Build tests" OFF)
    MESSAGE(STATUS "Additional Flags for Release mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    option(TESTING "Build tests" ON)
    MESSAGE(STATUS "Additional Flags for Debug mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -g -O0")
ENDIF()

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
FIND_PACKAGE(OpenMP REQUIRED)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

IF (CSM_LIBRARY)
    SET(libs gflags glog boost_system m X11 OpenMP::OpenMP_CXX)
    ADD_LIBRARY(csm src/CorrelativeScanMatcher.cc)
    TARGET_LINK_LIBRARIES(csm ${LIBS})
    SET_TARGET_PROPERTIES(csm PROPERTIES
                          COMPILE_FLAGS ${CMAKE_CXX_FLAGS})
    MESSAGE(STATUS "CSM Building as library")
    MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
ELSE()
    INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
    
    rosbuild_init()
    
    SET(ROS_BUILD_STATIC_LIBS true)
    SET(ROS_BUILD_SHARED_LIBS false)
    
    MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
    MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
    
    
    rosbuild_genmsg()
    SET(libs roslib roscpp rosbag console_bridge cv_bridge gflags glog boost_system m X11 OpenMP::OpenMP_CXX)
    ROSBUILD_ADD_EXECUTABLE(correlative_scan_match_pair
                            src/bag_pair_matcher.cc
                            src/pointcloud_helpers.cc
                            src/CorrelativeScanMatcher.cc)
    
    ROSBUILD_ADD_EXECUTABLE(correlative_scan_bag_avg
                            src/bag_avg_matcher.cc
                            src/pointcloud_helpers.cc
                            src/CorrelativeScanMatcher.cc)
    
    TARGET_LINK_LIBRARIES(correlative_scan_match_pair
                          ${libs}
                          ${CERES_LIBRARIES})
    TARGET_LINK_LIBRARIES(correlative_scan_bag_avg
                          ${libs}
                          ${CERES_LIBRARIES})
ENDIF()

if (TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
