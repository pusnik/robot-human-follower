cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(Detector src/OnlineBoosting/OS_specific.h)
rosbuild_add_library(Detector src/OnlineBoosting/windefs.h)
rosbuild_add_library(Detector src/OnlineBoosting/ImageRepresentation.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/Patches.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/StrongClassifier.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/StrongClassifierDirectSelection.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/BoostingDetector.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/Regions.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/BaseClassifier.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/ClassifierThreshold.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/EstimatedGaussDistribution.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/FeatureHaar.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/StrongClassifierStandard.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/WeakClassifier.cpp)
rosbuild_add_library(Detector src/OnlineBoosting/WeakClassifierHaarFeature.cpp)

rosbuild_add_library(Detector
											 src/preprocessing.cpp
											 )


