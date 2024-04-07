PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)
MESSAGE(STATUS "Eigen3 Include Dirs: ${EIGEN3_INCLUDE_DIRS}")

# # Enable CUDA
# FIND_PACKAGE(CUDA REQUIRED)
# enable_language(CUDA)
# message(STATUS "CUDA Include Dirs: ${CUDA_INCLUDE_DIRS}")
# message(STATUS "CUDA Libraries: ${CUDA_LIBRARIES}")

# set(CMAKE_CUDA_STANDARD 11)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()


# FIND_PACKAGE( Eigen3 REQUIRED )
# SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
# IF( NOT EIGEN3_INCLUDE_DIR )
#     MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
# ENDIF()
# INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl-shared-lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

ROSBUILD_ADD_EXECUTABLE(slam
                        src/slam/slam_main.cc
                        src/slam/slam.cc)
TARGET_LINK_LIBRARIES(slam shared_library ${libs})

# ADD_LIBRARY(particle_filter_cu
#   src/particle_filter/particle_filter.cu
# )

ROSBUILD_ADD_EXECUTABLE(particle_filter
                        src/particle_filter/particle_filter_main.cc
                        src/particle_filter/particle_filter.cc)
# set_target_properties(particle_filter PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
# TARGET_LINK_LIBRARIES(particle_filter particle_filter_cu shared_library ${libs} ${CUDA_LIBRARIES})
TARGET_LINK_LIBRARIES(particle_filter shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/motion_primitives.cc
                        src/navigation/constant_curvature_arc.cc
                        src/navigation/ackermann_motion_primitives.cc
                        src/navigation/high_level_nav_plan.cc)
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})

ADD_EXECUTABLE(eigen_tutorial
               src/eigen_tutorial.cc)

ADD_EXECUTABLE(simple_queue_test
               src/navigation/simple_queue_test.cc)

ADD_EXECUTABLE(cimg_example
               src/cimg_example.cc)
TARGET_LINK_LIBRARIES(cimg_example shared_library ${libs} X11)