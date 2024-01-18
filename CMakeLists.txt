# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(curvature_computation LANGUAGES C CXX)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  pcl_ros
  pcl_conversions
  eigen_conversions
  tf
  tf_conversions
  tf2_ros
  jsk_recognition_utils
  )

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

# openmp
find_package(OpenMP REQUIRED)

# Find the CGAL library
find_package(CGAL REQUIRED)

# Include CGAL's directories and libraries
include(${CGAL_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs geometry_msgs cv_bridge pcl_ros pcl_conversions
                 eigen_conversions tf tf_conversions tf2_ros jsk_recognition_utils
  )

include_directories(
  include
  # ${catkin_INCLUDE_DIRS}  # eigen version conflict
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(Draw src/curvServer.cpp)
target_link_libraries(Draw PRIVATE Open3D::Open3D PRIVATE OpenMP::OpenMP_CXX PRIVATE ${CGAL_LIBRARIES})
