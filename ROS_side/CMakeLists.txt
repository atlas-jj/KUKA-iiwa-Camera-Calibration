cmake_minimum_required(VERSION 2.8.3)
project(iiwa_kinect_calibration)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport ar_track_alvar_msgs)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
include_directories(${VISP_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(chris_calibrator src/calibrator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(chris_calibrator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(chris_calibrator lapack blas)
