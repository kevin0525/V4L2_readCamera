cmake_minimum_required(VERSION 2.8.3)
project(read_camera)

add_compile_options(-std=c++11)
set(OpenCV_DIR /usr/local/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${read_camera_SOURCE_DIR}/include/read_camera
  ${OpenCV_INCLUDE_DIRS}
  include
)
add_executable(read_camera_node src/read_camera.cpp)
target_link_libraries(read_camera_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
