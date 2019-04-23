cmake_minimum_required(VERSION 2.8.3)
project(undistortimage_xk)
set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
add_executable(undistortimage_xk_node src/undistortimage_xk.cpp)

target_link_libraries(undistortimage_xk_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)