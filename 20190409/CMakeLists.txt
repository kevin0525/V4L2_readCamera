cmake_minimum_required(VERSION 2.6)

project(camera)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE RELEASE)
set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)

file(GLOB_RECURSE sources "capture.cpp")

add_executable(camera ${sources})

target_link_libraries(camera ${OpenCV_LIBS})
