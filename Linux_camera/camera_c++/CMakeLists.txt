cmake_minimum_required(VERSION 2.6)

project(read_camera)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE RELEASE)
# set(OpenCV_DIR /usr/local/share/OpenCV/)

find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)

include_directories(${PROJECT_SOURCE_DIR}/include/)

file(GLOB_RECURSE sources "${PROJECT_SOURCE_DIR}/src/main.cpp" "${PROJECT_SOURCE_DIR}/src/*.c")

add_executable(read_camera ${sources})

target_link_libraries(read_camera ${OpenCV_LIBS})
