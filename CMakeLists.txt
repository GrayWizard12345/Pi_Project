cmake_minimum_required(VERSION 3.9.4)
project(Pi_Project)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
include_directories($OpenCV_INCLUDE_DIRS)

add_executable(traffic_sign_video src/traffic_sign_by_color_demo.cpp src/traffic_sign.cpp src/CascadeUtil.cpp)
target_link_libraries(traffic_sign_video ${OpenCV_LIBS})

add_executable(traffic_sign_img src/traffic_sign_by_color.cpp src/traffic_sign.cpp src/CascadeUtil.cpp src/ShapeDetector.cpp src/ShapeDetector.h)
target_link_libraries(traffic_sign_img ${OpenCV_LIBS})

add_executable(matching src/matching.cpp )
target_link_libraries(matching ${OpenCV_LIBS})