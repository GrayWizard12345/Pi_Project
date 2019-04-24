cmake_minimum_required(VERSION 3.9.4)
project(Pi_Project)

set(CMAKE_CXX_STANDARD 11)

add_executable(Pi_Project main.cpp LaneDetector.cpp motion.cpp crosswalk_detection.cpp sensors_controll.cpp traffic_light.cpp traffic_sign.cpp)
SET(CMAKE_CXX_FLAGS -lpthread -lwiringPi)