cmake_minimum_required(VERSION 3.9.4)
project(Pi_Project)

set(CMAKE_CXX_STANDARD 11)

add_executable(Pi_Project src/main.cpp src/LaneDetector.cpp src/motion.cpp src/sensors_controll.cpp src/traffic_light.cpp src/traffic_sign.cpp src/config_reader.cpp include/config_reader.h)
SET(CMAKE_CXX_FLAGS -lpthread -lwiringPi)