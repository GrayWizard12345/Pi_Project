cmake_minimum_required(VERSION 3.10)
project(Pi_Project)

set(CMAKE_CXX_STANDARD 11)

add_executable(Pi_Project main.cpp LaneDetector.cpp motion.cpp)
SET(CMAKE_CXX_FLAGS -lpthread -lwiringPi)