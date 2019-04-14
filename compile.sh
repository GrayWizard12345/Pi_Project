#!/bin/bash

g++ main.cpp LaneDetector.cpp  motion.cpp -o main -I /usr/local/include -L /usr/local/lib -L /opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` -lwiringPi -Wall -pthread
