#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include <csignal>
#include "motion.hpp"

#define RED_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 6
#define GREEN_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 7

/**
 * @brief for indicating the status of traffic light
 */
enum Status {
    NON_TRAFFIC_LIGHT,
    RED_LIGHT,
    GREEN_LIGHT
};
cv::Mat frame_for_traffic_light;
static Status trafficLightStatus = Status::NON_TRAFFIC_LIGHT;

/**
 *  @brief the thread function for red and green light detection
 */
void *trafficLightLoop(void *);

/**
 * @brief initializing traffic light thread
 * @param src is used to get a ROI from captured frame
 */
int initTrafficLightThread();

/**
 * @brief depending on the signal, it sets trafficLightStatus value
 * @param signum signal number either RED_TRAFFIC_LIGHT_SIGNAL or GREEN_TRAFFIC_LIGHT_SIGNAL
 */
void trafficLightHandler(int signum);

/**
 * @brief initializing the interrupt handler for traffic light signal
 */
void initTrafficLightSignal();