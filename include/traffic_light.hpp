#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include <csignal>
#include "motion.hpp"
#include "traffic_sign.h"

#define RED_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 7
#define GREEN_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 8


/**
 * @brief for indicating the status of traffic light
 */
enum Status {
    RED_LIGHT,
    GREEN_LIGHT
};

extern cv::Mat frame;
extern cv::Mat red_color_frame;
extern cv::Mat green_color_frame;
extern Status trafficLightStatus;
extern pthread_t trafficLightThread;
extern pthread_mutex_t motor_mutex;
extern int width;
extern int height;

extern int cascadeMinRadius;
extern int cascadeMaxRadius;

extern cv::Mat red_hue_image;
/**
 *  @brief the thread function for red and green light detection
 */
    void *trafficLightLoop(void*);

/**
 * @brief initializing traffic light thread
 * @param src is used to get a ROI from captured frame
 */
    int initTrafficLightThread();

void killSignThread();