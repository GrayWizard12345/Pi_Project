//
// Created by madina on 05/05/19.
//

#ifndef PI_PROJECT_TRAFFIC_SIGN_H
#define PI_PROJECT_TRAFFIC_SIGN_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <string>

#include <math.h>

using cv::Mat;
using cv::Scalar;

enum Sign {
    NO_SIGN, LEFT_TURN_SIGN, RIGHT_TURN_SIGN, STOP_SIGN, PEDESTRIAN_SIGN, PARKING_SIGN
};

extern Sign signDetected;

extern Scalar blue;
extern Scalar yellow;
extern Scalar green;
extern Scalar orange;
extern Scalar violet;
extern Scalar purple;
extern Scalar pink;

#endif //PI_PROJECT_TRAFFIC_SIGN_H
