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

enum ArrowDirection{
    NO_ARROW = -1, UP_ARROW, DOWN_ARROW, RIGHT_ARROW, LEFT_ARROW
};

cv::Mat getTrafficSignROI(cv::Mat bgr);

cv::Rect MatchingMethod(cv::Mat img, cv::Mat templ, int match_method =  cv::TM_CCORR_NORMED);

std::vector<cv::Vec3f> getBlueCircles(cv::Mat bgr, int high = 100, int low = 15, int minRadius = 60, int maxRadius = 100);

/**
 * @brief checks whether the rectangle is within Mat
 * @param circleBox
 * @param bgr
 * @return
 */
bool isWithMat(cv::Rect circleBox, cv::Mat bgr);


ArrowDirection isArrowDetected(cv::Mat bgr);

#endif //PI_PROJECT_TRAFFIC_SIGN_H
