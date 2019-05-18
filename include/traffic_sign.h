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
    NO_SIGN, LEFT_TURN, RIGHT_TURN, STOP, PEDESTRIAN, PARKING
};

extern Sign signDetected;

extern Scalar blue;
extern Scalar yellow;
extern Scalar green;
extern Scalar orange;
extern Scalar violet;
extern Scalar purple;
extern Scalar pink;
extern Mat src;

cv::Mat getTrafficSignROI(cv::Mat bgr);

cv::Rect MatchingMethod(cv::Mat img, cv::Mat templ, int match_method = cv::TM_CCORR_NORMED);

Mat
getEdges(cv::Mat bgr);


/**
 * @brief checks whether the rectangle is within Mat
 * @param circleBox
 * @param bgr
 * @return
 */
bool isWithMat(cv::Rect circleBox, cv::Mat bgr);


cv::Mat getRedMask(cv::Mat roi);

cv::Mat convertToYCrCb(cv::Mat input);

Mat gammaCorrection(const Mat &img, const double gamma_);

void initSignDetection();

void detectSign(Mat bgr);

std::vector<cv::Vec3f> getCircles(Mat edges, int high = 100, int low = 15, int minRadius = 60, int maxRadius = 100);

void getRectangles(Mat edges, Mat src);

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

#endif //PI_PROJECT_TRAFFIC_SIGN_H
