//
// Created by madina on 05/05/19.
//

#include "traffic_sign.h"



//TODO change the points of ROI
cv::Mat getTrafficSignROI(cv::Mat bgr) {


    cv::Rect roiBox(0, bgr.rows / 6, bgr.cols, bgr.rows / 4);

    cv::Mat roi(bgr, roiBox);


    return roi;
}

std::vector<cv::Vec3f> getBlueCircles(cv::Mat bgr, int high, int low, int minRadius, int maxRadius) {
    cv::Mat hsv_image;
    cvtColor(bgr, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat blue_hue_range;
    inRange(hsv_image, cv::Scalar(50, 0, 0), cv::Scalar(140, 255, 255), blue_hue_range);
    dilate(blue_hue_range, blue_hue_range, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    imshow("Blue", blue_hue_range);

    std::vector<cv::Vec3f> circles;
    HoughCircles(blue_hue_range, circles, cv::HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, minRadius, maxRadius);

    return circles;
}

bool isWithMat(cv::Rect circleBox, cv::Mat bgr){
    return  0 <= circleBox.x
            && 0 <= circleBox.width
            && circleBox.x + circleBox.width <= bgr.cols
            && 0 <= circleBox.y
            && 0 <= circleBox.height
            && circleBox.y + circleBox.height <= bgr.rows;
}