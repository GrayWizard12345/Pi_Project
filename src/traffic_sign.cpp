//
// Created by madina on 05/05/19.
//

#include "traffic_sign.h"

using namespace cv;

//TODO change the points of ROI
cv::Mat getTrafficSignROI(cv::Mat bgr) {

    cv::Rect roiBox(0, bgr.rows / 7, bgr.cols, bgr.rows / 4);

    cv::Mat roi(bgr, roiBox);


    return roi;
}

std::vector<cv::Vec3f> getBlueCircles(cv::Mat bgr, int high, int low, int minRadius, int maxRadius) {
    cv::Mat hsv_image;
    cvtColor(bgr, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat blue_hue_range;
    //85 60 60
    inRange(hsv_image, cv::Scalar(50, 0, 0), cv::Scalar(140, 255, 255), blue_hue_range);
    //dilate(blue_hue_range, blue_hue_range, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    imshow("Blue", blue_hue_range);

    std::vector<cv::Vec3f> circles;
    HoughCircles(blue_hue_range, circles, cv::HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, minRadius,
                 maxRadius);

    return circles;
}

Mat getRedMask(Mat roi) {
    cv::Mat hsv_image;
    cvtColor(roi, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat lower_hue, upper_hue;
    inRange(hsv_image, Scalar(0, 120, 70), Scalar(10, 255, 255), lower_hue);
    inRange(hsv_image, Scalar(170, 120, 70), Scalar(180, 255, 255), upper_hue);

    Mat red_hue_image;
    //use hue values from the both ranges (masks)
    addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, red_hue_image);
    erode(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    dilate(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
    GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    return red_hue_image;
}

bool isWithMat(cv::Rect circleBox, cv::Mat bgr) {
    return 0 <= circleBox.x
           && 0 <= circleBox.width
           && circleBox.x + circleBox.width <= bgr.cols
           && 0 <= circleBox.y
           && 0 <= circleBox.height
           && circleBox.y + circleBox.height <= bgr.rows;
}

float round(float num, int decimalPlaces) {
    return roundf(num * pow(10, decimalPlaces)) / pow(10, decimalPlaces);
}

Rect MatchingMethod(cv::Mat img, cv::Mat templ, int match_method) {
    Mat img_display;
    img.copyTo(img_display);

    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    Mat result;
    result.create(result_rows, result_cols, CV_32FC1);

    matchTemplate(img, templ, result, match_method);

    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
        matchLoc = minLoc;
    } else {
        matchLoc = maxLoc;
    }

    Rect rect(matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows));

    return rect;
}