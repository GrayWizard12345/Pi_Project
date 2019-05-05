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
    inRange(hsv_image, cv::Scalar(80, 65, 65), cv::Scalar(140, 255, 255), blue_hue_range);
    dilate(blue_hue_range, blue_hue_range, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    imshow("Blue", blue_hue_range);

    std::vector<cv::Vec3f> circles;
    HoughCircles(blue_hue_range, circles, cv::HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, minRadius,
                 maxRadius);

    return circles;
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

ArrowDirection isArrowDetected(cv::Mat circleROI) {
    static int index = 0;
    index++;

    cv::Mat gray;
    cvtColor(circleROI, gray, cv::COLOR_RGB2GRAY);

    //image processing for better accuracy
    dilate(gray, gray, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)));
    cv::Mat arrow;
    inRange(gray, 100, 255, arrow);
    erode(arrow, arrow, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)));

    cv::Mat edges;
    cv::Canny(arrow, edges, 50, 150);
    imshow(std::to_string(index) + "Canny", edges);
    cvWaitKey(0);

    std::vector<cv::Vec2f> lines;
    HoughLines(edges, lines, 1, CV_PI / 180, 50);

    int left[] = {0, 0};
    int right[] = {0, 0};
    int up[] = {0, 0};
    int down[] = {0, 0};

    printf("%d lines size %d\n", index, lines.size());

    for (size_t i = 0; i < lines.size(); i++) {

        float rho = lines[i][0], theta = lines[i][1];

        printf("rho - %d theta - %d\n", rho, theta);

        //cases for right/left arrows
        if ((round(theta, 2) >= 1.0 && round(theta, 2) <= 1.1) ||
            (round(theta, 2) >= 2.0 and round(theta, 2) <= 2.1)) {
            if (rho >= 20 and rho <= 30)
                left[0] += 1;
            else if (rho >= 60 and rho <= 65)
                left[1] += 1;
            else if (rho >= -73 and rho <= -57)
                right[0] += 1;
            else if (rho >= 148 and rho <= 176)
                right[1] += 1;
        } else if ((round(theta, 2) >= 0.4 && round(theta, 2) <= 0.6) ||
                   (round(theta, 2) >= 2.6 && round(theta, 2) <= 2.7)) {
            if (rho >= -63 and rho <= -15)
                up[0] += 1;
            else if (rho >= 67 and rho <= 74) {
                down[1] += 1;
                up[1] += 1;
            } else if (rho >= 160 and rho <= 171)
                down[0] += 1;
        }
    }

    //region With HoughLinesP
    cv::Point ini;
    cv::Point fini;

    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        printf("%lf\t", slope);


    }
    //endregion

    ArrowDirection turn = NO_ARROW;

    if (left[0] >= 1 and left[1] >= 1)
        turn = LEFT_ARROW;
    else if (right[0] >= 1 and right[1] >= 1)
        turn = RIGHT_ARROW;
    else if (up[0] >= 1 and up[1] >= 1)
        turn = UP_ARROW;
    else if (down[0] >= 1 and down[1] >= 1)
        turn = DOWN_ARROW;

    printf("turn %d %d\n\n\n", index, turn);

    return turn;
}