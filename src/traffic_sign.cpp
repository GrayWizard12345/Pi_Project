//
// Created by madina on 05/05/19.
//

#include "traffic_sign.h"

using namespace cv;

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
    //85 60 60
    inRange(hsv_image, cv::Scalar(50, 0, 0), cv::Scalar(140, 255, 255), blue_hue_range);
    //dilate(blue_hue_range, blue_hue_range, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

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
    cv::blur(edges, edges, cv::Size(3, 3));
    imshow(std::to_string(index) + "Canny", edges);
    cvWaitKey(0);

    //region With HoughLinesP
    std::vector<cv::Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 30, 10);

    cv::Point ini;
    cv::Point fini;

    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        cv::line(circleROI, ini, fini, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

        printf("%lf\t", slope);

    }
    //endregion

    ArrowDirection turn = NO_ARROW;


    printf("turn %d %d\n\n\n", index, turn);

    return turn;
}