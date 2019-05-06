#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

#include "traffic_sign.h"
#include "CascadeUtil.h"

using namespace cv;
using namespace std;

cv::Scalar orange = cv::Scalar(255, 178, 102);
cv::Scalar blue = cv::Scalar(255, 255, 51);
cv::Scalar green = cv::Scalar(153, 255, 51);
cv::Scalar yellow = cv::Scalar(51, 255, 255);
cv::Scalar violet = cv::Scalar(127, 0, 255);
cv::Scalar purple = cv::Scalar(255, 51, 255);
cv::Scalar pink = cv::Scalar(255, 51, 153);

int main(int argc, char **argv) {
    Mat bgr = imread(argv[1], IMREAD_COLOR);

    Mat roi = getTrafficSignROI(bgr);
    Mat templ = imread("/media/madina/Files/Projects/opencv/Pi_Project/src/templates/arrow_left2.png", IMREAD_COLOR);

    imshow("templ", templ);


    CascadeUtil cascadeUtil;

    std::vector<cv::Vec3f> circles = getBlueCircles(roi);

    printf("detected circles size %d\n", circles.size());

    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        //TODO maybe it should be changed
        if (radius < 80)
            continue;

        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
        circle(roi, center, radius, yellow, 2, 8, 0);

        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);

        if (isWithMat(circleBox, roi)) {

            Mat circleROI(roi, circleBox);

            Rect rect = MatchingMethod(circleROI, templ);

            if (isWithMat(rect, circleROI)) {
                rectangle(circleROI, rect, pink);
                imshow("circle ROI", circleROI);
            }

        } else {
            printf("outside\n");
        }

        imshow("ROI", roi);
    }

    namedWindow("Original", WINDOW_AUTOSIZE);
    imshow("Original", bgr);


    waitKey(0);
    return 0;
}