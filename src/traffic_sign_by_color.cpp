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

    CascadeUtil cascadeUtil;

    std::vector<cv::Vec3f> circles = getBlueCircles(roi);

    printf("detected circles size %d\n", circles.size());

    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
        circle(roi, center, radius, yellow, 2, 8, 0);

        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);

        if (isWithMat(circleBox, roi)) {

            Mat circleROI(roi, circleBox);

            /*
            Mat black = Mat::zeros(circleBox.size(), CV_8UC3);
            circle(black, Point(radius,radius), radius, Scalar::all(255), -1);
            imshow("Test", circleROI);
            circleROI = black & circleROI;
            Mat gray;
            cvtColor(circleROI, gray, COLOR_RGB2GRAY);
            dilate(gray, gray, getStructuringElement(MORPH_ELLIPSE, Size(6,6)));
            Mat arrow;
            inRange(gray, 100, 255, arrow);
            erode(arrow, arrow, getStructuringElement(MORPH_ELLIPSE, Size(6,6)));

            imshow("Gray", gray);
            imshow("Arrow", arrow);
             */

            cascadeUtil.setDetectionArea(circleROI);
            cascadeUtil.detectAllBlueSigns();


            for (unsigned j = 0; j < cascadeUtil.parking.size(); j++) {
                rectangle(roi, cascadeUtil.parking[j], yellow, 2, 1);
                putText(roi, "parking", Point(50, 90), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
            }

            for (unsigned k = 0; k < cascadeUtil.left_.size(); k++){
                rectangle(roi, cascadeUtil.left_[k], green, 2, 1);
                putText(roi, "left", Point(50, 110), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
            }


            for (unsigned n = 0; n < cascadeUtil.right_.size(); n++){
                rectangle(roi, cascadeUtil.right_[n], purple, 2, 1);
                putText(roi, "right", Point(50, 150), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
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