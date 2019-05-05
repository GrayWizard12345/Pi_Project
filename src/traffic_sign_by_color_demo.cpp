#include "opencv2/opencv.hpp"
#include "traffic_sign.h"
#include "CascadeUtil.h"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar orange = Scalar(255, 178, 102);
Scalar blue = Scalar(255, 255, 51);
Scalar green = Scalar(153, 255, 51);
Scalar yellow = Scalar(51, 255, 255);
Scalar violet = Scalar(127, 0, 255);
Scalar purple = Scalar(255, 51, 255);
Scalar pink = Scalar(255, 51, 153);

//FIXME blue range sees black objects

//TODO move all data into the method

//TODO divide the video into the parts by sign
//TODO detect rectangle for parking sign
//TODO add inRange for red signs

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Not enough parameters" << std::endl;
        return -1;
    }

    std::string source = argv[1];
    cv::VideoCapture cap(source);
    if (!cap.isOpened())
        return -1;


    Mat bgr;
    vector<Vec3f> circles;
    CascadeUtil cascadeUtil;

    int frameCount = 0;

    while (frameCount < INT_MAX) {
        // Capture frame
        if (!cap.read(bgr))
            break;

        bgr = getTrafficSignROI(bgr);

        std::vector<cv::Vec3f> circles = getBlueCircles(bgr);

        printf("detected circles size %d\n", circles.size());

        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
            circle(bgr, center, radius, yellow, 2, 8, 0);

            Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);

            if (isWithMat(circleBox, bgr)) {

                Mat circleROI(bgr, circleBox);

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
                    rectangle(bgr, cascadeUtil.parking[j], yellow, 2, 1);
                    putText(bgr, "parking", Point(50, 90), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
                }

                for (unsigned k = 0; k < cascadeUtil.left_.size(); k++){
                    rectangle(bgr, cascadeUtil.left_[k], green, 2, 1);
                    putText(bgr, "left", Point(50, 110), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
                }


                for (unsigned n = 0; n < cascadeUtil.right_.size(); n++){
                    rectangle(bgr, cascadeUtil.right_[n], purple, 2, 1);
                    putText(bgr, "right", Point(50, 150), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
                }

            } else {
                printf("outside\n");
            }
        }

        imshow("original", bgr);
        frameCount += 1;
        waitKey(25);
    }

    cap.release();
    destroyAllWindows();

    return 0;
}