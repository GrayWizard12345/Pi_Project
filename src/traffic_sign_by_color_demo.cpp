#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar orange = Scalar(255, 178, 102);
Scalar yellow = Scalar(255, 255, 51);
Scalar green = Scalar(153, 255, 51);
Scalar blue = Scalar(51, 255, 255);
Scalar violet = Scalar(127, 0, 255);
Scalar purple = Scalar(255, 51, 255);
Scalar pink = Scalar(255, 51, 153);

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Not enough parameters" << std::endl;
        return -1;
    }

    std::string source = argv[1];
    cv::VideoCapture cap(source);
    if (!cap.isOpened())
        return -1;

    int high = 100, low = 15;

    if (argc >= 4) {
        high = atoi(argv[2]);
        low = atoi(argv[3]);
    }

    int minRadius = 20, maxRadius = 60;
    if (argc >= 6) {
        minRadius = atoi(argv[4]);
        maxRadius = atoi(argv[5]);
    }

    Mat bgr;
    Mat hsv_image;
    Mat blue_hue_range;
    vector<Vec3f> circles;

    int frameCount = 0;

    while (frameCount < INT_MAX) {
        // Capture frame
        if (!cap.read(bgr))
            break;

        cvtColor(bgr, hsv_image, cv::COLOR_BGR2HSV);

        inRange(hsv_image, cv::Scalar(80, 65, 65, 0), cv::Scalar(140, 255, 255, 0), blue_hue_range);

        HoughCircles(blue_hue_range, circles, HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, minRadius,
                     maxRadius);

        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
            circle(bgr, center, radius, yellow, 2, 8, 0);

            Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);

            // check the box within the image plane
            if (0 <= circleBox.x
                && 0 <= circleBox.width
                && circleBox.x + circleBox.width <= bgr.cols
                && 0 <= circleBox.y
                && 0 <= circleBox.height
                && circleBox.y + circleBox.height <= bgr.rows) {
                //the area is within the image

                // obtain the image ROI:
                Mat circleROI(blue_hue_range, circleBox);

                int interval = 10;

                if (interval * 2 > circleROI.rows)
                    interval = circleROI.rows;


                Rect roiBox(0, circleROI.rows / 2 - interval, circleROI.cols, interval * 2);
                printf("circleROI - %d %d\n", circleROI.cols, circleROI.rows);

                printf("roiBox %d %d %d %d\n", roiBox.x, roiBox.y, roiBox.width, roiBox.height);

                if (!(0 <= roiBox.x
                      && 0 <= roiBox.width
                      && roiBox.x + roiBox.width <= circleROI.cols
                      && 0 <= roiBox.y
                      && 0 <= roiBox.height
                      && roiBox.y + roiBox.height <= circleROI.rows)) {
                    printf("out of roi range\n");
                    continue;
                }


                Mat roi(circleROI, roiBox);

                printf("got ROI\n");

                int width = roi.cols / 2;
                int height = roi.rows;

                Rect leftRec(0, 0, width, height);
                Rect rightRec(width, 0, width, height);

                Mat leftFrame(roi, leftRec);
                Mat rightFrame(roi, rightRec);

                int leftBlackNum = countNonZero(leftFrame);
                int rightBlackNum = countNonZero(rightFrame);

                //TODO maybe clash with circle traffic sign
                if (abs(leftBlackNum - rightBlackNum) > 10) {
                    //it is left/right turn traffic sign
                    if (leftBlackNum > rightBlackNum) {
                        circle(bgr, center, radius, yellow, 2, 8, 0);
                        putText(bgr, "left", Point(50, 50), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1,
                                CV_AA);
                    } else {
                        circle(bgr, center, radius, orange, 2, 8, 0);
                        putText(bgr, "right", Point(50, 50), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1,
                                CV_AA);
                    }
                }

                printf("%d mask \t %d - %d\n", i, leftBlackNum, rightBlackNum);
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