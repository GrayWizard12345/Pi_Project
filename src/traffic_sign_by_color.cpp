// Modify the program such that:
// 1. It takes raspicam input frames instead of imread
// 2. When red traffic signs are identified, then the motors should act accordingly (for that use the motor control functions such as goForward(), goRight(), stop(), goLeft() etc.)

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    Mat bgr_image = imread(argv[1], IMREAD_COLOR);
    Mat hsv_image;
    cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    int high = 100, low = 19;

    if (argc > 4) {
        high = atoi(argv[2]);
        low = atoi(argv[3]);
    }

    Mat blue_hue_range;
    inRange(hsv_image, cv::Scalar(80, 65, 65, 0), cv::Scalar(140, 255, 255, 0), blue_hue_range);

    imshow("Blue", blue_hue_range);

    vector<Vec3f> circles;
    HoughCircles(blue_hue_range, circles, HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, 30, 100);
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(bgr_image, center, radius, Scalar(0, 255, 0), 2, 8, 0);

        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);
        // check the box within the image plane
        if (0 <= circleBox.x
            && 0 <= circleBox.width
            && circleBox.x + circleBox.width <= bgr_image.cols
            && 0 <= circleBox.y
            && 0 <= circleBox.height
            && circleBox.y + circleBox.height <= bgr_image.rows) {
            //the area is within the image

            // obtain the image ROI:
            Mat circleROI(blue_hue_range, circleBox);

            int interval = 10;
            Rect roiBox(0, circleROI.rows / 2 + interval, circleROI.cols, interval * 2);

            printf("circleROI %d %d\n", circleROI.cols, circleROI.rows);
            printf("boxROI %d %d %d %d\n", roiBox.x, roiBox.y, roiBox.width, roiBox.height);

            Mat roi(circleROI, roiBox);

            int width = roi.cols / 2;
            int height = roi.rows;

            Rect leftRec(0, 0, width, height);
            Rect rightRec(width, 0, width, height);

            Mat leftFrame(roi, leftRec);
            Mat rightFrame(roi, rightRec);

            namedWindow("left mask " + to_string(i), WINDOW_AUTOSIZE);
            imshow("left mask " + to_string(i), leftFrame);
            namedWindow("right mask " + to_string(i), WINDOW_AUTOSIZE);
            imshow("right mask " + to_string(i), rightFrame);
            namedWindow("mask roi " + to_string(i), WINDOW_FULLSCREEN);
            imshow("mask roi" + to_string(i), roi);

            namedWindow("mask " + to_string(i), WINDOW_FULLSCREEN);
            imshow("mask " + to_string(i), circleROI);

            int leftBlackNum = countNonZero(leftFrame);
            int rightBlackNum = countNonZero(rightFrame);

            //TODO maybe clash with circle traffic sign
            if (abs(leftBlackNum - rightBlackNum) > 10) {
                //it is left/right turn traffic sign
                if (leftBlackNum < rightBlackNum){
                    printf("%d mask - turn left", i);
                }else{
                    printf("%d mask - turn right", i);
                }
            }

            printf("%d mask \t %d - %d\n", i, leftBlackNum, rightBlackNum);
        }
    }

    imshow("Original", bgr_image);

    waitKey(0);
    return 0;
}