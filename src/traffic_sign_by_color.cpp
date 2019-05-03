#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    Mat bgr = imread(argv[1], IMREAD_COLOR);
    Mat hsv_image;
    cvtColor(bgr, hsv_image, cv::COLOR_BGR2HSV);

    printf("Window size %d : %d", bgr.cols, bgr.rows);

    int high = 100, low = 15;

    if (argc >= 4) {
        high = atoi(argv[2]);
        low = atoi(argv[3]);

        printf("New values high: %d, low %d\n", high, low);
    }

    int minRadius = 20, maxRadius = 60;
    if (argc >= 6) {
        minRadius = atoi(argv[4]);
        maxRadius = atoi(argv[5]);
    }

    Scalar orange = Scalar(255, 178, 102);
    Scalar yellow = Scalar(255, 255, 51);
    Scalar green = Scalar(153, 255, 51);
    Scalar blue = Scalar(51, 255, 255);
    Scalar violet = Scalar(127, 0, 255);
    Scalar purple = Scalar(255, 51, 255);
    Scalar pink = Scalar(255, 51, 153);

    Mat blue_hue_range;
    inRange(hsv_image, cv::Scalar(50, 0, 0), cv::Scalar(140, 255, 255), blue_hue_range);
    dilate(blue_hue_range, blue_hue_range, getStructuringElement(MORPH_ELLIPSE, Size(9,9)));
    namedWindow("Blue", WINDOW_AUTOSIZE);
    imshow("Blue", blue_hue_range);

    vector<Vec3f> circles;
    HoughCircles(blue_hue_range, circles, HOUGH_GRADIENT, 1, blue_hue_range.rows / 8, high, low, minRadius, maxRadius);

    vector<Vec4i> lines;

    printf("detected circles size %d\n", circles.size());

    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]) - 5;

        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
//        circle(bgr, center, radius, yellow, 2, 8, 0);

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
            Mat black = Mat::zeros(circleBox.size(), CV_8UC3);
            Mat circleROI(bgr, circleBox);
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
        } else {
            printf("outside\n");
        }
    }

    namedWindow("Original", WINDOW_AUTOSIZE);
    imshow("Original", bgr);


    waitKey(0);
    return 0;
}