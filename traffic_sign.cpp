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

        Rect box(center.x - radius, center.y - radius, radius * 2, radius * 2);
        // check the box within the image plane
        if (0 <= box.x
            && 0 <= box.width
            && box.x + box.width <= bgr_image.cols
            && 0 <= box.y
            && 0 <= box.height
            && box.y + box.height <= bgr_image.rows) {
            //the area is within the image

            // obtain the image ROI:
            Mat roi(blue_hue_range, box);

            int width = roi.cols / 2;
            int height = roi.rows;

            Rect leftRec(0, 0, width, height);
            Rect rightRec(width, 0, width, height);

            Mat left(roi, leftRec);
            Mat right(roi, rightRec);

            namedWindow("mask " + to_string(i), WINDOW_AUTOSIZE);
            imshow("mask " + to_string(i), roi);
            namedWindow("left mask " + to_string(i), WINDOW_AUTOSIZE);
            imshow("left mask " + to_string(i), left);
            namedWindow("right mask " + to_string(i), WINDOW_AUTOSIZE);
            imshow("right mask " + to_string(i), right);


            //TODO use countNonZero()
        }
    }

    imshow("Original", bgr_image);

    waitKey(0);
    return 0;
}