//
// Created by madina on 30/04/19.
//

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cout << "Not enough parameters" << std::endl;
        return -1;
    }

    std::string source = argv[1];
    cv::VideoCapture cap(source);
    if (!cap.isOpened())
        return -1;

    // Initialise the cascade classifier containers and load them with the .xml file you obtained from the training
    CascadeClassifier cascade_left, cascade_parking, cascade_right, cascade_pedestrian;

    if (!cascade_pedestrian.load("./cascades_custom/cascade_pedestrian.xml"))
        printf("be careful - not successfully loaded\n");

    if (!cascade_parking.load("./cascades_custom/cascade_parking.xml"))
        printf("circle - not successfully loaded\n");

    if (!cascade_left.load("./cascades_custom/cascade_left.xml"))
        printf("left - not successfully loaded\n");

    if (!cascade_right.load("./cascades/cascade_right.xml"))
        printf("right - not successfully loaded\n");

    // Container of signs
    vector<Rect> pedestrian;
    vector<Rect> parking;
    vector<Rect> left_;
    vector<Rect> right_;

    Scalar orange = Scalar(255, 178, 102);
    Scalar yellow = Scalar(255, 255, 51);
    Scalar green = Scalar(153, 255, 51);
    Scalar blue = Scalar(51, 255, 255);
    Scalar violet = Scalar(127, 0, 255);
    Scalar purple = Scalar(255, 51, 255);
    Scalar pink = Scalar(255, 51, 153);

    Mat bgr;
    int frameCount = 0;

    cap.read(bgr);
    VideoWriter video("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(bgr.cols, bgr.rows));

    while (frameCount < 540) {
        // Capture frame
        if (!cap.read(bgr))
            break;

        // These are detection function where you invoke the classifiers on to the frame to detect the trained elements
        cascade_pedestrian.detectMultiScale(bgr, pedestrian, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(100, 100));
        cascade_parking.detectMultiScale(bgr, parking, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(100, 100));
        cascade_left.detectMultiScale(bgr, left_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(100, 100));
        cascade_right.detectMultiScale(bgr, right_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(100, 100));

        // To draw rectangles around detected objects accordingly
        for (unsigned i = 0; i < pedestrian.size(); i++) {
            rectangle(bgr, pedestrian[i], orange, 2, 1);
            putText(bgr, "be careful", Point(50, 50), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
        }

        for (unsigned j = 0; j < parking.size(); j++) {
            rectangle(bgr, parking[j], yellow, 2, 1);
            putText(bgr, "circular", Point(50, 50), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
        }

        for (unsigned k = 0; k < left_.size(); k++)
            rectangle(bgr, left_[k], green, 2, 1);


        for (unsigned n = 0; n < right_.size(); n++)
            rectangle(bgr, right_[n], purple, 2, 1);


        imshow("original", bgr);
        video.write(bgr);


        frameCount += 1;
        waitKey(25);

    }

    video.release();
    cap.release();

    destroyAllWindows();

    return 0;
}