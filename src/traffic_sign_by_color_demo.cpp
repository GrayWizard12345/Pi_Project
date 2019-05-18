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

    initSignDetection();

    int frameCount = 0;

    while (frameCount < INT_MAX) {
        // Capture frame
        if (!cap.read(bgr))
            break;



        imshow("original", bgr);
        frameCount += 1;
        waitKey(25);
    }

    cap.release();
    destroyAllWindows();

    return 0;
}