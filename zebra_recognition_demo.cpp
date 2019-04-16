//
// Created by madina on 14/04/19.
//

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
//#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include <csignal>
//#include "motion.hpp"

using namespace cv;
using namespace std;

#define ZEBRA_SIGNAL SIGRTMIN + 8

pthread_t zebraThread;
//raspicam::RaspiCam_Cv zebraCapture;
pthread_mutex_t zebraDetectionMutex;

int main() {

    Mat bgr = imread("zebra_crossing_line.jpg", CV_LOAD_IMAGE_COLOR);
    Mat src;

    int lowThreshold = 45;
    int ratio = 3;
    int kernel_size = 3;

    cv::blur(bgr, src, cv::Size(3, 3));

    // Filter the binary image to obtain the edges
    cv::Canny(src, bgr, lowThreshold, lowThreshold * ratio, kernel_size);
    imshow("Laplacian", bgr);
    waitKey(5000);
    Laplacian(bgr, src, CV_64F);

    //namedWindow( "Laplacian", WINDOW_AUTOSIZE );
    imshow("Laplacian", src);
    waitKey(5000);
    Mat sobelX, sobelY;
    Sobel(src, sobelX, CV_64F, 1, 0, 5);
    imshow("SobelX", sobelX);
    waitKey(5000);
    Sobel(src, sobelY, CV_64F, 0, 1, 5);
    imshow("SobelY", sobelY);
    waitKey(5000);

    convertScaleAbs(sobelX, sobelX);
    sobelX.convertTo(sobelX, CV_8U);
    imshow("SobelY", sobelY);
    waitKey(5000);



    while (1)
        waitKey(50);
}
