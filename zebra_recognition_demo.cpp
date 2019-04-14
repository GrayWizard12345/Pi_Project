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

    Laplacian(bgr, bgr, CV_64F);

    namedWindow( "Laplacian", WINDOW_AUTOSIZE );
    imshow("Laplacian", bgr);

    Mat sobelX, sobelY;
    Sobel(bgr, sobelX, CV_64F, 1, 0, 5);
    imshow("SobelX", sobelX);
    Sobel(bgr, sobelY, CV_64F, 0, 1, 5);
    imshow("SobelY", sobelY);


    convertScaleAbs(sobelX, sobelX);
    sobelX.convertTo(sobelX, CV_8U);

    waitKey(50);


    while (1);
}
