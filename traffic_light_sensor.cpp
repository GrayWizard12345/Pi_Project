#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include <csignal>
#include "motion.hpp"

using namespace cv;
using namespace std;

#define RED_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 6
#define GREEN_TRAFFIC_LIGHT_SIGNAL SIGRTMIN + 7

pthread_t trafficLightThread;
raspicam::RaspiCam_Cv trafficLightCapture;
int circleCount = 0;
pthread_mutex_t trafficLightMutex;

void *trafficLightLoop(void *) {
    printf("thread\n");
    Mat bgr;
    trafficLightCapture.open();
    Rect rec((1280 * 3) / 4, 0, 1280 / 4, 960 / 2);
    while (1) {
        trafficLightCapture.grab();
        trafficLightCapture.retrieve(bgr);

        Mat res = bgr(rec);

        Mat hsv;
        cvtColor(res, hsv, COLOR_BGR2HSV);

        //red color detection
        Mat redLowerMask, redHigherMask;
        inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), redLowerMask);
        inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), redHigherMask);

        Mat red_hue_image;
        addWeighted(redLowerMask, 1.0, redHigherMask, 1.0, 0.0, red_hue_image);
        GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        vector<Vec3f> circles;
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 110, 35, 0, 0);

        printf("circle count %d \n", circleCount);

        if ((circleCount = circles.size()) >= 1) {
            printf("red traffic light\n");
            raise(RED_TRAFFIC_LIGHT_SIGNAL);
        }

        for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
            cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
            int radius = std::round(circles[current_circle][2]);
            cv::circle(res, center, radius, cv::Scalar(0, 255, 0), 5);
        }

        imshow("Bck", bgr);
        imshow("Red", res);

        Mat greenMask;
        inRange(hsv, Scalar(65, 60, 60), Scalar(80, 255, 255), greenMask);

        Mat green_hue_image;
        GaussianBlur(greenMask, green_hue_image, Size(9, 9), 2, 2);
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, green_hue_image.rows / 8, 110, 35, 0, 0);


        if ((circleCount = circles.size()) > 1){
            printf("green traffic light");
            raise(GREEN_TRAFFIC_LIGHT_SIGNAL);
        }

        if (cvWaitKey(20) == 'q')
            break;
    }

    trafficLightCapture.release();
}

int initTrafficLightThread() {
    if (pthread_create(&trafficLightThread, NULL, trafficLightLoop, NULL)) {
        fprintf(stderr, "Error creating traffic light thread\n");
        return 1;
    }
}

void trafficLightHandler(int signum) {
    printf("red traffic light interrupt called\n");
    pthread_mutex_lock(&motor_mutex);
    pwmStop();
    delay(10000);
    //TODO change the default delay
    //while (circleCount != 0);
    pthread_mutex_unlock(&motor_mutex);
}

void initTrafficLightSignal() {
    signal(RED_TRAFFIC_LIGHT_SIGNAL, trafficLightHandler);
}

void signalHandler(int signum) {

    // cleanup and close up stuff here
    // terminate program
    printf("CNTR+C");

    pwmStop();
    stopDCMotor();
    trafficLightCapture.release();

    destroyAllWindows();
    exit(signum);
}

int main() {
    initTrafficLightThread();
    initTrafficLightSignal();
    signal(SIGINT, signalHandler);

    if (wiringPiSetup() == -1)
        return 0;
    pwmInitDCMotor();

    while (1) {
        pthread_mutex_lock(&motor_mutex);
        pwmGo(40);
        delay(10);
        pthread_mutex_unlock(&motor_mutex);
    }

    return 0;
}