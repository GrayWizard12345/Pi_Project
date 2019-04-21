#pragma once
#include "traffic_light.hpp"

using namespace cv;
using namespace std;

#define RED_TRAFFIC_LIGHT_SIGNAL (SIGRTMIN + 6)
#define GREEN_TRAFFIC_LIGHT_SIGNAL (SIGRTMIN + 7)

pthread_t trafficLightThread;

void *trafficLightLoop(void*) {
    //delay(1000);
    trafficLightStatus = Status::NON_TRAFFIC_LIGHT;
    printf("\nTraffic light thread\n");
    delay(1000);
    printf("\nTraffic light thread - width: %d\n", frame.size().width);
    Rect rec((1280 * 3) / 4, 0, 1280 / 4, 960 / 2);
    int circleCount = 0;
    while (1) {

        Mat rightTopBgr = frame(rec);
        Mat hsv;
        cvtColor(rightTopBgr, hsv, COLOR_BGR2HSV);

        //red color detection
        Mat redLowerMask, redHigherMask;
        inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), redLowerMask);
        inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), redHigherMask);

        Mat red_hue_image;
        //use hue values from the both ranges (masks)
        addWeighted(redLowerMask, 1.0, redHigherMask, 1.0, 0.0, red_hue_image);
        GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        vector<Vec3f> circles;
        //detect red circles
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 110, 22, 0, 0);

        printf("circle count %d \n", circleCount);

        if ((circleCount = circles.size()) >= 1) {
            for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
                int radius = round(circles[current_circle][2]);

                //draw circle on the background
                circle(traffic_light, center, radius, Scalar(0, 255, 0), 5);
            }
            traffic_light = red_hue_image;
            raise(RED_TRAFFIC_LIGHT_SIGNAL);
        }

        Mat greenMask;
        inRange(hsv, Scalar(65, 60, 60), Scalar(80, 255, 255), greenMask);

        Mat green_hue_image;
        GaussianBlur(greenMask, green_hue_image, Size(9, 9), 2, 2);
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, green_hue_image.rows / 8, 110, 22, 0, 0);


        if ((circleCount = circles.size()) > 1) {
            for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
                int radius = round(circles[current_circle][2]);

                //draw circle on the background
                circle(traffic_light, center, radius, Scalar(0, 0, 255), 5);
            }
            traffic_light = green_hue_image;
            raise(GREEN_TRAFFIC_LIGHT_SIGNAL);
        }
    }
}

int initTrafficLightThread() {
    if (pthread_create(&trafficLightThread, NULL, trafficLightLoop, NULL)) {
        fprintf(stderr, "Error creating traffic light thread\n");
        return 1;
    }
}

void trafficLightHandler(int signum) {
    if (signum == RED_TRAFFIC_LIGHT_SIGNAL)
        trafficLightStatus = Status::RED_LIGHT;
    else if (signum == GREEN_TRAFFIC_LIGHT_SIGNAL) {
        trafficLightStatus = Status::GREEN_LIGHT;
    }
}

void initTrafficLightSignal() {
    signal(RED_TRAFFIC_LIGHT_SIGNAL, trafficLightHandler);
    signal(GREEN_TRAFFIC_LIGHT_SIGNAL, trafficLightHandler);
}