#pragma once
#include "traffic_light.hpp"

using namespace cv;
using namespace std;

pthread_t trafficLightThread;

void *trafficLightLoop(void*) {
    //delay(1000);
    trafficLightStatus = Status::RED_LIGHT;
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
        erode(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size( 9, 9)));
        dilate(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size( 3, 3)));
        GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        vector<Vec3f> circles;
        //detect red circles
        HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 110, 20, 0, 0);

        if ((circleCount = circles.size()) >= 1) {
            for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
                int radius = round(circles[current_circle][2]);

                //draw circle on the background
                circle(red_hue_image, center, radius, Scalar(0, 255, 0), 5);
            }
            printf("\nRed light detected - circles: %d \n", circleCount);
            trafficLightStatus = RED_LIGHT;
        }
        red_color_frame = red_hue_image;

        Mat hsv_green;
        cvtColor(rightTopBgr, hsv_green, COLOR_BGR2HSV);
        Mat green_mask;
        inRange(hsv_green, Scalar(45, 100, 100), Scalar(80, 255, 255), green_mask);

        //erode(greenHigherMask, green_hue_image, getStructuringElement(MORPH_ELLIPSE, Size( 9, 9)));
        green_color_frame = green_mask;
        threshold(green_mask, green_mask, 128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        GaussianBlur(green_mask, green_mask, Size(9, 9), 2, 2);
//        erode(green_mask, green_mask, getStructuringElement(MORPH_ELLIPSE, Size( 9, 9)));
        dilate(green_mask, green_mask, getStructuringElement(MORPH_ELLIPSE, Size( 15, 15)));
        HoughCircles(green_mask, circles, CV_HOUGH_GRADIENT, 1, green_mask.rows / 8, 110, 10, 0, 50);



        if ((circleCount = circles.size()) > 1) {
            for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
                Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
                int radius = round(circles[current_circle][2]);

                //draw circle on the background
                circle(green_mask, center, radius, Scalar(0, 0, 255), 5);
            }
            printf("\nGreen light detected - circles: %d \n", circleCount);
            trafficLightStatus = GREEN_LIGHT;
        }



    }
}

int initTrafficLightThread() {
    if (pthread_create(&trafficLightThread, NULL, trafficLightLoop, NULL)) {
        fprintf(stderr, "Error creating traffic light thread\n");
        return 1;
    }
}
