#include "../include/traffic_light.hpp"
#include "../include/traffic_sign.h"
#include "../include/CascadeUtil.h"

using namespace cv;
using namespace std;

Mat roi;
pthread_t signThread;

//TODO verify the area for false positive
void *sign_detect(void *) {

    delay(1000);
    Sign lastDetectedSign = NO_SIGN;
    CascadeUtil cascadeUtil;
    cascadeUtil.loadAll();

    while (true) {
        Mat sign_detection_frame = roi;

        //region Color detection + cascade
//    std::vector<cv::Vec3f> circles = getEdges(sign_detection_frame);
//
//    printf("detected circles size %d\n", circles.size());
//
//    for (size_t i = 0; i < circles.size(); i++) {
//        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);
//
//        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
//        circle(sign_detection_frame, center, radius, yellow, 2, 8, 0);
//
//        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);
//
//        if (isWithMat(circleBox, sign_detection_frame)) {
//
//            Mat circleROI(sign_detection_frame, circleBox);
//
//            cascadeUtil.setDetectionArea(circleROI);
//            cascadeUtil.detectAllCircleBlueSigns();
//
//            for (unsigned k = 0; k < cascadeUtil.left_.size(); k++) {
//                rectangle(sign_detection_frame, cascadeUtil.left_[k], green, 2, 1);
//                putText(sign_detection_frame, "left", Point(50, 110), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//
//
//            for (unsigned n = 0; n < cascadeUtil.right_.size(); n++) {
//                rectangle(sign_detection_frame, cascadeUtil.right_[n], purple, 2, 1);
//                putText(sign_detection_frame, "right", Point(50, 150), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//        } else {
//            printf("outside\n");
//        }
//    }
        //endregion

        cascadeUtil.setDetectionArea(sign_detection_frame);

        cascadeUtil.detectRightTurn();
        cascadeUtil.detectLeftTurn();
        cascadeUtil.detectParking();
        cascadeUtil.detectParking();

        if (cascadeUtil.isStopDetected && lastDetectedSign == STOP_SIGN) {
            signDetected = NO_SIGN;
            continue;
        } else if (cascadeUtil.isStopDetected)
            signDetected = STOP_SIGN;
        else if (cascadeUtil.isRightTurnDetected)
            signDetected = RIGHT_TURN_SIGN;
        else if (cascadeUtil.isLeftTurnDetected)
            signDetected = LEFT_TURN_SIGN;
        else if (cascadeUtil.isParkingDetected)
            signDetected = PARKING_SIGN;
        else if (cascadeUtil.isPedestrianDetected)
            signDetected = PEDESTRIAN_SIGN;
        else
            signDetected = NO_SIGN;


        for (auto r: cascadeUtil.stop) {
            rectangle(sign_detection_frame, r, yellow);
        }

        for (auto r : cascadeUtil.left_) {
            rectangle(sign_detection_frame, r, green);
        }

        for (auto r : cascadeUtil.right_) {
            rectangle(sign_detection_frame, r, purple);
        }

        for (auto r : cascadeUtil.pedestrian) {
            rectangle(sign_detection_frame, r, orange);
        }

        for (auto r : cascadeUtil.parking) {
            rectangle(sign_detection_frame, r, blue);
        }

        lastDetectedSign = signDetected;
    }
}

void *trafficLightLoop(void *) {
    //delay(1000);
    trafficLightStatus = Status::RED_LIGHT;
    printf("\nTraffic light thread\n");
    printf("\nTraffic light thread - width: %d\n", frame.size().width);
    Rect rec(0, 0, width, height / 2);
    int circleCount = 0;

    if (pthread_create(&signThread, nullptr, sign_detect, nullptr)) {
        fprintf(stderr, "Error creating sign thread\n");
        return nullptr;
    }

    while (1) {
        roi = frame(rec);

        while (signDetected == STOP_SIGN);

        Mat hsv;
        cvtColor(roi, hsv, COLOR_BGR2HSV);

        //red color detection
        Mat redLowerMask, redHigherMask;
        inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), redLowerMask);
        inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), redHigherMask);

        //use hue values from the both ranges (masks)
        addWeighted(redLowerMask, 1.0, redHigherMask, 1.0, 0.0, red_hue_image);
        erode(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
        dilate(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
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
            pthread_mutex_lock(&motor_mutex);
            pwmStop();
            delay(20);
            pthread_mutex_unlock(&motor_mutex);
        } else {
            trafficLightStatus = GREEN_LIGHT;
            printf("\nNo Red light detected => Green Light is ON");
        }

    }
}

void killSignThread(){
    pthread_kill(signThread, 0);
}

int initTrafficLightThread() {
    if (pthread_create(&trafficLightThread, NULL, trafficLightLoop, NULL)) {
        fprintf(stderr, "Error creating traffic light thread\n");
        return 1;
    }
}
