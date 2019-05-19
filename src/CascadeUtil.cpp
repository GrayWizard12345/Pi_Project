//
// Created by madina on 05/05/19.
//

#include "../include/CascadeUtil.h"

using namespace cv;

//TODO change the path to relative
//TODO change the path for Raspberry Pi

void CascadeUtil::loadLeftTurn() {
    if (!cascade_left.load("cascades/cascade_left.xml"))
        printf("left - not successfully loaded\n");
    else
        printf("left - success\n");
}


void CascadeUtil::loadRightTurn() {
    if (!cascade_right.load("cascades/cascade_right.xml"))
        printf("right - not successfully loaded\n");
    else
        printf("right - success\n");
}

void CascadeUtil::loadParking() {
    if (!cascade_parking.load("cascades/cascade_parking.xml"))
        printf("parking - not successfully loaded\n");
    else
        printf("parking - success\n");
}

void CascadeUtil::loadPedestrian() {
    if (!cascade_pedestrian.load("cascades/cascade_pedestrian.xml"))
        printf("pedestrian - not successfully loaded\n");
    else
        printf("pedestrian - success\n");
}

void CascadeUtil::loadStop() {
    if (!cascade_stop.load("cascades/stop.xml"))
        printf("stop - not successfully loaded\n");
    else
        printf("stop - success\n");
}

void CascadeUtil::loadAll() {
    loadLeftTurn();
    loadRightTurn();
    loadParking();
    loadPedestrian();
    loadStop();
}

CascadeUtil::CascadeUtil() {}

void CascadeUtil::setDetectionArea(cv::Mat bgr, bool isPreProcessingNeeded) {
    cv::Mat gray;
    if (isPreProcessingNeeded) {
        cv::cvtColor(bgr, gray, CV_BGR2GRAY);
        equalizeHist(gray, gray);
        gray.copyTo(detectionArea);
    } else {
        bgr.copyTo(detectionArea);
    }
}

std::vector<cv::Rect> CascadeUtil::detectPedestrian(int minRadius, int maxRadius) {
    cascade_pedestrian.detectMultiScale(detectionArea, pedestrian, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE,
                                        cv::Size(minRadius, minRadius), cv::Size(maxRadius, maxRadius));
    isPedestrianDetected = !pedestrian.empty();
    return pedestrian;
}

std::vector<cv::Rect> CascadeUtil::detectStop(int minRadius, int maxRadius) {
    cascade_stop.detectMultiScale(detectionArea, stop, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE,
                                        cv::Size(minRadius, minRadius), cv::Size(maxRadius, maxRadius));
    isStopDetected = !stop.empty();
    return pedestrian;
}

std::vector<cv::Rect> CascadeUtil::detectParking(int minRadius, int maxRadius) {
    cascade_parking.detectMultiScale(detectionArea, parking, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE,
                                        cv::Size(minRadius, minRadius), cv::Size(maxRadius, maxRadius));
    isParkingDetected = !parking.empty();
    return pedestrian;
}

std::vector<cv::Rect> CascadeUtil::detectLeftTurn(int minRadius, int maxRadius) {
    cascade_left.detectMultiScale(detectionArea, left_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(minRadius, minRadius), cv::Size(maxRadius, maxRadius));
    isLeftTurnDetected = !left_.empty();
    return left_;
}

std::vector<cv::Rect> CascadeUtil::detectRightTurn(int minRadius, int maxRadius) {
    cascade_right.detectMultiScale(detectionArea, right_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(minRadius, minRadius), cv::Size(maxRadius, maxRadius));
    isRightTurnDetected = !right_.empty();
    return right_;
}

void CascadeUtil::detectAllSigns() {
    detectPedestrian();
    detectParking();
    detectLeftTurn();
    detectRightTurn();
    detectStop();
}


void CascadeUtil::detectAllCircleBlueSigns() {
    detectLeftTurn();
    detectRightTurn();
}
