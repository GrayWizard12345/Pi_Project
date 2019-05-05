//
// Created by madina on 05/05/19.
//

#include "CascadeUtil.h"

//TODO change the path to relative
//TODO change the path for Raspberry Pi

void CascadeUtil::loadLeftTurn() {
    if (!cascade_left.load("/home/madina/cascades/cascade_left.xml"))
        printf("left - not successfully loaded\n");
}


void CascadeUtil::loadRightTurn() {
    if (!cascade_right.load("/home/madina/cascades/cascade_right.xml"))
        printf("right - not successfully loaded\n");
}

void CascadeUtil::loadParking() {
    if (!cascade_parking.load("/media/madina/Files/Projects/opencv/Pi_Project/src/cascades/cascade_parking.xml"))
        printf("parking - not successfully loaded\n");
}

void CascadeUtil::loadPedestrian() {
    if (cascade_pedestrian.load("/media/madina/Files/Projects/opencv/Pi_Project/src/cascades/cascade_pedestrian.xml"))
        printf("pedestrian - not successfully loaded\n");
}

void CascadeUtil::loadAll() {
    loadLeftTurn();
    loadRightTurn();
    loadParking();
    loadPedestrian();
}

CascadeUtil::CascadeUtil() {
    loadAll();
}

void CascadeUtil::setDetectionArea(cv::Mat bgr) {
    bgr.copyTo(detectionArea);
}

std::vector<cv::Rect> CascadeUtil::detectPedestrian() {
    cascade_pedestrian.detectMultiScale(detectionArea, pedestrian, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(120, 120));
    isPedestrianDetected = pedestrian.empty();
    return pedestrian;
}

std::vector<cv::Rect> CascadeUtil::detectParking() {
    cascade_parking.detectMultiScale(detectionArea, parking, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(120, 120));
    isParkingDetected = parking.empty();
    return parking;
}

std::vector<cv::Rect> CascadeUtil::detectLeftTurn() {
    cascade_left.detectMultiScale(detectionArea, left_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(120, 120));
    isLeftTurnDetected = left_.empty();
    return left_;
}

std::vector<cv::Rect> CascadeUtil::detectRightTurn() {
    cascade_right.detectMultiScale(detectionArea, right_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(120, 120));
    isRightTurnDetected = right_.empty();
    return right_;
}

void CascadeUtil::detectAllSigns() {
    detectPedestrian();
    detectParking();
    detectLeftTurn();
    detectRightTurn();
}


void CascadeUtil::detectAllBlueSigns() {
    detectParking();
    detectLeftTurn();
    detectRightTurn();
}
