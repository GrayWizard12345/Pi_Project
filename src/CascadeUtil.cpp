//
// Created by madina on 05/05/19.
//

#include "CascadeUtil.h"

using namespace cv;

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
    if (!cascade_pedestrian.load("/media/madina/Files/Projects/opencv/Pi_Project/src/cascades/cascade_pedestrian.xml"))
        printf("pedestrian - not successfully loaded\n");
}

void CascadeUtil::loadStop() {
    if (!cascade_stop.load("/media/madina/Files/Projects/opencv/Pi_Project/src/cascades/cascade_stop.xml"))
        printf("stop - not successfully loaded\n");
}


void CascadeUtil::loadAll() {
    loadLeftTurn();
    loadRightTurn();
    loadParking();
    loadPedestrian();
    loadStop();
}

CascadeUtil::CascadeUtil() {
}

void CascadeUtil::setDetectionArea(cv::Mat bgr) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, CV_BGR2GRAY);
    equalizeHist(gray, gray);
    gray.copyTo(detectionArea);
}

std::vector<cv::Rect> CascadeUtil::detectPedestrian(int width, int height) {
    cascade_pedestrian.detectMultiScale(detectionArea, pedestrian, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(width, height));
    isPedestrianDetected = pedestrian.empty();
    return pedestrian;
}

std::vector<cv::Rect> CascadeUtil::detectStop(int width, int height) {
    cascade_stop.detectMultiScale(detectionArea, stop, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(width, height));
    isStopDetected = pedestrian.empty();
    return stop;
}

std::vector<cv::Rect> CascadeUtil::detectParking(int width, int height) {
    cascade_parking.detectMultiScale(detectionArea, parking, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(width, height));
    isParkingDetected = parking.empty();
    return parking;
}

std::vector<cv::Rect> CascadeUtil::detectLeftTurn(int width, int height) {
    cascade_left.detectMultiScale(detectionArea, left_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(width, height));
    isLeftTurnDetected = left_.empty();
    return left_;
}

std::vector<cv::Rect> CascadeUtil::detectRightTurn(int width, int height) {
    cascade_right.detectMultiScale(detectionArea, right_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(width, height));
    isRightTurnDetected = right_.empty();
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
