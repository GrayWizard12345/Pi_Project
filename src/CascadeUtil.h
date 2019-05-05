//
// Created by madina on 05/05/19.
//

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/opencv.hpp"

class CascadeUtil {

    cv::CascadeClassifier cascade_left, cascade_parking, cascade_right, cascade_pedestrian;

    cv::Mat detectionArea;

public:

    std::vector<cv::Rect> pedestrian;
    std::vector<cv::Rect> parking;
    std::vector<cv::Rect> left_;
    std::vector<cv::Rect> right_;

    bool isPedestrianDetected = false;
    bool isParkingDetected = false;
    bool isLeftTurnDetected = false;
    bool isRightTurnDetected = false;

    /**
     * @brief loads all cascades needed
     */
    CascadeUtil();


    /**
     * @brief set the area for further detection, until the area is changed, all detections are made on it
     * @param bgr
     */
    void setDetectionArea(cv::Mat bgr);

    void loadPedestrian();

    void loadParking();

    void loadLeftTurn();

    void loadRightTurn();

    void loadAll();

    std::vector<cv::Rect> detectPedestrian();

    std::vector<cv::Rect> detectParking();

    std::vector<cv::Rect> detectLeftTurn();

    std::vector<cv::Rect> detectRightTurn();

    /**
     * @brief detects all signs in bgr, and sets according vector and boolean values
     */
    void detectAllSigns();

    void detectAllCircleBlueSigns();
};