/** MIT License
Copyright (c) 2017 Miguel Maestre Trueba
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *@copyright Copyright 2017 Miguel Maestre Trueba
 *@file LaneDetector.hpp
 *@author Miguel Maestre Trueba
 *@brief Header file for the LaneDetector class. Functions are developed in LaneDetector.cpp
 */

#pragma once

#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include "opencv2/opencv.hpp"

#define ZEBRA_CROSSING_SIGNAL SIGRTMIN + 6

enum Turn {
    INVALID = -1,
    LEFT,
    STRAIGHT,
    RIGHT
};


/**
 *@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
 *@brief Activity diagram and UML Class diagram.
 *@brief It detects the lanes in an image if a highway and outputs the
 *@brief same image with the plotted lane.
 */
class LaneDetector {
private:
    double img_size;
    double img_center;
    bool left_flag = false;  // Tells us if there's left boundary of lane detected
    bool right_flag = false;  // Tells us if there's right boundary of lane detected
    cv::Point right_b;  // Members of both line equations of the lane boundaries:
    cv::Point left_b;  //
public:
    double right_m;  // y = m*x + b
    double left_m;  //

    cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
    cv::Mat edgeDetector(cv::Mat img_noise);  // Filter the image to obtain only edges
    cv::Mat mask(cv::Mat img_edges);  // Mask the edges image to only care about ROI
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);  // Detect Hough lines in masked edges image
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines,
                                                        cv::Mat img_edges);  // Sprt detected lines by their slope into right and left lines
    std::vector<std::vector<cv::Vec4i>> left_frame_lineSeparation(std::vector<cv::Vec4i> lines,
                                                                  cv::Mat img_edges);

    std::vector<std::vector<cv::Vec4i>> right_frame_lineSeparation(std::vector<cv::Vec4i> lines,
                                                                   cv::Mat img_edges);

    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines,
                                      cv::Mat inputImage);  // Get only one line for each side of the lane
    int left_frame_predictTurn(int &output,
                               cv::Mat source);  // Determine if the lane is turning or not by calculating the position of the vanishing point
    int right_frame_predictTurn(int &output, cv::Mat source);

    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane,
                               std::string turn, std::string frameName);  // Plot the resultant lane and turn prediction in the frame.

    cv::Mat mask_left_bottom(cv::Mat img_edges);

    cv::Mat mask_right_bottom(cv::Mat img_edges);

    cv::Mat mask_center_bottom(cv::Mat img_edges);

    void predictTurn(int &output);

    unsigned long look_for_cross_walk(cv::Mat &src);
};
