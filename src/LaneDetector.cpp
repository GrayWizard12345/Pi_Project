/** MIT License
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
 *@file LaneDetector.cpp
 *@author Miguel Maestre Trueba
 *@brief Definition of all the function that form part of the LaneDetector class.
 *@brief The class will take RGB images as inputs and will output the same RGB image but
 *@brief with the plot of the detected lanes and the turn prediction.
 */

#include <wiringPi.h>
#include "../include/LaneDetector.hpp"
#include "../include/config_reader.h"
using namespace cv;
using namespace std;
// IMAGE BLURRING
/**
 *@brief Apply gaussian filter to the input image to denoise it
 *@param inputImage is the frame of a video in which the
 *@param lane is going to be detected
 *@return Blurred and denoised image
 */
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
    cv::Mat output;

    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

    return output;
}

// EDGE DETECTION
/**
 *@brief Detect all the edges in the blurred frame by filtering the image
 *@param img_noise is the previously blurred frame
 *@return Binary image with only the edges represented in white
 */
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {

    cv::Mat output;
    if(stoi(vars["COLOR_DETECTION"])) {
        int H, S, V, H2, S2, V2;
        H = stoi(vars["LANE_DETECTOR_SCALAR_H"]);
        S = stoi(vars["LANE_DETECTOR_SCALAR_S"]);
        V = stoi(vars["LANE_DETECTOR_SCALAR_V"]);

        H2 = stoi(vars["LANE_DETECTOR_SCALAR_H2"]);
        S2 = stoi(vars["LANE_DETECTOR_SCALAR_S2"]);
        V2 = stoi(vars["LANE_DETECTOR_SCALAR_V2"]);
        // Convert image from RGB to HSV
        cv::cvtColor(img_noise, output, cv::COLOR_BGR2HSV);
        cv::inRange(output, cv::Scalar(H, S, V), cv::Scalar(H2, S2, V2), output);

//    cv::dilate(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 3, 3)));
//    cv::erode(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 9, 9)));

    } else
    {
        cv::cvtColor(img_noise, output, cv::COLOR_BGR2GRAY);
//        cv::dilate(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
//        imshow("deliate", output);
    }
//    cv::GaussianBlur(output, output, cv::Size(9, 9), 2, 2);
    blur(output, output, cv::Size(3, 3));
    int lowThreshold = 35;
    int ratio = 3;
    int kernel_size = 3;
    // Filter the binary image to obtain the edges
    cv::Canny(output, output, lowThreshold, lowThreshold * ratio, kernel_size);
    //printf("\nEdge detector: %d -- %d", output.cols, output.rows);
    return output;
}

// MASK THE EDGE IMAGE
/**
 *@brief Mask the image so that only the edges that form part of the lane are detected
 *@param img_edges is the edges image from the previous function
 *@return Binary image with only the desired edges being represented
 */
cv::Mat LaneDetector::mask(cv::Mat img_edges) {
    cv::Mat output;
    int y = stoi(vars["ROI_Y"]);
    cv::Rect rect(0, y, width, height - y);

    output = img_edges(rect);

    return output;
}
// HOUGH LINES
/**
 *@brief Obtain all the line segments in the masked images which are going to be part of the lane boundaries
 *@param img_mask is the masked binary image from the previous function
 *@return Vector that contains all the detected lines in the image
 */
std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
    std::vector<cv::Vec4i> line;

    // rho and theta are selected by trial and error
    HoughLinesP(img_mask, line, 1, CV_PI / 180, 20, 20, 30);

    return line;
}

// SORT RIGHT AND LEFT LINES
/**
 *@brief Sort all the detected Hough lines by slope.
 *@brief The lines are classified into right or left depending
 *@brief on the sign of their slope and their approximate location
 *@param lines is the vector that contains all the detected lines
 *@param img_edges is used for determining the image center
 *@return The output is a vector(2) that contains all the classified lines
 */
std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
    std::vector<std::vector<cv::Vec4i> > output(2);
    size_t j = 0;
    cv::Point ini;
    cv::Point fini;
    double slope_thresh_low = 0.17;     //TODO changed from 0.3
    double slope_thresh_high = 200;
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Vec4i> right_lines, left_lines;

    // Calculate the slope of all the detected lines
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) > slope_thresh_low && std::abs(slope) < slope_thresh_high) {
            slopes.push_back(slope);
            selected_lines.push_back(i);
        }
    }

    // Split the lines into right and left lines
    img_center = static_cast<double>((img_edges.cols / 2));
    while (j < selected_lines.size()) {
        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

        // Condition to classify line as left side or right side
        if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
            right_lines.push_back(selected_lines[j]);
            right_flag = true;
        } else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
            left_lines.push_back(selected_lines[j]);
            left_flag = true;
        }
        j++;
    }

    output[0] = right_lines;
    output[1] = left_lines;

    return output;
}


// REGRESSION FOR LEFT AND RIGHT LINES
/**
 *@brief Regression takes all the classified line segments initial and final points and fits a new lines out of them using the method of least squares.
 *@brief This is done for both sides, left and right.
 *@param left_right_lines is the output of the lineSeparation function
 *@param inputImage is used to select where do the lines will end
 *@return output contains the initial and final points of both lane boundary lines
 */
std::vector<cv::Point>
LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
    std::vector<cv::Point> output(4);
    cv::Point ini;
    cv::Point fini;
    cv::Point ini2;
    cv::Point fini2;
    cv::Vec4d right_line;
    cv::Vec4d left_line;
    std::vector<cv::Point> right_pts;
    std::vector<cv::Point> left_pts;

    // If right lines are being detected, fit a line using all the init and final points of the lines
    Point r_point;
    Point l_point;
    if (right_flag == true) {
        for (auto i : left_right_lines[0]) {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            line(inputImage, ini, fini, Scalar(250,250,250), 15);
            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) {
            // The right line is formed here
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];
            right_b = cv::Point(right_line[2], right_line[3]);
            r_point.x = right_b.x - 200 * right_line[0];
            r_point.y = right_b.y - 200 * right_line[1];
        }
    }

    // If left lines are being detected, fit a line using all the init and final points of the lines
    if (left_flag == true) {
        for (auto j : left_right_lines[1]) {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);
            line(inputImage, ini2, fini2, Scalar(250,250,250), 15);
            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) {
            // The left line is formed here
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            left_m = left_line[1] / left_line[0];
            left_b = cv::Point(left_line[2], left_line[3]);
            l_point.x = left_b.x - 200 * left_line[0];
            l_point.y = left_b.y - 200 * left_line[1];
        }
    }

    // One the slope and offset points have been obtained, apply the line equation to obtain the line points
//    int ini_y = inputImage.rows;
    int ini_y = height;
    int fin_y = height / 2;
//    int fin_y = inputImage.cols;

    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(right_fin_x, fin_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(left_fin_x, fin_y);
//
//        output[0] = right_b;
//        output[1] = r_point;
//        output[2] = left_b;
//        output[3] = l_point;

    return output;
}


// TURN PREDICTION
/**
 *@brief Predict if the lane is turning left, right or if it is going straight
 *@brief It is done by seeing where the vanishing point is with respect to the center of the image
 *@return String that says if there is left or right turn or if the road is straight
 */
double LaneDetector::predictTurn(int &output) {
    double vanish_x;
    double thr_vp = stoi(vars["LANE_DETECTION_CENTER_INTERVAL"]);

    // The vanishing point is the point where both lane boundary lines intersect
    vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

    // The vanishing points location determines where is the road turning
    if (vanish_x < (img_center - thr_vp))
        output = LEFT;
    else if (vanish_x > (img_center + thr_vp))
        output = RIGHT;
    else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp))
        output = STRAIGHT;

    return vanish_x;

}

// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param init is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */
Mat LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn, std::string frameName) {
    std::vector<cv::Point> poly_points;
    cv::Mat output;
    cv::Mat drawing;
    // Create the transparent polygon for a better visualization of the lane
    inputImage.copyTo(output);
    inputImage.copyTo(drawing);
    poly_points.push_back(lane[2]);
    poly_points.push_back(lane[0]);
    poly_points.push_back(lane[1]);
    poly_points.push_back(lane[3]);
    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
    cv::addWeighted(output, 0.3, drawing, 1.0 - 0.3, 0, drawing);

    // Plot both lines of the lane boundary
    cv::line(drawing, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
    cv::line(drawing, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

    auto vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));
    line(drawing, Point(vanish_x, inputImage.rows / 2 - 10), Point(vanish_x, drawing.rows / 2 + 10), Scalar(250, 255, 255), 25, CV_AA);

    // Plot the turn message
//    resize(inputImage, inputImage, Size(), 0.6, 0.6);
    cv::putText(drawing, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);

    // Show the final output image
    cv::namedWindow(frameName, CV_WINDOW_AUTOSIZE);

    cv::imshow(frameName, drawing);
    return drawing;
}

void* look_for_cross_walk(void*) {

    delay(3000);
    zeros = Mat(width, 60, CV_8UC1, 0.0);
    Mat crosswalk_grayscale;
    int width;
    int height;
    while (true) {

        cout << "Test1" << endl;
        width = src.cols;
        height = src.rows;
        cout << "\nLOOK FOR CROSSWALK GOT FRAME OF WIDTH :" << width << endl;

        zeros = cv::Mat::zeros(Size(width, 60), CV_8UC1);

        Rect rect(0, height - 100, width, 50);
        Rect rect2(0, 5, width, 50);

        Mat crosswalk = src(rect);
        cvtColor(crosswalk, crosswalk_grayscale, COLOR_RGB2GRAY);
        dilate(crosswalk_grayscale, crosswalk_grayscale, getStructuringElement(MORPH_RECT, Size(6, 6)));
        GaussianBlur(crosswalk_grayscale, crosswalk_grayscale, cv::Size(9, 9), 2, 2);
        erode(crosswalk_grayscale, crosswalk_grayscale, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));

        Mat combi = zeros(rect2);

        crosswalk_grayscale.copyTo(combi);

        int lowThreshold = 30;
        int ratio = 3;
        int kernel_size = 3;
        cv::Canny(zeros, zeros, lowThreshold, lowThreshold * ratio, kernel_size);

        vector<vector<Point>> contours;
        vector<Point> approx;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(zeros, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        drawing = cv::Mat::zeros(zeros.size(), CV_8UC3);

        int counter = 0;

        vector<vector<Point>> horizontals;
        int c = 0;
        for (int i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(0, 100, 0);
            drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point());


            approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true) * 0.05, true);
            //0.5
            Scalar color1(0, 255, 0);
            Scalar color2(0, 0, 255);
            if (approx.size() == 4) {

                Scalar colorus = color2;
                if (c % 2 == 0)
                    colorus = color1;
                c++;

                double len1 = sqrt(pow(approx[0].x - approx[1].x, 2) + pow(approx[0].y - approx[1].y, 2));
                double len2 = sqrt(pow(approx[1].x - approx[2].x, 2) + pow(approx[1].y - approx[2].y, 2));
                double len = max(len1, len2);

                if (len > 15) {

                    vector<vector<Point>> hlines;
                    for (int m = 0, n = 0; m < 4; m++, n++) {
                        if (n == 3)
                            n = -1;
                        if (abs(approx[m].y - approx[n + 1].y) < 30) {
                            vector<Point> arr(2);
                            arr[0] = approx[m];
                            arr[1] = approx[n + 1];
                            hlines.push_back(arr);
                        }
                    }
                    vector<Point> min(2);
                    if (!hlines.empty()) {
                        min = hlines[0];
                        for (auto l: hlines) {
                            if (l[0].y + l[1].y < min[0].y + min[1].y) {
                                min = l;
                            }
                        }
                    }

                    horizontals.push_back(min);


                    cv::line(drawing, approx[0], approx[1], colorus);
                    cv::line(drawing, approx[1], approx[2], colorus);
                    cv::line(drawing, approx[2], approx[3], colorus);
                    cv::line(drawing, approx[3], approx[0], colorus);

                    counter++;
                }

            }
        }

        cout << "\nHorizontal lines " << horizontals.size() << endl;
        if (!horizontals.empty()) {
            Vec4d lane;
            vector<Point> points;
            for (auto p:horizontals) {
                points.push_back(p[0]);
                points.push_back(p[1]);
            }
            fitLine(points, lane, CV_DIST_L2, 0, 0.01, 0.01);
            int x0 = lane[2];                       // a point on the line
            int y0 = lane[3];
            int x1 = x0 - 200 * lane[0];     // add a vector of length 200
            int y1 = y0 - 200 * lane[1];   // using the unit vector
            line(crosswalk, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0), 3);
            if (abs(lane[1] / lane[0]) < 0.4 && counter >= 5)
            {
                ir_tracers_are_on = 0;
                int temp = speed;
                speed = slowSpeed;
                delay(3000);
                speed = temp;
                ir_tracers_are_on = 1;

                cout << "CROSSROAD WITH SLOPE:" << lane[1] / lane[0] << " WAS DETECTED1" << endl;
            }
        }
    }

}
