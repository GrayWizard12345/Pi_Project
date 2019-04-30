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

#include "../include/LaneDetector.hpp"
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
    left_m = 0;
    right_m = 0;
    left_flag = false;
    right_flag = false;
    cv::Mat output;
    //cv::Mat kernel;
    //cv::Point anchor;

    // Convert image from RGB to gray
    cv::cvtColor(img_noise, output, cv::COLOR_BGR2HSV);
    cv::inRange(output, cv::Scalar(20, 100, 100), cv::Scalar(45, 255, 255), output);
//  (20, 100, 100)(20, 50, 50)
//    cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
    // Binarize gray image
    //cv::threshold(output, output, 140, 255, cv::THRESH_BINARY);

    cv::erode(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 9, 9)));
    cv::dilate(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 3, 3)));

    // Create the kernel [-1 0 1]
    // This kernel is based on the one found in the
    // Lane Departure Warning System by Mathworks
    int lowThreshold = 35;
    int ratio = 3;
    int kernel_size = 3;
    cv::Mat blur_mat;
    cv::GaussianBlur(output, output, cv::Size(9, 9), 2, 2);


    // Filter the binary image to obtain the edges
//    cv::Canny(output, output, lowThreshold, lowThreshold * ratio, kernel_size);
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
    cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
    cv::Point pts[4] = {
            cv::Point(0, 960),
            cv::Point(0, 420),
            cv::Point(1280, 420),
            cv::Point(1280, 960)
    };

    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(img_edges, mask, output);

    return output;
}

cv::Mat LaneDetector::mask_right_bottom(cv::Mat img_edges) {
    cv::Mat output;
    cv::Rect rec(1280 / 2, 540, 1280 / 2, 960 - 540);
    output = img_edges(rec);

    return output;
}

cv::Mat LaneDetector::mask_center_bottom(cv::Mat img_edges) {
    cv::Mat output;
    cv::Rect rec(1280 / 4, 540, 1280 / 2, 960 - 540);
    output = img_edges(rec);
    return output;
}

cv::Mat LaneDetector::mask_left_bottom(cv::Mat img_edges) {
    cv::Mat output;
    cv::Rect rec(0, 540, 1280 / 2, 960 - 540);
    output = img_edges(rec);
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
    double slope_thresh = 0.17;     //TODO changed from 0.3
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Vec4i> right_lines, left_lines;


    right_flag = false;
    left_flag = false;

    // Calculate the slope of all the detected lines
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) > slope_thresh) {
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
        if (slopes[j] > 0) {
            right_lines.push_back(selected_lines[j]);
            right_flag = true;
        } else if (slopes[j] < 0) {
            left_lines.push_back(selected_lines[j]);
            left_flag = true;
        }
        j++;
    }

    output[0] = right_lines;
    output[1] = left_lines;

    return output;
}

std::vector<std::vector<cv::Vec4i>>
LaneDetector::left_frame_lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
    std::vector<std::vector<cv::Vec4i>> output(2);
    size_t j = 0;
    cv::Point ini;
    cv::Point fini;
    double slope_thresh = 0.17;     //TODO changed from 0.3
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;

    // Calculate the slope of all the detected lines
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) > slope_thresh) {
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
        if (slopes[j] < 0) {
            output[1].push_back(selected_lines[j]);
            left_flag = true;
        }
        j++;
    }

    return output;
}

std::vector<std::vector<cv::Vec4i>>
LaneDetector::right_frame_lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
    std::vector<std::vector<cv::Vec4i>> output(2);
    size_t j = 0;
    cv::Point ini;
    cv::Point fini;
    double slope_thresh = 0.17;     //TODO changed from 0.3
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;

    // Calculate the slope of all the detected lines
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) > slope_thresh) {
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
        if (slopes[j] > 0) {
            output[0].push_back(selected_lines[j]);
            right_flag = true;
        }
        j++;
    }

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
    if (right_flag == true) {
        for (auto i : left_right_lines[0]) {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) {
            // The right line is formed here
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];
            right_b = cv::Point(right_line[2], right_line[3]);
        }
    }

    // If left lines are being detected, fit a line using all the init and final points of the lines
    if (left_flag == true) {
        for (auto j : left_right_lines[1]) {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);

            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) {
            // The left line is formed here
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            left_m = left_line[1] / left_line[0];
            left_b = cv::Point(left_line[2], left_line[3]);
        }
    }

    // One the slope and offset points have been obtained, apply the line equation to obtain the line points
    int ini_y = inputImage.rows;
    //int fin_y = 470;
    int fin_y = inputImage.cols;

    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(right_fin_x, fin_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(left_fin_x, fin_y);

    return output;
}


int LaneDetector::left_frame_predictTurn(int &output, cv::Mat source) {

    double vanish_x;
    double thr_vp = 7;

//    // The vanishing point is the point where both lane boundary lines intersect
//    vanish_x = static_cast<double>(((right_m * right_b.x) - (left_m * left_b.x) - right_b.y + left_b.y) /
//                                       (right_m - left_m));

//    double y1 = left_m * img_center + left_b.y;
    if (left_m >= 0) {
        output = Turn::LEFT;
    } else {
        output = Turn::RIGHT;
    }
//    if ((source.rows / 2) - y1 - 20 > y1)
//    {
//        output = Turn::STRAIGHT;
//    }
//    // The vanishing points location determines where is the road turning
//    if (vanish_x < (img_center - thr_vp))
//        output = Turn::LEFT;
//    else if (vanish_x > (img_center + thr_vp))
//        output = Turn::RIGHT;
//    else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp))
//        output = Turn::STRAIGHT;
//    //printf("\nTurn in predict turn %d", output);
    return output;
}

int LaneDetector::right_frame_predictTurn(int &output, cv::Mat source) {
//    double y1 = right_m * img_center + right_b.y;
    if (right_m > 0) {
        output = Turn::LEFT;
    } else {
        output = Turn::RIGHT;
    }
//    if ((source.rows / 2) - y1 - 20> y1)
//    {
//        output = Turn::STRAIGHT;
//    }

    return output;
}

// TURN PREDICTION
/**
 *@brief Predict if the lane is turning left, right or if it is going straight
 *@brief It is done by seeing where the vanishing point is with respect to the center of the image
 *@return String that says if there is left or right turn or if the road is straight
 */
void LaneDetector::predictTurn(int &output) {
    double vanish_x;
    int threashold = 20;
    // The vanishing point is the point where both lane boundary lines intersect
    vanish_x = ((right_m * right_b.x) - (left_m * left_b.x) - right_b.y + left_b.y) /
               (right_m - left_m);

    // The vanishing points location determines where is the road turning
    if (vanish_x <= img_center)
        output = LEFT;
    else
        output = RIGHT;

    if (output == LEFT && left_flag) {
        output = RIGHT;
    }

    if (output == RIGHT && right_flag) {
        output = LEFT;
    }
}

// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param init is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */
int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn, std::string frameName) {
    std::vector<cv::Point> poly_points;
    cv::Mat output;

    // Create the transparent polygon for a better visualization of the lane
    inputImage.copyTo(output);
    poly_points.push_back(lane[2]);
    poly_points.push_back(lane[0]);
    poly_points.push_back(lane[1]);
    poly_points.push_back(lane[3]);
    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
    cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

    // Plot both lines of the lane boundary
    cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
    cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

    // Plot the turn message
    cv::putText(inputImage, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);

    // Show the final output image
    cv::namedWindow(frameName, CV_WINDOW_AUTOSIZE);
    cv::imshow(frameName, inputImage);
    return 0;
}

unsigned long LaneDetector::look_for_cross_walk(cv::Mat &src) {
/*
    cv::Point ini;
    cv::Point fini;
    double slope_thresh = 1.5;     //TODO changed from 0.3
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Point> poly_points;
    int c = 0;
    // Calculate the slope of all the detected lines
    for (auto i : houghLines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                       (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope

        if (std::abs(slope) > slope_thresh) {
            //printf("%lf\n", slope);
            slopes.push_back(slope);
            selected_lines.push_back(i);
            cv::line(src, ini, fini, cv::Scalar(245, 40, c += 20), 5, CV_AA);
        }
        //printf("Num of lines: %lu\n" ,selected_lines.size());
        if (selected_lines.size() > 15)
            return selected_lines.size();
    }

    return selected_lines.size();
*/
    Mat crosswalk_g;
    Mat crosswalk;
    cvtColor(src, crosswalk_g, COLOR_RGB2GRAY);
    cvtColor(crosswalk_g, src, COLOR_GRAY2BGR);
    cvtColor(src, crosswalk, COLOR_RGB2HSV);
    inRange(crosswalk, Scalar(0, 0, 215) ,Scalar(255, 50, 255),crosswalk);

//    GaussianBlur(crosswalk, crosswalk, cv::Size(3, 3), 0, 0);
    blur(crosswalk, crosswalk, cv::Size(3, 3));

    int lowThreshold = 35;
    int ratio = 3;
    int kernel_size = 3;
    cv::Canny(crosswalk, crosswalk, lowThreshold, lowThreshold * ratio, kernel_size);

    vector<vector<Point>> contours;
    vector<Point> approx;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(crosswalk,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


    cv::Mat drawing = cv::Mat::zeros( crosswalk.size(), CV_8UC3 );
    int counter = 0;
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

        approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true)*0.06, true);
        if (approx.size() == 4){

            cv::line(src, approx[0], approx[1], cv::Scalar(0,255,0));
            cv::line(src, approx[1], approx[2], cv::Scalar(0,255,0));
            cv::line(src, approx[2], approx[3], cv::Scalar(0,255,0));
            cv::line(src, approx[3], approx[0], cv::Scalar(0,255,0));

            cv::line(drawing, approx[0], approx[1], cv::Scalar(255,255,0));
            cv::line(drawing, approx[1], approx[2], cv::Scalar(255,255,0));
            cv::line(drawing, approx[2], approx[3], cv::Scalar(255,255,0));
            cv::line(drawing, approx[3], approx[0], cv::Scalar(255,255,0));

            counter++;
        }
    }

    if(counter >= 4)
        putText(crosswalk, "CrossWalk detected - r#:" + to_string(counter), Point(50,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 0, 0), 1, CV_AA);
    cout << counter;
    imshow("drawing", drawing);
    imshow("Looking here", src);


}

