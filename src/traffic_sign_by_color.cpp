#include "ShapeDetector.h"
#include "traffic_sign.h"
#include "CascadeUtil.h"

#include <string>
#include <iostream>
#include <dirent.h>

#include <assert.h>
#include <cstdarg>

#include <string>
#include <sstream>
#include <regex>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

void preprocessSample(int i) {
    char buf1[526];
    sprintf(buf1, "/home/madina/data/negatives/data%d.jpg", i);

    cout << buf1 << endl;

    string p(buf1);

    Mat img = imread(p.c_str());

    Mat gray_image;

    cvtColor(img, gray_image, COLOR_BGR2GRAY);


    normalize(gray_image, gray_image, 0, 1, NORM_MINMAX, CV_32F);

//        Mat res;
//        cv::GaussianBlur(gray_image, res, cv::Size(0, 0), 3);
//        cv::addWeighted(gray_image, 1.5, res, -0.5, 0, res);

//        cv::erode(gray_image, gray_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));
//        cv::dilate(gray_image, gray_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));


    char buf2[526];
    sprintf(buf2, "/home/madina/data/nnegatives/data%d.jpg", i);

    cout << buf2 << endl;

    string p2(buf2);

    gray_image.convertTo(gray_image, CV_8UC3, 255.0);
    imwrite(p2.c_str(), gray_image);
    imshow("img", gray_image);
}

void showSigns(Mat src, vector<Rect> rec, Scalar color) {
    for (auto r: rec) {
        rectangle(src, r, color);
    }
}

Mat sharpen2D(Mat src) {
    // Create a kernel that we will use to sharpen our image

    // Construct kernel (all entries initialized to 1)
    cv::Mat kernel(3, 3, CV_32F, cv::Scalar(1));
    // assigns kernel values
    kernel.at<float>(1, 1) = -8.0;

    // an approximation of second derivative, a quite strong kernel
    // do the laplacian filtering as it is
    // well, we need to convert everything in something more deeper then CV_8U
    // because the kernel has some negative values,
    // and we can expect in general to have a Laplacian image with negative values
    // BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
    // so the possible negative number will be truncated
    Mat imgLaplacian;
    filter2D(src, imgLaplacian, CV_32F, kernel);
    Mat sharp;
    src.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // imshow( "Laplace Filtered Image", imgLaplacian );
    return imgResult;
}

bool isEqual(const Vec4i &_l1, const Vec4i &_l2) {
    Vec4i l1(_l1), l2(_l2);

    float length1 = sqrtf((l1[2] - l1[0]) * (l1[2] - l1[0]) + (l1[3] - l1[1]) * (l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0]) * (l2[2] - l2[0]) + (l2[3] - l2[1]) * (l2[3] - l2[1]));

    float product = (l1[2] - l1[0]) * (l2[2] - l2[0]) + (l1[3] - l1[1]) * (l2[3] - l2[1]);

    if (fabs(product / (length1 * length2)) < cos(CV_PI / 30))
        return false;

    float mx1 = (l1[0] + l1[2]) * 0.5f;
    float mx2 = (l2[0] + l2[2]) * 0.5f;

    float my1 = (l1[1] + l1[3]) * 0.5f;
    float my2 = (l2[1] + l2[3]) * 0.5f;
    float dist = sqrtf((mx1 - mx2) * (mx1 - mx2) + (my1 - my2) * (my1 - my2));

    if (dist > std::max(length1, length2) * 0.5f)
        return false;

    return true;
}

int main(int argc, char **argv) {
    /// Load an image
    Mat src = imread("/home/madina/test.png");

    //resize(src, src, Size(), 0.3, 0.3);

    Mat imgResult = sharpen2D(src);

    // imshow( "Laplace Filtered Image", imgLaplacian );
    imshow("New Sharped Image", imgResult);
    //! [sharp]

    //! [bin]
    // Create binary image from source image
    Mat bw;
    cvtColor(imgResult, bw, COLOR_BGR2GRAY);
    threshold(bw, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
    imshow("Binary Image", bw);
    Mat blur;
    GaussianBlur(bw, blur, Size(3, 3), 0, 0);
    Mat edges = getEdges(blur);
    imshow("edges", edges);
    //! [bin]

    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
    Mat closed;
    cv::morphologyEx(edges, closed, cv::MORPH_CLOSE, structuringElement);

    vector<vector<Point>> contours;

//    Mat resized;
//    float ratio = 0.5;
//    resize(src, resized, Size(), ratio);

    imshow("closed", closed);

    findContours(closed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i< contours.size(); i++) {
//        Moments M = moments(c);
//
//        int cX = M.m10 / M.m00;
//        int cY = M.m01 / M.m00;

        drawContours(src, c, i++, yellow, 2);
    }

    imshow("src", src);
    waitKey(0);

    return 0;
}