//
// Created by madina on 05/05/19.
//

#include "../include/traffic_sign.h"
#include "../include/CascadeUtil.h"

using namespace cv;

static CascadeUtil cascadeUtil;

Sign signDetected = NO_SIGN;

Scalar blue = Scalar(255, 178, 102);
Scalar yellow = Scalar(255, 255, 51);
Scalar green = Scalar(153, 255, 51);
Scalar orange = Scalar(51, 255, 255);
Scalar violet = Scalar(127, 0, 255);
Scalar purple = Scalar(255, 51, 255);
Scalar pink = Scalar(255, 51, 153);

void initSignDetection() {
    cascadeUtil.loadAll();
}

void detectSign(Mat inputImage) {

    Mat bgr;
    bgr = getTrafficSignROI(inputImage);

    //region Color detection + cascade
//    std::vector<cv::Vec3f> circles = getEdges(bgr);
//
//    printf("detected circles size %d\n", circles.size());
//
//    for (size_t i = 0; i < circles.size(); i++) {
//        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);
//
//        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
//        circle(bgr, center, radius, yellow, 2, 8, 0);
//
//        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);
//
//        if (isWithMat(circleBox, bgr)) {
//
//            Mat circleROI(bgr, circleBox);
//
//            cascadeUtil.setDetectionArea(circleROI);
//            cascadeUtil.detectAllCircleBlueSigns();
//
//            for (unsigned k = 0; k < cascadeUtil.left_.size(); k++) {
//                rectangle(bgr, cascadeUtil.left_[k], green, 2, 1);
//                putText(bgr, "left", Point(50, 110), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//
//
//            for (unsigned n = 0; n < cascadeUtil.right_.size(); n++) {
//                rectangle(bgr, cascadeUtil.right_[n], purple, 2, 1);
//                putText(bgr, "right", Point(50, 150), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//        } else {
//            printf("outside\n");
//        }
//    }
    //endregion

    cascadeUtil.setDetectionArea(bgr);

    cascadeUtil.detectRightTurn();
    cascadeUtil.detectLeftTurn();
    cascadeUtil.detectParking();
    cascadeUtil.detectParking();

    if (cascadeUtil.isRightTurnDetected)
        signDetected = RIGHT_TURN;
    else if (cascadeUtil.isLeftTurnDetected)
        signDetected = LEFT_TURN;
    else if (cascadeUtil.isParkingDetected)
        signDetected = PARKING;
    else if (cascadeUtil.isPedestrianDetected)
        signDetected = PEDESTRIAN;
    else
        signDetected = NO_SIGN;
}

//TODO change the points of ROI
cv::Mat getTrafficSignROI(cv::Mat bgr) {

    cv::Rect roiBox(0, bgr.rows / 12, bgr.cols, bgr.rows / 4);

    cv::Mat roi(bgr, roiBox);


    return roi;
}

Mat getEdges(cv::Mat bgr) {
    Mat output;
    bgr.copyTo(output);

    int lowThreshold = 35;
    int ratio = 3;
    int kernel_size = 3;


    Mat edges;
    cv::Canny(output, edges, lowThreshold, lowThreshold * ratio, kernel_size);

//    Mat blurred;
//    cv::blur(edges, blurred, cv::Size(9, 9));


    return edges;
}

std::vector<Vec3f> getCircles(Mat edges, int high, int low, int minRadius, int maxRadius) {
    std::vector<cv::Vec3f> circles;
    HoughCircles(edges, circles, cv::HOUGH_GRADIENT, 1, edges.rows / 8, high, low, minRadius,
                 maxRadius);

    return circles;
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void getRectangles(Mat edges, Mat src) {
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    // find contours and store them all as a list
    findContours(edges, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    std::vector<Point> approx;
    std::vector<std::vector<Point>> rectangles;

    // test each contour
    for (size_t i = 0; i < contours.size(); i++) {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if (approx.size() == 4)
//            isContourConvex(Mat(approx))) {
//            double maxCosine = 0;

//            for (int j = 2; j < 5; j++) {
//                // find the maximum cosine of the angle between joint edges
//                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
//                maxCosine = MAX(maxCosine, cosine);
//            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
//            if (maxCosine < 0.3) {
            cv::line(src, approx[0], approx[1], cv::Scalar(0, 255, 0));
        cv::line(src, approx[1], approx[2], cv::Scalar(0, 255, 0));
        cv::line(src, approx[2], approx[3], cv::Scalar(0, 255, 0));
        cv::line(src, approx[3], approx[0], cv::Scalar(0, 255, 0));
//            }
//        }
    }
}

Mat getRedMask(Mat roi) {
    cv::Mat hsv_image;
    cvtColor(roi, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat lower_hue, upper_hue;
    inRange(hsv_image, Scalar(0, 120, 70), Scalar(10, 255, 255), lower_hue);
    inRange(hsv_image, Scalar(170, 120, 70), Scalar(180, 255, 255), upper_hue);

    Mat red_hue_image;
    //use hue values from the both ranges (masks)
    addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, red_hue_image);
    erode(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    dilate(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
    GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    return red_hue_image;
}

bool isWithMat(cv::Rect circleBox, cv::Mat bgr) {
    return 0 <= circleBox.x
           && 0 <= circleBox.width
           && circleBox.x + circleBox.width <= bgr.cols
           && 0 <= circleBox.y
           && 0 <= circleBox.height
           && circleBox.y + circleBox.height <= bgr.rows;
}

Mat gammaCorrection(const Mat &img, const double gamma_) {
    CV_Assert(gamma_ >= 0);

    //! [changing-contrast-brightness-gamma-correction]

    Mat lookUpTable(1, 256, CV_8U);
    uchar *p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);

    Mat res = img.clone();
    LUT(img, lookUpTable, res);

    return res;
}


float round(float num, int decimalPlaces) {
    return roundf(num * pow(10, decimalPlaces)) / pow(10, decimalPlaces);
}

Rect MatchingMethod(cv::Mat img, cv::Mat templ, int match_method) {
    Mat img_display;
    img.copyTo(img_display);

    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    Mat result;
    result.create(result_rows, result_cols, CV_32FC1);

    matchTemplate(img, templ, result, match_method);

    normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
        matchLoc = minLoc;
    } else {
        matchLoc = maxLoc;
    }

    Rect rect(matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows));

    return rect;
}

cv::Mat convertToYCrCb(cv::Mat input) {
    //sample input and output
    float data[3][1] = {98, 76, 88};
    Mat output(input.rows, input.cols, CV_32FC3);

    //iterate over all pixels
    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            //get bgr pixel
            Vec3f bgrPixel = input.at<Vec3f>(i, j);

            float B = bgrPixel[0];
            float G = bgrPixel[1];
            float R = bgrPixel[2];

            //actual conversion from BGR to YCrCb
            float delta = 0.5f;
            float Y = 0.299f * R + 0.587f * G + 0.114f * B;
            float Cb = (B - Y) * 0.564f + delta;
            float Cr = (R - Y) * 0.713f + delta;

            //store into result image
            Vec3f yCrCbPixel(Y, Cr, Cb);
            output.at<Vec3f>(i, j) = yCrCbPixel;
        }
    }

    return output;
}

//TODO verify the area for false positive
void *sign_detection(void *) {
    initSignDetection();

    Sign lastDetectedSign = NO_SIGN;
    Mat bgr;
    while (true) {
        Mat bgr;
        bgr = getTrafficSignROI(src);

        //region Color detection + cascade
//    std::vector<cv::Vec3f> circles = getEdges(bgr);
//
//    printf("detected circles size %d\n", circles.size());
//
//    for (size_t i = 0; i < circles.size(); i++) {
//        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        int radius = cvRound(circles[i][2]);
//
//        printf("x: %d y: %d radius: %d\n", center.x, center.y, radius);
//        circle(bgr, center, radius, yellow, 2, 8, 0);
//
//        Rect circleBox(center.x - radius, center.y - radius, radius * 2, radius * 2);
//
//        if (isWithMat(circleBox, bgr)) {
//
//            Mat circleROI(bgr, circleBox);
//
//            cascadeUtil.setDetectionArea(circleROI);
//            cascadeUtil.detectAllCircleBlueSigns();
//
//            for (unsigned k = 0; k < cascadeUtil.left_.size(); k++) {
//                rectangle(bgr, cascadeUtil.left_[k], green, 2, 1);
//                putText(bgr, "left", Point(50, 110), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//
//
//            for (unsigned n = 0; n < cascadeUtil.right_.size(); n++) {
//                rectangle(bgr, cascadeUtil.right_[n], purple, 2, 1);
//                putText(bgr, "right", Point(50, 150), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//            }
//        } else {
//            printf("outside\n");
//        }
//    }
        //endregion

        cascadeUtil.setDetectionArea(bgr);

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
            rectangle(bgr, r, yellow);
        }

        for (auto r : cascadeUtil.left_) {
            rectangle(bgr, r, green);
        }

        for (auto r : cascadeUtil.right_) {
            rectangle(bgr, r, purple);
        }

        for (auto r : cascadeUtil.pedestrian) {
            rectangle(bgr, r, orange);
        }

        for (auto r : cascadeUtil.parking) {
            rectangle(bgr, r, blue);
        }

        imshow("Traffic Sign", bgr);
        waitKey(0);


        lastDetectedSign = signDetected;
    }
}