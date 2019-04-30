#include "sensors_controll.cpp"
#include <cstring>
#include <string>
#include <cstdlib>
#include <csignal>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include "LaneDetector.hpp"
#include "traffic_light.hpp"
#include <cmath>

using namespace std;
using namespace cv;

//TODO make possible variables local, not global



int main(int argc, char **argv) {

    Mat image = imread("zebra.jpg", CV_LOAD_IMAGE_COLOR);

    if (!image.data) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    LaneDetector lanedetector;

    image = lanedetector.mask_center_bottom(image);

    Mat img_denoise, img_edges, img_mask;

    // Denoise the image using a Gaussian filter
    img_denoise = lanedetector.deNoise(image);

    // Detect edges in the image
    img_edges = lanedetector.edgeDetector(img_denoise);

    cv::namedWindow("Edges", CV_WINDOW_AUTOSIZE);
    cv::imshow("Edges", img_edges);

    vector<Vec4i> lines = lanedetector.houghLines(img_edges);

    if (!lines.empty()) {
        printf("%d line count", lines.size());
    } else {
        printf("empty lines");
    }

    waitKey(25);

    return 0;
}