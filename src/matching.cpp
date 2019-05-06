
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat img;
Mat templ;
Mat mask;
Mat result;
const char *image_window = "Source Image";
const char *result_window = "Result window";
int match_method;
int max_Trackbar = 5;

Rect MatchingMethod(int match_method, cv::Mat img, cv::Mat templ);

bool isWithMat(cv::Rect circleBox, cv::Mat bgr) {
    return 0 <= circleBox.x
           && 0 <= circleBox.width
           && circleBox.x + circleBox.width <= bgr.cols
           && 0 <= circleBox.y
           && 0 <= circleBox.height
           && circleBox.y + circleBox.height <= bgr.rows;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        cout << "Not enough parameters" << endl;
        cout << "Usage:\n./MatchTemplate_Demo <image_name> <template_name> [<mask_name>]" << endl;
        return -1;
    }

    img = imread(argv[1], IMREAD_COLOR);
    templ = imread(argv[2], IMREAD_COLOR);

    namedWindow(image_window, WINDOW_AUTOSIZE);
    namedWindow(result_window, WINDOW_AUTOSIZE);

    MatchingMethod(5, img, templ);

    waitKey(0);
    return 0;
}

Rect MatchingMethod(int match_method, cv::Mat img, cv::Mat templ) {
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

    if(isWithMat(rect, img_display)){
        rectangle(img_display, rect , Scalar::all(0), 2, 8, 0);
        rectangle(result, rect, Scalar::all(0), 2, 8, 0);
    }

    imshow(image_window, img_display);
    imshow(result_window, result);
}