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
#include <cmath>

using namespace std;
using namespace cv;


raspicam::RaspiCam_Cv capture; // initialise the raspicam object
VideoWriter *video;
VideoWriter *video_edge;
VideoWriter *video_mask;
static int turn;
static int left_bottom_turn_predictor;
static int right_bottom_turn_predictor;
static int center_bottom_turn_predictor;

char *turnAsString[] = {const_cast<char *>("LEFT"), const_cast<char *>("STRAIGHT"), const_cast<char *>("RIGHT")};
int speed;
int ratio_;

void signalHandler(int signum) {

    // cleanup and close up stuff here
    // terminate program
    printf("CNTR+C");

    pwmStop();
    stopDCMotor();
    capture.release();
    video->release();
    //video_edge->release();
    //video_mask->release();

    // Closes all the windows
    destroyAllWindows();
    exit(signum);
}
void crosswalk_handler(int sigNum)
{
    pthread_mutex_lock(&motor_mutex);
    pwmStop();
    delay(3000);
    pwmGo(speed);
    delay(1500);
    pthread_mutex_unlock(&motor_mutex);
}

LaneDetector laneDetector;
pthread_mutex_t frame_mutex;

void *video_loop(void *) {
    //Initialise the image as a matrix container
    Mat src;

    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;

    std::vector<cv::Point> lane_left_buttom_frame;
    std::vector<cv::Point> lane_right_buttom_frame;
    std::vector<cv::Point> lane_center_buttom_frame;

    cv::Mat img_left_bottom_mask;
    cv::Mat img_right_buttom_mask;
    cv::Mat img_center_buttom_mask;

    std::vector<cv::Vec4i> left_buttom_lines;
    std::vector<cv::Vec4i> right_buttom_lines;
    std::vector<cv::Vec4i> center_buttom_lines;

    std::vector<std::vector<cv::Vec4i> > left_right_lines_left_buttom_frame;
    std::vector<std::vector<cv::Vec4i> > left_right_lines_right_buttom_frame;
    std::vector<std::vector<cv::Vec4i> > left_right_lines_center_buttom_frame;

    if (pthread_mutex_init(&frame_mutex, nullptr) == -1) {
        printf("\nCannot create mutex!\n");
        return nullptr;

    }


    capture.open(); // activate the raspicam object

    capture.grab(); //grab the scene using raspicam
    capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
    img_edges = laneDetector.edgeDetector(src);
    int width = src.size().width;
    int height = src.size().height;
    video = new VideoWriter("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(width, height));
    //video_edge = new VideoWriter("edge.avi",CV_FOURCC('M','J','P','G'),10, Size(img_edges.size().width, img_edges.size().height));
    //video_mask = new VideoWriter("mask.avi",CV_FOURCC('M','J','P','G'),10, Size(img_mask.size().width, img_mask.size().height));
    while (true) {
        pthread_mutex_lock(&frame_mutex);
        capture.grab(); //grab the scene using raspicam
        capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container

        //img_denoise = laneDetector.deNoise(src);

        img_edges = laneDetector.edgeDetector(src);
/*
        // Mask the image so that we only get the ROI
        img_mask = laneDetector.mask(img_edges);

        // Obtain Hough lines in the cropped image
        lines = laneDetector.houghLines(img_mask);

        if (!lines.empty()) {
            // Separate lines into left and right lines
            left_right_lines = laneDetector.lineSeparation(lines, img_edges);

            // Apply regression to obtain only one line for each side of the lane
            lane = laneDetector.regression(left_right_lines, src);

            // Predict the turn by determining the vanishing point of the the lines
            laneDetector.predictTurn(turn);
            printf("\nTurn %s", turnAsString[turn]);
            // Plot lane detection
            //flag_plot = laneDetector.plotLane(src, lane, turnAsString[turn] + to_string(laneDetector.right_m));


        }
*/

        //LEFT BUTTOM FRAME HERE
        img_left_bottom_mask = laneDetector.mask_left_buttom(img_edges);
        left_buttom_lines = laneDetector.houghLines(img_left_bottom_mask);

        if (!left_buttom_lines.empty()) {
            // Separate lines into left and right lines
            left_right_lines_left_buttom_frame = laneDetector.left_frame_lineSeparation(left_buttom_lines, img_left_bottom_mask);

            // Apply regression to obtain only one line for each side of the lane
            lane_left_buttom_frame = laneDetector.regression(left_right_lines_left_buttom_frame, img_left_bottom_mask);

            // Predict the turn by determining the vanishing point of the the lines
            left_bottom_turn_predictor = 1;
            laneDetector.left_frame_predictTurn(left_bottom_turn_predictor, img_left_bottom_mask);
            printf("\n%s", turnAsString[left_bottom_turn_predictor]);
            // Plot lane detection
            //laneDetector.plotLane(img_left_bottom_mask, lane_left_buttom_frame, turnAsString[left_bottom_turn_predictor]);


        }

        //CENTER BUTTOM FRAME HERE
        img_center_buttom_mask = laneDetector.mask_center_bottom(img_edges);
        center_buttom_lines = laneDetector.houghLines(img_center_buttom_mask);

        Mat center = img_center_buttom_mask.clone();

        if (!center_buttom_lines.empty()) {
            // Separate lines into left and right lines
            unsigned  long zebra_lines = laneDetector.look_for_cross_walk(center_buttom_lines, center);
            if (zebra_lines > 15)
                raise(SIGRTMIN + 6);
            left_right_lines_center_buttom_frame = laneDetector.lineSeparation(center_buttom_lines,
                                                                               img_center_buttom_mask);

            // Apply regression to obtain only one line for each side of the lane
            lane_center_buttom_frame = laneDetector.regression(left_right_lines_center_buttom_frame,
                                                               img_center_buttom_mask);

            // Predict the turn by determining the vanishing point of the the lines
            center_bottom_turn_predictor = 1;
            laneDetector.predictTurn_center_bottom_frame(center_bottom_turn_predictor);
            printf(" - %s", turnAsString[center_bottom_turn_predictor]);
            // Plot lane detection
            //laneDetector.plotLane(img_center_buttom_mask, lane_center_buttom_frame, turnAsString[turn]);


        }

        //RIGHT BUTTOM FRAME HERE
        img_right_buttom_mask = laneDetector.mask_right_bottom(img_edges);
        right_buttom_lines = laneDetector.houghLines(img_right_buttom_mask);

        if (!right_buttom_lines.empty()) {
            // Separate lines into left and right lines
            left_right_lines_right_buttom_frame = laneDetector.right_frame_lineSeparation(right_buttom_lines,
                                                                              img_right_buttom_mask);

            // Apply regression to obtain only one line for each side of the lane
            lane_right_buttom_frame = laneDetector.regression(left_right_lines_right_buttom_frame,
                                                              img_right_buttom_mask);

            // Predict the turn by determining the vanishing point of the the lines
            right_bottom_turn_predictor = 1;
            laneDetector.right_frame_predictTurn(right_bottom_turn_predictor, img_right_buttom_mask);
            printf(" - %s", turnAsString[right_bottom_turn_predictor]);
            // Plot lane detection
            //laneDetector.plotLane(img_right_buttom_mask, lane_right_buttom_frame, turnAsString[turn]);


        }

        turn = (center_bottom_turn_predictor + left_bottom_turn_predictor + right_bottom_turn_predictor + 3 - 1)/3;

        //video_mask->write(img_mask);
        video->write(src);
        //video_edge->write(img_edges);
        //imshow("Video", img_edges);
        //imshow("Video", center);
        //imshow("LEFT_BUTTOM", img_left_bottom_mask);
        //imshow("RIGHT_BUTTOM", img_right_buttom_mask);
        //imshow("CENTER_BUTTOM", img_center_buttom_mask);

        printf("\n%s", turnAsString[turn]);
        //namedWindow("Hello world");
        cvWaitKey(1);
        pthread_mutex_unlock(&frame_mutex);
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    signal(SIGUSR1, left_interupt);
    signal(SIGUSR2, right_interupt);
    signal(SIGRTMIN + 5, obstacle_signal_handler);
    signal(SIGRTMIN + 6, crosswalk_handler);
    /* this variable is our reference to the second thread */
    pthread_t video_thread;

    unsigned int del;
    speed = 60;
    ratio_ = 3;
    del = 2500;
    if (argc > 1) {
        speed = atoi(argv[1]);
        ratio_ = atoi(argv[2]);
        del = static_cast<unsigned int>(atoi(argv[3]));
    }
    printf("Speed %d ", speed);
    if (wiringPiSetup() == -1)
        return 0;
    pwmInitDCMotor();
    sensor_setup();
    sensor_thread_setup();

    if (pthread_create(&video_thread, nullptr, video_loop, nullptr)) {

        fprintf(stderr, "Error creating thread\n");
        return 1;

    }
    turn = STRAIGHT;
    //while(1);
    delay(5000);
    while (true) {
        //pthread_mutex_lock(&frame_mutex);
        pthread_mutex_lock(&motor_mutex);
        //int del;
        double temp;
        double temp2;
        if (turn == STRAIGHT) {
            pwmGo(speed);
            delay(10);
        }
        if (turn == LEFT) {
            temp = abs(laneDetector.right_m);
            //del = 20 * temp;
            pwm_left_smooth_turn(speed, ratio_);
            pthread_mutex_unlock(&motor_mutex);
            delay(del);
        }
        if (turn == RIGHT) {
            temp2 = abs(laneDetector.left_m);
            //del = 20 * temp2;
            pwm_right_smooth_turn(speed, ratio_);
            pthread_mutex_unlock(&motor_mutex);
            delay(del);
        }
        //printf("\nLeft: %lf\nRight : %lf", temp, temp2);
        pthread_mutex_unlock(&motor_mutex);
        //pthread_mutex_unlock(&frame_mutex);
        delay(10);
        //pwmStop();

    }


    return 0;
}

// left / 5 left
