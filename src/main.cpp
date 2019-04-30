#include "sensors_controll.cpp"
#include <cstring>
#include <string>
#include <cstdlib>
#include <csignal>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include "../include/LaneDetector.hpp"
#include "../include/traffic_light.hpp"
#include <cmath>

using namespace std;
using namespace cv;

//TODO make possible variables local, not global

raspicam::RaspiCam_Cv capture;
Mat src;
Mat frame;
Mat red_color_frame;
Mat green_color_frame;

VideoWriter *video;

static int turn;
static int left_frame_turn;
static int right_frame_turn;
static int center_bottom_turn_predictor;

Status trafficLightStatus = GREEN_LIGHT;

string turnAsString[] = {"L", "S", "R"};

int speed;
int ratio_;

LaneDetector laneDetector;
pthread_mutex_t frame_mutex;

std::vector<cv::Point> lane_left_bottom_frame;
std::vector<cv::Point> lane_right_bottom_frame;
std::vector<cv::Point> lane_center_buttom_frame;

double slope;

void signalHandler(int signum);

void crosswalk_handler(int sigNum);

void *video_loop(void *);

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    signal(SIGUSR1, left_interupt);
    signal(SIGUSR2, right_interupt);
    signal(SIGRTMIN + 5, obstacle_signal_handler);
//    signal(ZEBRA_CROSSING_SIGNAL, crosswalk_handler);
    /* this variable is our reference to the second thread */
    pthread_t video_thread;

    unsigned int del;
    speed = 80;
    ratio_ = 3;
    del = 250;
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

    if (pthread_mutex_init(&motor_mutex, nullptr) == -1) {
        printf("\nCannot create mutex!\n");
        return 1;
    }

    if (pthread_create(&video_thread, nullptr, video_loop, nullptr)) {

        fprintf(stderr, "Error creating thread\n");
        return 1;

    }
    //Traffic light thread initializations
    initTrafficLightThread();

    turn = STRAIGHT;
    delay(3000);
    int speedLeft = speed, speedRight = speed;
    while (true) {

        pthread_mutex_lock(&motor_mutex);
        if (turn == Turn::STRAIGHT) {
            speedLeft = speed;
            speedRight = speed;
        } else if (turn == Turn::LEFT) {
            speedRight = speed;
            speedLeft = speed / 4;
        } else if (turn == Turn::RIGHT) {
            speedRight = speed / 4;
            speedLeft = speed;
        }

        //pthread_mutex_lock(&frame_mutex);
        if (trafficLightStatus != RED_LIGHT) {
//            pwm_go_smooth(speedLeft, speedRight);
        }
        //pthread_mutex_unlock(&frame_mutex);
        pthread_mutex_unlock(&motor_mutex);

        printf("\nspeed L: %d, speed R: %d, turn: %d , slope: %fl\n", speedLeft, speedRight, turn ,slope);

        delay(100);

    }


    return 0;
}

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
    //destroyAllWindows();
    exit(signum);
}

void crosswalk_handler(int sigNum) {
    pthread_mutex_lock(&motor_mutex);
    pwmStop();
    delay(3000);
    pwmGo(speed);
    delay(1500);
    pthread_mutex_unlock(&motor_mutex);
}

void *video_loop(void *) {
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;

    cv::Mat left_mask;
    cv::Mat left_edged;

    cv::Mat right_mask;
    cv::Mat right_edged;

    cv::Mat center_mask;
    cv::Mat center_edged;

    std::vector<cv::Vec4i> left_lines;
    std::vector<cv::Vec4i> right_lines;
    std::vector<cv::Vec4i> center_bottom_lines;

    std::vector<std::vector<cv::Vec4i> > left_lines_separated;
    std::vector<std::vector<cv::Vec4i> > right_lines_separated;
    std::vector<std::vector<cv::Vec4i> > left_right_lines_center_buttom_frame;

    if (pthread_mutex_init(&frame_mutex, nullptr) == -1) {
        printf("\nCannot create mutex!\n");
        return nullptr;
    }


    capture.open(); // activate the raspicam object

    capture.grab(); //grab the scene using raspicam
    capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
    //img_edges = laneDetector.edgeDetector(src);
    int width = src.size().width;
    int height = src.size().height;
    frame = src;
    delay(1000);
    double left_slope;
    double right_slope;
    video = new VideoWriter("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 7, Size(width, height));
    //video_edge = new VideoWriter("edge.avi",CV_FOURCC('M','J','P','G'),10, Size(img_edges.size().width, img_edges.size().height));
    //video_mask = new VideoWriter("mask.avi",CV_FOURCC('M','J','P','G'),10, Size(img_mask.size().width, img_mask.size().height));

    while (true) {

        pthread_mutex_lock(&frame_mutex);
        capture.grab(); //grab the scene using raspicam
        capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
        frame = src;

        //LEFT BOTTOM FRAME
        left_mask = laneDetector.mask_left_bottom(src);
        left_edged = laneDetector.edgeDetector(left_mask);
        left_lines = laneDetector.houghLines(left_edged);

        if (!left_lines.empty()) {
            left_lines_separated = laneDetector.lineSeparation(left_lines, left_edged);

            lane_left_bottom_frame = laneDetector.regression(left_lines_separated, left_mask);

            laneDetector.predictTurn(left_frame_turn);

            if (left_frame_turn == LEFT)
                left_slope = laneDetector.right_m;
            else
                left_slope = laneDetector.left_m;

            //laneDetector.plotLane(left_mask, lane_left_bottom_frame, turnAsString[left_frame_turn].append(to_string(left_slope)), "LEFT FRAME");
        } else {
            left_frame_turn = LEFT;
            left_slope = -1;
        }


        //CENTER BOTTOM FRAME HERE
        center_mask = laneDetector.mask_center_bottom(src);
//        center_edged = laneDetector.edgeDetector(center_mask);
        //center_bottom_lines = laneDetector.houghLines(center_edged);

        Mat center = center_mask.clone();

        unsigned long zebra_lines = laneDetector.look_for_cross_walk(center);
        //if (zebra_lines > 15)
//                raise(ZEBRA_CROSSING_SIGNAL);


        //RIGHT BOTTOM FRAME HERE
        right_mask = laneDetector.mask_right_bottom(src);
        right_edged = laneDetector.edgeDetector(right_mask);
        right_lines = laneDetector.houghLines(right_edged);

        if (!right_lines.empty()) {
            right_lines_separated = laneDetector.lineSeparation(right_lines, right_edged);

            lane_right_bottom_frame = laneDetector.regression(right_lines_separated, right_mask);

            laneDetector.predictTurn(right_frame_turn);

            if (right_frame_turn == RIGHT)
                right_slope = laneDetector.left_m;
            else
                right_slope = laneDetector.right_m;

            //laneDetector.plotLane(right_mask, lane_right_bottom_frame, turnAsString[right_frame_turn].append(to_string(right_slope)), "RIGHT FRAME");

        } else {
            right_frame_turn = RIGHT;
            right_slope = 1;
        }

        slope = (abs(left_slope) + abs(right_slope)) / 2;

        turn = (left_frame_turn + right_frame_turn) / 2;

        int rows = max(right_edged.rows, left_edged.rows);
        int cols = left_edged.cols + right_edged.cols;

//        Mat3b res(rows, cols, Vec3b(0,0,0));
//
//        left_edged.copyTo(res(Rect(0, 0, left_edged.cols, left_edged.rows)));
//        right_edged.copyTo(res(Rect(left_edged.cols, 0, right_edged.cols, right_edged.rows)));
//        //printf("L : %lf --- R: %lf", left_slope, right_slope);
//        putText(res, turnAsString[left_frame_turn] +  " " + to_string(left_slope) + " "+ turnAsString[left_frame_turn] + " " + to_string(right_slope), Point(35, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
//        resize(res, res, Size(), 0.75, 0.75);
//        putText(res, "General Turn: " + turnAsString[turn], Point(35, 140), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);

        //line(res, left_ini, right_fini, Scalar(0, 255, 0), 5, LINE_AA);
        //line(res, right_ini, right_fini, Scalar(0, 0, 255), 5, LINE_AA);
        //line(res, Point(0,0), Point(50,50), Scalar(0, 0, 255), 5, LINE_AA);
        //video_mask->write(img_mask);
        //video_mask->write(img_mask);
        video->write(src);
//        imshow("Road", res);
        //video_edge->write(img_edges);
        //imshow("RED", red_color_frame);
        //imshow("GREEN", green_color_frame);
        imshow("LEFT_BUTTOM", left_edged);
        imshow("RIGHT_BUTTOM", right_edged);
//        imshow("LEFT_BUTTOM_EDGED", left_mask);
//        imshow("RIGHT_BUTTOM_EDGED", right_mask);
//        //imshow("traffic_light", traffic_light);


        //printf("\n%s", turnAsString[turn]);
        //namedWindow("Hello world");
        waitKey(1);
        pthread_mutex_unlock(&frame_mutex);
    }
}