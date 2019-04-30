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
#include "../include/config_reader.h"
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
Status trafficLightStatus = GREEN_LIGHT;

string turnAsString[] = {"L", "S", "R"};

std::map vars<std::string, std::string>;

int speed;
int ratio_;

LaneDetector laneDetector;
pthread_mutex_t frame_mutex;

double slope;

void init_vars();

void signalHandler(int signum);

void crosswalk_handler(int sigNum);

void *video_loop(void *);

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    signal(SIGUSR1, left_interupt);
    signal(SIGUSR2, right_interupt);
    signal(SIGRTMIN + 5, obstacle_signal_handler);
//    signal(ZEBRA_CROSSING_SIGNAL, crosswalk_handler);


    pthread_t video_thread;

    speed = 80;
    ratio_ = 3;
    init_vars();

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
   
    while (true) {

        pthread_mutex_lock(&frame_mutex);
        capture.grab(); //grab the scene using raspicam
        capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
        frame = src;
        
        img_mask = laneDetector.mask(src);
        img_edges = laneDetector.edgeDetector(img_mask);
        lines = laneDetector.houghLines(img_edges);

        if (!lines.empty()) {
            left_right_lines = laneDetector.lineSeparation(lines, img_edges);

            lane = laneDetector.regression(left_right_lines, img_mask);

            int degree = laneDetector.predictTurn(turn);

            laneDetector.plotLane(img_mask, lane, turnAsString[turn].append(to_string(right_slope)), "Lane Detection");

        } else {
            turn = STRAIGHT;
        }
//
//        slope = (abs(left_slope) + abs(right_slope)) / 2;
//
//        turn = (left_frame_turn + right_frame_turn) / 2;
//
//        int rows = max(right_edged.rows, left_edged.rows);
//        int cols = left_edged.cols + right_edged.cols;
//
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


        //printf("\n%s", turnAsString[turn]);
        //namedWindow("Hello world");
        waitKey(1);
        pthread_mutex_unlock(&frame_mutex);
    }
}

void init_vars()
{
    speed = vars["SPEED"];
    ratio_ = vars["RATIO"];
}