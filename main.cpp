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


raspicam::RaspiCam_Cv capture; // initialise the raspicam object
//Initialise the image as a matrix container
Mat src;
Mat frame;

VideoWriter *video;
VideoWriter *video_edge;
VideoWriter *video_mask;
static int turn;
static int left_frame_turn;
static int right_frame_turn;
static int center_bottom_turn_predictor;

Status trafficLightStatus = Status::RED_LIGHT;


string turnAsString[] = {"LEFT", "STRAIGHT", "RIGHT"};
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

void crosswalk_handler(int sigNum) {
    pthread_mutex_lock(&motor_mutex);
    pwmStop();
    delay(3000);
    pwmGo(speed);
    delay(1500);
    pthread_mutex_unlock(&motor_mutex);
}

LaneDetector laneDetector;
pthread_mutex_t frame_mutex;

std::vector<cv::Point> lane_left_bottom_frame;
std::vector<cv::Point> lane_right_buttom_frame;
std::vector<cv::Point> lane_center_buttom_frame;

struct Line {
    double slope;
    double length;
};

Line centerLine;
double slope;
Line getSlope(Point_<int> &leftInitial, Point_<int> &leftFinal, Point_<int> &rightInitial, Point_<int> &rightFinal) {

    Point_<int> final;
    final.y = (leftFinal.y + rightFinal.y) / 2;
    final.x = (leftFinal.x + rightFinal.x) / 2;

    Point_<int> initial;
    initial.y = (leftInitial.y + rightInitial.y) / 2;
    initial.x = (rightInitial.x + rightInitial.x) / 2;

    double slope = (static_cast<double>(final.y) - static_cast<double>(initial.y)) /
                   (static_cast<double>(final.x) - static_cast<double>(initial.x) + 0.00001);

    Line line;
    line.slope = slope;
    line.length = sqrt(pow(final.y - initial.y, 2) + pow(initial.x - initial.x, 2));

    return line;
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
    std::vector<cv::Vec4i> right_bottom_lines;
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
    double left_slope;
    double right_slope;
    video = new VideoWriter("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 6, Size(width, height));
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
            // Separate lines into left and right lines
            left_lines_separated = laneDetector.left_frame_lineSeparation(left_lines, left_edged);

            // Apply regression to obtain only one line for each side of the lane
            lane_left_bottom_frame = laneDetector.regression(left_lines_separated, left_mask);

            // Predict the turn by determining the vanishing point of the the lines
//            laneDetector.left_frame_predictTurn(left_frame_turn, left_mask);
            laneDetector.predictTurn_center_bottom_frame(left_frame_turn);

            // Plot lane detection
//            printf("\n%s", turnAsString[left_frame_turn]);
            laneDetector.plotLane(left_mask, lane_left_bottom_frame[2], lane_left_bottom_frame[3],
                                  turnAsString[left_frame_turn] + " " + to_string(laneDetector.left_m), "left_bottom");
            left_slope = laneDetector.left_m;
        } else {
            left_frame_turn = LEFT;
        }

        //CENTER BOTTOM FRAME HERE
        center_mask = laneDetector.mask_center_bottom(src);
        center_edged = laneDetector.edgeDetector(center_mask);
        center_bottom_lines = laneDetector.houghLines(center_edged);

        Mat center = center_mask.clone();

        if (!center_bottom_lines.empty()) {
            // Separate lines into left and right lines
            unsigned long zebra_lines = laneDetector.look_for_cross_walk(center_bottom_lines, center);
            if (zebra_lines > 15)
                raise(SIGRTMIN + 6);

        }

        //RIGHT BOTTOM FRAME HERE
        right_mask = laneDetector.mask_right_bottom(src);
        right_edged = laneDetector.edgeDetector(right_mask);
        right_bottom_lines = laneDetector.houghLines(right_edged);

        if (!right_bottom_lines.empty()) {
            // Separate lines into left and right lines
            right_lines_separated = laneDetector.right_frame_lineSeparation(right_bottom_lines, right_edged);

            // Apply regression to obtain only one line for each side of the lane
            lane_right_buttom_frame = laneDetector.regression(right_lines_separated, right_mask);

            // Predict the turn by determining the vanishing point of the the lines
            //laneDetector.right_frame_predictTurn(right_frame_turn, right_mask);
            laneDetector.predictTurn_center_bottom_frame(right_frame_turn);
            // Plot lane detection
//            printf(" - %s", turnAsString[right_frame_turn]);
            laneDetector.plotLane(right_mask, lane_right_buttom_frame[0], lane_right_buttom_frame[1],
                                  turnAsString[right_frame_turn] + " " + to_string(laneDetector.right_m),
                                  "Right_bottom");
            right_slope = laneDetector.right_m;
        } else {
            right_frame_turn = RIGHT;
        }

        turn = (left_frame_turn + right_frame_turn) / 2;
        printf("turn: %s\n", turnAsString[turn]);
        slope = (abs(right_slope) + abs(left_slope)) / 2;
        //video_mask->write(img_mask);
        //video_mask->write(img_mask);
        video->write(src);
        //video_edge->write(img_edges);
        //imshow("Video", img_edges);
        //imshow("Video", center);
        //imshow("LEFT_BUTTOM", left_mask);
        //imshow("RIGHT_BUTTOM", right_mask);
//        imshow("LEFT_BUTTOM_EDGED", left_mask);
//        imshow("RIGHT_BUTTOM_EDGED", right_mask);
//        //imshow("traffic_light", traffic_light);


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

    if (pthread_create(&video_thread, nullptr, video_loop, nullptr)) {

        fprintf(stderr, "Error creating thread\n");
        return 1;

    }
    //Taffic light thread initializations
    initTrafficLightThread();

    turn = STRAIGHT;
    delay(3000);
    int speedLeft, speedRight;
    while (true) {
        //pthread_mutex_lock(&frame_mutex);
        pthread_mutex_lock(&motor_mutex);

        if (turn == Turn::STRAIGHT) {
            speedLeft = speed;
            speedRight = speed;
        } else {
            if (turn == Turn::LEFT) {
                speedRight = speed;
                if (slope > 0.37) {
                    //speedLeft = speed * line.slope;
                    speedLeft = speed / 2;
                } else {
                    //speedLeft = speed * (1 - (1 / line.slope));
                    speedLeft = speed / 3;
                    speedRight = speed * 1.5;
                }
            } else {
                speedLeft = speed;
                if (slope > 0.37) {
                    //speedRight = speed * line.slope;
                    speedRight = speed / 2;
                } else {
                    //speedRight = speed * (1 - (1 / line.slope));
                    speedRight = speed / 3;
                    speedLeft = speed * 1.5;
                }
            }
        }

        if (trafficLightStatus != RED_LIGHT)
            pwm_go_smooth(speedLeft, speedRight);

        printf("\nspeed left: %d, speed right: %d, slope: %fl\n", speedLeft, speedRight, slope);


        pthread_mutex_unlock(&motor_mutex);
        //pthread_mutex_unlock(&frame_mutex);
        delay(100);

    }


    return 0;
}