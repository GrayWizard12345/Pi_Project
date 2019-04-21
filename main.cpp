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
VideoWriter *video;
VideoWriter *video_edge;
VideoWriter *video_mask;
static int turn;
static int left_frame_turn;
static int right_frame_turn;
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

std::vector<cv::Point> lane_left_buttom_frame;
std::vector<cv::Point> lane_right_buttom_frame;
std::vector<cv::Point> lane_center_buttom_frame;

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
    std::vector<cv::Vec4i> right_buttom_lines;
    std::vector<cv::Vec4i> center_buttom_lines;

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
    video = new VideoWriter("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 6, Size(width, height));
    //video_edge = new VideoWriter("edge.avi",CV_FOURCC('M','J','P','G'),10, Size(img_edges.size().width, img_edges.size().height));
    //video_mask = new VideoWriter("mask.avi",CV_FOURCC('M','J','P','G'),10, Size(img_mask.size().width, img_mask.size().height));
    while (true) {
        pthread_mutex_lock(&frame_mutex);
        capture.grab(); //grab the scene using raspicam
        capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
        frame_for_traffic_light = src;
        //LEFT BOTTOM FRAME
        left_mask = laneDetector.mask_left_buttom(src);
        left_edged = laneDetector.edgeDetector(left_mask);
        left_lines = laneDetector.houghLines(left_edged);

        if (!left_lines.empty()) {
            // Separate lines into left and right lines
            left_lines_separated = laneDetector.left_frame_lineSeparation(left_lines, left_edged);

            // Apply regression to obtain only one line for each side of the lane
            lane_left_buttom_frame = laneDetector.regression(left_lines_separated, left_mask);

            // Predict the turn by determining the vanishing point of the the lines
            left_frame_turn = Turn::STRAIGHT;
            laneDetector.left_frame_predictTurn(left_frame_turn, left_mask);

            // Plot lane detection
            printf("\n%s", turnAsString[left_frame_turn]);
            laneDetector.plotLane(left_mask, lane_left_buttom_frame, turnAsString[left_frame_turn], "left_bottom");
        } else {
            left_frame_turn = Turn::RIGHT;
        }

        //CENTER BUTTOM FRAME HERE
        center_mask = laneDetector.mask_center_bottom(src);
        center_edged = laneDetector.edgeDetector(center_mask);
        center_buttom_lines = laneDetector.houghLines(center_edged);

        Mat center = center_mask.clone();

        if (!center_buttom_lines.empty()) {
            // Separate lines into left and right lines
            unsigned long zebra_lines = laneDetector.look_for_cross_walk(center_buttom_lines, center);
            if (zebra_lines > 15)
                raise(SIGRTMIN + 6);
/*        left_right_lines_center_buttom_frame = laneDetector.lineSeparation(center_buttom_lines,
                                                                               img_center_buttom_mask);

            // Apply regression to obtain only one line for each side of the lane
            lane_center_buttom_frame = laneDetector.regression(left_right_lines_center_buttom_frame,
                                                               img_center_buttom_mask);

            // Predict the turn by determining the vanishing point of the the lines
            center_bottom_turn_predictor = 1;
            laneDetector.predictTurn_center_bottom_frame(center_bottom_turn_predictor);
            printf(" - %s", turnAsString[center_bottom_turn_predictor]);
             Plot lane detection
            laneDetector.plotLane(img_center_buttom_mask, lane_center_buttom_frame, turnAsString[turn]);
*/
        }

        //RIGHT BOTTOM FRAME HERE
        right_mask = laneDetector.mask_right_bottom(src);
        right_edged = laneDetector.edgeDetector(right_mask);
        right_buttom_lines = laneDetector.houghLines(right_edged);

        if (!right_buttom_lines.empty()) {
            // Separate lines into left and right lines
            right_lines_separated = laneDetector.right_frame_lineSeparation(right_buttom_lines, right_edged);

            // Apply regression to obtain only one line for each side of the lane
            lane_right_buttom_frame = laneDetector.regression(right_lines_separated, right_mask);

            // Predict the turn by determining the vanishing point of the the lines
            right_frame_turn = Turn::STRAIGHT;
            laneDetector.right_frame_predictTurn(right_frame_turn, right_mask);

            // Plot lane detection
            printf(" - %s", turnAsString[right_frame_turn]);
            laneDetector.plotLane(right_mask, lane_right_buttom_frame, turnAsString[turn], "Right_bottom");
        }

        turn = (left_frame_turn + right_frame_turn) / 2;

        //video_mask->write(img_mask);
        //video_mask->write(img_mask);
        video->write(src);
        //video_edge->write(img_edges);
        //imshow("Video", img_edges);
        //imshow("Video", center);
        //imshow("LEFT_BUTTOM", left_mask);
        //imshow("RIGHT_BUTTOM", right_mask);
        imshow("LEFT_BUTTOM_EDGED", left_edged);
        imshow("RIGHT_BUTTOM_EDGED", right_edged);
        //imshow("CENTER_BUTTOM", img_center_buttom_mask);

        printf("\n%s", turnAsString[turn]);
        //namedWindow("Hello world");
        cvWaitKey(1);
        pthread_mutex_unlock(&frame_mutex);
    }
}

struct Line {
    double slope;
    double length;
};

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

    initTrafficLightSignal();
    initTrafficLightThread();

    turn = STRAIGHT;
    //while(1);
    delay(15000);
    int speedLeft, speedRight;
    Line line;
    while (true) {
        //pthread_mutex_lock(&frame_mutex);
        pthread_mutex_lock(&motor_mutex);

        line = getSlope(lane_left_buttom_frame.at(2), lane_left_buttom_frame.at(3), lane_right_buttom_frame.at(0),
                        lane_right_buttom_frame.at(1));

        if (turn == Turn::STRAIGHT) {
            speedLeft = speed;
            speedRight = speed;
        } else {
            line.slope = abs(line.slope);
            if (turn == Turn::LEFT) {
                speedRight = speed;
                if (line.slope <= 1) {
                    speedLeft = speed * line.slope;
                } else {
                    speedLeft = speed * (1 - (1 / line.slope));
                }
            } else {
                speedLeft = speed;
                if (line.slope <= 1) {
                    speedRight = speed * line.slope;
                } else {
                    speedRight = speed * (1 - (1 / line.slope));
                }
            }
        }

       if(trafficLightStatus != RED_LIGHT)
            pwm_go_smooth(speedLeft, speedRight);

        printf("speed left: %d, speed right: %d, slope: %fl\n", speedLeft, speedRight, line.slope);


        //printf("\nLeft: %lf\nRight : %lf", temp, temp2);
        pthread_mutex_unlock(&motor_mutex);
        //pthread_mutex_unlock(&frame_mutex);
        delay(100);

    }


    return 0;
}