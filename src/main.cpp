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

pthread_t motor_thread;
pthread_t tracer_thread;
pthread_t ultrasonic_thread;
pthread_t trafficLightThread;
pthread_t crosswalk_thread;

VideoWriter *video;

static int turn;
Status trafficLightStatus = GREEN_LIGHT;

string turnAsString[] = {"L", "S", "R"};

int speed;
int ratio_;

LaneDetector laneDetector;
pthread_mutex_t frame_mutex;
std::map<std::string, std::string> vars;


double slope;

int show_edges;

int ultrasonic_is_on;

int ir_tracers_are_on;

int del;

bool crosswalk_detected = true;

int obstacle_avoidance_left_turn_delay;

int obstacle_avoidance_go_dalay;

int width;

int height;

cv::Mat red_hue_image;

int obstacle_avoidance_right_turn_delay;

void init_vars();

void signalHandler(int signum);

void crosswalk_handler(int sigNum);

void *motor_loop(void *);


void* motor_loop(void*) {

    if (wiringPiSetup() == -1)
        exit(-1);

    pwmInitDCMotor();
    sensor_setup();
    sensor_thread_setup();

    if (pthread_mutex_init(&motor_mutex, nullptr) == -1) {
        printf("\nCannot create mutex!\n");
        exit(-1);
    }



    turn = STRAIGHT;
    delay(3000);
    int speedLeft = speed, speedRight = speed;
    double max_turning = speed / ratio_;
    double speed_per_turn = (speed - max_turning) / (50 - 100);
    while (true) {

        pthread_mutex_lock(&motor_mutex);

//        auto turn_speed = static_cast<int>(speed + (slope - 50) * speed_per_turn);
        int turn_speed = speed / ratio_;
        if (turn_speed > speed)
            turn_speed = speed;

        if (turn == Turn::STRAIGHT) {
            speedLeft = speed;
            speedRight = speed;
        } else if (turn == Turn::LEFT) {
            delay(del);
            speedRight = speed;
            speedLeft = turn_speed;
        } else if (turn == Turn::RIGHT) {
            delay(del);
            speedRight = turn_speed;
            speedLeft = speed;
        }

        if (trafficLightStatus == GREEN_LIGHT) {
            pwm_go_smooth(speedLeft, speedRight);
        }
        //pthread_mutex_unlock(&frame_mutex);
        pthread_mutex_unlock(&motor_mutex);

        printf("\nspeed L: %d, speed R: %d, turn: %d , slope: %lf, traffic_light_status:%d\n", speedLeft, speedRight,
               turn, slope, trafficLightStatus);

        delay(100);

    }

}

void signalHandler(int signum) {

    // cleanup and close up stuff here
    // terminate program
    printf("CNTR+C");
    pwmStop();
    stopDCMotor();
    //Destroy all threads
    pthread_kill(motor_thread, 0);
    pthread_kill(tracer_thread, 0);
    pthread_kill(ultrasonic_thread, 0);
    pthread_kill(trafficLightThread, 0);

    // Closes all the windows
    destroyAllWindows();

    capture.release();
    video->release();

    pwmStop();
    stopDCMotor();


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

int main () {

    signal(SIGINT, signalHandler);
    signal(SIGUSR1, left_interupt);
    signal(SIGUSR2, right_interupt);
    signal(SIGRTMIN + 5, obstacle_signal_handler);
//    signal(ZEBRA_CROSSING_SIGNAL, crosswalk_handler);

    speed = 80;
    ratio_ = 3;
    init_vars();

    printf("Speed %d ", speed);

    if (pthread_create(&motor_thread, nullptr, motor_loop, nullptr)) {

        fprintf(stderr, "Error creating thread\n");
        return 1;

    }

    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;

    if (pthread_mutex_init(&frame_mutex, nullptr) == -1) {
        printf("\nCannot create mutex!\n");
        return -1;
    }


    capture.open(); // activate the raspicam object

    capture.grab(); //grab the scene using raspicam
    capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container

    resize(src, src, Size(width, height));
    cout << "\nWidth:" << src.cols << " Height:" << src.rows << endl;
    src.copyTo(frame);

    video = new VideoWriter("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 7, Size(width, height));

    //Traffic light thread initializations
    initTrafficLightThread();


    //Crosswalk detection thread
    if (pthread_create(&crosswalk_thread, nullptr, look_for_cross_walk, (void *) &img_mask)) {
        printf("\nError wile creating crosswalk thread!\n");
        exit(-1);
    }

    while (true) {

//        pthread_mutex_lock(&frame_mutex);
        capture.grab(); //grab the scene using raspicam
        capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
        resize(src, src, Size(width, height));

        src.copyTo(frame);



        img_mask = laneDetector.mask(src);
        img_edges = laneDetector.edgeDetector(img_mask);
        lines = laneDetector.houghLines(img_edges);


        if (!lines.empty()) {
            left_right_lines = laneDetector.lineSeparation(lines, img_edges);

            lane = laneDetector.regression(left_right_lines, img_edges);

            double vanish_x = laneDetector.predictTurn(turn);

            double full = img_mask.cols;
            slope = vanish_x * 100 / full;

            laneDetector.plotLane(src, lane, turnAsString[turn] + " " + to_string(slope), "Lane Detection");

        } else {
            turn = STRAIGHT;
        }

        if (show_edges) {
            imshow("edges", img_edges);
        }
        video->write(src);

//        imshow("TRAFFIC_LIGHT", red_hue_image);
        cvWaitKey(1);
//        pthread_mutex_unlock(&frame_mutex);
    }
}

void init_vars() {
    read_data();
    speed = stoi(vars["SPEED"]);
    ratio_ = stoi(vars["RATIO"]);
    show_edges = stoi(vars["SHOW_EDGES"]);
    ultrasonic_is_on = stoi(vars["ULTRASONIC_ON"]);
    ir_tracers_are_on = stoi(vars["IR_TRACERS_ON"]);
    del = stoi(vars["BEFORE_TURN_DELAY"]);
    obstacle_avoidance_left_turn_delay = stoi(vars["OBSTACLE_AVOIDANCE_LEFT_TURN_DELAY"]);
    obstacle_avoidance_right_turn_delay = stoi(vars["OBSTACLE_AVOIDANCE_RIGHT_TURN_DELAY"]);
    obstacle_avoidance_go_dalay = stoi(vars["OBSTACLE_AVOIDANCE_GO_DELAY"]);
    width = stoi(vars["WIDTH"]);
    height = stoi(vars["HEIGHT"]);
}