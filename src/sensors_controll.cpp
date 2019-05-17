#include "../include/motion.hpp"
#include <string.h>
#include <stdio.h>
#include<stdlib.h>
#include <csignal>
#include <pthread.h>
#include "opencv2/opencv.hpp"
#include "opencv2/opencv.hpp"


using namespace std;
using namespace cv;
#define get_left_lane digitalRead(LEFT_TRACER_PIN)
#define get_right_lane digitalRead(RIGHT_TRACER_PIN)

#define LEFT_TRACER_PIN 10
#define RIGHT_TRACER_PIN 11

#define TRIG_PIN 28
#define ECHO_PIN 29

#define LEFT_SIGNAL_NUM SIGUSR1
#define RIGHT_SIGNAL_NUM SIGUSR2
#define STOP_SIGNAL SIGRTMIN + 5

#define BLACK 1
#define WHITE 0

#define MAX_SPEED 50

double measure_distance(int timeout);

long recordPulseLength();

void *ultrasonic_loop(void *);

extern int speed;
extern int ultrasonic_is_on;
extern int ir_tracers_are_on;
extern Mat src;

int left_ir_val;
int right_ir_val;
int distance = 0;

pthread_mutex_t motor_mutex;
extern pthread_t tracer_thread;
extern pthread_t ultrasonic_thread;


void* check_if_suddent_pedestrian();
void sensor_setup() {
    // IR sensor pins
    pinMode(LEFT_TRACER_PIN, INPUT);
    pinMode(RIGHT_TRACER_PIN, INPUT);
    //pinMode(LEFT_IR_PIN, INPUT);
    //pinMode(RIGHT_IR_PIN, INPUT); 

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

}


void left_interupt(int sig) {

    printf("\nleft_interupt called!");
    pthread_mutex_lock(&motor_mutex);
    pwmGoBack(speed);
    delay(100);
    pwm_left_point_turn(speed);
    while (!get_left_lane);
    delay(175);
    //pwmStop();
    pthread_mutex_unlock(&motor_mutex);

}

void right_interupt(int sig) {
    printf("\nright_interupt called!");
    pthread_mutex_lock(&motor_mutex);
    pwmGoBack(speed);
    delay(100);
    pwm_right_point_turn(speed);
    while (!get_right_lane);
    delay(175);
    //pwmStop();
    pthread_mutex_unlock(&motor_mutex);
}

void *wait_left_ir_thread(void *) {
    for (int i = 0; i < 25; i++) {
        left_ir_val = get_left_lane;
        if (left_ir_val == WHITE)
            return nullptr;
        delay(5);
    }

}

void *wait_right_ir_thread(void *) {
    for (int i = 0; i < 25; i++) {
        right_ir_val = get_right_lane;
        if (right_ir_val == WHITE)
            return nullptr;
        delay(5);
    }

}

void *IR_tracer_loop(void *) {


    while (1) {
        pthread_t left_ir_thread;
        pthread_t right_ir_thread;

        if (pthread_create(&left_ir_thread, nullptr, wait_left_ir_thread, nullptr)) {
            printf("\nFailed to create a thread!\n");
            return nullptr;
        }

        if (pthread_create(&right_ir_thread, nullptr, wait_right_ir_thread, nullptr)) {
            printf("\nFailed to create a thread!\n");
            return nullptr;
        }
        pthread_join(left_ir_thread, nullptr);
        pthread_join(right_ir_thread, nullptr);

        if (left_ir_val == BLACK && right_ir_val == WHITE) {
            if(ir_tracers_are_on)
                raise(LEFT_SIGNAL_NUM);
        }
        if (left_ir_val == WHITE && right_ir_val == BLACK) {
            if(ir_tracers_are_on)
                raise(RIGHT_SIGNAL_NUM);
        }
        if (left_ir_val == WHITE && right_ir_val == WHITE) {
            //TODO maybe add stop here
            pthread_mutex_lock(&motor_mutex);
            pwmStop();
            delay(400);
            pthread_mutex_unlock(&motor_mutex);
        }

    }

}

void obstacle_signal_handler(int signum) {
    pthread_mutex_lock(&motor_mutex);
    do {
        auto temp = static_cast<int>(measure_distance(30000));
        if (temp != 0)
            distance = temp;
        pwmStop();
        delay(100);
    } while (distance < 10);
    pthread_mutex_unlock(&motor_mutex);
}


double measure_distance(int timeout) {
    delay(10);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long now = micros();
    long travelTimeUsec;

    while (digitalRead(ECHO_PIN) == LOW && micros() - now < timeout);
    travelTimeUsec = recordPulseLength();

    double distanceMeters = 100 * ((travelTimeUsec / 1000000.0) * 340.29) / 2;

    return distanceMeters;
}

long startTimeUsec;
long endTimeUsec;

long recordPulseLength() {
    startTimeUsec = micros();
    while (digitalRead(ECHO_PIN) == HIGH);
    endTimeUsec = micros();
    return endTimeUsec - startTimeUsec;
}

void *ultrasonic_loop(void *) {
    printf("\nUltrasonic loop has started!");
    while (1) {
        auto temp = static_cast<int>(measure_distance(30000));
        if (temp != 0)
            distance = temp;
        if (distance < 10) {
            printf("\nObstacle detected!!");
            if(ultrasonic_is_on)
                raise(SIGRTMIN + 5);
        }
    }
}

int sensor_thread_setup() {


    if (pthread_create(&tracer_thread, nullptr, IR_tracer_loop, nullptr)) {
        printf("Failed to create a thread!");
        exit(-1);
    }


    if (pthread_create(&ultrasonic_thread, nullptr, ultrasonic_loop, nullptr)) {
        printf("Failed to create a thread!");
        exit(-1);
    }

}

void* check_if_suddent_pedestrian(){
    Mat checkFrame_hsv;
    cv::cvtColor(src, checkFrame_hsv, cv::COLOR_BGR2HSV);
    Mat redLowerMask, redHigherMask;
    inRange(checkFrame_hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), redLowerMask);
    inRange(checkFrame_hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), redHigherMask);
    Mat red_hue_image;
    //use hue values from the both ranges (masks)
    addWeighted(redLowerMask, 1.0, redHigherMask, 1.0, 0.0, red_hue_image);
    erode(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size( 9, 9)));
    dilate(red_hue_image, red_hue_image, getStructuringElement(MORPH_ELLIPSE, Size( 3, 3)));
    GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
    int reds = countNonZero(red_hue_image);
    cout << reds << "number of red pixels on the frame" << endl;
    if(reds > 2000)
    {

    }

}