#pragma once

#include "motion.hpp"
#include <string.h>
#include <stdio.h>
#include<stdlib.h>
#include <csignal>
#include <pthread.h>

using namespace std;

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

int left_ir_val;
int right_ir_val;
int horizontal_line_counter = 0;
int distance = 0;

pthread_mutex_t motor_mutex;
pthread_t tracer_thread;
pthread_t ultrasonic_thread;

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
    delay(150);
    pwm_left_point_turn(speed * 1.2);
    while (!get_left_lane);
    delay(300);
    //pwmStop();
    pthread_mutex_unlock(&motor_mutex);

}

void right_interupt(int sig) {
    printf("\nright_interupt called!");
    pthread_mutex_lock(&motor_mutex);
    pwmGoBack(speed);
    delay(150);
    pwm_right_point_turn(speed * 1.2);
    while (!get_right_lane);
    delay(300);
    //pwmStop();
    pthread_mutex_unlock(&motor_mutex);
}

void *wait_left_ir_thread(void *) {
    for (int i = 0; i < 25; i++) {
        left_ir_val = get_left_lane;
        if (left_ir_val == WHITE)
            return NULL;
        delay(5);
    }

}

void *wait_right_ir_thread(void *) {
    for (int i = 0; i < 25; i++) {
        right_ir_val = get_right_lane;
        if (right_ir_val == WHITE)
            return NULL;
        delay(5);
    }

}

void *IR_tracer_loop(void *) {


    while (1) {
        pthread_t left_ir_thread;
        pthread_t right_ir_thread;

        if (pthread_create(&left_ir_thread, NULL, wait_left_ir_thread, NULL)) {
            printf("\nFailed to create a thread!\n");
            return NULL;
        }

        if (pthread_create(&right_ir_thread, NULL, wait_right_ir_thread, NULL)) {
            printf("\nFailed to create a thread!\n");
            return NULL;
        }
        pthread_join(left_ir_thread, NULL);
        pthread_join(right_ir_thread, NULL);
        //printf("\nRight IR: %d,  Left ir: %d", left_ir_val, right_ir_val);
        //printf("\nDistance to obstacle: %d", distance);
        if (left_ir_val == BLACK && right_ir_val == WHITE) {
            raise(LEFT_SIGNAL_NUM);
        }
        if (left_ir_val == WHITE && right_ir_val == BLACK) {
            raise(RIGHT_SIGNAL_NUM);
        }
        if (left_ir_val == WHITE && right_ir_val == WHITE) {
//            horizontal_line_counter++;
            delay(200);
        }

        //delay(10);

    }

}

void obstacle_signal_handler(int signum) {
    pthread_mutex_lock(&motor_mutex);
    do {
        int temp = measure_distance(30000);
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
        int temp = measure_distance(30000);
        if (temp != 0)
            distance = temp;
        if (distance < 10) {
            printf("\nObstacle detected!!");
            raise(SIGRTMIN + 5);
        }
    }
}

int sensor_thread_setup() {

    if (pthread_create(&tracer_thread, NULL, IR_tracer_loop, NULL)) {
        printf("Failed to create a thread!");
        exit(-1);
    }

    if (pthread_create(&ultrasonic_thread, NULL, ultrasonic_loop, NULL)) {
        printf("Failed to create a thread!");
        exit(-1);
    }

}
