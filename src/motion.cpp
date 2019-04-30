/**
  * Paradox v0.0.1 by PARALLAX
  * - Autonomus Car
  *
  * Raspberry Pi 3 (QuadCore Processer, 1GB RAM)
  * * Wifi, Bluetooth, LAN port, 4 USBv2.0 ports, HDMI port
  * 
  * Additional hardware:
  * Raspberry Pi Camera: 1
  * DC motors: 4
  * IR transrecievers: 2
  * IR Line Tracers: 2
  * Ultrasonic Sensor: 1
  *	
  * Copyright 2019 © PARALLAX Team
  *
  * Expected GNU v2.0 licence release in 2020
  */

#include <cstdio>
#include "../include/motion.hpp"

void initDCMotor() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, HIGH);

}

void goForward() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void goBackward() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void goLeft() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
}

void goRight() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}

void stopDCMotor() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

// Motor Initialize
void pwmInitDCMotor() {
    pinMode(IN1_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN2_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN3_PIN, SOFT_PWM_OUTPUT);
    pinMode(IN4_PIN, SOFT_PWM_OUTPUT);
    softPwmCreate(IN1_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN2_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN3_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(IN4_PIN, MIN_SPEED, MAX_SPEED);
    //printf("pwm Init DC Motor\n");
}

// DC motor go
void pwmGo(int gSpeed) {
    softPwmWrite(IN1_PIN, gSpeed);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, gSpeed - 0.04 * gSpeed);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    //printf("PWM go\n");
}

// DC motor go back
void pwmGoBack(int gSpeed) {
    softPwmWrite(IN1_PIN, MIN_SPEED);
    softPwmWrite(IN2_PIN, gSpeed);
    softPwmWrite(IN3_PIN, MIN_SPEED);
    softPwmWrite(IN4_PIN, gSpeed);
    //printf("PWM go\n");
}

// DC motor stop
void pwmStop() {
    softPwmWrite(IN1_PIN, MIN_SPEED);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, MIN_SPEED);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    printf("Pwm Stop\n");
}

// Smooth turn is achieved with difference in speed in left and right motors
void pwm_right_smooth_turn(int speed, int ratio) {
    softPwmWrite(IN1_PIN, speed);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, speed / ratio);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    printf("PWM go_1\n");
}

void pwm_left_smooth_turn(int speed, int ratio) {
    softPwmWrite(IN1_PIN, speed / ratio);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, speed);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    printf("PWM go_1\n");
}

void pwm_left_point_turn(int speed) {
    softPwmWrite(IN1_PIN, MIN_SPEED);
    softPwmWrite(IN2_PIN, speed);
    softPwmWrite(IN3_PIN, speed);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    printf("PWM point left turn\n");
}

void pwm_right_point_turn(int speed) {
    softPwmWrite(IN1_PIN, speed);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, MIN_SPEED);
    softPwmWrite(IN4_PIN, speed);
    //printf("PWM point right turn\n");
}

void turn_left_by_slope(int speed, double slope) {
    if (slope > 1)
        slope -= 1;
    softPwmWrite(IN1_PIN, int(speed * slope));
    softPwmWrite(IN2_PIN, LOW);
    softPwmWrite(IN3_PIN, speed);
    softPwmWrite(IN4_PIN, LOW);
    //printf("PWM go_1\n");
}

void turn_right_by_slope(int speed, double slope) {
    if (slope > 1)
        slope -= 1;
    softPwmWrite(IN1_PIN, speed);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, int(speed * slope));
    softPwmWrite(IN4_PIN, MIN_SPEED);
    //printf("PWM go_1\n");
}

// Smooth turn is achieved with difference in speed in left and right motors
void pwm_go_smooth(int speedLeft, int speedRight) {
    softPwmWrite(IN1_PIN, speedLeft);
    softPwmWrite(IN2_PIN, MIN_SPEED);
    softPwmWrite(IN3_PIN, speedRight);
    softPwmWrite(IN4_PIN, MIN_SPEED);
    printf("\n\t----Go called!----");
}