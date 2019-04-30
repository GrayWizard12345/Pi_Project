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

// Include guard
#pragma once

#include<wiringPi.h>
#include<softPwm.h>

// pin connections for DC motor
// left DC motors
#define IN1_PIN 1
#define IN2_PIN 4
// right DC motors
#define IN3_PIN 5
#define IN4_PIN 6

// constants for pwm generation for DC motors
#define MAX_SPEED   50
#define MIN_SPEED   0
//#define SOFT_PWM_OUTPUT 
// delay (in ms) to cooldown DC Motors
#define COOLDOWN_DELAY delay(300)
// delay (in ms) to check sensor values
#define SENSOR_CHECK_DELAY delay(500)

// DC motor direction switch functions
void initDCMotor();

void goForward();

void goBackward();

void goLeft();

void goRight();

void stopDCMotor();

// Functions for pwm generation for DC motors
void pwmInitDCMotor();

void pwmGo(int gSpeed);

void pwmStop();

void pwmGoBack(int gSpeed);


// Smooth turn is achieved with difference in speed in left and right motors
void pwm_right_smooth_turn(int speed, int ratio);

void pwm_left_smooth_turn(int speed, int ratio);

void pwm_left_point_turn(int speed);

void pwm_right_point_turn(int speed);

void turn_left_by_slope(int speed, double slope);

void turn_right_by_slope(int speed, double slope);

void pwm_go_smooth(int speedLeft, int speedRight);