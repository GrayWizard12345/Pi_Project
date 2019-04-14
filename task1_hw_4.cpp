//
// Created by hamlet on 3/29/19.
//

#include <cstdio>
#include<stdlib.h>
#include <wiringPi.h>
#include "motion.cpp"
#include <csignal>
#include <pthread.h>

#define LEFT_IR_PIN 27
#define RIGHT_IR_PIN 26

int speed;
pthread_mutex_t motor_mutex;
void signalHandler( int signum ) {

    // cleanup and close up stuff here
    // terminate program
    printf("\nCNTR+C\n");

    pwmStop();
    stopDCMotor();

    exit(signum);
}


void* left_ir_handler_thread(void * arg)
{
    int left_ir_value;
	
    while(1)
    {
        pthread_mutex_lock(&motor_mutex);
        while(!(left_ir_value = digitalRead(LEFT_IR_PIN)))
        {
            pwm_right_point_turn(speed);
        }        
        pthread_mutex_unlock(&motor_mutex);
        delay(100);
    }
    
}
void* right_ir_handler_thread(void* arg)
{
	
	int right_ir_value;
	
    while(1)
    {
        pthread_mutex_lock(&motor_mutex);
        while(!(right_ir_value = digitalRead(RIGHT_IR_PIN)))
        {
            pwm_left_point_turn(speed);
        }        
        pthread_mutex_unlock(&motor_mutex);
        delay(100);
    }
    
}


int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    if(wiringPiSetup() == -1)
        return 0;
    pthread_t left_ir_thread;
    pthread_t right_ir_thread;
    
    pwmInitDCMotor();


    pinMode(LEFT_IR_PIN, INPUT); 
    pinMode(RIGHT_IR_PIN, INPUT); 

    speed = atoi(argv[1]);
    
    if (pthread_mutex_init(&motor_mutex, NULL))
    {
        printf("\nFailed to created a mutex!");
        return -1;
    }
    if (pthread_create(&left_ir_thread, NULL, left_ir_handler_thread, NULL ))
    {
        printf("\nFailed to created a left_thread!");
        return -1;
    }
    if (pthread_create(&right_ir_thread, NULL, right_ir_handler_thread, NULL))
    {
        printf("\nFailed to created a right_thread!");
        return -1;
    }
    
    

    
    while(1)
    {
     
        pthread_mutex_lock(&motor_mutex);
        pwmGo(speed);
        pthread_mutex_unlock(&motor_mutex);
        
	}

    pwmStop();
    stopDCMotor();
}
