#include "motion.cpp"
#include <string.h>
#include <stdlib.h>
#include <csignal>
#include <pthread.h>
using namespace std;

#define LANE_DETECTED 1
#define LANE_NOT_DETECTED 0

#define get_left_lane digitalRead(LEFT_TRACER_PIN)
#define get_right_lane digitalRead(RIGHT_TRACER_PIN)

#define LEFT_TRACER_PIN 10
#define RIGHT_TRACER_PIN 11

#define MAX_SPEED 50

double measure_distance(int timeout); 	
long recordPulseLength();


static int speed;
static int Ratio;
static int del;
static int del_ratio;


int left_ir_val;
int right_ir_val;
int horizontal_line_counter = 0;

pthread_mutex_t motor_mutex;

void setUpIRTracer()
{
    pinMode(LEFT_TRACER_PIN, INPUT);
    pinMode(RIGHT_TRACER_PIN, INPUT);
}


void* wait_left_ir_thread(void*)
{
	for(int i = 0; i < 20; i++)
	{
		left_ir_val = get_left_lane;
		if(left_ir_val == 1)
			return NULL;
		delay(10);
	}
	
}

void* wait_right_ir_thread(void*)
{
	for(int i = 0; i < 20; i++)
	{
		right_ir_val = get_right_lane;
		if (right_ir_val == 1)
			return NULL;
		delay(10);
	}
	
}

void* IR_tracer_loop(void*){

	while (1){
		pthread_t left_ir_thread;
		pthread_t right_ir_thread;
	
		if (pthread_create(&left_ir_thread, NULL, wait_left_ir_thread, NULL))
		{
			printf("\nFailed to create a thread!\n");
			return NULL;
		}
	
		if (pthread_create(&right_ir_thread, NULL, wait_right_ir_thread, NULL))
		{
			printf("\nFailed to create a thread!\n");
			return NULL;
		}
		pthread_join(left_ir_thread, NULL);
		pthread_join(right_ir_thread, NULL);
		printf("\nRight IR: %d,  Left ir: %d", left_ir_val, right_ir_val);
		
		if(left_ir_val == 1 && right_ir_val == 1)
		{
			horizontal_line_counter++;
		}
		
		delay(50);
		
	}

}


void signalHandler( int signum ) {

    // cleanup and close up stuff here
    // terminate program
    printf("CNTR+C");

    pwmStop();
    stopDCMotor();

    exit(signum);
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
	
    speed = 50;
    Ratio = 6;
    del_ratio = 2;
    del = 3000;

    printf("Given speed: %d \nGiven turn ratio: %d\n", speed, Ratio);
    if(wiringPiSetup() == -1){
    	printf("wiringPiSetup error\n");
        return 400;
    }
    if (pthread_mutex_init(&motor_mutex, NULL))
    {
        printf("\nFailed to created a mutex!");
        return -1;
    }
    
    // IR sensor pins
    pinMode(LEFT_TRACER_PIN, INPUT);
    pinMode(RIGHT_TRACER_PIN, INPUT);
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT); 

    pwmInitDCMotor();

	pthread_t tracer_thread;
	if(pthread_create(&tracer_thread,NULL, IR_tracer_loop, NULL))
	{
		printf("Failed to create a thread!");
		return -1;
	}

    while(1){
		pthread_mutex_lock(&motor_mutex);
		pwmGo(speed);
		delay(10);
		pthread_mutex_unlock(&motor_mutex);
		
		printf("\nHorizontal line counter: %d", horizontal_line_counter);
		if (horizontal_line_counter > 1)
		{
			break;
		}
   }
   pthread_kill(tracer_thread, 0);
   pwmStop();
   stopDCMotor();

   return 0;
}
