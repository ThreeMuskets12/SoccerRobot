/*
 * timer_driver.c
 *
 * Created: 1/23/2023 4:59:09 PM
 *  Author: evanv
 *  Defines and initializes tasks/timer
 * timer resolution is defined in atmel start
 * resolution is currently 1ms
 */ 
#include "timer_driver.h"

bool time_to_pid = 0; //flag for when to run PID for motors
bool time_to_adc = 0; //flag for when to read ADCs

void timer_task_PID_cb(const struct timer_task *const timer_task){ //sets flag for PID
	time_to_pid = 1;
}

void initialize_task_PID(void){ //initializes timer 0
	task_0.interval = 10; //amount of counts before repeating task
	task_0.cb = timer_task_PID_cb; //tells timer what task to execute
	task_0.mode = TIMER_TASK_REPEAT; //tells timer to repeat task

	timer_add_task(&TIMER_0, &task_0);
	timer_start(&TIMER_0);
}

void timer_task_ADC_cb(const struct timer_task *const timer_task){ //sets flag for ADC
	time_to_adc = 1;
}

void initialize_task_ADC(void){ //initializes timer 1
	task_1.interval = 500; //amount of counts before repeating task
	task_1.cb = timer_task_ADC_cb; //tells timer what task to execute
	task_1.mode = TIMER_TASK_REPEAT; //tells timer to repeat task

	timer_add_task(&TIMER_1, &task_1);
	timer_start(&TIMER_1);
}