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
#include <atmel_start.h>

static void timer_task_cb(const struct timer_task *const timer_task){
	time_to_pid = !time_to_pid;
	
}

void initialize_task(void){
	task_0.interval = 1; //amount of counts before repeating task
	task_0.cb = timer_task_cb;
	task_0.mode = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &task_0);
	timer_start(&TIMER_0);
}