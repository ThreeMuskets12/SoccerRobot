/*
 * timer_driver.h
 *
 * Created: 1/23/2023 5:00:40 PM
 *  Author: evanv
 */ 


#ifndef TIMER_DRIVER_H_
#define TIMER_DRIVER_H_

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

struct timer_task task_0;

struct timer_task task_1;

extern bool time_to_pid;

extern bool time_to_adc;

void timer_task_PID_cb(const struct timer_task *const timer_task);

void initialize_task_PID(void);

void timer_task_ADC_cb(const struct timer_task *const timer_task);

void initialize_task_ADC(void);

#endif /* TIMER_DRIVER_H_ */