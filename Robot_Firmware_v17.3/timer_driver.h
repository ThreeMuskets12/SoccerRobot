/*
 * timer_driver.h
 *
 * Created: 1/23/2023 5:00:40 PM
 *  Author: evanv
 */ 


#ifndef TIMER_DRIVER_H_
#define TIMER_DRIVER_H_

static struct timer_task task_0;

static char time_to_pid = 0;

static void timer_task_cb(const struct timer_task *const timer_task);

void initialize_task(void);

#endif /* TIMER_DRIVER_H_ */