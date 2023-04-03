/*
 * encoders.h
 *
 * Created: 2/1/2023 5:28:42 PM
 *  Author: tonat
 */ 


#ifndef ENCODERS_H_
#define ENCODERS_H_
#include "atmel_start_pins.h"

extern int back_left_counter;
extern int back_right_counter;
extern int front_left_counter;
extern int front_right_counter;

void encoders_init();
void interrupt_front_left();
void interrupt_front_right();
void interrupt_back_left();
void interrupt_back_right();
int front_left_counts();
int front_right_counts();
int back_left_counts();
int back_right_counts();
#endif /* ENCODERS_H_ */