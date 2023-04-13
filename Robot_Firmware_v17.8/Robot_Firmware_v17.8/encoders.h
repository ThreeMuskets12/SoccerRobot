/*
 * encoder_driver.h
 *
 * Created: 2/23/2023 11:55:36 PM
 *  Author: evanv
 */ 
#ifndef ENCODERSH
#define ENCODERSH
#include "atmel_start_pins.h"
#include "atmel_start.h"

void encoders_init();
void interrupt_front_left();
void interrupt_front_right();
void interrupt_back_left();
void interrupt_back_right();
int front_left_counts();
int front_right_counts();
int back_left_counts();
int back_right_counts();
#endif /* ENCODERSH */