
/*
 * NPP.h
 *
 * Created: 1/21/2023 3:42:04 PM
 *  Author: bensc
 */ 

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MOTOR_0_BYTE 1 //least significant byte for motor 0 data
#define MOTOR_1_BYTE 3 //least significant byte for motor 1 data
#define MOTOR_2_BYTE 5 //least significant byte for motor 2 data
#define MOTOR_3_BYTE 7 //least significant byte for motor 3 data
#define MOTOR_DRIBBLER_BYTE 9 //least significant byte for dribbler motor data
#define KICKER_BYTE 11 //byte for kicker data
#define CHIPPER_BYTE 12 //byte for chipper data
#define VELOCITY_MODIFIER 100 //constant for transffering float to int and back

extern float velocity_motor_0; //motor 0's target velocity
extern float velocity_motor_1;	//motor 1's target velocity
extern float velocity_motor_2; //motor 2's target velocity
extern float velocity_motor_3; //motor 3's target velocity
extern float velocity_motor_dribbler; //dribbler motor's target velocity
extern uint8_t kicker; //kicker information
extern uint8_t chipper; //chipper information

void NPP_init(uint8_t *robot_ID);

void NPP_process(uint8_t *data, uint8_t *robot_ID);

void two_byte_to_float(float *velocity, uint8_t *data, uint8_t byte_location);

void float_to_two_byte(float *velocity, uint8_t *data, uint8_t byte_location);