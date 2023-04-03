
/*
 * motor_controller.h
 *
 * Created: 1/22/2023 1:45:03 PM
 *  Author: bensc
 */ 

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "NPP.h"
#include "PWM_driver.h"
#include "encoder_driver.h"

//physical constants
#define W_DIAMETER (0.07) //wheel diameter [m]
#define DRIBBLER_D (1.0) //outer diameter of dribbler module
#define PPR (2048) //variable encoder resolution

//PID constants
#define KP (15.7)
#define KI (4.3)
#define PID_I_Limit (15.7) //need to be determined experimentally
#define FREQ (100) //desired PID freq.[Hz]
#define DELTA_T (0.01) // [s]
//math constants
#define PI (3.1415)
#define PWM_PER (2928) //2928 pulses per pwm period.
#define RATED_LOAD_W_D (545)// rated angular speed for dribbler motor [rad/s]
#define PWM_MAX 2737
#define PWM_ZERO 2445
#define PWM_MAX_NEG 2143
#define V_CONSTANT_DRIBBLER (0.1859) //rad/s /PWM step


//Define encoder interrupts and have quadrature channel processing in separate file

void resetErrorSum(void);

float wheel_speed_front_left();

float wheel_speed_front_left();

float wheel_speed_back_right();

float wheel_speed_back_left();

double dribblerSpeed();

void wheelMotorPID(float target_fr, float target_fl, float target_bl, float target_br);

void setDribblerMotorEffort(void);

void setWheelMotorEffort(float effort_front_right, float effort_front_left, float effort_back_left, float effort_back_right);

//Non-hardcoded versions of wheel speeds:

float calcWheelSpeed(int wheel);

long int getEncoder(int wheel);

long int getOldEncoder(int wheel);

void setOldEncoder(int wheel);