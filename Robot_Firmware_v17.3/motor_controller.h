
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
#define WHEEL_D (7.0) //wheel diameter [cm]
#define DRIBBLER_D (1.0) //outer diameter of dribbler module
#define ENCODER (1024) //variable encoder resolution
#define SHAFT (0.4) //wheel radius [cm]
#define S_R (0.05714) //speed ratio of 1:17.5 (wheel:shaft)
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
#define V_CONSTANT_DRIBBLER (0.1859) //rad/s /PWM step
//Define encoder interrupts and have quadrature channel processing in separate file

void resetErrorSum(void);

long int getEncoder(int wheel);

long int getOldEncoder(int wheel);

float calcWheelSpeed(int wheel);

double dribblerSpeed();

void wheelMotorPID(void);

void setDribblerMotorEffort(void);

void setWheelMotorEffort(float effort0, float effort1, float effort2, float effort3);