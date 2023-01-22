
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
//Define encoder interrupts and have quadrature channel processing in separate file

void resetErrorSum(void);

int getEncoder(int wheel);

int getOldEncoder(int wheel);

float wheelSpeed(int wheel);

double dribblerSpeed();

int mapPWM(int effort);

void computeEffort(void);