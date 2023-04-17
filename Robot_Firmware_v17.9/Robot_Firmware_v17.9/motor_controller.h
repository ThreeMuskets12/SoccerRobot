
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
#include "encoders.h"

//physical constants
#define W_DIAMETER (0.07) //wheel diameter [m]
#define DRIBBLER_D (1.0) //outer diameter of dribbler module
#define PPR (256) //variable encoder resolution

//PID constants
#define KP (1.0)
#define KP_M (27.25)
#define KI (1.0)
#define KD (0.00)
#define PID_I_Limit (7200)
#define FREQ (100) //desired PID freq.[Hz]
#define DELTA_T (0.01) // [s]
#define DEADBAND (54) //for deadband correction
//math constants
#define PI (3.1415)
#define PWM_PER (5856) //2928 pulses per pwm period.
#define RATED_LOAD_W_D (545)// rated angular speed for dribbler motor [rad/s]
#define PWM_MAX 5474
#define PWM_ZERO 4908
#define PWM_MAX_NEG 4286
#define V_CONSTANT_DRIBBLER (0.1859) //rad/s /PWM step


//Define encoder interrupts and have quadrature channel processing in separate file

void resetErrorSum(void);

float wheel_speed_front_right();

float wheel_speed_front_left();

float wheel_speed_back_right();

float wheel_speed_back_left();

double dribblerSpeed();

int wheelMotorPID(float target_fr, float target_fl, float target_bl, float target_br);

void setDribblerMotorEffort(void);

void setWheelMotorEffort(float effort_front_right, float effort_front_left, float effort_back_left, float effort_back_right);

float float_abs(float number);

//Non-hardcoded versions of wheel speeds:

float calcWheelSpeed(int wheel);

long int getEncoder(int wheel);

long int getOldEncoder(int wheel);

void setOldEncoder(int wheel);

float convert_linear_to_pwm(int flip, float error);