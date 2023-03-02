/*
 * motor_controller.c
 * Motor controller code
 * Created: 11/16/2022 9:58:17 PM
 *  Author: evanv
* Two bytes for a specified motor are input and are handled in the main firmware loop. This Driver implements the
* closed loop speed controller with encoders for the drive motors and the open loop controller for the dribbler.
*/
#include "motor_controller.h"
//motor direction
const char CCW = 1;
const char CW = 0;

//back left encoder
extern long int back_left_counter;
long int back_left_counter_old = 0;
//back right encoder 
extern long int back_right_counter;
long int back_right_counter_old = 0;
//front left encoder
extern long int front_left_counter;
long int front_left_counter_old = 0;
//front right encoder
extern long int front_right_counter;
long int front_right_counter_old = 0;

//sum of error
static float error_sum0=0;
static float error_sum1=0;
static float error_sum2=0;
static float error_sum3=0;

float ppr = 48;
float diameter = 70;

float getRads(){
	
	;
}

//initialize velocity constant once based on the encoder resolution, etc
//[rad/s]
float v_c_r = (2.00*PI)/(ppr*DELTA_T); //convert d(enc) to [rad/s]
//[m/s]
float v_c_l = (PI*diameter)/(ppr*DELTA_T);

//resets error sum of certain PI controller to 0 based on new command
void resetErrorSum(){
	error_sum0=0;
	error_sum1=0;
	error_sum2=0;
	error_sum3=0;
}
//gets current count of encoder based on wheel
// 0 - FR, 1 - FL, 2- BL, 3 - BR
long int getEncoder(int wheel){
	switch(wheel){
		case 0:
			return front_right_counter;
		case 1:
			return front_left_counter;
		case 2:
			return back_left_counter;
		case 3:
			return back_right_counter;
	}
}

//gets old encoder count before PID update
long int getOldEncoder(int wheel){
	switch(wheel){
		case 0:
			return front_right_counter_old;
		case 1:
			return front_left_counter_old;
		case 2:
			return back_left_counter_old;
		case 3:
			return back_right_counter_old;
	}
}

//calculate wheel speeds [rad/s]
//wheel is ID
float calcWheelSpeed(int wheel){
	float current_speed;
	//get encoder counts new and old
	float enc_n = (float) getEncoder(wheel);
	float enc_o = (float) getOldEncoder(wheel);
	current_speed = (float)(enc_n - enc_o)*v_c_r; //[rad/s] or [m/s]
	return current_speed; // [rad/s]
}

//calculates efforts based on error target speed, [rad/s]
//target speeds included from NPP 
//maximum change in angular velocity is 285.71 rad/s
void wheelMotorPID(){
	//calculate error for each motors
	float error0 = velocity_motor_0 - calcWheelSpeed(0);
	float error1 = velocity_motor_1 - calcWheelSpeed(1);
	float error2 = velocity_motor_2 - calcWheelSpeed(2);
	float error3 = velocity_motor_3 - calcWheelSpeed(3);

	//update each error sum
	error_sum0 += error0;
	error_sum1 += error1;
	error_sum2 += error2;
	error_sum3 += error3;

	//check error sums against I-limit and adjust
	//0
	if ((error_sum0)> PID_I_Limit) error_sum0= PID_I_Limit;
	if ((error_sum0)< -PID_I_Limit) error_sum0=-PID_I_Limit;
	//1
	if ((error_sum1)> PID_I_Limit) error_sum1= PID_I_Limit;
	if ((error_sum1)< -PID_I_Limit) error_sum1=-PID_I_Limit;
	//2
	if ((error_sum2)> PID_I_Limit) error_sum2= PID_I_Limit;
	if ((error_sum2)< -PID_I_Limit) error_sum2=-PID_I_Limit;
	//3
	if ((error_sum3)> PID_I_Limit) error_sum3= PID_I_Limit;
	if ((error_sum3)< -PID_I_Limit) error_sum3=-PID_I_Limit;
	
	//compute efforts using PI control
	float effort0 = KP * error0 + KI * error_sum0;
	float effort1 = KP * error1 + KI * error_sum1;
	float effort2 = KP * error2 + KI * error_sum2;
	float effort3 = KP * error3 + KI * error_sum3;
	
	//calculate the general min/max range of effort before mapping to PWM
	//0
	effort0 = (effort0 >= PWM_PER) ? PWM_PER : effort0;
	effort0 = (effort0 <= -PWM_PER) ? -PWM_PER : effort0;
	//1
	effort1 = (effort1 >= PWM_PER) ? PWM_PER : effort1;
	effort1 = (effort1 <= -PWM_PER) ? -PWM_PER : effort1;
	//2
	effort2 = (effort2 >= PWM_PER) ? PWM_PER : effort2;
	effort2 = (effort2 <= -PWM_PER) ? -PWM_PER : effort2;
	//3
	effort3 = (effort3 >= PWM_PER) ? PWM_PER : effort3;
	effort3 = (effort3 <= -PWM_PER) ? -PWM_PER : effort3;
	
	setWheelMotorEffort(effort0, effort1, effort2, effort3);
	
}

//handles magnitude and direction of motor
//FIGURE OUT CCW vs CW HIGH/LOW for motor controller
void setWheelMotorEffort(float effort0, float effort1, float effort2, float effort3){
	//set PWM duty cycle
	set_pwm_motor_0(effort0);
	set_pwm_motor_1(effort1);
	set_pwm_motor_2(effort2);
	set_pwm_motor_3(effort3);
	//set directions for motors based on effort
	gpio_set_pin_level(Motor_0_Dir, ((effort0 > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_1_Dir, ((effort1 > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_2_Dir, ((effort2 > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_3_Dir, ((effort3 > 0) ? CCW : CW));	
}

//target velocity in [rad/s]
void setDribblerMotorEffort(){
	int dribbler_pwm = velocity_motor_dribbler / V_CONSTANT_DRIBBLER;
	gpio_set_pin_level(Dribbler_Motor_Dir, CCW);
	set_pwm_dribbler_motor(dribbler_pwm);	
}