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
static long int back_left_counter_old = 0;
//back right encoder 
extern long int back_right_counter;
static long int back_right_counter_old = 0;
//front left encoder
extern long int front_left_counter;
static long int front_left_counter_old = 0;
//front right encoder
extern long int front_right_counter;
static long int front_right_counter_old = 0;

//sum of error
static float error_sum_front_right=0;
static float error_sum_front_left=0;
static float error_sum_back_left=0;
static float error_sum_back_right=0;

//initialize velocity constant once
// angular velocity of motor shaft
float v_c_r = (2.00*PI)/(PPR*DELTA_T); //convert d(enc) to rad/s
// linear velocity of wheel
float v_c_l = (PI*W_DIAMETER)/(PPR*DELTA_T); //convert d(end) to m/s

//resets error sum of certain PI controller to 0 based on new command
void resetErrorSum(){
	error_sum_front_left=0;
	error_sum_front_right=0;
	error_sum_back_left=0;
	error_sum_back_right=0;
}

//hardcoded wheel speed calculations
float wheel_speed_front_right(){
	float current_speed;
	current_speed = (float)(front_right_counter - front_right_counter_old)*v_c_l; //rad/s or m/s
	//set encoder previous encoder count
	front_right_counter_old = front_right_counter;
	return current_speed;
}

float wheel_speed_front_left(){
	float current_speed;
	current_speed = (float)(front_left_counter - front_left_counter_old)*v_c_l; //rad/s or m/s
	//set encoder previous encoder count
	front_left_counter_old = front_left_counter;
	return current_speed;
}

float wheel_speed_back_left(){
	float current_speed;
	current_speed = (float)(back_left_counter - back_left_counter_old)*v_c_l; //rad/s or m/s
	//set encoder previous encoder count
	back_left_counter_old = back_left_counter;
	return current_speed;
}

float wheel_speed_back_right(){
	float current_speed;
	current_speed = (float)(back_right_counter - back_right_counter_old)*v_c_l; //rad/s or m/s
	//set encoder previous encoder count
	back_right_counter_old = back_right_counter;
	return current_speed;
}

//calculates efforts based on error target speed, [rad/s]
//target speeds included from NPP 
//maximum change in angular velocity is 285.71 rad/s
void wheelMotorPID(float target_fr, float target_fl, float target_bl, float target_br){
	//calculate error for each motors
	float error_front_right = target_fr - wheel_speed_front_right();
	float error_front_left = target_fl - wheel_speed_front_left();
	float error_back_left = target_bl - wheel_speed_back_left();
	float error_back_right = target_br - wheel_speed_back_right();

	//update each error sum
	error_sum_front_right += error_front_right;
	error_sum_front_left += error_front_left;
	error_sum_back_left += error_back_left;
	error_sum_back_right += error_sum_back_right;

	//check error sums against I-limit and adjust
	//0
	if ((error_sum_front_right)> PID_I_Limit) error_sum_front_right= PID_I_Limit;
	if ((error_sum_front_right)< -PID_I_Limit) error_sum_front_right=-PID_I_Limit;
	//1
	if ((error_sum_front_left)> PID_I_Limit) error_sum_front_left= PID_I_Limit;
	if ((error_sum_front_left)< -PID_I_Limit) error_sum_front_left=-PID_I_Limit;
	//2
	if ((error_sum_back_left)> PID_I_Limit) error_sum_back_left= PID_I_Limit;
	if ((error_sum_back_left)< -PID_I_Limit) error_sum_back_left=-PID_I_Limit;
	//3
	if ((error_sum_back_right)> PID_I_Limit) error_sum_back_right= PID_I_Limit;
	if ((error_sum_back_right)< -PID_I_Limit) error_sum_back_right=-PID_I_Limit;
	
	//compute efforts using PI control
	float effort_front_right = KP * error_front_right + KI * error_sum_front_right;
	float effort_front_left = KP * error_front_left + KI * error_sum_front_left;
	float effort_back_left = KP * error_back_left + KI * error_sum_back_left;
	float effort_back_right = KP * error_back_right + KI * error_sum_back_right;
	
	//calculate the general min/max range of effort before mapping to PWM
	//0
	effort_front_right = (effort_front_right >= PWM_MAX) ? PWM_MAX : effort_front_right;
	effort_front_right = (effort_front_right <= -PWM_MAX_NEG) ? -PWM_MAX_NEG : effort_front_right;
	//1
	effort_front_left = (effort_front_left >= PWM_MAX) ? PWM_MAX : effort_front_left;
	effort_front_left = (effort_front_left <= -PWM_MAX_NEG) ? -PWM_MAX_NEG : effort_front_left;
	//2
	effort_back_left = (effort_back_left >= PWM_MAX) ? PWM_MAX : effort_back_left;
	effort_back_left = (effort_back_left <= -PWM_MAX_NEG) ? -PWM_MAX_NEG : effort_back_left;
	//3
	effort_back_right = (effort_back_right >= PWM_MAX) ? PWM_MAX : effort_back_right;
	effort_back_right = (effort_back_right <= -PWM_MAX_NEG) ? -PWM_MAX_NEG : effort_back_right;
	
	setWheelMotorEffort(effort_front_right, effort_front_left, effort_back_left, effort_back_right);
	
}

//handles magnitude and direction of motor
//FIGURE OUT CCW vs CW HIGH/LOW for motor controller
void setWheelMotorEffort(float effort_front_right, float effort_front_left, float effort_back_left, float effort_back_right){
	//set PWM duty cycle
	set_pwm_motor_0(effort_front_right);
	set_pwm_motor_1(effort_front_left);
	set_pwm_motor_2(effort_back_left);
	set_pwm_motor_3(effort_back_right);
	//set directions for motors based on effort
	/*gpio_set_pin_level(Motor_0_Dir, ((effort_front_right > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_1_Dir, ((effort_front_left > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_2_Dir, ((effort_back_left > 0) ? CCW : CW));
	gpio_set_pin_level(Motor_3_Dir, ((effort_back_right > 0) ? CCW : CW));	*/
}

//dribbler target velocity in rad/s
void setDribblerMotorEffort(){
	int dribbler_pwm = velocity_motor_dribbler / V_CONSTANT_DRIBBLER;
	gpio_set_pin_level(Dribbler_Motor_Dir, CCW);
	set_pwm_dribbler_motor(dribbler_pwm);	
}

/*
	Non-hardcoded versions of encoder updates below (not in use currently)
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

	//sets old encoder count after PID update
	void setOldEncoder(int wheel){
		switch(wheel){
			case 0:
			front_right_counter_old = front_right_counter;
			case 1:
			front_left_counter_old = front_left_counter;
			case 2:
			back_left_counter_old = back_left_counter;
			case 3:
			back_right_counter_old = back_right_counter;
		}
	}

	//calculate wheel speeds (units depend on velocity constant chosen)
	//wheel is ID
	float calcWheelSpeed(int wheel){
		float current_speed;
		//get encoder counts new and old
		float enc_n = (float) getEncoder(wheel);
		float enc_o = (float) getOldEncoder(wheel);
		current_speed = (float)(enc_n - enc_o)*v_c_l; //rad/s or m/s
		//set encoder previous encoder count
		setOldEncoder(wheel);
		return current_speed;
	}
*/