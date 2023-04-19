/*
 * motor_controller.c
 * Motor controller code
 * Created: 11/16/2022 9:58:17 PM
 *  Author: evanv
* Two bytes for a specified motor are input and are handled in the main firmware loop. This Driver implements the
* closed loop speed controller with encoders for the drive motors and the open loop controller for the dribbler.
*/
#include "motor_controller.h"

//externs for printing/plotting purposes
extern float v_fr[1000];
extern float v_fl[1000];
extern float v_br[1000];
extern float v_bl[1000];
extern int v_counter;

//back left encoder
extern long double back_left_counter;
static long double back_left_counter_old = 0;
//back right encoder 
extern long double back_right_counter;
static long double back_right_counter_old = 0;
//front left encoder
extern long double front_left_counter;
static long double front_left_counter_old = 0;
//front right encoder
extern long double front_right_counter;
static long double front_right_counter_old = 0;

//sum of errors
static float error_sum_front_right=0;
static float error_sum_front_left=0;
static float error_sum_back_left=0;
static float error_sum_back_right=0;
//previous errors
static float prev_error_front_right=0;
static float prev_error_front_left=0;
static float prev_error_back_left=0;
static float prev_error_back_right=0;


//initialize velocity constant once
// angular velocity of motor shaft
float v_c_r = (2.00*PI)/(PPR*DELTA_T); //convert d(enc) to rad/s
// linear velocity of wheel
float v_c_l = (PI*W_DIAMETER)/(PPR*DELTA_T); //convert d(end) to m/s

//resets error sum of PID controller to 0, needs to be called at start of new target velocity
void resetErrorSum(){
	error_sum_front_left=0;
	error_sum_front_right=0;
	error_sum_back_left=0;
	error_sum_back_right=0;
	
	prev_error_front_right=0;
	prev_error_front_left=0;
	prev_error_back_left=0;
	prev_error_back_right=0;
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
	//return v_c_r;
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

//converts linear velocity to the range of pwm for error signal
float convert_linear_to_pwm(int flip, float error){
	volatile float correction;
	if(flip){
		correction = PWM_ZERO - error*KP_M - DEADBAND;
	}
	else{correction = error*KP_M + PWM_ZERO + DEADBAND;}
	
	return correction;
}

//calculates efforts based on error target speed, each in [m/s]
int wheelMotorPID(float target_fr, float target_fl, float target_bl, float target_br){
	//calculate linear speed of each motor
	volatile float fl_vel = wheel_speed_front_left();
	volatile float fr_vel = wheel_speed_front_right();
	volatile float bl_vel = wheel_speed_back_left();
	volatile float br_vel = wheel_speed_back_right();
	//calc error between target and set point
	volatile float error_front_right = target_fr - fr_vel;
	volatile float error_front_left = target_fl - fl_vel;
	volatile float error_back_left = target_bl - bl_vel;
	volatile float error_back_right = target_br - br_vel;
	
	//update each error sum
	error_sum_front_right += error_front_right;
	error_sum_front_left += error_front_left;
	error_sum_back_left += error_back_left;
	error_sum_back_right += error_back_right;
	
	//determine if errors are within accepted range of velocities
	/*if((float_abs(error_front_right) <= 0.1) && (float_abs(error_front_left) <= 0.1) && (float_abs(error_back_left) <= 0.1) && (float_abs(error_back_right) <= 0.1)){
		return 1;
	}*/
	
	//only check front left motor (tuning purposes only)
	/*if((float_abs(error_front_left) <= 0.05)){
		return 1;
	}*/
	//roughly map [min_linear, max_linear] to [min_pwm, max_pwm] (functions as KP)
	error_front_right = convert_linear_to_pwm(0, error_front_right);
	error_front_left = convert_linear_to_pwm(1, error_front_left);
	error_back_right = convert_linear_to_pwm(1, error_back_right);
	error_back_left = convert_linear_to_pwm(1, error_back_left);

	//update variable trackers for plotting afterward
	if(v_counter <= 1000){
		v_fr[v_counter] = fr_vel;
		v_fl[v_counter] = fl_vel;
		v_br[v_counter] = br_vel;
		v_bl[v_counter] = bl_vel;
		v_counter++;
	}
	
	//check error sums against I-limit and adjust
	//front right	
	/*
	if ((error_sum_front_right)> PID_I_Limit) error_sum_front_right= PID_I_Limit;
	if ((error_sum_front_right)< -PID_I_Limit) error_sum_front_right=-PID_I_Limit;
	//front left
	if ((error_sum_front_left)> PID_I_Limit) error_sum_front_left= PID_I_Limit;
	if ((error_sum_front_left)< -PID_I_Limit) error_sum_front_left=-PID_I_Limit;
	//back left
	if ((error_sum_back_left)> PID_I_Limit) error_sum_back_left= PID_I_Limit;
	if ((error_sum_back_left)< -PID_I_Limit) error_sum_back_left=-PID_I_Limit;
	//back right
	if ((error_sum_back_right)> PID_I_Limit) error_sum_back_right= PID_I_Limit;
	if ((error_sum_back_right)< -PID_I_Limit) error_sum_back_right=-PID_I_Limit;
	*/
	
	//compute efforts using PI control
	//some PWM channels read the inverse of the 
	//TODO: add KD for overshoot
	float effort_front_right = KP * error_front_right + KI * error_sum_front_right;
	float effort_front_left = KP * error_front_left - KI * error_sum_front_left;
	float effort_back_left = KP * error_back_left - KI * error_sum_back_left;
	float effort_back_right = KP * error_back_right - KI * error_sum_back_right;
	
	//PID version
	/*
	float effort_front_right = KP * error_front_right + KI * error_sum_front_right + KD * (error_front_right - prev_error_front_right);
	float effort_front_left = KP * error_front_left - KI * error_sum_front_left - KD * (error_front_left - prev_error_front_left);
	float effort_back_left = KP * error_back_left - KI * error_sum_back_left  - KD * (error_back_left - prev_error_back_left);
	float effort_back_right = KP * error_back_right - KI * error_sum_back_right - KD * (error_back_right - prev_error_back_right);
	//update previous errors
	prev_error_front_right = error_front_right;
	prev_error_front_left = error_front_left;
	prev_error_back_right = error_back_right;
	prev_error_back_left = error_back_left;
	*/

	
	//set to middle of deadzone if zero velocity is desired
	if(target_fr == 0){
		effort_front_right = PWM_ZERO;
	}
	
	if(target_fl == 0){
		effort_front_left = PWM_ZERO;
	}
	
	if(target_bl == 0){
		effort_back_left = PWM_ZERO;
	}
	
	if(target_br == 0){
		effort_back_right = PWM_ZERO;
	}
	//threshold PWM signal
	//front right
	/*
	effort_front_right = (effort_front_right >= PWM_MAX) ? PWM_MAX : effort_front_right;
	effort_front_right = (effort_front_right <= PWM_MAX_NEG) ? PWM_MAX_NEG : effort_front_right;
	//front left
	effort_front_left = (effort_front_left >= PWM_MAX) ? PWM_MAX : effort_front_left;
	effort_front_left = (effort_front_left <= PWM_MAX_NEG) ? PWM_MAX_NEG : effort_front_left;
	//back left
	effort_back_left = (effort_back_left >= PWM_MAX) ? PWM_MAX : effort_back_left;
	effort_back_left = (effort_back_left <= PWM_MAX_NEG) ? PWM_MAX_NEG : effort_back_left;
	//back right
	effort_back_right = (effort_back_right >= PWM_MAX) ? PWM_MAX : effort_back_right;
	effort_back_right = (effort_back_right <= PWM_MAX_NEG) ? PWM_MAX_NEG : effort_back_right;
	*/
	
	//update motor efforts
	//setWheelMotorEffort(effort_front_right, effort_front_left, effort_back_left, effort_back_right);
	setWheelMotorEffort(effort_front_right, effort_front_left, PWM_ZERO, PWM_ZERO);

	//still need effort adjustment
	return 0;
	

}

//handles magnitude and direction of motor
//FIGURE OUT CCW vs CW HIGH/LOW for motor controller
void setWheelMotorEffort(float effort_front_right, float effort_front_left, float effort_back_left, float effort_back_right){
	//set PWM duty cycle
	set_pwm_drive_motor(0, effort_front_left);
	set_pwm_drive_motor(1, effort_back_left);
	set_pwm_drive_motor(2, effort_back_right);
	set_pwm_drive_motor(3, effort_front_right);
}

//dribbler target velocity in rad/s
void setDribblerMotorEffort(){
	int dribbler_pwm = velocity_motor_dribbler / V_CONSTANT_DRIBBLER;
	//gpio_set_pin_level(Dribbler_Motor_Dir, CCW);
	set_pwm_dribbler_motor(dribbler_pwm);	
}

float float_abs(float number){
	if(number < 0){
		return -number;
	}
	else{
		return number;
	}
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