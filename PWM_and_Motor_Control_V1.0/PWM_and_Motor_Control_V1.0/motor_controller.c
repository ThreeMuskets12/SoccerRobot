/*
 * motor_controller.c
 * Motor PI controller code
 * Created: 11/16/2022 9:58:17 PM
 *  Author: evanv
* 
*/

#define WHEEL_D (7.0) //wheel diameter [cm]
#define PI (3.1415926)
#define ENCODER (500) //variable encoder resolution
#define FREQ (100) //desired PID freq.[Hz]
#define DELTA_T (1/FREQ)//time step for measurement [s]
#define SHAFT (0.4) //wheel radius [cm]
#define S_R (0.05714) //speed ratio of 1:17.5 (wheel:shaft)
#define KP (15.7)
#define KI (4.3)
#define PID_I_Limit (15.7) //need to be determined experimentally

// Define encoder interrupts and have quadrature channel processing
int enc_n = 0;
int enc_o = 0;

double error_sum=0;

//initialize velocity constant ONCE based on the encoder resolution, etc
static double v_c = ((PI * WHEEL_D)/ENCODER)*(S_R);

//calculate wheel speed
double wheelSpeed(){
	double current_speed = double(enc_n - enc_o)/(DELTA_T)*v_c;
	return current_speed; // [cm/s]
}

//maps the PWM effort to the duty cycle that is constrained from [146, 292]
int mapPWM(int effort){
	return (effort / 10)*0.72 + 220;
}

//calculates effort based on error target speed, [cm/s]
int computeEffort(double current_speed, double target_speed){
	double error = target_speed - current_speed;
	error_sum += error;
	//check I-limit and adjust
	if ((error_sum)> PID_I_Limit) error_sum= PID_I_Limit;
	if ((error_sum)< -PID_I_Limit) error_sum=-PID_I_Limit;
	
	//compute effort using PI
	double effort = KP * error + KI * error_sum;
	
	//PID values defined from -1000 - 1000
	effort = (effort >= 1000) ? 1000 : effort;
	//configure based on controller
	effort = (effort <= -1000) ? -1000 : effort;
	//map effort to duty-cycle
	return mapPWM((int)effort);
}