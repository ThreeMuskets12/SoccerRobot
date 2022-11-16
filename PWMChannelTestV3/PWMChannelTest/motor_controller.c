/* 
* Motor PI controller code
*/

#define WHEEL_D (7.0) //wheel diameter (const)
#define PI (3.1415926)
#define ENCODER (500) //variable encoder resolution
#define FREQ (100) //desired PID freq.
#define DELTA_T (1/FREQ)//time step for measurement
#define SHAFT (0.4) //wheel radius
#define S_R (0.05714) //speed ratio of 1:17.5 (wheel:shaft)
#define KP (110)
#define KI (30)
#define PID_I_Limit (110) //need to be determined experimentally

// Define encoder interrupts and have quadrature channel processing
int enc_n = 0;
int enc_o = 0;

double error_sum=0;
//initialize velocity constant ONCE based on the encoder resolution, etc
static double v_c =(PI*SHAFT) / (ENCODER) * (S_R);

//calculate wheel speed
double wheelSpeed(){
	double current_speed = double(enc_n - enc_o)/(DELTA_T)*v_c;
	return current_speed;
}

//calculates effort based on error target speed
double computeEffort(double current_speed, double target_speed){
	double error = target_speed - current_speed;
	error_sum += error;
	
	//check I-limit and adjust
	if ((error_sum)> PID_I_Limit) error_sum= PID_I_Limit;
	if ((error_sum)<-PID_I_Limit) error_sum=-PID_I_Limit;
	
	//compute PWM using PI
	float PWM = KP * error + KI * error_sum;
	
	//will need to be determined upon looking at Atmel chip PWM implementation
	PWM = (PWM >= 7995) ? 7995 : PWM;
	PWM = (PWM <= -7995) ? -7995 : PWM;
	
	return PWM;
}