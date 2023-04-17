
/*
 * NPP.c
 *
 * Created: 1/21/2023 3:22:17 PM
 *  Author: bensc
 */ 

#include "NPP.h"

float velocity_motor_fl; //motor 0's target velocity in m/s
float velocity_motor_bl;	//motor 1's target velocity in m/s
float velocity_motor_br; //motor 2's target velocity in m/s
float velocity_motor_fr; //motor 3's target velocity in m/s
float velocity_motor_dribbler; //dribbler motor's target velocity in m/s
uint8_t kicker; //kicker solenoid information
uint8_t chipper; //chipper solenoid information


void NPP_init(uint8_t *robot_ID){
	*robot_ID = 0x0F & read_dipswitch(); //reads dip switches 3-0 to set robot_ID
}

void NPP_process(uint8_t *data, uint8_t *robot_ID, uint16_t *robot_stop){
	if((data[0] & 0x0F) == *robot_ID){ //verifies if message is for this robot
		*robot_stop = (data[0] & 0xF0) >> 4;
		two_byte_to_float(&velocity_motor_fl, data, MOTOR_0_BYTE);
		two_byte_to_float(&velocity_motor_bl, data, MOTOR_1_BYTE);
		two_byte_to_float(&velocity_motor_br, data, MOTOR_2_BYTE);
		two_byte_to_float(&velocity_motor_fr, data, MOTOR_3_BYTE);
		two_byte_to_float(&velocity_motor_dribbler, data, MOTOR_DRIBBLER_BYTE);
		kicker = data[KICKER_BYTE];
		chipper = data[CHIPPER_BYTE];
	}
}

//converts two bytes into floating point (function needed to receive float)
void two_byte_to_float(float *velocity, uint8_t data[], uint8_t byte_location){
	int16_t velocity_mod = data[byte_location] + (data[byte_location + 1] << 8);
	*velocity = (float)velocity_mod/VELOCITY_MODIFIER;
}

//converts floating point into two bytes (function needed to transmit float)
void float_to_two_byte(float velocity, uint8_t data[], uint8_t byte_location){
	int16_t velocity_mod = velocity * VELOCITY_MODIFIER;
	data[byte_location] = velocity_mod & 0x00FF;
	data[byte_location + 1] = velocity_mod >> 8;
}