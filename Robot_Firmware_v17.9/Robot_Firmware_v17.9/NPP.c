
/*
 * NPP.c
 *
 * Created: 1/21/2023 3:22:17 PM
 *  Author: bensc
 */ 

#include "NPP.h"

float velocity_motor_0; //motor 0's target velocity in rad/s
float velocity_motor_1;	//motor 1's target velocity in rad/s
float velocity_motor_2; //motor 2's target velocity in rad/s
float velocity_motor_3; //motor 3's target velocity in rad/s
float velocity_motor_dribbler; //dribbler motor's target velocity in rad/s
uint8_t kicker; //kicker solenoid information
uint8_t chipper; //chipper solenoid information


void NPP_init(uint8_t *robot_ID){
	*robot_ID = gpio_get_pin_level(DipSwitch0) + //reads dip switches 3-0 to set robot_ID
				(gpio_get_pin_level(DipSwitch1) << 1) +
				(gpio_get_pin_level(DipSwitch2) << 2) +
				(gpio_get_pin_level(DipSwitch3) << 3);
}

void NPP_process(uint8_t *data, uint8_t *robot_ID){
	if((data[0] & 0xF) == *robot_ID){ //verifies if message is for this robot
		//two_byte_to_float(&velocity_motor_0, data, MOTOR_0_BYTE);
		//two_byte_to_float(&velocity_motor_1, data, MOTOR_1_BYTE);
		//two_byte_to_float(&velocity_motor_2, data, MOTOR_2_BYTE);
		//two_byte_to_float(&velocity_motor_3, data, MOTOR_3_BYTE);
		//two_byte_to_float(&velocity_motor_dribbler, data, MOTOR_DRIBBLER_BYTE);
		kicker = data[KICKER_BYTE];
		chipper = data[CHIPPER_BYTE];
	}
}

//converts two bytes into floating point (function needed to receive float)
void two_byte_to_float(float velocity, uint8_t data[], uint8_t byte_location){
	int16_t velocity_mod = data[byte_location] + (data[byte_location + 1] << 8);
	velocity = (float)velocity_mod/VELOCITY_MODIFIER;
	gpio_set_pin_level(LED1, true);
}

//converts floating point into two bytes (function needed to transmit float)
void float_to_two_byte(float velocity, uint8_t data[], uint8_t byte_location){
	int16_t velocity_mod = velocity * VELOCITY_MODIFIER;
	data[byte_location] = velocity_mod & 0x00FF;
	data[byte_location + 1] = velocity_mod >> 8;
}