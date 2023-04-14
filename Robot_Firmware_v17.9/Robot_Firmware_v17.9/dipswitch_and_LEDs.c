/*
 * dipswitch_and_LEDs.c
 *
 * Created: 4/12/2023 7:16:25 PM
 *  Author: bensc
 */ 

#include "dipswitch_and_LEDs.h"

uint8_t read_dipswitch(){
	uint8_t total_state = 0;
	if(gpio_get_pin_level(DipSwitch0)){
		total_state |= 0b00000001;
	}
	if(gpio_get_pin_level(DipSwitch1)){
		total_state |= 0b00000010;
	}
	if(gpio_get_pin_level(DipSwitch2)){
		total_state |= 0b00000100;
	}
	if(gpio_get_pin_level(DipSwitch3)){
		total_state |= 0b00001000;
	}
	if(gpio_get_pin_level(DipSwitch4)){
		total_state |= 0b00010000;
	}
	if(gpio_get_pin_level(DipSwitch5)){
		total_state |= 0b00100000;
	}
	if(gpio_get_pin_level(DipSwitch6)){
		total_state |= 0b01000000;
	}
	if(gpio_get_pin_level(DipSwitch7)){
		total_state |= 0b10000000;
	}
	
	return total_state;
}

void set_LEDs(uint8_t state){
	gpio_set_pin_level(LED0, (0b0001 & state));
	gpio_set_pin_level(LED1, (0b0010 & state));
	gpio_set_pin_level(LED2, (0b0100 & state));
	gpio_set_pin_level(LED3, (0b1000 & state));
}