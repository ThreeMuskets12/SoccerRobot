/*
 * PWM_driver.c
 *
 * Created: 11/21/2022 8:45:31 PM
 *  Author: evanv
 */ 
#include "PWM_driver.h"
//pwm_set_parameters(&PWM_0, PWM_PERIOD, pwm_duty);
//struct _pwm_device *const device
//uint8_t channel
//uint32_t period
//uint32_t duty_cycle
//sets duty cycle / period of a specified PWM channel
void set_pwm_channel(struct _pwm_device *const device, uint8_t channel, uint32_t duty_cycle){
	hri_pwm_write_CDTYUPD_reg(device->hw, channel, duty_cycle);
	hri_pwm_write_CPRDUPD_reg(device->hw, channel, PWM_PERIOD);
}

void set_pwm_drive_motor(uint8_t target, uint32_t duty_cycle){
	switch(target){
		case 0: set_pwm_channel(&(PWM_0.device), PWM_0_CHANNEL_0, duty_cycle); //motor 0, PA13
			break;
		case 1: set_pwm_channel(&(PWM_0.device), PWM_0_CHANNEL_1, duty_cycle); //motor 1, PA17
			break;
		case 2: set_pwm_channel(&(PWM_0.device), PWM_0_CHANNEL_2, duty_cycle); //motor 2, PA2
			break;
		case 3: set_pwm_channel(&(PWM_0.device), PWM_0_CHANNEL_3, duty_cycle); //motor 3, PD11
			break;
	}
}

//motor dribbler PA12
void set_pwm_dribbler_motor(uint32_t duty_cycle){
	set_pwm_channel(&(PWM_1.device), PWM_1_CHANNEL_0, duty_cycle);
}