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

//motor 0, D5
void set_pwm_motor_0(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_0, duty_cycle);
}
//motor 1, D7
void set_pwm_motor_1(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_1, duty_cycle);
}
//motor 2, D6
void set_pwm_motor_2(struct _pwm_device *const device,  uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_2, duty_cycle);
}
//motor 3, A4 (PC13)
void set_pwm_motor_3(struct _pwm_device *const device,  uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_3, duty_cycle);
}

void config_PWM(void){
	atmel_start_init();
	pwm_enable(&PWM_0);
}
