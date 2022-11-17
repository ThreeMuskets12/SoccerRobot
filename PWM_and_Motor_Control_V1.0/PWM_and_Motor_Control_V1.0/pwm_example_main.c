/**
 *
 * Created: 11/16/2022 9:58:17 PM
 *  Author: evanv
 */
#include "atmel_start.h"
#include "atmel_start_pins.h"
#include <hal_delay.h>

/* MOVE defines to driver init as externs*/

//PWM channels
#define CHANNEL_0 (0) // Pin D5
#define CHANNEL_1 (1) // Pin D7
#define CHANNEL_2 (2) // Pin D6
#define CHANNEL_3 (3) // Pin A4

/* On Atmel Start configure using master clock / 1024 = 14648 Hz */
/* PWM_PERIOD breaks up the period even further. 14648 / 732 = 200Hz. */
/* The ESC accepts 1000us - 2000us pulses, where 1000us = -100% effort, 1500us = 0%, 2000us = 100%. Therefore, the controller is constrained for this*/
#define PWM_PERIOD (732)

//sets duty cycle / period of a specified PWM channel on a specified instance
//use Atmel start to configure both instances
//channel may be from 0-3
void set_pwm_channel(struct _pwm_device *const device, uint8_t channel, uint32_t duty_cycle){
	if((channel < 4) && (channel >= 0)){
		hri_pwm_write_CDTYUPD_reg(device->hw, channel, duty_cycle); //set duty-cycle for channel
		hri_pwm_write_CPRDUPD_reg(device->hw, channel, PWM_PERIOD); //set period for channel
	}
}

/*
 * Functions for each individual motor for sake of processing time.
*/
//motor 0
void set_pmw_motor_0(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_0, duty_cycle);
}
//motor 1
void set_pmw_motor_1(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_1, duty_cycle);
}
//motor 2
void set_pmw_motor_2(struct _pwm_device *const device,  uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_2, duty_cycle);
}
//motor 3
void set_pmw_motor_3(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_3, duty_cycle);
}

int main(void)
{
	atmel_start_init();

	pwm_enable(&PWM_0);
	set_pmw_motor_0(&(PWM_0.device), 146);
	set_pmw_motor_1(&(PWM_0.device), 220);
	set_pmw_motor_2(&(PWM_0.device), 292);
	
	while (1) {
		delay_ms(100);
	}
}
