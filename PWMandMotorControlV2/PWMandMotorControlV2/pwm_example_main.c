/**
 * \file
 *
 * \brief Application implement
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "atmel_start.h"
#include "atmel_start_pins.h"
#include "pwm_example_config.h"
#include <hal_delay.h>

//PWM channels
#define CHANNEL_0 (0) // Pin D5
#define CHANNEL_1 (1) // Pin D7
#define CHANNEL_2 (2) // Pin D6
#define CHANNEL_3 (3) // Pin A4
/* the pwm period for application is 10ms */
#define PWM_PERIOD 1464

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
void set_pmw_motor_0(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_0, duty_cycle);
}
//motor 1, D7
void set_pmw_motor_1(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_1, duty_cycle);
}
//motor 2, D6
void set_pmw_motor_2(struct _pwm_device *const device,  uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_2, duty_cycle);
}
//motor 3, A4
void set_pmw_motor_3(struct _pwm_device *const device,  uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_3, duty_cycle);
}


int main(void)
{
	atmel_start_init();

	pwm_enable(&PWM_0);
	
	//set_pwm_channel(&(PWM_0.device), CHANNEL_0, 5000); //D5
	//set_pwm_channel(&(PWM_0.device), CHANNEL_1, 1000); //D7
	//set_pwm_channel(&(PWM_0.device), CHANNEL_2,2500); //D6
	

	set_pmw_motor_0(&(PWM_0.device), 100); //5
	set_pmw_motor_1(&(PWM_0.device), 700); //7
	set_pmw_motor_2(&(PWM_0.device), 1000); //6



	while (1) {

		delay_ms(100);
	}
}
