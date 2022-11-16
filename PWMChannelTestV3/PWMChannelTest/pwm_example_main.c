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
#include <hal_delay.h>

/* MOVE defines to driver init as externs*/

//PWM channels
#define CHANNEL_0 (0) // Pin D5
#define CHANNEL_1 (1) // Pin D7
#define CHANNEL_2 (2) // Pin D6
#define CHANNEL_3 (3) // Pin A4

/* PWM_PERIOD constant acts a prescaler for the chosen clock source. Use dividers in driver where appropriate to not overload software.*/ 
/* On Atmel Start configure using master clock / 256 = 58.59 Hz */

#define PWM_PERIOD 10000

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
	set_pmw_motor_0(&(PWM_0.device), 0);
	set_pmw_motor_1(&(PWM_0.device), 5000);
	set_pmw_motor_2(&(PWM_0.device), 9999);
	
	while (1) {
		delay_ms(100);
	}
}
