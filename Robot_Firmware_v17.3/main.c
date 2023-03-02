#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "bno085_driver.h"
#include "nRF24_driver.h"
#include "PWM_driver.h"
#include "BQ76925_driver.h"
#include "NPP.h"
#include "motor_controller.h"
#include "timer_driver.h"
#include "esc_calibration.h"
#include "ADC_driver.h"

int main(void)
{
	atmel_start_init();
	encoders_init();
	
	gpio_set_pin_level(LED0, 0);
	gpio_set_pin_level(LED1, 0);
	gpio_set_pin_level(LED2, 0);
	gpio_set_pin_level(LED3, 0);
	
	uint8_t robot_ID = 0;
	uint8_t data_store[32];
	
	NPP_init(&robot_ID);
	memset(&data_store[0], 0, sizeof(uint8_t)*32);
	nRF24_init(data_store);
	delay_us(200); //Should be 200 us, setting higher for testing
	nRF24_enter_receive();
	
	pwm_enable(&PWM_0);
	pwm_enable(&PWM_1);
	
	set_pwm_motor_0(0);
	set_pwm_motor_1(0);
	set_pwm_motor_2(0);
	set_pwm_motor_3(0);
	set_pwm_dribbler_motor(0);
	
	float adc_value_battery_current = 0;
	float adc_value_battery_voltage = 0;
	float adc_value_cap_charge = 0;
	
	adc_init();
	
	while (1) {
		velocity_motor_0 = 10;
		velocity_motor_1 = 10;
		velocity_motor_2 = 10;
		velocity_motor_3 = 10;

		
	}
	
}