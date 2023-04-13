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
#include "encoders.h"
#include "dipswitch_and_LEDs.h"

//temp definitions
#define MOTOR_TARGET 3

int main(void)
{
	volatile float current_wheel_speed = 0.0;
	volatile uint8_t dipswitch_state = 0;
	
	uint8_t robot_stop = 0;
	
	atmel_start_init();
	
	//disable LEDs
	set_LEDs(0);
	
	uint8_t robot_ID = 0;
	uint8_t data_store[32];
	
	//initialize wireless data processing
	NPP_init(&robot_ID);
	memset(&data_store[0], 0, sizeof(uint8_t)*32);
	nRF24_init(data_store);
	delay_us(500); //Should be 200 us, setting higher for testing
	nRF24_enter_receive();
	
	//initialize encoders
	encoders_init();
	
	// PWMs
	pwm_enable(&PWM_0);
	pwm_enable(&PWM_1);
	
	//disable motors
	set_pwm_drive_motor(0, 0);
	set_pwm_drive_motor(1, 0);
	set_pwm_drive_motor(2, 0);
	set_pwm_drive_motor(3, 0);
	set_pwm_dribbler_motor(0);
	
	float adc_value_battery_current = 0;
	float adc_value_battery_voltage = 0;
	float adc_value_cap_charge = 0;
	
	//initialize ADCs
	adc_init();
	
	//initialize timers
	initialize_task_PID();
	initialize_task_ADC();
	
	delay_ms(1000);
	
	set_pwm_drive_motor(0, PWM_ZERO);
	set_pwm_drive_motor(1, PWM_ZERO);
	set_pwm_drive_motor(2, PWM_ZERO);
	set_pwm_drive_motor(3, PWM_ZERO);
	
	delay_ms(4000);
	
	while (1) {
		
		//process information sent from hub
		if(nRF_24_is_data_available(1)){ //check to see if data was received
			nRF24_receive_data(data_store);
			//NPP_process(&data_store[0], &robot_ID); //process data
			robot_stop = data_store[0];
			//target_speed_0 = data_store[1] + (data_store[2] << 8);
			//target_speed_1 = data_store[3] + (data_store[4] << 8);
			//target_speed_2 = data_store[5] + (data_store[6] << 8);
			//target_speed_3 = data_store[7] + (data_store[8] << 8);
			//gpio_set_pin_level(LED1, data_store[9]);
			//gpio_set_pin_level(LED2, data_store[10]);
			//gpio_set_pin_level(LED3, data_store[11]);
			memset(&data_store[0], 0, sizeof(uint8_t)*32); //clear data_store array
		}
		/*
		if((robot_stop == 1) || (robot_stop == 2)){
			set_pwm_motor_0(target_speed_0);
			set_pwm_motor_1(target_speed_1);
			set_pwm_motor_2(target_speed_2);
			set_pwm_motor_3(target_speed_3);
			gpio_set_pin_level(LED0, false);
		}
		else{
			set_pwm_motor_0(PWM_ZERO);
			set_pwm_motor_1(PWM_ZERO);
			set_pwm_motor_2(PWM_ZERO);
			set_pwm_motor_3(PWM_ZERO);
			gpio_set_pin_level(LED0, true);
		}*/
		
		//wheel and dribbler time
		if(time_to_pid){
			wheelMotorPID(0, 0, 10, 0);
			//setWheelMotorEffort(50, 150, -225, -100); //for testing motor controllers
			//set_pwm_dribbler_motor(100);
			//robot_stop--;
			time_to_pid = 0;
			/*
			dipswitch_state = dipswitch_read();
			switch(dipswitch_read()){
				case 0b0000: set_pwm_drive_motor(MOTOR_TARGET, PWM_ZERO);
					break;
				case 0b0001: set_pwm_drive_motor(MOTOR_TARGET, 2500);
					break;
				case 0b0011: set_pwm_drive_motor(MOTOR_TARGET, 2550);
					break;
				case 0b0101: set_pwm_drive_motor(MOTOR_TARGET, 2620);
					break;
				case 0b0111: set_pwm_drive_motor(MOTOR_TARGET, PWM_MAX);
					break;
				case 0b1001: set_pwm_drive_motor(MOTOR_TARGET, 2400);
					break;
				case 0b1011: set_pwm_drive_motor(MOTOR_TARGET, 2350);
					break;
				case 0b1101: set_pwm_drive_motor(MOTOR_TARGET, 2280);
					break;
				case 0b1111: set_pwm_drive_motor(MOTOR_TARGET, PWM_MAX_NEG);
					break;
				default: set_pwm_drive_motor(MOTOR_TARGET, PWM_ZERO);
					break;
			}
			
			set_pwm_drive_motor(0, PWM_ZERO);
			set_pwm_drive_motor(1, PWM_ZERO);
			set_pwm_drive_motor(2, PWM_ZERO);
			
			current_wheel_speed = wheel_speed_front_right();
			LED_set((uint8_t)current_wheel_speed);
			*/
		}
		
		//adc time
		if(time_to_adc){
			adc_read(&adc_value_battery_current, ADC_BATTERY_CURRENT);
			adc_read(&adc_value_battery_voltage, ADC_BATTERY_VOLTAGE);
			adc_read(&adc_value_cap_charge, ADC_CAP_CHARGE);
			time_to_adc = 0;
		}
		
	}
	
}