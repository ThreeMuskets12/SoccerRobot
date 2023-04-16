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
#define MOTOR_TARGET 1

float v_a[1000];
float error_a[1000];
float v_b[1000];
float v_c[1000];
float v_d[1000];
int v_counter=0;


int main(void)
{
	//print out velocities
	memset(v_a, 0, 1000);
	memset(error_a, 0, 1000);
	
	volatile float current_wheel_speed = 0.0;
	
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
	
	int pid_done = 0;
	
	volatile float target_speed_0 = 0;
	volatile float target_speed_1 = 0;
	volatile float target_speed_2 = 0;
	volatile float target_speed_3 = 0;
	
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
			two_byte_to_float(target_speed_0, &data_store[0], 1);
			two_byte_to_float(target_speed_1, &data_store[0], 3);
			two_byte_to_float(target_speed_2, &data_store[0], 5);
			two_byte_to_float(target_speed_3, &data_store[0], 7);
			//gpio_set_pin_level(LED1, data_store[9]);
			//gpio_set_pin_level(LED2, data_store[10]);
			//gpio_set_pin_level(LED3, data_store[11]);
			memset(&data_store[0], 0, sizeof(uint8_t)*32); //clear data_store array
		}
		/*
		if(!((robot_stop == 1) || (robot_stop == 2))){
			set_pwm_drive_motor(0, PWM_ZERO);
			set_pwm_drive_motor(1, PWM_ZERO);
			set_pwm_drive_motor(2, PWM_ZERO);
			set_pwm_drive_motor(3, PWM_ZERO);
		}*/
		/*
		switch(0x0F & read_dipswitch()){
			case 0b0000: target_speed = 0;
			resetErrorSum();
			break;
			case 0b0001: target_speed = 0.5;
			resetErrorSum();
			break;
			case 0b0011: target_speed = 2.5;
			resetErrorSum();
			break;
			case 0b0101: target_speed = 5;
			resetErrorSum();
			break;
			case 0b0111: target_speed = 7.5;
			resetErrorSum();
			break;
			case 0b1001: target_speed = -0.5;
			resetErrorSum();
			break;
			case 0b1011: target_speed = -2.5;
			resetErrorSum();
			break;
			case 0b1101: target_speed = -5;
			resetErrorSum();
			break;
			case 0b1111: target_speed = -7.5;
			resetErrorSum();
			break;
			default: target_speed = 0;
			resetErrorSum();
			break;
		}
		
		if(target_speed != target_speed_old){
			pid_done = 0;
		}
		target_speed_old = target_speed;*/
		
		//wheel and dribbler time
		if(time_to_pid &&!pid_done){
			/*pid
			if((robot_stop == 1) || (robot_stop == 2)){
			*/
				pid_done = wheelMotorPID(2.0, 2.0, 2.0, 2.0);
			}
			/*
			else{
				wheelMotorPID(0, 0, 0, 0);
				set_LEDs(1);
			}
			set_LEDs(pid_done);
			//set_pwm_dribbler_motor(100);
			robot_stop--;
			*/
			time_to_pid = 0;
			
		
		
		//adc time
		/*
		if(time_to_adc){
			adc_read(&adc_value_battery_current, ADC_BATTERY_CURRENT);
			adc_read(&adc_value_battery_voltage, ADC_BATTERY_VOLTAGE);
			adc_read(&adc_value_cap_charge, ADC_CAP_CHARGE);
			time_to_adc = 0;
		}*/
		/*if(pid_done){
			//print_pid_values(v_a);
			if(v_counter <= 999){
				volatile float p = -wheel_speed_front_left();
				v_a[v_counter] = p;
			}
		} */
	}
	
	
}

//"prints" the values velocity values during the tuning
void print_pid_values(float *velocities){
	int x=0;
	float c_v;
	int y = 0;
	
	for(x; x<999; x++){
		c_v = *(velocities+x);
		y = 1;
		
	}
	
}