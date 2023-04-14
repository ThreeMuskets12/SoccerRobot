/*
 * ESC_calibration.c
 *
 * Created: 1/25/2023 4:27:51 PM
 *  Author: evanv
 * For calibrating the ESC's. 
 * Note: unnecessary if switched to motor controllers.
 */ 
#include "PWM_driver.h"
#include "esc_calibration.h"

/*
1) no power to ESC, but send 100% speed command to ESC non stop
2) power to ESC while still sending 100% speed command to ESC
3) wait like 8 or more seconds
4) disconnect power to ESC while still sending 100% speed command to ESC
5) wait like 2 or more seconds
6) reconnect power to ESC while still sending 100% speed command to ESC
7) slowly sweep speed command from 100% down to 0% over the course of like 4 or so seconds
8) wait like 5 seconds
9) ESC now good to go
*/
void initializeESC(void){
	gpio_set_pin_level(LED0, 0);
	gpio_set_pin_level(LED1, 0);
	gpio_set_pin_level(LED2, 0);
	gpio_set_pin_level(LED3, 0);
	
	//1 - 100% command, no power
	for(int i=0; i<20000; i++){	
		set_pwm_drive_motor(0, PWM_MAX);
	}
	gpio_set_pin_level(LED0, 1);
	//wait for power supply
	
	//delay_ms(4000);
	
	//2 - 100%, supply power, 8 sec high
	for(int i=0; i<8000; i++){
		set_pwm_drive_motor(0, PWM_MAX);
		delay_ms(1);
	}
	gpio_set_pin_level(LED1, 1);
	//delay_ms(4000);
	
	//4 - disconnected power, 100% command
	for(int i=0; i<1000; i++){
		set_pwm_drive_motor(0, PWM_MAX);
		delay_ms(1);
	}
	//wait 2 seconds
	gpio_set_pin_level(LED2, 1);
	delay_ms(2000);
	//reconnect power
	for(int i=0; i<1000; i++){
		set_pwm_drive_motor(0, PWM_MAX);
	}
	//sweep from 100% to 0% in about 4 seconds
	for(int k=PWM_MAX; k>PWM_ZERO; k--){
		set_pwm_drive_motor(0, k);
		delay_ms(13);
	}
	delay_ms(1000);
	gpio_set_pin_level(LED3, 1);	
}
