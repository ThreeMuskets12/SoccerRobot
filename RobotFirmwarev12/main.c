#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include "bno085_driver.h"
#include "nRF24_driver.h"

#define CHANNEL_0 (0) // Pin D5

/* On Atmel Start configure using master clock / 1024 = 14648 Hz */
/* PWM_PERIOD breaks up the period even further. 14648 / 732 = 200Hz. */
/* The ESC accepts 1000us - 2000us pulses, where 1000us = -100% effort, 1500us = 0%, 2000us = 100%. Therefore, the controller is constrained for this*/
#define PWM_PERIOD (732)

//sets duty cycle / period of a specified PWM channel on a specified instance
//use Atmel start to configure both instances
//channel may be from 0-3
void set_pwm_channel(struct _pwm_device *const device, uint8_t channel, uint32_t duty_cycle){
	if((channel < 4) && (channel >= 0)){
		hri_pwm_write_CPRDUPD_reg(device->hw, channel, PWM_PERIOD); //set period for channel
		hri_pwm_write_CDTYUPD_reg(device->hw, channel, duty_cycle); //set duty-cycle for channel
	}
}

/*
 * Functions for each individual motor for sake of processing time.
*/
//motor 0
void set_pmw_motor_0(struct _pwm_device *const device, uint32_t duty_cycle){
	set_pwm_channel(device, CHANNEL_0, duty_cycle);
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	/*Create a float array of length 4 to hold the real, i, j, and k quaternion coefficients that get_IMU_quaternion() returns
	and then initialize it to zero */
	
	//float imu_vector_buffer[4];
	//memset(&imu_vector_buffer[0], 0, sizeof(float)*4);
	
	/*Initialize the IMU, making sure to pass in the address of the start of the float array that will hold the quaternion coefficients*/
	
	//init_IMU(&imu_vector_buffer[0]);
	
	uint8_t data_store[32];
	memset(&data_store[0], 0, sizeof(uint8_t)*32);
	nRF24_init(data_store);
	delay_us(200); //Should be 200 us, setting higher for testing
	nRF24_enter_receive();
	while (1) {
		/*There are two ways to read information from the IMU: See the driver manual for more information. Shown here is mode 0,
		where the higher level robot code (which would typically run within this while(1) loop in main.c) calls get_IMU_quaternion()
		whenever it wants to fetch the most recent quaternion coefficients. In this case, the float array is only updated with the
		latest quaternion coefficients when get_IMU_quaternion() is called. So in this below example, we repeatedly call get_IMU_quaternion(),
		representing a situation in which the main robot code would want to update the quaternion coefficients once per loop of the main robot code.
		
		The other way (mode 1) to read information from the IMU is to enable interrupt mode. The driver manual explains how to do this. When in
		interrupt mode, the main loop is interrupted every time the IMU has new coefficients ready, and the MCU will read this data in and store it
		in the float array before returning back to continue executing the main loop. In this mode, the higher level robot code never needs to
		call get_IMU_quaternion(), and instead, can just assume that the most recent coefficients are ALWAYS up to date inside of the float array.
		This is not recommended, because this will tie up a lot of CPU clock cycles handling the IMU interrupts, when instead the most recent data can
		simply be fetched right before it is needed. (Which may not be very often depending on how the higher level robot code is written! */
		 
		/*This delay was chosen because in the real robot[ code, IMU data will not need to be read every single loop of the code, and the
		IMU is set to update at a period of 50ms currently (20 Hz), so a delay of 55ms is fast enough that we shouldn't be missing new
		IMU quaternion coefficient updates (only relevant for the demo code, in reality it doesn't matter if we miss new updates, as we
		should only fetch when needed, regardless of whether we have missed some or not), while being slow enough that we aren't asking the
		IMU for new quaternion coefficients before it is ready with the next set of them. (also really only relevant for the demo code, in
		reality it doesn't matter if we ask for coefficients before the IMU is ready with new ones, as the function will just return the last
		valid coefficients it collected in the event that the IMU isn't ready with new ones) */
		/*if(nRF_24_is_data_available(1)){ //There is data for me to collect :)
			nRF24_receive_data(&data_store[0]);
			for(int i = 0; i < 32; i++){
				printf("0x%02x\r\n", data_store[i]);
			}
			printf("%s\r\n",data_store); //If you want to print the received number to console you would have to add 48!!!
			uint8_t m_period;
			m_period = data_store[0];
			set_pmw_motor_0(&(PWM_0.device), m_period);
			memset(&data_store[0], 0, sizeof(uint8_t)*32);
		}*/
	}
}
