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

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	//-----------------IMU 
	/*Create a float array of length 4 to hold the real, i, j, and k quaternion coefficients that get_IMU_quaternion() returns
	and then initialize it to zero */
	
	//float imu_vector_buffer[4];
	//memset(&imu_vector_buffer[0], 0, sizeof(float)*4);
	
	/*Initialize the IMU, making sure to pass in the address of the start of the float array that will hold the quaternion coefficients*/
	
	//init_IMU(&imu_vector_buffer[0]);
	//----------------END IMU
	
	//---------------NPP
	uint8_t robot_ID;
	NPP_init(&robot_ID);
	
	gpio_set_pin_level(LED0, robot_ID & 0b0001);
	gpio_set_pin_level(LED1, robot_ID & 0b0010);
	gpio_set_pin_level(LED2, robot_ID & 0b0100);
	gpio_set_pin_level(LED3, robot_ID & 0b1000);
	//------------------END NPP
	
	//------------------nRF24
	uint8_t data_store[32];
	memset(&data_store[0], 0, sizeof(uint8_t)*32);
	nRF24_init(data_store);
	delay_us(500); //Should be 200 us, setting higher for testing
	nRF24_enter_receive();
	//------------------END nRF24
	
	//------------------PWM
	pwm_enable(&PWM_0);
	pwm_enable(&PWM_1);
	/*
	set_pwm_motor_0(&(PWM_0.device), 878);
	set_pwm_motor_1(&(PWM_0.device), 878);
	set_pwm_motor_2(&(PWM_0.device), 878);
	set_pwm_motor_3(&(PWM_0.device), 878);
	set_pwm_dribbler_motor(&(PWM_1.device), 878);
	*/
	//----------------END PWM
	
	adc_sync_enable_channel(&ADC_0, 0);
    uint8_t buff[2];
    //second param = channel 
    //length of buffer
    //adc_sync_read_channel(&ADC_0, 0, buff, 2);
    int32_t value; // = buff[0] + ((buff[1] & 0x0F) << 8);
	
	//float* v_buff;
	//init_IMU(v_buff);
	uint8_t values[2];//location to place register values in
	char* buffer;
	char* buffer2;
	char* buffer3;
	uint8_t* ptr;
	//set_ref_voltage(buffer, 1);
	//setup_BQ7_for_adc(buffer2, CELL_1);
	//setup_BQ7_for_adc(buffer3, CELL_2);
	//adc_sync_enable_channel(&ADC_0, 0);
	//float y = v_cell();
	
	//setup_BQ7_for_adc(buffer, CELL_2);
	//setup_BQ7_for_adc(buffer, CELL_3);
	
	//initializeESC();
	
	while (1) {
		//gpio_set_pin_level(LED0, 0);
		
		if(gpio_get_pin_level(DipSwitch7)){
			set_pwm_motor_0(878);
		}
		else if(gpio_get_pin_level(DipSwitch6)){
			set_pwm_motor_0(1171);
		}
		else if(gpio_get_pin_level(DipSwitch5)){
			set_pwm_motor_0(600);
		}
		
		//wheel and dribbler time
		//if(time_to_pid){
		//	wheelMotorPID();
		//}
		/*
		gpio_set_pin_level(LED0, gpio_get_pin_level(DipSwitch0));
		gpio_set_pin_level(LED1, gpio_get_pin_level(DipSwitch1));
		gpio_set_pin_level(LED2, gpio_get_pin_level(DipSwitch2));
		gpio_set_pin_level(LED3, gpio_get_pin_level(DipSwitch3));
		*/
				
		//adc_sync_read_channel(&ADC_0, 0, buff, 2);
		//value = buff[0] + ((buff[1] & 0x0F) << 8);
			
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
		
		//if(nRF_24_is_data_available(1)){ //There is data for me to collect :)
			//nRF24_receive_data(data_store);
			//for(int i = 0; i < 32; i++){
				//printf("0x%02x\r\n", data_store[i]);
			//}
			//printf("%s\r\n",data_store); //If you want to print the received number to console you would have to add 48!!!
			//uint8_t m_period;
			//m_period = data_store[0];
			//set_pmw_motor_0(&(PWM_0.device), m_period);		
			/*	
			gpio_set_pin_level(LED0, data_store[0] & 0b0001);
			gpio_set_pin_level(LED1, data_store[0] & 0b0010);
			gpio_set_pin_level(LED2, data_store[0] & 0b0100);
			gpio_set_pin_level(LED3, data_store[0] & 0b1000);
			memset(&data_store[0], 0, sizeof(uint8_t)*32);
			*/
			
			//NPP_process(&data_store[0], &robot_ID);
			//memset(&data_store[0], 0, sizeof(uint8_t)*32);
			
			
		//}
		//get_IMU_quaternion();
		////printf("%s", imu_vector_buffer[0]);
		//float f = 32.3;
		//set_ref_voltage(buffer, 1);
	
	}
	
}
