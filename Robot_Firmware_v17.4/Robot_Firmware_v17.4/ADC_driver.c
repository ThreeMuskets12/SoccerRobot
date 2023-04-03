/*
 * ADC_driver.c
 *
 * Created: 1/30/2023 8:54:07 AM
 *  Author: bensc
 */ 

#include "ADC_driver.h"

void adc_init(){ //initializes the ADCs
	adc_sync_enable_channel(&ADC_0, ADC_BATTERY_CURRENT);
	adc_sync_enable_channel(&ADC_0, ADC_BATTERY_VOLTAGE);
	adc_sync_enable_channel(&ADC_0, ADC_CAP_CHARGE);
}

void adc_read(float *adc_value, uint8_t channel){ //reads chosen ADC channel and stores in float
	uint8_t adc_buff[2];
	adc_sync_read_channel(&ADC_0, channel, &adc_buff[0], 2);
	*adc_value = (3.3/4095) * (float)(adc_buff[0] + ((adc_buff[1] & 0x0F) << 8)); //converts 12-bit ADC value to Voltage
}