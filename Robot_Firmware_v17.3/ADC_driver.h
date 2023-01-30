/*
 * ADC_driver.h
 *
 * Created: 1/30/2023 8:54:27 AM
 *  Author: bensc
 */ 


#ifndef ADC_DRIVER_H_
#define ADC_DRIVER_H_

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define ADC_BATTERY_CURRENT 0
#define ADC_BATTERY_VOLTAGE 5
#define ADC_CAP_CHARGE 7

void adc_init(void);

void adc_read(float *adc_value, uint8_t channel);

#endif /* ADC_DRIVER_H_ */