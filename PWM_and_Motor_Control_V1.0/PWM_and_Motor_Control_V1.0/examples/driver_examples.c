/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

static void convert_cb_IO1_LIGHT_SENS_CHANNEL_0(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

/**
 * Example of using IO1_LIGHT_SENS to generate waveform.
 */
void IO1_LIGHT_SENS_example(void)
{
	adc_async_register_callback(
	    &IO1_LIGHT_SENS, CONF_IO1_LIGHT_SENS_CHANNEL_0, ADC_ASYNC_CONVERT_CB, convert_cb_IO1_LIGHT_SENS_CHANNEL_0);
	adc_async_enable_channel(&IO1_LIGHT_SENS, CONF_IO1_LIGHT_SENS_CHANNEL_0);
	adc_async_start_conversion(&IO1_LIGHT_SENS);
}

static void period_cb_PWM_0(const struct pwm_descriptor *const descr)
{
	/* period interrupt */
}
/**
 * Example of using PWM_0.
 */
void PWM_0_example(void)
{
	pwm_register_callback(&PWM_0, PWM_PERIOD_CB, period_cb_PWM_0);
	pwm_enable(&PWM_0);
}
