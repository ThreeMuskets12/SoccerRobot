/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <hal_init.h>
#include <hpl_pmc.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hpl_pwm_base.h>

/* The channel amount for ADC */
#define IO1_LIGHT_SENS_CH_AMOUNT 1

/* The buffer size for ADC */
#define IO1_LIGHT_SENS_CH0_BUF_SIZE 16

/* The maximal channel number of enabled channels */
#define IO1_LIGHT_SENS_CH_MAX 0

struct adc_async_descriptor IO1_LIGHT_SENS;
#if IO1_LIGHT_SENS_CH_AMOUNT < 1
/* Avoid compiling errors. */
struct adc_async_channel_descriptor IO1_LIGHT_SENS_ch[1];
#warning none of ADC channel is enabled, please check
#else
struct adc_async_channel_descriptor IO1_LIGHT_SENS_ch[IO1_LIGHT_SENS_CH_AMOUNT];
#endif

static uint8_t IO1_LIGHT_SENS_ch0_buf[IO1_LIGHT_SENS_CH0_BUF_SIZE];

#ifdef IO1_LIGHT_SENS_CH_MAX
static uint8_t IO1_LIGHT_SENS_map[IO1_LIGHT_SENS_CH_MAX + 1];
#endif

struct pwm_descriptor PWM_0;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
static void IO1_LIGHT_SENS_init(void)
{
	_pmc_enable_periph_clock(ID_AFEC0);
#ifdef IO1_LIGHT_SENS_CH_MAX
	adc_async_init(&IO1_LIGHT_SENS,
	               AFEC0,
	               IO1_LIGHT_SENS_map,
	               IO1_LIGHT_SENS_CH_MAX,
	               IO1_LIGHT_SENS_CH_AMOUNT,
	               &IO1_LIGHT_SENS_ch[0],
	               (void *)NULL);
#endif
	adc_async_register_channel_buffer(
	    &IO1_LIGHT_SENS, CONF_IO1_LIGHT_SENS_CHANNEL_0, IO1_LIGHT_SENS_ch0_buf, IO1_LIGHT_SENS_CH0_BUF_SIZE);

	gpio_set_pin_function(PD30, GPIO_PIN_FUNCTION_OFF);
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(PD11, MUX_PD11B_PWM0_PWMH0);

	gpio_set_pin_function(PA2, MUX_PA2A_PWM0_PWMH1);

	gpio_set_pin_function(PC19, MUX_PC19B_PWM0_PWMH2);

	gpio_set_pin_function(PC13, MUX_PC13B_PWM0_PWMH3);
}

void PWM_0_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_PWM0);
}

void PWM_0_init(void)
{
	PWM_0_CLOCK_init();
	PWM_0_PORT_init();
	pwm_init(&PWM_0, PWM0, _pwm_get_pwm());
}

void system_init(void)
{
	init_mcu();

	/* Disable Watchdog */
	hri_wdt_set_MR_WDDIS_bit(WDT);

	IO1_LIGHT_SENS_init();

	PWM_0_init();
}
