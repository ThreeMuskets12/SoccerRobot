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

/**
 * Example of using ADC_0 to generate waveform.
 */
void ADC_0_example(void)
{
	uint8_t buffer_ch0[2];
	uint8_t buffer_ch5[2];
	uint8_t buffer_ch7[2];

	adc_sync_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_0);
	adc_sync_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_5);
	adc_sync_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_7);

	while (1) {
		adc_sync_read_channel(&ADC_0, CONF_ADC_0_CHANNEL_0, buffer_ch0, 2);
		adc_sync_read_channel(&ADC_0, CONF_ADC_0_CHANNEL_5, buffer_ch5, 2);
		adc_sync_read_channel(&ADC_0, CONF_ADC_0_CHANNEL_7, buffer_ch7, 2);
	}
}

static void button_on_PB1_pressed(void)
{
}

/**
 * Example of using EXTERNAL_IRQ_1
 */
void EXTERNAL_IRQ_1_example(void)
{
	ext_irq_register(PIO_PB1_IDX, button_on_PB1_pressed);
}

static void button_on_PA15_pressed(void)
{
}
static void button_on_PA22_pressed(void)
{
}
static void button_on_PA23_pressed(void)
{
}
static void button_on_PA24_pressed(void)
{
}
static void button_on_PA25_pressed(void)
{
}

/**
 * Example of using EXTERNAL_IRQ_0
 */
void EXTERNAL_IRQ_0_example(void)
{
	ext_irq_register(PIO_PA15_IDX, button_on_PA15_pressed);
	ext_irq_register(PIO_PA22_IDX, button_on_PA22_pressed);
	ext_irq_register(PIO_PA23_IDX, button_on_PA23_pressed);
	ext_irq_register(PIO_PA24_IDX, button_on_PA24_pressed);
	ext_irq_register(PIO_PA25_IDX, button_on_PA25_pressed);
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

static void period_cb_PWM_1(const struct pwm_descriptor *const descr)
{
	/* period interrupt */
}
/**
 * Example of using PWM_1.
 */
void PWM_1_example(void)
{
	pwm_register_callback(&PWM_1, PWM_PERIOD_CB, period_cb_PWM_1);
	pwm_enable(&PWM_1);
}

/**
 * Example of using SPI_0 to write "Hello World" using the IO abstraction.
 */
static uint8_t example_SPI_0[12] = "Hello World!";

void SPI_0_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);

	spi_m_sync_enable(&SPI_0);
	io_write(io, example_SPI_0, 12);
}

/**
 * Example of using TIMER_0.
 */
static struct timer_task TIMER_0_task1, TIMER_0_task2;

static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}

/**
 * Example of using TIMER_1.
 */
static struct timer_task TIMER_1_task1, TIMER_1_task2;

static void TIMER_1_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_1_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_1_example(void)
{
	TIMER_1_task1.interval = 100;
	TIMER_1_task1.cb       = TIMER_1_task1_cb;
	TIMER_1_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_1_task2.interval = 200;
	TIMER_1_task2.cb       = TIMER_1_task2_cb;
	TIMER_1_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_1, &TIMER_1_task1);
	timer_add_task(&TIMER_1, &TIMER_1_task2);
	timer_start(&TIMER_1);
}

void I2C_0_example(void)
{
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, 0x12, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)"Hello World!", 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

/**
 * Example of using WDT_0.
 */
void WDT_0_example(void)
{
	/**
	 * ATSAMS70N20B do not support wdt_set_timeout_period()
	 * and wdt_enable(), the watch dog will start counting once
	 * wdt_init() been called.
	 * The application should call wdt_feed(&WDT_0) at
	 * regular intervals before the timer underflow.
	 */
	wdt_feed(&WDT_0);
}
