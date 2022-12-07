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

#include <hpl_spi_base.h>
#include <hpl_usart_base.h>

struct spi_m_sync_descriptor SPI_0;

struct pwm_descriptor PWM_0;

struct i2c_m_sync_desc I2C_0;

struct usart_sync_descriptor TARGET_IO;

void EXTERNAL_IRQ_1_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(PB1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PB1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PB1, GPIO_PIN_FUNCTION_OFF);
}

void EXTERNAL_IRQ_0_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(PA5, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA5,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA5, GPIO_PIN_FUNCTION_OFF);
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

void SPI_0_PORT_init(void)
{

	gpio_set_pin_function(PD20, MUX_PD20B_SPI0_MISO);

	gpio_set_pin_function(PD21, MUX_PD21B_SPI0_MOSI);

	gpio_set_pin_function(PD22, MUX_PD22B_SPI0_SPCK);
}

void SPI_0_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_SPI0);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_set_func_ptr(&SPI_0, _spi_get_spi_m_sync());
	spi_m_sync_init(&SPI_0, SPI0);
	SPI_0_PORT_init();
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_function(PA4, MUX_PA4A_TWIHS0_TWCK0);

	gpio_set_pin_function(PA3, MUX_PA3A_TWIHS0_TWD0);
}

void I2C_0_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_TWIHS0);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();

	i2c_m_sync_init(&I2C_0, TWIHS0);

	I2C_0_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void TARGET_IO_PORT_init(void)
{

	gpio_set_pin_function(PA21, MUX_PA21A_USART1_RXD1);

	gpio_set_pin_function(PB4, MUX_PB4D_USART1_TXD1);
}

void TARGET_IO_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_USART1);
}

void TARGET_IO_init(void)
{
	TARGET_IO_CLOCK_init();
	TARGET_IO_PORT_init();
	usart_sync_init(&TARGET_IO, USART1, _usart_get_usart_sync());
}

void system_init(void)
{
	init_mcu();

	_pmc_enable_periph_clock(ID_PIOA);

	_pmc_enable_periph_clock(ID_PIOB);

	_pmc_enable_periph_clock(ID_PIOC);

	_pmc_enable_periph_clock(ID_PIOD);

	/* Disable Watchdog */
	hri_wdt_set_MR_WDDIS_bit(WDT);

	/* GPIO on PA0 */

	gpio_set_pin_level(IMU_RST,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(IMU_RST, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(IMU_RST, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PC9 */

	gpio_set_pin_level(RF24_CSN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RF24_CSN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RF24_CSN, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD25 */

	gpio_set_pin_level(RF24_CE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RF24_CE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RF24_CE, GPIO_PIN_FUNCTION_OFF);

	EXTERNAL_IRQ_1_init();
	EXTERNAL_IRQ_0_init();

	PWM_0_init();

	SPI_0_init();

	I2C_0_init();

	delay_driver_init();

	TARGET_IO_init();

	ext_irq_init();
}
