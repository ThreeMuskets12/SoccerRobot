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

#include <hpl_pwm_base.h>

#include <hpl_spi_base.h>

struct spi_m_sync_descriptor SPI_0;

struct adc_sync_descriptor ADC_0;

struct pwm_descriptor PWM_0;

struct pwm_descriptor PWM_1;

struct i2c_m_sync_desc I2C_0;

struct usart_sync_descriptor TARGET_IO;

struct wdt_descriptor WDT_0;

void ADC_0_PORT_init(void)
{

	gpio_set_pin_function(battery_current, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(battery_voltage, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(capacitor_charge, GPIO_PIN_FUNCTION_OFF);
}

void ADC_0_CLOCK_init(void)
{

	_pmc_enable_periph_clock(ID_AFEC0);
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, AFEC0, (void *)NULL);
}

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
	gpio_set_pin_direction(PA15, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PA15,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA15, GPIO_PIN_FUNCTION_OFF);

	// Set pin direction to input
	gpio_set_pin_direction(encoder1_A, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder1_A,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder1_A, GPIO_PIN_FUNCTION_OFF);

	// Set pin direction to input
	gpio_set_pin_direction(encoder2_A, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder2_A,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder2_A, GPIO_PIN_FUNCTION_OFF);

	// Set pin direction to input
	gpio_set_pin_direction(Encoder3_A, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Encoder3_A,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(Encoder3_A, GPIO_PIN_FUNCTION_OFF);

	// Set pin direction to input
	gpio_set_pin_direction(Encoder4_A, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Encoder4_A,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(Encoder4_A, GPIO_PIN_FUNCTION_OFF);
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(PD11, MUX_PD11B_PWM0_PWMH0);

	gpio_set_pin_function(PA2, MUX_PA2A_PWM0_PWMH1);

	gpio_set_pin_function(PA13, MUX_PA13B_PWM0_PWMH2);

	gpio_set_pin_function(PA17, MUX_PA17C_PWM0_PWMH3);
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

void PWM_1_PORT_init(void)
{

	gpio_set_pin_function(PA12, MUX_PA12C_PWM1_PWMH0);
}

void PWM_1_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_PWM1);
}

void PWM_1_init(void)
{
	PWM_1_CLOCK_init();
	PWM_1_PORT_init();
	pwm_init(&PWM_1, PWM1, _pwm_get_pwm());
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
}

void TARGET_IO_CLOCK_init(void)
{
	_pmc_enable_periph_clock(ID_UART0);
}

void TARGET_IO_init(void)
{
	TARGET_IO_CLOCK_init();
	usart_sync_init(&TARGET_IO, UART0, _uart_get_usart_sync());
	TARGET_IO_PORT_init();
}

void WDT_0_init(void)
{
	wdt_init(&WDT_0, WDT);
}

void system_init(void)
{
	init_mcu();

	_pmc_enable_periph_clock(ID_PIOA);

	_pmc_enable_periph_clock(ID_PIOB);

	_pmc_enable_periph_clock(ID_PIOD);

	/* GPIO on PA27 */

	// Set pin direction to input
	gpio_set_pin_direction(encoder1_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder1_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder1_B, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PA28 */

	// Set pin direction to input
	gpio_set_pin_direction(encoder2_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder2_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder2_B, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PA30 */

	// Set pin direction to input
	gpio_set_pin_direction(encoder3_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder3_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder3_B, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PA31 */

	// Set pin direction to input
	gpio_set_pin_direction(encoder4_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(encoder4_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(encoder4_B, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PB3 */

	gpio_set_pin_level(RF24_CSN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RF24_CSN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RF24_CSN, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD2 */

	gpio_set_pin_level(RF24_CE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RF24_CE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RF24_CE, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD3 */

	gpio_set_pin_level(IMU_RST,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(IMU_RST, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(IMU_RST, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD4 */

	gpio_set_pin_level(toggle_kicker,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(toggle_kicker, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(toggle_kicker, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD5 */

	gpio_set_pin_level(Chipper_toggle,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(Chipper_toggle, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(Chipper_toggle, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD6 */

	gpio_set_pin_level(toggle_charging,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(toggle_charging, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(toggle_charging, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD7 */

	gpio_set_pin_level(LED0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(LED0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED0, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD8 */

	gpio_set_pin_level(LED1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED1, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD9 */

	gpio_set_pin_level(LED2,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED2, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD10 */

	gpio_set_pin_level(LED3,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED3, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD12 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch0, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD13 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch1, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD14 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch2, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch2,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch2, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD15 */

	gpio_set_pin_level(DipSwitch3,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(DipSwitch3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(DipSwitch3, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD16 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch4, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch4,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch4, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD17 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch5, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch5,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch5, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD18 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch6, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch6,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch6, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD19 */

	// Set pin direction to input
	gpio_set_pin_direction(DipSwitch7, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(DipSwitch7,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DipSwitch7, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();
	EXTERNAL_IRQ_1_init();
	EXTERNAL_IRQ_0_init();

	PWM_0_init();

	PWM_1_init();

	SPI_0_init();

	I2C_0_init();

	delay_driver_init();

	TARGET_IO_init();

	WDT_0_init();

	ext_irq_init();
}
