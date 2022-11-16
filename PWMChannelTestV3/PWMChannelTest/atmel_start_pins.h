/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMV71 has 4 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3

#define PA2 GPIO(GPIO_PORTA, 2)
#define PC13 GPIO(GPIO_PORTC, 13)
#define PC19 GPIO(GPIO_PORTC, 19)
#define PD11 GPIO(GPIO_PORTD, 11)
#define PD30 GPIO(GPIO_PORTD, 30)

#endif // ATMEL_START_PINS_H_INCLUDED
