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

#define IMU_RST GPIO(GPIO_PORTA, 0)
#define PA2 GPIO(GPIO_PORTA, 2)
#define PA3 GPIO(GPIO_PORTA, 3)
#define PA4 GPIO(GPIO_PORTA, 4)
#define PA5 GPIO(GPIO_PORTA, 5)
#define PA21 GPIO(GPIO_PORTA, 21)
#define PB1 GPIO(GPIO_PORTB, 1)
#define PB4 GPIO(GPIO_PORTB, 4)
#define RF24_CSN GPIO(GPIO_PORTC, 9)
#define PC13 GPIO(GPIO_PORTC, 13)
#define PC19 GPIO(GPIO_PORTC, 19)
#define PD11 GPIO(GPIO_PORTD, 11)
#define PD20 GPIO(GPIO_PORTD, 20)
#define PD21 GPIO(GPIO_PORTD, 21)
#define PD22 GPIO(GPIO_PORTD, 22)
#define RF24_CE GPIO(GPIO_PORTD, 25)

#endif // ATMEL_START_PINS_H_INCLUDED
