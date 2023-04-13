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

// SAMS70 has 4 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3

#define PA2 GPIO(GPIO_PORTA, 2)
#define PA3 GPIO(GPIO_PORTA, 3)
#define PA4 GPIO(GPIO_PORTA, 4)
#define PA12 GPIO(GPIO_PORTA, 12)
#define PA13 GPIO(GPIO_PORTA, 13)
#define PA15 GPIO(GPIO_PORTA, 15)
#define PA17 GPIO(GPIO_PORTA, 17)
#define capacitor_charge GPIO(GPIO_PORTA, 18)
#define encoder1_A GPIO(GPIO_PORTA, 22)
#define encoder2_A GPIO(GPIO_PORTA, 23)
#define Encoder3_A GPIO(GPIO_PORTA, 24)
#define Encoder4_A GPIO(GPIO_PORTA, 25)
#define encoder1_B GPIO(GPIO_PORTA, 27)
#define encoder2_B GPIO(GPIO_PORTA, 28)
#define encoder3_B GPIO(GPIO_PORTA, 30)
#define encoder4_B GPIO(GPIO_PORTA, 31)
#define PB1 GPIO(GPIO_PORTB, 1)
#define battery_voltage GPIO(GPIO_PORTB, 2)
#define RF24_CSN GPIO(GPIO_PORTB, 3)
#define RF24_CE GPIO(GPIO_PORTD, 2)
#define IMU_RST GPIO(GPIO_PORTD, 3)
#define toggle_kicker GPIO(GPIO_PORTD, 4)
#define Chipper_toggle GPIO(GPIO_PORTD, 5)
#define toggle_charging GPIO(GPIO_PORTD, 6)
#define LED0 GPIO(GPIO_PORTD, 7)
#define LED1 GPIO(GPIO_PORTD, 8)
#define LED2 GPIO(GPIO_PORTD, 9)
#define LED3 GPIO(GPIO_PORTD, 10)
#define PD11 GPIO(GPIO_PORTD, 11)
#define DipSwitch0 GPIO(GPIO_PORTD, 12)
#define DipSwitch1 GPIO(GPIO_PORTD, 13)
#define DipSwitch2 GPIO(GPIO_PORTD, 14)
#define DipSwitch3 GPIO(GPIO_PORTD, 15)
#define DipSwitch4 GPIO(GPIO_PORTD, 16)
#define DipSwitch5 GPIO(GPIO_PORTD, 17)
#define DipSwitch6 GPIO(GPIO_PORTD, 18)
#define DipSwitch7 GPIO(GPIO_PORTD, 19)
#define PD20 GPIO(GPIO_PORTD, 20)
#define PD21 GPIO(GPIO_PORTD, 21)
#define PD22 GPIO(GPIO_PORTD, 22)
#define Motor_0_Dir GPIO(GPIO_PORTD, 24)
#define Motor_1_Dir GPIO(GPIO_PORTD, 25)
#define Motor_2_Dir GPIO(GPIO_PORTD, 26)
#define Motor_3_Dir GPIO(GPIO_PORTD, 27)
#define Dribbler_Motor_Dir GPIO(GPIO_PORTD, 28)
#define battery_current GPIO(GPIO_PORTD, 30)

#endif // ATMEL_START_PINS_H_INCLUDED
