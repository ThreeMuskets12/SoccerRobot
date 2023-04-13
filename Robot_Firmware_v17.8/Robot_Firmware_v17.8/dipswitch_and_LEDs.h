/*
 * dipswitch_and_LEDs.h
 *
 * Created: 4/12/2023 7:16:56 PM
 *  Author: bensc
 */ 


#ifndef DIPSWITCH_AND_LEDS_H_
#define DIPSWITCH_AND_LEDS_H_

#include "atmel_start.h"
#include "atmel_start_pins.h"


uint8_t read_dipswitch(void); //reads the state of the dipswitch array and interprets it as a byte

void set_LEDs(uint8_t state); //sets the state of LEDs using 4 bits, lsb is LED 0 and msb is LED 3

#endif /* DIPSWITCH_AND_LEDS_H_ */