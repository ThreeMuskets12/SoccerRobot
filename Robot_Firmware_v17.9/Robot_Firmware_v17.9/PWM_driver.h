/*
 * PWM_driver.h
 *
 * Created: 11/21/2022 8:46:16 PM
 *  Author: evanv
 */ 
#include "atmel_start.h"
#include "atmel_start_pins.h"
#include <hal_delay.h>
//PWM channels for either instance (0 or 1)
#define PWM_0_CHANNEL_0 (2) // Pin PA13
#define PWM_0_CHANNEL_1 (3) // Pin PA17
#define PWM_0_CHANNEL_2 (1) // Pin PA2
#define PWM_0_CHANNEL_3 (0) // Pin PD11
#define PWM_1_CHANNEL_0 (0) // Pin PA12

/* the pwm period for application is 200Hz */
#define PWM_PERIOD 2928*2

/*
* Sets a specific channel to the given duty-cycle
*    device is a constant pointer to the pwm device instance
*    channel [0-3] is the PWM channel of the instance we wish to set
*    duty_cycle is the duty cycle that we wish to set (see documentation for effort mapping)
*/
void set_pwm_channel(struct _pwm_device *const device, uint8_t channel, uint32_t duty_cycle);
/*
* The following four functions set the duty-cycle of the corresponding motor (which in turn have corresponding channels)
* All operate at the defined 200Hz
*    device is a constant pointer to the pwm device instance
*    duty_cycle is the duty cycle that we wish to set (see documentation for effort mapping)
*/
//for drive motors
void set_pwm_drive_motor(uint8_t target, uint32_t duty_cycle);
//for dribbler motor, on second PWM instance, PWM_1
void set_pwm_dribbler_motor(uint32_t duty_cycle);
