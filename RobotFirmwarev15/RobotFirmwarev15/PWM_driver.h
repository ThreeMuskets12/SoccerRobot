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
#define CHANNEL_0 (0) // Pin D5
#define CHANNEL_1 (1) // Pin D7
#define CHANNEL_2 (2) // Pin D6
#define CHANNEL_3 (3) // Pin A4

/* the pwm period for application is 200Hz */
#define PWM_PERIOD 2928

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
//motor 0, output on pin D5
void set_pwm_motor_0(struct _pwm_device *const device, uint32_t duty_cycle);
//motor 1, output on pin D7
void set_pwm_motor_1(struct _pwm_device *const device, uint32_t duty_cycle);
//motor 2, output on pin D6
void set_pwm_motor_2(struct _pwm_device *const device, uint32_t duty_cycle);
//motor 3, output on pin A4
void set_pwm_motor_3(struct _pwm_device *const device, uint32_t duty_cycle);

//enables PWM and selected pins/clock dividers via Atmel.start
void config_PWM(void);