===========
PWM Example
===========

IO1 Xplained pro board must be connected to EXT1 extension header on
SAMC21/SAMD11/SAML10/SAML11 boards and to EXT2 on
SAMD21/SAML21/SAML22/SAMV71/SAME70 boards for the
example to work.

This example periodically reads data from the light sensor using the ADC driver
and uses read data to control PWM duty cycle.


Drivers
-------
* Asynchronous ADC
* PWM
* Delay
* GPIO

Default interface settings
--------------------------
* ADC

  * Internal band-gap reference
  * 8-bit resolution
