/*
 * bno085_driver.h
 *
 * Created: 11/2/2022 3:23:08 PM
 *  Author: nynoa
 */ 


#ifndef BNO085_DRIVER_H_
#define BNO085_DRIVER_H_

/* If you would like to put debugging information to the serial console, switch the VERBOSE flag from a 0 to a 1 */
#define VERBOSE 1

/* If you would like to put the quaternion coefficients to serial console, for the purposes of running
the Python 3D IMU visualizer, switch the VERBOSE flag from a 0 to a 1 */

#define PRINT_QUATERNION_COEFFS 1

/* If you would like to use operating mode 0 (the "fetch" configuration, where the main robot code calls
get_IMU_quaternion() whenever the code needs the quaternion buffer updated), then set this flag to 0.
If you would like to use operating mode 1 (the "push" configuration, where the main robot code can
assume that the quaternion buffer is always up to date with the most recent quaternions, and
never has to call get_IMU_quaternion() because the interrupt handler takes care of that), then set
this flag to a 1. */

#define OPERATING_MODE 0

/* The byte of the report ID that we should use when running enable_report_ID(). 0x05 corresponds to
Rotation Vector. If you change this to be something other than 0x05, make sure you change the empty parts
of the switch statement in get_IMU_quaternion() that processes that particular report ID. */

#define SELECTED_SENSOR_REPORT_ID 0x05

/* The speed at which the IMU should return sensor reports to the MCU. Units are in uS. C350 = 50,000uS. (20 Hz)
The maximum speed for a given report ID can be found in the BNO085 user manual (For Rotation Vector, max is 400 Hz).
Only increase this speed if you know your robot code is sampling the IMU faster than this frequency. */

#define IMU_REPORT_PERIOD_MSB 0xC3
#define IMU_REPORT_PERIOD_LSB 0x50

/* 0x4a is the default I2C slave address for the BNO085 chip, do not change this unless you know what you are doing */
#define BNO085_SLAVE_ADDR 0x4a

/* This will use like 3 out of 384 KBytes in memory, but I think that is a fair price to pay to avoid the complexity of dynamically trying to allocate this */
#define MAXIMUM_CHANNEL_NAME_LENGTH 14

/* Don't change the values below! */

#define MAXIMUM_CHANNELS 256 //There can actually only be 256 channels, since the channel # is defined by a single byte
#define SHTP_HEADER_BYTES 4
#define QP(n) (1.0f / (1 << n))

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>

struct SHTP_header{
	uint8_t channel;
	uint8_t counter;
	uint16_t length;
	};

extern uint32_t spec_MaxCargoPlusHeaderWrite;
extern uint32_t spec_MaxCargoPlusHeaderRead;
extern uint32_t spec_MaxTransferWrite;
extern uint32_t spec_MaxTransferRead;
extern uint32_t spec_SHTPVersion;
	
extern uint32_t valid_channels[MAXIMUM_CHANNELS];
extern char valid_channel_names[MAXIMUM_CHANNELS][MAXIMUM_CHANNEL_NAME_LENGTH];
extern uint8_t valid_channel_messageCounter[MAXIMUM_CHANNELS];
extern struct io_descriptor *i2c_0_io;
extern float* location_of_imu_vector_buffer;

void init_IMU(float* imu_vector_buffer);

void get_IMU_quaternion();

int enable_report_ID();

int find_SH2_control_channel();

int find_SH2_inputNormal_channel();

uint32_t byte_array_to_uint(uint8_t* byte_array, uint8_t length);

#endif /* BNO085_DRIVER_H_ */