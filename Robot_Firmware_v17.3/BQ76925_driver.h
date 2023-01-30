/*
 * BQ76925_driver.h
 *
 * Created: 12/5/2022 8:10:07 PM
 *  Author: evanv
 */ 

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>

//Register Addresses as a slave address
#define STATUS_BQ7 0b0100000
#define CELL_CTL 0b0100001
#define BAL_CTL 0b0100010
#define CONFIG_1 0b0100011
#define CONFIG_2 0b0100100
#define POWER_CTL 0b0100101
#define CHIP_ID 0b01000111
#define VREF_CAL 0b0110000
#define VC1_CAL 0b0110001
#define VC2_CAL 0b0110010
#define VC3_CAL 0b0110011
#define VC4_CAL 0b0110100
#define VC5_CAL 0b0110101
#define VC6_CAL 0b1010110
#define VC_CAL_EXT_1 0b1010111
#define VC_CAL_EXT_2 0b1011000
#define VREF_CAL_EXT 0b1011001

//masks for cell we wish to read
#define CELL_1 (0)
#define CELL_2 (1)
#define CELL_3 (2)
#define CELL_4 (3)
#define CELL_5 (4)
#define CELL_6 (5)

//ADC
#define CONF_ADC_CHANNEL (0)

void init_BQ76925(uint8_t *buffer, uint8_t slave_address);
void change_BQ76925_cell(uint8_t cell);
uint8_t handleMasking(uint8_t single_byte, uint8_t cell);
void setup_BQ7_for_adc(uint8_t *buffer, uint8_t cell);
void set_ref_voltage(uint8_t *buffer, uint8_t selection);

//adc functions
void init_ADC0(void);
float v_cell(void);