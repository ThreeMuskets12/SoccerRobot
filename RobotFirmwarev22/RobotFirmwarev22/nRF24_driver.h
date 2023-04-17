/*
 * nRF24_driver.h
 *
 * Created: 11/13/2022 8:13:35 PM
 *  Author: nynoa
 */ 


#ifndef NRF24_DRIVER_H_
#define NRF24_DRIVER_H_

#include <atmel_start.h>
#include <string.h>
#include <stdlib.h>

//Register addresses:
#define CONFIG 0
#define EN_AA 1
#define EN_RXADDR 2
#define SETUP_AW 3
#define SETUP_RETR 4
#define RF_CH 5
#define RF_SETUP 6
#define STATUS 7
#define OBSERVE_TX 8
#define RPD 9
#define RX_ADDR_P0 10
#define RX_ADDR_P1 11
#define RX_ADDR_P2 12
#define RX_ADDR_P3 13
#define RX_ADDR_P4 14
#define RX_ADDR_P5 15
#define TX_ADDR 16
#define RX_PW_P0 17
#define RX_PW_P1 18
#define RX_PW_P2 19
#define RX_PW_P3 20
#define RX_PW_P4 21
#define RX_PW_P5 22
#define FIFO_STATUS 23
#define DYN_PD 28
#define FEATURE 29

//SPI Commands
#define R_RX_PAYLOAD 97
#define W_TX_PAYLOAD 160
#define FLUSH_TX 225
#define FLUSH_RX 226
#define REUSE_TX_PL 227
#define R_RX_PL_WID 96
#define W_TX_PAYLOAD_NO_ACK 176

extern struct io_descriptor *spi_0_io;

extern uint8_t* global_data_pointer;

extern void nRF24_write_to_register(uint8_t registerd, uint8_t data);

extern void nRF24_write_to_register_multi_byte(uint8_t registerd, uint8_t *data, int length);

uint8_t nRF24_read_from_register(uint8_t registerd);

void nRF24_read_from_register_multi_byte(uint8_t registerd, uint8_t *data_pointer, int length);

void nRF24_send_SPI_command(uint8_t command);

extern void nRF24_init(uint8_t *data_pointer);

void nRF24_transmit(uint8_t *data);

void nRF24_receive_data();

void nRF24_enter_receive();

uint8_t nRF_24_is_data_available(int pipe_num);

void enter_standby();

#endif /* NRF24_DRIVER_H_ */