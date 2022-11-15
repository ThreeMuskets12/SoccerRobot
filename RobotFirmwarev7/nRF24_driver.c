/*
 * nRF24_driver.c
 *
 * Created: 11/13/2022 8:15:29 PM
 *  Author: nynoa
 */ 

#include "nRF24_driver.h"

struct io_descriptor *spi_0_io;

void nRF24_write_to_register(uint8_t registerd, uint8_t data){ //SPI Command "W_REGISTER"
	uint8_t buffer[2];
	memset(&buffer[0], 0, sizeof(uint8_t)*2);
	buffer[0] = buffer[0] | 32; //Command word (binary) = 001A AAAA. 32 = 0010 0000. This will place a 1 in the right position for the command word for W_REGISTER
	buffer[0] = (buffer[0]) | registerd; //For example, if register 4 was passed in as a parameter. buffer[0] currently equals 32. 32|4 = 36. 36 = 0010 0100. This complies
	//with what the chip is expecting of 001A AAAA, where AAAAA would equal 0100, or 4.
	buffer[1] = data;
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, buffer, 2); //Careful never to put too much math inside of here, you only get 38 clock cycles max between driving CSN to low and when data needs to start being transmitted
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
}

void nRF24_write_to_register_multi_byte(uint8_t registerd, uint8_t *data, int length){ //SPI Command "W_REGISTER"
	uint8_t buffer;
	buffer = 0;
	buffer = buffer | 32; //Command word (binary) = 001A AAAA. 32 = 0010 0000. This will place a 1 in the right position for the command word for W_REGISTER
	buffer = buffer | registerd; //For example, if register 4 was passed in as a parameter. buffer[0] currently equals 32. 32|4 = 36. 36 = 0010 0100. This complies
	//with what the chip is expecting of 001A AAAA, where AAAAA would equal 0100, or 4.
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1);
	io_write(spi_0_io, data, length); 
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
}

uint8_t nRF24_read_from_register(uint8_t registerd){ //SPI Command "R_REGISTER"
	uint8_t in_byte;
	uint8_t buffer;
	buffer = 0;
	buffer = buffer | registerd;
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1); //Careful never to put too much math inside of here, you only get 38 clock cycles max between driving CSN to low and when data needs to start being transmitted
	io_read(spi_0_io, &in_byte, 1);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
	return in_byte;
}

void nRF24_read_from_register_multi_byte(uint8_t registerd, uint8_t *data_pointer, int length){ //SPI Command "R_REGISTER"
	uint8_t buffer;
	buffer = 0;
	buffer = buffer | registerd;
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1);
	io_read(spi_0_io, data_pointer, length);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
}

void nRF24_send_SPI_command(uint8_t command){
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &command, 1);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
}

void nRF24_SPI_init(){ //You are in standby-1 at the end of this call
	spi_m_sync_get_io_descriptor(&SPI_0, &spi_0_io);
	spi_m_sync_enable(&SPI_0);
	gpio_set_pin_level(RF24_CE, false); //Keeps us set to be ready to enter standby-1
	nRF24_write_to_register(CONFIG,0); //0000 0000 keep us in power down, disable checksums
	nRF24_write_to_register(EN_AA,0); //0000 0000 no auto ACK
	nRF24_write_to_register(EN_RXADDR,0); //0000 0000 disable all RX pipes
	nRF24_write_to_register(SETUP_AW,3); //0000 0011 5 byte tx rx address fields
	nRF24_write_to_register(SETUP_RETR,0); //0000 0000 no auto retransmission
	nRF24_write_to_register(RF_CH,120); //0011 1111 first bit must be 0, 011 1111 = 63 freq = 2400 + 63 = 2463 = 2.463 GHz
	nRF24_write_to_register(RF_SETUP,2); //0000 0010 1 Mbps and -12 db
	uint8_t tx_address[5];
	memset(&tx_address[0], 231, sizeof(uint8_t)*5);
	nRF24_write_to_register_multi_byte(TX_ADDR, &tx_address[0], 5); //Set TX addr as e7e7e7e7e7
	nRF24_write_to_register(CONFIG,2); //0000 0020 enter standby-1, disable checksums
}

void nRF24_transmit(uint8_t *data){ //You should be in standby-1 at the beginning of this call
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	uint8_t x = W_TX_PAYLOAD;
	io_write(spi_0_io, &x, 1);
	io_write(spi_0_io, data, 32);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
	//Now there should be a payload in the TX_FIFO
	gpio_set_pin_level(RF24_CE, true);
	delay_us(11); //This will put us into TX mode and empty the FIFO queue into the air
	gpio_set_pin_level(RF24_CE, false); //Return to standby-1 mode!
}

void nRF24_power_up(){
	nRF24_SPI_init();
	gpio_set_pin_level(RF24_CE, false); //Keeps us set to be ready to enter standby-1
}

void at_least_150ns_delay(){ //At this level, due to pipelining and predictive instruction execution, it is REALLY hard to delay in the ns resolution.
//However, given that there is the need often to wait 150ns before doing something, this function should take AT LEAST 150ns to complete. (Actually much more).
//One clock cycle is 3.3ns when master clock is 300 Mhz. Conditionals take longer than 1 clock cycle, and incrementing i is at least 1 clock cycle. Do this 75 times,
//a conditional and an increment, and you get something that is guaranteed to be longer than 150ns.
	int x = 0;
	for(int i = 0; i < 75; i++){
		x++;
	}
}
