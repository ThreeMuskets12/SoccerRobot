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

void nRF24_init(){ //You are in standby-1 at the end of this call
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
	uint8_t tx_address[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
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

uint8_t nRF_24_is_data_available(int pipe_num){
	uint8_t status_reg;
	status_reg = nRF24_read_from_register(CONFIG);
	if((status_reg&(1<<6))&&(status_reg&(1<<1))){ //1<<6 is the data ready rx fifo interrupt and 1<<1 is the data from pipe 1 ready to read 
		nRF24_write_to_register(STATUS, (1<<6)); //clear data ready rx fifo
		return 1;
	}
	return 0;
}

void nRF24_enter_receive(){ //You are in receive at the end of this call
	uint8_t config_reg;
	config_reg = nRF24_read_from_register(CONFIG);
	config_reg = config_reg | 1; //XXXX XXX1 PRIM_RX to 1
	nRF24_write_to_register(CONFIG, config_reg);
	nRF24_write_to_register(EN_RXADDR, 2); //0000 0010 set data pipe 1 to on
	uint8_t rx_address[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
	nRF24_write_to_register_multi_byte(RX_ADDR_P1, &rx_address[0], 5);
	nRF24_write_to_register(RX_PW_P1,32); //32 bytes packet size
	gpio_set_pin_level(RF24_CE, true);
}

void nRF24_receive_data(uint8_t *data_pointer){
	uint8_t cmd = R_RX_PAYLOAD;
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &cmd, 1);
	io_read(spi_0_io, data_pointer, 32);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
	delay_us(11); //Make sure we had enough time to grab the data before flushing
	cmd = FLUSH_RX;
	nRF24_send_SPI_command(cmd);
}

void enter_standby(){
	uint8_t config_reg;
	config_reg = nRF24_read_from_register(CONFIG);
	config_reg = config_reg & 254; //XXXX XXX0 PRIM_RX to 0
	nRF24_write_to_register(CONFIG, config_reg);
	gpio_set_pin_level(RF24_CE, false);
}