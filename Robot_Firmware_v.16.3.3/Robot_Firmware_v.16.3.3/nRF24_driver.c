/*
 * nRF24_driver.c
 *
 * Created: 11/13/2022 8:15:29 PM
 *  Author: nynoa
 */ 

#include "nRF24_driver.h"

/* The io_descriptor and pointer for the received byte array are declared. */
struct io_descriptor *spi_0_io;
uint8_t* global_data_pointer;

/* This function takes in a byte indicating which of the nRF24's registers should be written to, and then a single byte of data to write to that register */
void nRF24_write_to_register(uint8_t registerd, uint8_t data){
	/* Initialize a 2 byte buffer to all zeros. Bits 7:5 will be the Write to Register SPI command word (001), and then bits 4:0 will be the 5 bit long register number to write to */
	uint8_t buffer[2];
	memset(&buffer[0], 0, sizeof(uint8_t)*2);
	/* Set bits 7:5 to be equal to the Write to Register SPI command word (001). 32 = 0010 000. */
	buffer[0] = buffer[0] | 32;
	/* Set bits 4:0 to be equal to the 5 bit long register number to write to. If editing this code later, be careful not to use the word register as a variable, as it is a reserved
	word by another library we use. */
	buffer[0] = (buffer[0]) | registerd;
	/* Set the second byte of the buffer equal to the passed in data to be written to the register */
	buffer[1] = data;
	/* Send the constructed packet to the nRF24 over SPI */
	gpio_set_pin_level(RF24_CSN, false); //Drive low before doing SPI transmissions
	io_write(spi_0_io, buffer, 2); //Write the constructed 2 byte packet to the nRF24 over SPI
	gpio_set_pin_level(RF24_CSN, true); //Drive back to high after doing SPI transmissions
}

/* This function takes in a byte indicating which of the nRF24's registers should be written to, a pointer to multiple bytes of data to be written to that register, and the length
(in number of bytes) of the data to be sent to the register */
void nRF24_write_to_register_multi_byte(uint8_t registerd, uint8_t *data, int length){
	/* Initialize a 1 byte buffer to all zeros. Bits 7:5 will be the Write to Register SPI command word (001), and then bits 4:0 will be the 5 bit long register number to write to */
	uint8_t buffer;
	buffer = 0;
	/* Set bits 7:5 to be equal to the Write to Register SPI command word (001). 32 = 0010 000. */
	buffer = buffer | 32;
	/* Set bits 4:0 to be equal to the 5 bit long register number to write to. If editing this code later, be careful not to use the word register as a variable, as it is a reserved
	word by another library we use. */
	buffer = buffer | registerd;
	/* Send the constructed packet to the nRF24 over SPI */
	gpio_set_pin_level(RF24_CSN, false); //Drive low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1); //Write the constructed 1 byte (does not contain any of the data to be written to the register) packet to the nRF24 over SPI
	io_write(spi_0_io, data, length); //Write all of the data to be written to the register to the nRF24 over SPI
	gpio_set_pin_level(RF24_CSN, true); //Drive back to high after doing SPI transmissions
}

/* This function takes in a byte indicating which of the nRF24's registers should be read from, and then reads in a single byte from that register and returns it to the function caller */
uint8_t nRF24_read_from_register(uint8_t registerd){
	/* Initialize a 1 byte buffer to be used to store the data byte read from the register */
	uint8_t in_byte;
	/* Initialize a 1 byte buffer to all zeros. Bits 7:5 will be the Read to Register SPI command word (000), and then bits 4:0 will be the 5 bit long register number to read from */
	uint8_t buffer;
	buffer = 0;
	/* Set bits 4:0 to be equal to the 5 bit long register number to write to. If editing this code later, be careful not to use the word register as a variable, as it is a reserved
	word by another library we use. */
	buffer = buffer | registerd;
	gpio_set_pin_level(RF24_CSN, false); //Drive low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1); /* Write the constructed 1 byte packet to the nRF24 over SPI. This will tell the nRF24 that the next time we read, we would like it to return the 1 byte
	of data from the target register */
	io_read(spi_0_io, &in_byte, 1); /* Read the 1 byte of data into the in_byte buffer */
	gpio_set_pin_level(RF24_CSN, true); //Drive back to high after doing SPI transmissions
	/* Return the 1 read byte back to the function caller */
	return in_byte;
}

/* This function takes in a byte indicating which of the nRF24's registers should be read from, a pointer to the location where the read data from that register should be stored,
 and the length (in number of bytes) of the data to be read from the register */
void nRF24_read_from_register_multi_byte(uint8_t registerd, uint8_t *data_pointer, int length){
	/* Initialize a 1 byte buffer to all zeros. Bits 7:5 will be the Read to Register SPI command word (000), and then bits 4:0 will be the 5 bit long register number to read from */
	uint8_t buffer;
	buffer = 0;
	/* Set bits 4:0 to be equal to the 5 bit long register number to write to. If editing this code later, be careful not to use the word register as a variable, as it is a reserved
	word by another library we use. */
	buffer = buffer | registerd;
	gpio_set_pin_level(RF24_CSN, false); //Drive low before doing SPI transmissions
	io_write(spi_0_io, &buffer, 1); /* Write the constructed 1 byte packet to the nRF24 over SPI. This will tell the nRF24 that the next time we read, we would like it to return the 1 byte
	of data from the target register */
	io_read(spi_0_io, data_pointer, length); /* Read the multiple bytes of data into the pointer that was passed in as a parameter */
	gpio_set_pin_level(RF24_CSN, true); //Drive back to high after doing SPI transmissions
}

/* This function will send a raw SPI command to the nRF24, without reading or writing to or from a particular register. */
void nRF24_send_SPI_command(uint8_t command){
	gpio_set_pin_level(RF24_CSN, false); //Drive low before doing SPI transmissions
	io_write(spi_0_io, &command, 1); //Send the 1 byte command that was received as a parameter to the nRF24 over SPI
	gpio_set_pin_level(RF24_CSN, true); //Drive back to high after doing SPI transmissions
}

/* This function sets up the nRF24 to be used in accordance to the Noah's Packet Protocol v1.0, enables the interrupt pin, and has the nRF24 enter standby-1 mode when completed */
void nRF24_init(uint8_t* data_pointer){
	/* Setup the SPI connection */
	spi_m_sync_get_io_descriptor(&SPI_0, &spi_0_io);
	spi_m_sync_enable(&SPI_0);
	/* Set the global data pointer to the data pointer that was passed in as a parameter. This needs to be done because a function setup to be used as an interrupt cannot accept parameters */
	global_data_pointer = data_pointer;
	/*This should already be set low, but this will make sure we do not accidentally enter TX or RX modes upon turning on the PWR_UP bit */
	gpio_set_pin_level(RF24_CE, false);
	nRF24_write_to_register(CONFIG,0); //0000 0000 Keep us in power down mode, IRQ pin assertion set to default, CRC disabled, RX / TX control in RX mode
	/* Read from the STATUS register, write a 1 in bit 6 to clear the IRQ pin assertion if present */
	uint8_t status_read = nRF24_read_from_register(STATUS);
	status_read = status_read | 64;
	nRF24_write_to_register(STATUS, status_read);
	nRF24_write_to_register(EN_AA,0); //0000 0000 no auto ACK
	nRF24_write_to_register(EN_RXADDR,0); //0000 0000 disable all RX pipes
	nRF24_write_to_register(SETUP_AW,3); //0000 0011 5 byte tx rx address fields
	nRF24_write_to_register(SETUP_RETR,0); //0000 0000 no auto retransmission
	nRF24_write_to_register(RF_CH,120); //0111 1000 first bit must be 0, 0111 1000 = 120, freq = 2400 + 120 = 2520 = 2.52 GHz, tested to have low interference in UH
	nRF24_write_to_register(RF_SETUP,6); //0000 0110 1 Mbps and maximum power output
	uint8_t tx_address[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA}; //Needs to be the same on the other nRF24L01+ device
	nRF24_write_to_register_multi_byte(TX_ADDR, &tx_address[0], 5);
	nRF24_write_to_register(CONFIG,2); //0000 0020 enter standby-1, disable checksums
	ext_irq_register(PB1, nRF24_receive_data); //enable interrupt
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
	status_reg = nRF24_read_from_register(STATUS);
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

void nRF24_receive_data(){
	uint8_t cmd = R_RX_PAYLOAD;
	gpio_set_pin_level(RF24_CSN, false); //drive this low before doing SPI transmissions
	io_write(spi_0_io, &cmd, 1);
	io_read(spi_0_io, global_data_pointer, 32);
	gpio_set_pin_level(RF24_CSN, true); //return to high after SPI transmissions
	delay_us(11); //Make sure we had enough time to grab the data before flushing
	cmd = FLUSH_RX;
	nRF24_send_SPI_command(cmd);
	uint8_t status_read = nRF24_read_from_register(STATUS);
	status_read = status_read | 64;
	nRF24_write_to_register(STATUS, status_read);
	//uint8_t charray[64];
	for(int x = 0; x < 32; x++){
		printf("%02x ", global_data_pointer[x]);
	}
	printf("\r\n");
}

void enter_standby(){
	uint8_t config_reg;
	config_reg = nRF24_read_from_register(CONFIG);
	config_reg = config_reg & 254; //XXXX XXX0 PRIM_RX to 0
	nRF24_write_to_register(CONFIG, config_reg);
	gpio_set_pin_level(RF24_CE, false);
}