/*
 * BQ76925_driver.c
 *
 * Created: 12/5/2022 8:10:33 PM
 *  Author: evanv
 */ 
#include "BQ76925_driver.h"

uint8_t add_values[2];//location to place register values in

struct io_descriptor *i2c_0_io_PD; //io descriptor
uint8_t* global_buffer_location; //global buffer location
//uint8_t* global_buffer_location[6]; //global buffer location, all 6 cells
uint8_t curr_slave_address; //register we wish to read/write

//initialize I2C based on the slave address we wish to read from (see .h file)
void init_BQ76925(uint8_t *buffer, uint8_t slave_address){
	i2c_m_sync_get_io_descriptor(&I2C_0, &i2c_0_io_PD);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, slave_address, I2C_M_SEVEN); //replace w/ address
	
	global_buffer_location = buffer;	
	curr_slave_address = slave_address;
}

//changes the cell on VCOUT pin on the mux
//cell is the input battery we wish to read [1, 6] which are defined in register as [0, 5]
void change_BQ76925_cell(uint8_t cell){
	if(cell < 0 || cell > 5)return; //check cell
	//printf("Changing Pin\n");
	uint8_t single_Byte; 
	single_Byte = 0; //initialize to 0
	io_read(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
	//bit mask single_Byte according to the information we want
	single_Byte = handleMasking(single_Byte, cell);
	//write to selected slave address to configure mux
	io_write(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
	//io_read(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
	
}

//sets BQ7 for ADC conversion on VCOUT
//requires a buffer for I2C and a cell that we wish to change
void setup_BQ7_for_adc(uint8_t *buffer, uint8_t cell){
	//initialize descriptor to have CELL_CTL as slave address
	init_BQ76925(buffer, CELL_CTL);
	//configure MUX
	change_BQ76925_cell(cell);
}

//masking for CELL selection
// bits [5, 4] must be [0, 1] respectively for VCOUT_SEL to be put on VCOUT pin
uint8_t handleMasking(uint8_t single_byte, uint8_t cell){
	single_byte &= 0b11001000;	 //keep bits 7, 6, 3 same, revert others
	single_byte |= 0b00010000; //set bits 5, 4 to 0, 1, respectively, to ensure cell goes on VCOUT pin
	uint8_t mask = 0;
	switch(cell){ //cell selection (last three bits of CELL_CTL register)
		case 0: // 0 0 0
		break;
		case 1: // 0 0 1
		mask |= 0b001;
		break;
		case 2: // 0 1 0
		mask |= 0b010;
		break;
		case 3: // 0 1 1
		mask |= 0b011;
		break;
		case 4: // 1 0 0
		mask |= 0b100;
		break;
		case 5: // 1 0 1
		mask |= 0b101;
		break;
	}
	single_byte |= mask;
	return single_byte;
}

//reference voltage configuration
//selection is
//	0: 0.25 - 1.25
//	1: 0.5 - 2.5
// CRC_EN is set to 0 b/c makes more difficult
// register bit layout
//   D7		D6   D5    D4     D3     D2     D1     D0
// CRC_EN   -    -     -      -      -      -    REF_SEL
void set_ref_voltage(uint8_t *buffer, uint8_t selection){
	if(selection != 0 && selection != 1) return; //check bounds
	init_BQ76925(buffer, CONFIG_2);
	//uint8_t single_Byte; //buffer to read register
	//single_Byte = 0;
	//io_read(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
	uint8_t single_Byte = (0b01111110 | selection); //disable CRC, keep bits 6-1 same, toggle bit 0
	
	io_write(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
	//io_read(i2c_0_io_PD, &single_Byte, sizeof(uint8_t));
}


//initializes channel 0 for battery voltage mux
void init_ADC0(void){
	//enable the channel
	 adc_sync_enable_channel(&ADC_0, 0);
}
	
//does ADC conversion and calculates voltage of cell
//voltage will always be between 0.5 and 2.5, as defined by the CONFIG_2 register
//therefore, [0.5, 2.5] must map to the battery's actual voltage of [0, 3.7]
float v_cell(void){
	adc_sync_read_channel(&ADC_0, 0, add_values, 2);
	
	int32_t value = add_values[0] + ((add_values[1] & 0x0F) << 8);
	
	float voltage = (value / 4096.0)*3.3; //voltage from 0-3.3
	
	voltage = voltage*(1.85) - 0.925; //map to correct domain
	
	return voltage;
}


