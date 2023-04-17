/*
 * bno085_driver.c
 *
 * Created: 11/2/2022 3:20:58 PM
 *  Author: Noah Page
 */ 

#include "bno085_driver.h"

/* Initialize variables */

/* These 3 arrays are used for tracking the channel information that the BNO085 sends over during the SHTP Advertisement
packet. The first one is an array of unsigned integers that tracks which channels the BNO085 has advertised as valid. This array
is initialized to all 0s initially, with index 0 being a 1. (This is because channel 0, the command channel, is always
considered a valid channel). As the advertisement packet is processed, this array is updated. Each index in the array
corresponds to the channel #, and the value at the index is 0 if invalid, and 1 if valid. For example, if array index
5 is a 1, that means that channel 5 has been advertised as a valid channel during the SHTP Advertisement packet. The
channel names 2D array works in the same way, where each index corresponds to a channel number, and stored at each
index is an array of characters that corresponds to the name of that channel number. For example, channel 0 will always
be called "command", so valid_channel_names[0] will consist of an array of characters equal to "command". These channel
names are also communicated during the SHTP Advertisement packet. The last array consists of the messageCounter for
each channel. According to the SHTP protocol, each channel has its own messageCounter that is incremented for every
message that is sent over the channel. This is also kept track of in the form of channel numbers corresponding to
array indices.*/

uint valid_channels[MAXIMUM_CHANNELS];
char valid_channel_names[MAXIMUM_CHANNELS][MAXIMUM_CHANNEL_NAME_LENGTH];
uint8_t valid_channel_messageCounter[MAXIMUM_CHANNELS];

/* The following variables store information about the way the BNO085 will communicate with the MCU over I2C.
Once again, this information is obtained from the SHTP Advertisement packet. Right now the driver does not
actually use any of these values, as it is assumed that with only 1 or 2 report IDs activated for the BNO085,
that we will never be attempting to read or write more than the MaxCargo amount anyways, although a check for
this should eventually be implemented. */

uint spec_MaxCargoPlusHeaderWrite = 0;
uint spec_MaxCargoPlusHeaderRead = 0;
uint spec_MaxTransferWrite = 0;
uint spec_MaxTransferRead = 0;
uint spec_SHTPVersion = 0;

/* Next the io_descriptor and pointer for the float array are declared. */

struct io_descriptor *i2c_0_io;
float* location_of_imu_vector_buffer;

/* This function accepts a pointer to a float array as a parameter, and enables
I2C communication with the BNO085 using the Microchip-provided I2C driver.
It then makes the global variable that stores the pointer to the float array
for returning quaternion coefficients equal to the pointer it accepted as its parameter.
It also zeros out any of the variables declared earlier that were arrays, as well as
setting the command channel in the valid_channels array as valid. It then calls
get_IMU_quaternion() once, (to process the SHTP advertisement packet), before calling
enable_report_ID() to tell the IMU to begin sending sensor fused reports. */

void init_IMU(float* imu_vector_buffer)
{
	/* These 3 lines initialize the I2C communication. The "I2C_0" variable
	should be equal to the name of the I2C driver instance that was setup
	when configuring the project in Atmel Start. */
	
	i2c_m_sync_get_io_descriptor(&I2C_0, &i2c_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, BNO085_SLAVE_ADDR, I2C_M_SEVEN);
	
	/* Set the global quaternion float storage pointer equal to the passed in
	pointer */
	
	location_of_imu_vector_buffer = imu_vector_buffer;
	
	/* Zero out variables that were declared as arrays */
	memset(&valid_channels[0], 0, sizeof(uint8_t)*MAXIMUM_CHANNELS);
	memset(&valid_channel_messageCounter[0], 0, sizeof(uint8_t)*MAXIMUM_CHANNELS);
	memset(&valid_channel_names[0][0], 0, sizeof(uint8_t)*MAXIMUM_CHANNELS*MAXIMUM_CHANNEL_NAME_LENGTH);
	
	/* Set the command channel as a valid channel */
	valid_channels[0] = 1;
	
	/* Call get_IMU_quaternion() once to process the SHTP Advertisement packet that
	should have been sent by now, and then call enable_report_ID() to tell the IMU 
	to begin sending sensor fused reports. */
	
	get_IMU_quaternion();
	delay_ms(5);
	enable_report_ID();
	
	/* Uncomment this line to enable "Mode 1" for getting quaternion data,
	this sets the the H_INT signal to be an interrupt that calls the
	get_IMU_quaternion() every time there is a falling edge on pin PA05. */
	
	//ext_irq_register(PA5, get_IMU_quaternion);
}

/* A very basic linear search function that looks through the valid channel name
array to find which channel is the SH2 control channel, and returns it as an int.
Returns 1000 if it couldn't find the channel name "control". */

int find_SH2_control_channel()
{
	/* This for loop starts at 1 because index 0 is the command channel, which has the name
	"command", which is not the same as the SH2 command channel, which also has the name "command".
	It is unfortunate that this is very confusing from a naming perspective. */
	
	for(int i = 1; i < MAXIMUM_CHANNELS; i ++){
		if(strcmp(valid_channel_names[i],"control") == 0){
			return i;
		}
	}
	return 1000;
}

/* A very basic linear search function that looks through the valid channel name
array to find which channel is the SH2 inputNormal channel, and returns it as an int.
Returns 1000 if it couldn't find the channel name "inputNormal". */

int find_SH2_inputNormal_channel()
{
	/* This for loop starts at 1 because index 0 is the command channel, which has we
	know is not the inputNormal channel. So might as well start at 1 for consistency with the
	find_SH2_control_channel(). */
	
	for(int i = 1; i < MAXIMUM_CHANNELS; i ++){
		if(strcmp(valid_channel_names[i],"inputNormal") == 0){
			return i;
		}
	}
	return 1000;
}

/* A very basic function that accepts a pointer to a byte array of length 4 or less that represents
a uint but is in Least Significant Byte to Most Significant Byte (left to right), as well as the length
(between 0 and 4 inclusively), and returns a uint that represents the 4 ordered correctly to be a uint.
Returns 0 if a length outside of the range 0 to 4 inclusive was given. */

uint byte_array_to_uint(uint8_t* byte_array, uint8_t length)
{
	if(length == 1){
		return (uint)byte_array[0];
	}
	else if(length == 2){
		return (uint)((uint)byte_array[0] | ((uint)byte_array[1] << 8));
	}
	else if(length == 3){
		return (uint)((uint)byte_array[0] | ((uint)byte_array[1] << 8) | ((uint)byte_array[2] << 16));
	}
	else if(length == 4){
		return (uint)((uint)byte_array[0] | ((uint)byte_array[1] << 8) | ((uint)byte_array[2] << 16) | ((uint)byte_array[3] << 24));
	}
	else{
		return 0; //Error, uint can only store 4 bytes!
	}
}

/* This function is the bulk of the driver. Calling this function will update the 4 values inside of the global
float array that stores the quaternion coefficients with the most recent quaternion coefficients available
by reading the BNO085 incoming information if available. */

void get_IMU_quaternion()
{
	/* Initialize a new SHTP_header struct called my header to be used to store the SHTP_header information,
	and zero it out */
	
	struct SHTP_header my_Header = {0};
	uint8_t header_Bytes[SHTP_HEADER_BYTES];
	memset(&header_Bytes[0], 0, SHTP_HEADER_BYTES*sizeof(uint8_t));
	
	/* Read one byte from I2C, and see if it is an empty byte. The first 2 bytes of an SHTP packet are
	the first 2 bytes of the SHTP header. The first 2 bytes of the SHTP header are the length of the
	incoming packet in bytes (including the length of the header itself). If the first byte is empty,
	check if the second byte is empty as well. If both bytes are empty, this is a packet of length 0,
	which means there is no packet ready to be read. In this case, simply exit the function without
	doing anything, as the most recent quaternion coefficients should still be stored in the global
	array, so leaving them unchanged leaves them as the most recent received coefficients. */
	
	uint8_t single_Byte[2];
	memset(&single_Byte[0], 0, 2*sizeof(uint8_t));
	io_read(i2c_0_io, &single_Byte[0], 2);
	if(single_Byte[0]==0 && single_Byte[1]==0){
		return;
	}
	/*Load the SHTP_header struct with the 3 pieces of information available inside of the SHTP
	header bytes. For the length, mask the first bit away, as it is reserved according to the protocol. */
	
	io_read(i2c_0_io, &header_Bytes[0], SHTP_HEADER_BYTES);
	my_Header.length = ((header_Bytes[1]<<8)|header_Bytes[0]) & 32767; //01111111 11111111 = 32767
	my_Header.channel = header_Bytes[2];
	my_Header.counter = header_Bytes[3];
	
	/*Dynamically allocate an array that is large enough to hold the entire packet
	(and zero it out), using the byte length of the whole packet from the SHTP header as
	calculated earlier. */
	
	uint8_t* packetPointer;
	packetPointer = (uint8_t*)malloc(my_Header.length*sizeof(uint8_t));
	memset(packetPointer, 0, my_Header.length);
	
	/* Read the full packet bytes into this dynamically allocated array. */
	
	io_read(i2c_0_io, packetPointer, my_Header.length);
	
	/*Since we have received a message at this point, increment the messageCounter on
	the channel we received the message on, regardless of whether we consider it an
	already initialized channel. */
	
	valid_channel_messageCounter[my_Header.channel] = my_Header.counter;
	
	/*Check if the channel we are receiving over is a channel we have marked as valid.
	On the first call of this function, only the command channel (channel 0) is valid.
	However, upon receiving the SHTP Advertisement packet on channel 0, new valid channels
	will be updated and will becoming available to process on. If the channel is not valid,
	just skip to the end of the function, where the dynamically allocated memory is freed,
	and then the function call ends. */
	
	if(valid_channels[my_Header.channel] != 1){
		if(VERBOSE == 1){
			printf("Attempted to read a packet on a channel that has not been initialized through an advertisement yet!\r\n");
		}
		return;
	}
	else{ // Entering this means the channel was valid
		if(my_Header.channel == 0){ // Entering this means the channel is the command channel
			
			/* The first byte outside of the SHTP header in a packet on the command channel
			represents whether the packet is an SHTP Advertisement packet (a byte of a 0),
			or if it is a SHTP Error packet (a byte of a 1). Entering this means the byte was a 0, 
			which represents an SHTP Advertisement. */
			
			if(packetPointer[4]==0){ // Entering this means the packet is a SHTP Advertisement (See above)
				
				/* valid_channel_index keeps track of the number of channels that have been marked as valid
				within the SHTP Advertisement packet. */
				
				int valid_channel_index = 0;
				if(VERBOSE == 1){
					printf("SHTP Advertisement output:\r\n");
				}
				
				/* byte_tracker will be used to keep track of which byte in the packet we are processing.
				It starts at 5, as bytes 0-3 are the SHTP header, and byte 4 indicates SHTP Advertisement
				or SHTP Error packet. (So the data starts at 5) */
				
				int byte_tracker = 5;
				
				/* The data inside of an SHTP Advertisement (starting with byte 5) comes in the format of
				repeated TLVs. A TLV is a byte segment of arbitrary length, where the first byte in the
				segment is the "T" section, standing for "Tag". The tag byte indicates what type of data is
				contained in the TLV section. All of the tag meanings are defined in the SHTP manual. The next
				byte is the "L" section, standing for "Length". The length byte indicates how many bytes long the
				"V" section will be. The next L bytes, (so if L is 5, the next 5 bytes) are the "V" section,
				standing for "Value". These bytes are the actual value of the given tag type. */
				
				/* Initialize a byte array to hold the current TVL we are trying to process. Technically we
				could dynamically allocate this every time based on the L value of the TLV we are processing,
				but thats a lot of work. 1 byte can only hold a uint of maximum size 256, and because L is
				only 1 byte long, we can just allocate this as 256 bytes and guarantee it will never overflow. */
				
				uint8_t tvl_value[256] = {0};
				
				/* This while loop will loop through the received packet until all the bytes in the packet (equal
				to the length of the packet as specified in the SHTP header) are processed. */
				
				while(byte_tracker < my_Header.length){
					/* At this point in the while loop, the byte_tracker is pointing to the index in the packet
					that corresponds to a "Tag" byte. */
					
					/* Set tvl_length equal to the "L" value of this TLV section. (Which we know is always 1 byte ahead
					of the "T" byte. */
					
					uint8_t tvl_length = packetPointer[byte_tracker+1];
					
					/* Zero out the tvl_value byte array, and then copy the bytes of the "V" section of this TVL
					segment into the array (We know that the "V" section starts 2 bytes past the "T" byte). */
					
					memset(tvl_value, 0, 256);
					memcpy(tvl_value, &packetPointer[byte_tracker+2], tvl_length);
					
					/* This switch statement is loaded with all of the meanings for each of the "T" bytes,
					according to the SHTP documentation. For information that we don't need to store for later,
					it will print the information to the serial console if VERBOSE is enabled. Otherwise, it does
					nothing. For information that we do need to store for later, it will process and store the
					information in the appropriate place. */
					
					switch(packetPointer[byte_tracker]){
						case 0:
						break;
						case 1:
							if(VERBOSE == 1){
								printf("GUID = %u\r\n", byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length));
							}
						break;
						case 2:
							spec_MaxCargoPlusHeaderWrite = byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length);
							if(VERBOSE == 1){
								printf("MaxCargoPlusHeaderWrite = %u\r\n", spec_MaxCargoPlusHeaderWrite);
							}
						break;
						case 3:
							spec_MaxCargoPlusHeaderRead = byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length);
							if(VERBOSE == 1){
								printf("MaxCargoPlusHeaderRead = %u\r\n", spec_MaxCargoPlusHeaderRead);
							}
						break;
						case 4:
							spec_MaxTransferWrite = byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length);
							if(VERBOSE == 1){
								printf("MaxTransferWrite = %u\r\n", spec_MaxTransferWrite);
							}
						break;
						case 5:
							spec_MaxTransferRead = byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length);
							if(VERBOSE == 1){
								printf("MaxTransferRead = %u\r\n", spec_MaxTransferRead);
							}
						break;
						case 6:
							valid_channels[valid_channel_index] = 1;
							if(VERBOSE == 1){
								printf("NormalChannel = %u\r\n", valid_channels[valid_channel_index]);
							}
						break;
						case 7:
							if(VERBOSE == 1){
								printf("WakeChannel = %u\r\n", byte_array_to_uint(&packetPointer[byte_tracker+2], tvl_length));
							}
						break;
						case 8:
							if(VERBOSE == 1){
								printf("AppName = %s\r\n", &packetPointer[byte_tracker+2]);
							}
						break;
						case 9:
							memcpy(&valid_channel_names[valid_channel_index][0], &packetPointer[byte_tracker+2], tvl_length);
							if(VERBOSE == 1){
								printf("ChannelName = %s\r\n", valid_channel_names[valid_channel_index]);
							}
							valid_channel_index++;
						break;
						case 128:
							if(VERBOSE == 1){
								printf("SHTP Version = %s\r\n",&packetPointer[byte_tracker+2]);
							}
						break;
						case 129:
						break;
						default:
							if(VERBOSE == 1){
								printf("Cannot understand advertisement packet tag value");
							}
						break;
					}
					
				/* Move the byte_tracket to the location where the next "T" byte sits. */
				
				byte_tracker = byte_tracker + tvl_length + 2;
				}
			}
			else if(packetPointer[4]==1){  // Entering this means the packet is a SHTP Error
				
				/* The data inside of an SHTP Error is always just a list of single byte error messages.
				The meanings of each byte and what error message that byte corresponds to is found in the
				SHTP documentation. Since we only want to print errors if VERBOSE is enabled, this
				entire section is wrapped in an VERBOSE == 1 if statement. This section uses a for loop
				starting at byte 5 (for the same reason we started at byte 5 up above) and goes byte by
				byte printing out the corresponding error message for the byte according to the SHTP
				documentation, until it runs out of bytes to print (Known thanks to the SHTP header
				length value). */
				
				if(VERBOSE == 1){
					printf("SHTP error output:\r\n");
					for(int x = 5; x < my_Header.length; x++){
						switch(packetPointer[x]){
							case 0:
							printf("No error\r\n");
							break;
							case 1:
							printf("Hub application attempted to exceed maximum read cargo length\r\n");
							break;
							case 2:
							printf("Host write was too short (need at least a 4-byte header)\r\n");
							break;
							case 3:
							printf("Host wrote a header with length greater than maximum write cargo length\r\n");
							break;
							case 4:
							printf("Host wrote a header with length less than or equal to header length (either invalid or no payload). Note that a length of 0 is permitted, indicating �no cargo�\r\n");
							break;
							case 5:
							printf("Host wrote beginning of fragmented cargo (transfer length was less than full cargo length), fragmentation not supported\r\n");
							break;
							case 6:
							printf("Host wrote continuation of fragmented cargo (continuation bit sent), fragmentation not supported\r\n");
							break;
							case 7:
							printf("Unrecognized command on control channel\r\n");
							break;
							case 8:
							printf("Unrecognized parameter to get-advertisement command\r\n");
							break;
							case 9:
							printf("Host wrote to unrecognized channel\r\n");
							break;
							case 10:
							printf("Advertisement request received while Advertisement Response was pending\r\n");
							break;
							case 11:
							printf("Host performed a write operation before the hub had finished sending its advertisement response\r\n");
							break;
							case 12:
							printf("Error list too long to send, truncated\r\n");
							break;
							default:
							printf("Unknown error\r\n");
							break;
						}
					}
				}
			}
			
			/* We shouldn't ever be in here, as this would indicate that the command channel packet type
			byte was neither a 0 (Advertisement) or a 1 (Error), so print a message to inform of the problem. */
			
			else{
				if(VERBOSE == 1){
					printf("Command channel command type was a reserved bit!\r\n");
				}
			}
		}
		
		
		else if((int)my_Header.channel == find_SH2_control_channel()){ // Entering this means the channel is the SH2 command channel
			if(VERBOSE == 1){
				printf("SH-2 Control Channel Output:\r\n");
			}
			
			/* The first byte (after the SHTP header bytes) in a packet coming over the SH2 command channel 
			indicates the report ID of the packet. Each report ID has a specific format for what bytes within
			that packet correspond to what pieces of information (This is all specified in the SH2 manual).
			This switch statement is loaded with the meanings of all report IDs, and if VERBOSE is 1, prints the
			encoded information inside that report ID to the serial console. */
			
			switch(packetPointer[4]){
				case 252: //0xFC
					if(VERBOSE == 1){
						printf("Report ID = 0xFC 'Get Feature Response'\r\n");
						printf("Feature Report ID = %u\r\n", (uint)packetPointer[5]);
						printf("Feature flags = %u\r\n", (uint)packetPointer[6]);
						printf("Change sensitivity = %u\r\n", byte_array_to_uint(&packetPointer[7], 2));
						printf("Report interval = %u\r\n", byte_array_to_uint(&packetPointer[9], 4));
						printf("Batch interval = %u\r\n", byte_array_to_uint(&packetPointer[13], 4));
						printf("Sensor-specific configuration word = %u\r\n", byte_array_to_uint(&packetPointer[17], 4));
					}
				break;
				case 248: //0xF8
					if(VERBOSE == 1){
						printf("Report ID = 0xF8 'Product ID Response'\r\n");
						printf("Reset Cause = %u\r\n", (uint)packetPointer[5]);
						printf("SW Version Major = %u\r\n", (uint)packetPointer[6]);
						printf("SW Version Minor = %u\r\n", (uint)packetPointer[7]);
						printf("SW Part Number = %u\r\n", byte_array_to_uint(&packetPointer[8], 4));
						printf("SW Build Number = %u\r\n", byte_array_to_uint(&packetPointer[12], 4));
						printf("SW Version Patch = %u\r\n", byte_array_to_uint(&packetPointer[16], 2));
					}
				break;
				case 245: //0xF5
					if(VERBOSE == 1){
						printf("Report ID = 0xF5 'FRS Write Response'\r\n");
						printf("Status/Error = %u\r\n", (uint)packetPointer[5]);
						printf("Word Offset = %u\r\n", byte_array_to_uint(&packetPointer[6], 2));
					}
				break;
				case 243: //0xF3
					if(VERBOSE == 1){
						printf("Report ID = 0xF3 'FRS Read Response'\r\n");
						uint8_t data_length = (packetPointer[5] >> 4);
						printf("Data Length = %u\r\n", (uint)data_length);
						printf("Status = %u\r\n", (uint)(packetPointer[5]&15));
						printf("Word Offset = %u\r\n", byte_array_to_uint(&packetPointer[6], 2));
						if(data_length==1){
							printf("Data0 = %u\r\n", byte_array_to_uint(&packetPointer[8], 4));
							printf("FRS Type = %u\r\n", byte_array_to_uint(&packetPointer[16], 2));
						}
						else if(data_length==1){
							printf("Data0 = %u\r\n", byte_array_to_uint(&packetPointer[8], 4));
							printf("Data0 = %u\r\n", byte_array_to_uint(&packetPointer[12], 4));
							printf("FRS Type = %u\r\n", byte_array_to_uint(&packetPointer[16], 2));
						}
						else{
							printf("FRS Type = %u\r\n", byte_array_to_uint(&packetPointer[16], 2));
						}
					}
				break;
				case 241: //0xF1
					if(VERBOSE == 1){
						printf("Report ID = 0xF1 'Command Response'\r\n");
						printf("Sequence Number = %u\r\n", (uint)packetPointer[5]);
						printf("Command = %u\r\n", (uint)packetPointer[6]);
						printf("Command Sequence Number = %u\r\n", (uint)packetPointer[7]);
						printf("Response Sequence Number = %u\r\n", (uint)packetPointer[8]);
						printf("R0 Value = %u\r\n", (uint)packetPointer[9]);
						printf("R1 Value = %u\r\n", (uint)packetPointer[10]);
						printf("R2 Value = %u\r\n", (uint)packetPointer[11]);
						printf("R3 Value = %u\r\n", (uint)packetPointer[12]);
						printf("R4 Value = %u\r\n", (uint)packetPointer[13]);
						printf("R5 Value = %u\r\n", (uint)packetPointer[14]);
						printf("R6 Value = %u\r\n", (uint)packetPointer[15]);
						printf("R7 Value = %u\r\n", (uint)packetPointer[16]);
						printf("R8 Value = %u\r\n", (uint)packetPointer[17]);
						printf("R9 Value = %u\r\n", (uint)packetPointer[18]);
						printf("R10 Value = %u\r\n", (uint)packetPointer[19]);
					}
				break;
				default:
					if(VERBOSE == 1){
						printf("Received report ID over SH2 control channel that was not recognized");
					}
				break;
				
			}
		}
		
		else if((int)my_Header.channel == find_SH2_inputNormal_channel()){ // Entering this means the channel is the SH2 inputNormal channel (This is where sensor data will be sent)
			if(VERBOSE == 1){
				printf("SH-2 inputNormal Channel Output:\r\n");
				
				/* The bytes at packet index 5, 6, 7, and 8 correspond to the Base Delta of the following report.
				The Base Delta is a piece of information that helps relay what the delay of the incoming information is. */
				
				printf("Base Delta = %u\r\n", byte_array_to_uint(packetPointer[5],4));
			}
			
			/* The first byte (after the SHTP header bytes and the Base Delta bytes) in a
			packet coming over the SH2 inputNormal channel indicates the report ID of the packet.
			Each report ID has a specific format for what bytes within that packet correspond 
			to what pieces of information (This is all specified in the SH2 manual).
			This switch statement is loaded with the meanings of all report IDs, and if VERBOSE is 1, prints the
			encoded information inside that report ID to the serial console. 
			Additionally, this switch statement will make sure that the quaternions from the report IDs are
			placed inside of the float quaternion coefficient array. */
			
			/* Note that because we are only using the Rotation Vector report, that is the only
			switch statement that I have populated with information. However, if you are looking
			to use a different report than Rotation Vector, it should be pretty easy to fill
			in the correct information in this section. */
			
			switch(packetPointer[9]){
				memset(&location_of_imu_vector_buffer[0], 0, sizeof(uint)*4);
				case 20: //Raw Accelerometer
				break;
				case 1: //Accelerometer
				break;
				case 2: //Gyroscope Calibrated
				break;
				case 3: //Magnetic Field Calibrated
				break;
				case 4: //Linear Acceleration
				break;
				case 5: //Rotation Vector
					if(VERBOSE == 1){
						printf("Report ID = 0x05 'Rotation Vector'\r\n");
						printf("Sequence Number = %u\r\n", (uint)packetPointer[10]);
						printf("Status = %u\r\n", (uint)packetPointer[11]);
						printf("Delay = %u\r\n", (uint)packetPointer[12]);
						printf("Unit Quaternion i = %f\r\n", QP(14)*(int16_t)(((int16_t)packetPointer[14] << 8) | packetPointer[13]));
						printf("Unit Quaternion j = %f\r\n", QP(14)*(int16_t)(((int16_t)packetPointer[16] << 8) | packetPointer[15]));
						printf("Unit Quaternion k = %f\r\n", QP(14)*(int16_t)(((int16_t)packetPointer[18] << 8) | packetPointer[17]));
						printf("Unit Quaternion real = %f\r\n", QP(14)*(int16_t)(((int16_t)packetPointer[20] << 8) | packetPointer[19]));
						printf("Accuracy estimate = %f\r\n", QP(12)*(int16_t)(((int16_t)packetPointer[22] << 8) | packetPointer[21]));
					}
					location_of_imu_vector_buffer[0] = QP(14)*(int16_t)(((int16_t)packetPointer[20] << 8) | packetPointer[19]);
					location_of_imu_vector_buffer[1] = QP(14)*(int16_t)(((int16_t)packetPointer[14] << 8) | packetPointer[13]);
					location_of_imu_vector_buffer[2] = QP(14)*(int16_t)(((int16_t)packetPointer[16] << 8) | packetPointer[15]);
					location_of_imu_vector_buffer[3] = QP(14)*(int16_t)(((int16_t)packetPointer[18] << 8) | packetPointer[17]);
					if(PRINT_QUATERNION_COEFFS == 1){
						printf("%f,%f,%f,%f\r\n", location_of_imu_vector_buffer[0], location_of_imu_vector_buffer[1], location_of_imu_vector_buffer[2], location_of_imu_vector_buffer[3]);
					}
				break;
				case 6: //Gravity
				break;
				case 7: //Gyroscope Uncalibrated
				break;
				case 8: //Game Rotation Vector
				break;
				case 9: //Geomagnetic Rotation Vector
				break;
				case 15: //Magnetic Field Uncalibrated
				break;
				case 21: //Raw Gyroscope
				break;
				case 22: //Raw Magnetometer
				break;
				default:
					if(VERBOSE == 1){
						printf("Received report ID over SH2 inputNormal channel that was not recognized");
					}
				break;
			}
		}
		/* Free the memory that was used to dynamically allocated space for the packet that was read. */
		
		free(packetPointer);
	}
}

/* This function handles sending a packet in the proper SHTP format to command the BNO085 to begin
sending sensor reports of the specified type and at the specified period. It returns a -1 if
the function could not find the proper SH2 command channel to broadcast the packet over. It
returns a 1 upon a successful broadcast. */

int enable_report_ID(void)
{
	/* If SH2_control_channel can't find the control channel (returned error code 1000), then we should send the reset signal,
	and then wait 1 ms before trying to read the Advertisement packet and see if we find the control signal.
	Normally I am hesitant of using the delay_ms() function inside of a driver, this part only gets called once
	on system boot, so I think it is fine to have a 1 ms delay inside of here. */
	
	if(find_SH2_control_channel() > 256 || find_SH2_control_channel() < 0){
		gpio_set_pin_level(IMU_RST, false);
		delay_ms(1);
		gpio_set_pin_level(IMU_RST, true);
		delay_ms(500);
		get_IMU_quaternion(); //This read should process the (hopefully) received Advertisement packet
		get_IMU_quaternion();
		get_IMU_quaternion();
	}
	
	/* If SH2_control_channel still can't find the control channel (returned error code 1000), even after sending the reset signal and
	trying to read the Advertisement packet, then we should return -1 as a failure. */
	
	if(find_SH2_control_channel() > 256 || find_SH2_control_channel() < 0){
		printf("Failed to find the SH-2 control channel number! (Likely no SHTP advertisement packet read) Is the BNO085 connected?\r\n");
		return -1; //Failure
	}
	
	/* Send the command to the BNO085 to begin broadcasting the selected sensor report ID at the selected period. The first 4 bytes
	are the SHTP header, and the 5th byte is 0xFD, which indicates that this is a SET FEATURE COMMAND as per the manual.
	The following bytes are in the format that a SET FEATURE COMMAND should have. See the manual for more details. */
	
	uint8_t send_feature_command_bytes[21] = {0x15, 0x00, (uint8_t)find_SH2_control_channel(), valid_channel_messageCounter[(uint8_t)find_SH2_control_channel()]+1, 0xFD, SELECTED_SENSOR_REPORT_ID, 0x00, 0x00, 0x00, IMU_REPORT_PERIOD_LSB, IMU_REPORT_PERIOD_MSB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	valid_channel_messageCounter[(uint8_t)find_SH2_control_channel()]++;
	io_write(i2c_0_io, &send_feature_command_bytes[0], 21);
	if(VERBOSE == 1){
		printf("Successfully sent Set Feature Command to BNO085\r\n");
	}
	return 1; //Success
}