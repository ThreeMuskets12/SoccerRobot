/*
 */

#include <SPI.h>
#include <string.h>
#include "printf.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(9, 4);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[5] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t byte_packet[32];
long tx_counter = 0;

void setup() {
  memset(&byte_packet[0], 0, sizeof(uint8_t) * 32);
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(32);  // float datatype occupies 4 bytes
  radio.setChannel(120);
  radio.disableAckPayload();
  radio.disableDynamicPayloads();
  radio.disableCRC();
  radio.setAutoAck(false);
  radio.setDataRate(0);

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address);  // always uses pipe 0

  radio.stopListening();  // put radio in TX mode
}

uint8_t hex2int(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    return -1;
}

void loop() {
  // This device is a TX node
  memset(&byte_packet[0], 0, sizeof(uint8_t) * 32);
  while (!Serial.available()) {
    // wait for user input
  }
  delay(3);
int availableBytes = Serial.available();
for(int i=0; i<availableBytes; i++){
  byte_packet[i] = Serial.read();
  if(i>=32){
    break;
  }
}
while(Serial.available()){
  Serial.read();
}
  for(int z = 0; z <32; z++){
    Serial.print((char)byte_packet[z]);
  }
  Serial.println("\n");
  bool report = radio.write(&byte_packet, sizeof(byte_packet));  // transmit & save the report
  tx_counter++;

  if (report) {
    // transmission successful; wait for response and print results
    // print summary of transactions
    Serial.print(F("Transmission "));  // payload was delivered
    Serial.print(tx_counter);
    Serial.println(" Successful!");
    uint8_t pipe;
  } else {
    Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
  }                                                         // report

  // to make this example readable in the serial monitor
  //delay(1000);  // slow transmissions down by 1 second

}  // loop