#include <SPI.h> // Import SPI library
#include <RH_RF95.h> // RF95 from RadioHead Library

#define RFM95_CS 5   // NSS pin for LoRa connected to GPIO 5 (CS)
#define RFM95_RST 14 // RST pin for LoRa connected to GPIO 14
#define RFM95_DIO0 2 // DIO0 pin for LoRa connected to GPIO 26 (INT)

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_DIO0);

char value = 48;

void setup() 
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Reset LoRa Module 
  pinMode(RFM95_RST, OUTPUT); 
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize LoRa Module
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  // Set the default frequency 434.0MHz
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  // Print ready message
  Serial.println("LoRa transmitter ready");
}

void loop() {
  char radiopacket[1] = {char(value)};
  Serial.print("Send: ");
  Serial.println((char)value);
  rf95.send((uint8_t *)radiopacket, 1);
  delay(1000);
  value++;
  if (value > '9'){
    value = 48;
  }
}