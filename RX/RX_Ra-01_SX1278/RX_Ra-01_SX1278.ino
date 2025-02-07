#include <SPI.h> // Import SPI library
#include <RH_RF95.h> // RF95 from RadioHead Library

#define RFM95_CS 5   // NSS pin for LoRa connected to GPIO 5 (CS)
#define RFM95_RST 14 // RST pin for LoRa connected to GPIO 14
#define RFM95_DIO0 2 // DIO0 pin for LoRa connected to GPIO 26 (INT)


// Change to 434.0 or other frequency, must match TX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_DIO0);

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
  Serial.println("LoRa receiver ready");
}

void loop()
{
  if (rf95.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("Received: ");
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  }
  delay(1000);
}