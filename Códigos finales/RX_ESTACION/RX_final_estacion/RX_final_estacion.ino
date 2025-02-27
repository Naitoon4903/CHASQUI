#include <SPI.h>
#include <RH_RF95.h>

// Definir pines utilizados por el módulo LoRa
#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

// Frecuencia de operación del módulo LoRa (418 MHz)
#define RF95_FREQ 500.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // Inicializar la comunicación serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Esperar a que el monitor esté listo
  }
  Serial.println("Ground Station Receiver");

  // Configurar el pin de reinicio y establecerlo en alto
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Inicializar el módulo LoRa
  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  Serial.println("LoRa init succeeded");

  // Configurar la frecuencia
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setSignalBandwidth(125E3); // 125 kHz
  rf95.setSpreadingFactor(12); // SF12
  rf95.setCodingRate4(5); // CR 4/5
}

void loop() {
  // Buffer para almacenar el mensaje recibido
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Verificar si se recibió algún mensaje
  if (rf95.waitAvailableTimeout(1000)) {
    // Intentar recibir el mensaje
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';  
      Serial.print("Received: ");
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  }
}