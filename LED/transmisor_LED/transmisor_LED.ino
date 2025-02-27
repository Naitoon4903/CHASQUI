//////////////  TRANSMNISOR - LED

#include <SPI.h>
#include <RH_RF95.h>

// Definir pines del módulo LoRa
#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 2

// Definir frecuencia del LoRa
#define RF95_FREQ 500.0

// Crear instancia del driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

const int ledPin = 26;   // Pin del LED

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  while (!Serial);

  // Inicializar el módulo LoRa
  Serial.println("Inicializando LoRa...");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("Inicialización de LoRa fallida!");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Configuración de frecuencia fallida!");
    while (1);
  }

  Serial.println("LoRa inicializado correctamente.");
}

void loop() {
  if (rf95.available()) {
    // Buffer para el mensaje recibido
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Intentar recibir el mensaje
    if (rf95.recv(buf, &len)) {
      bool receivedState = buf[0];

      // Encender o apagar el LED basado en el estado recibido
      digitalWrite(ledPin, receivedState);

      // Mostrar el estado recibido en el monitor serie
      Serial.print("Estado recibido: ");
      Serial.println(receivedState);
    } else {
      Serial.println("Recepción fallida.");
    }
  }
}