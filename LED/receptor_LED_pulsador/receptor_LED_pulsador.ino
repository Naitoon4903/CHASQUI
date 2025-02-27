/////////// PULSADOR - RECEPTORRRRRR

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

const int buttonPin = 25; // Pin del pulsador
bool buttonState = false;    // Estado actual del pulsador
bool lastButtonState = false; // Estado anterior del pulsador
bool ledState = false;        // Estado del LED

void setup() {
  pinMode(buttonPin, INPUT);
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

  // Configurar potencia de transmisión
  rf95.setTxPower(13, false);

  Serial.println("LoRa inicializado correctamente.");
}

void loop() {
  buttonState = digitalRead(buttonPin); // Leer el estado del pulsador

  if (buttonState != lastButtonState) { // Detectar cambio en el estado del pulsador
    if (buttonState == HIGH) { // Si el pulsador está presionado
      ledState = !ledState; // Cambiar el estado del LED
    }
    
    // Enviar el estado del LED
    uint8_t data[1] = { ledState };
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();

    Serial.print("Estado enviado: ");
    Serial.println(ledState);

    delay(50); // Agregar un pequeño retraso para el debouncing
  }

  lastButtonState = buttonState; // Actualizar el estado del pulsador
}