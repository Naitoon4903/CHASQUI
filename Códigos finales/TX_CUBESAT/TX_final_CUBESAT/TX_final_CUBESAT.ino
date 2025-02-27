//********************************************LIBRERIAS*********************************
//LIBRERIAS LORA
#include <SPI.h>
#include <RH_RF95.h>

// LIBRERÍAS SENSORES
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ML8511.h>
#include <TinyGPS++.h>


//*******************************************DEFINIR VARIABLES**************************
// Definir pines utilizados por el módulo LoRa
#define RFM95_CS 5 //CS
#define RFM95_RST 14
#define RFM95_INT 2 //DIO0

// Frecuencia de operación del módulo LoRa (418 MHz)
#define RF95_FREQ 500.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// VARIABLES GLOBALES

#define DHTPIN 4     // Pin donde está conectado el sensor
#define DHTTYPE DHT22   // Sensor DHT22

#define I2C_SDA 21
#define I2C_SCL 22

#define ANALOGPINUV 32

// VARIABLES DE INSTANCIA GENERAL

DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
ML8511 light(ANALOGPINUV);

HardwareSerial gpsSerial(2); // Usamos Serial2
TinyGPSPlus gps;

// VARIABLES DE INSTANCIA PARTICULAR

long tiempoUltimaLectura = 0; //Para guardar el tiempo de la última lectura
float sum = 0;
int_fast64_t count = 0;

String outdata_send;
float loaddata_send[14];

//***********************************FUNCIONES DE RUTINA**************************
// LECT ESP32 - RASP
void prueba_lectura() { // SOLO PARA PROBAR LA LECTURA!
  const char* message = "Hola desde el ESP32!";
  Serial.println(message); // Envía mensaje a través del UART
  delay(1000); // Envía un mensaje cada segundo
}

// SENSOR DHT22
void get_dhtdata(int self_delay) {
  if (millis() - tiempoUltimaLectura > self_delay) {
    loaddata_send[0] = dht.readHumidity(); // Leemos la Humedad
    loaddata_send[1] = dht.readTemperature(); // Leemos la temperatura en grados Celsius
    tiempoUltimaLectura = millis(); // Actualizamos el tiempo de la última lectura
  }
}

// ACELERÓMETRO
void setup_aclr() {
  while (!Serial)
    delay(10); // Pausa hasta que se abra la consola serial

  // Intentar inicializar
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void get_aclrdata() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  loaddata_send[2] = a.acceleration.x;
  loaddata_send[3] = a.acceleration.y;
  loaddata_send[4] = a.acceleration.z; // m/s2
  loaddata_send[5] = g.gyro.x;
  loaddata_send[6] = g.gyro.y;
  loaddata_send[7] = g.gyro.z; // rad/s
  loaddata_send[1] += temp.temperature;
  loaddata_send[1] /= 2;
}

// SENSOR UV

void get_uvdata() {
  loaddata_send[8] = light.getUV(); // mW/cm2
  float fctr = light.getDUVfactor() / loaddata_send[8];
  count++;
  sum += fctr;
  loaddata_send[9] = sum / count;
}

// GPS

void get_gpsval() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c); // Procesar datos con TinyGPS++
  }
  loaddata_send[10] = gps.location.isValid() ? gps.location.lat() : 0.0;
  loaddata_send[11] = gps.location.isValid() ? gps.location.lng() : 0.0;
  loaddata_send[12] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  loaddata_send[13] = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

//*******************************CÓDIGO GENERAL -- ATENCIÓN Y CUIDADO********************************

void setup() {
  // Configuración de UART1 (Serial Monitor)
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Esperar a que el monitor esté listo
  }
  Serial.println("Monitor Serial inicializado.");

  // Configuración de UART2 (GPSserial)
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  dht.begin(); // DHT SENSOR
  Wire.begin(I2C_SDA, I2C_SCL); // I2C
  
  // ACELERÓMETRO
  setup_aclr();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // SENSOR UV
  light.setDUVfactor(1.61);
  light.enable();

  //MODULO LORA
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  Serial.println("LoRa init succeeded");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(13, false);
  rf95.setSignalBandwidth(125E3); // 125 kHz
  rf95.setSpreadingFactor(12); // SF12
  rf95.setCodingRate4(5); // CR 4/5


  // LECTRAS
  outdata_send = String();
}

void loop() {
  int_fast8_t iterdata;
  outdata_send = "DSTART_";

  get_dhtdata(250);
  get_aclrdata();
  get_uvdata();
  get_gpsval();

  for (iterdata = 0; iterdata != 14; iterdata++) {
    outdata_send += loaddata_send[iterdata];
    outdata_send += "_";
  }

  outdata_send += "DEND";
  
  Serial.println(outdata_send);

  //Envio de data
  rf95.send((uint8_t *)outdata_send.c_str(), outdata_send.length());
  rf95.waitPacketSent();

  delay(250);

  outdata_send = String();
}