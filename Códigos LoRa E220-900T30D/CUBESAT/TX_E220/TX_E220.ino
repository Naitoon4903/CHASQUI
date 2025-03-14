// TRANSMISOR
// Frecuencia base nodo LoRa
#define FREQUENCY_868

///******************************************************************* LIBRERIAS
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ML8511.h>
#include <TinyGPS++.h>

#include <LoRa_E220.h>

///****************************************************************** VARIABLES GLOBALES

#define DHTPIN 14      // Pin donde está conectado el sensor
#define DHTTYPE DHT22   // Sensor DHT22

#define I2C_SDA 21
#define I2C_SCL 22

#define ANALOGPINUV 32

//GPIO ESP32 - LoRa
#define UART1_TX  26 // ESP32 TX -- E220 RX
#define UART1_RX  27 // ESP32 RX -- E220 TX
#define AUX_PIN   18 // AUX
#define M0_PIN    4  // M0
#define M1_PIN    5  // M1

///// ************************************************************************* VARIABLES DE INSTANCIA GENERAL

DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
ML8511 light(ANALOGPINUV);

HardwareSerial gpsSerial(2); // Usamos Serial2
TinyGPSPlus gps;

HardwareSerial LoRaSerial(1);
LoRa_E220 e220ttl(UART1_RX, UART1_TX, &LoRaSerial, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

/////******************************************************************************* VARIABLES DE INSTANCIA PARTICULAR

long tiempoUltimaLectura = 0; //Para guardar el tiempo de la última lectura
float    sum   = 0;
int_fast64_t count = 0;

//String outdata_send;
float loaddata_send[14];

//******************************************************************************** FUNCIONES

//SENSOR DHT22
void get_dhtdata(int self_delay){
  if(millis()-tiempoUltimaLectura > self_delay)
  {    
    loaddata_send[0] = dht.readHumidity(); //Leemos la Humedad
    loaddata_send[1] = dht.readTemperature(); //Leemos la temperatura en grados Celsius
    tiempoUltimaLectura=millis(); //actualizamos el tiempo de la última lectura
  }
}

//ACELEROMETRO
void setup_aclr(){
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void get_aclrdata(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  loaddata_send[2] = a.acceleration.x; loaddata_send[3] = a.acceleration.y; loaddata_send[4] = a.acceleration.z; // m/s2
  loaddata_send[5] = g.gyro.x; loaddata_send[6] = g.gyro.y; loaddata_send[7] = g.gyro.z; // rad/s
  loaddata_send[1] += temp.temperature; loaddata_send[1] /= 2;
}

//SENSOR UV
void get_uvdata(){
  loaddata_send[8] = light.getUV(); // mW/cm2
  float fctr = light.getDUVfactor()/loaddata_send[8];
  count++; sum += fctr;
  loaddata_send[9] = sum / count;
}

//GPS
void get_gpsval(){
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c); // Procesar datos con TinyGPS++
  }
  loaddata_send[10] = gps.location.isValid() ? gps.location.lat() : 0.0;
  loaddata_send[11] = gps.location.isValid() ? gps.location.lng() : 0.0;
  loaddata_send[12] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  loaddata_send[13] = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

//LoRa
void LoRa_Init(void);

//*************************************************************************** Estructura para el manejo de datos
typedef struct
{
  uint16_t idNodo;
  uint16_t readingId;
  //int      numerito;
  float    loaddata_send[14];
} Message_t;

// Variables de aplicacion
Message_t sendMsg = {0};
uint8_t led_state = 0;

////////******************************************************************** CÓDIGO ----- TENER CUIDADO
void setup() 
{
  // Configuración de UART1 (Serial Monitor)
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Esperar a que el monitor esté listo
  }
  Serial.println("Monitor Serial inicializado.");
  // Configuración de UART2 (GPSserial)
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  dht.begin(); //DHT SENSOR
  Wire.begin(I2C_SDA, I2C_SCL); // I2C
  // ACELERÓMETRO
  setup_aclr();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // SENSOR UV
  light.setDUVfactor(1.61);
  light.enable();
  // LECTURAS
 
  // Configuracion de nodo LoRa
  LoRa_Init();
  //Set ID Node
  sendMsg.idNodo = 0x00F1; // Puede ser cualquier ID dentro del 16 bits
}

void loop() 
{
  //Lectura de variables
  get_dhtdata(250);
  get_aclrdata();
  get_uvdata();
  get_gpsval(); // Asegúrate de que también estás llamando a get_gpsval

  memcpy(sendMsg.loaddata_send, loaddata_send, sizeof(loaddata_send));
  
  //Mostrar datos en el monitor serial
  Serial.print("Datos para enviar: ");
  for (int i = 0; i < 14; i++) {
    Serial.print(sendMsg.loaddata_send[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  //Envio de mensaje a gateway canal 65 = 850.125 + 65 = 915.125 Mhz
  ResponseStatus rsTx = e220ttl.sendFixedMessage(0x00, 0x1A, 67, (const uint8_t *)&sendMsg, sizeof(sendMsg));

  //La información se envía correctamente? (UTILIZAR COMO DEPURACION EN CASO NO SE ENVIEN DATOS)
  /*if(rsTx.code == 1)
  {
    Serial.printf("Envio de mensaje al gateway Ok \r\n");
  }
  else
  {
    Serial.println("Envio de mensaje al gateway fallido \r\n\r\n");
  }*/

  //delay(100);
}


// Funcion para incializar el modulo LoRa
void LoRa_Init(void)
{
  //Variable para obtener configuracion del modulo LoRa
  ResponseStructContainer rc;

  // Inicia todos los pines y UART
	e220ttl.begin();

  //Leer configuracion actual
  rc = e220ttl.getConfiguration();

  if(rc.status.code == 1)
  {
    Serial.println("Lectura de configuracion exitosa");
  }
  else {
    Serial.println("Error en la lectura de configuracion");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  //CONFIGURACION DE REGISTROS DEL MODULO LORA
  config.ADDH = 0x00; 
  config.ADDL = 0x4A;  

  config.CHAN = 67; // Communication channel = 850.125 + 65 = 915.125 Mhz

  config.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
  config.SPED.airDataRate  = AIR_DATA_RATE_000_24; // Air baud rate 2.4Kbps
  config.SPED.uartParity   = MODE_00_8N1; // Parity bit

  config.OPTION.subPacketSetting = SPS_200_00; // Packet size, 200bytes
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
  config.OPTION.transmissionPower = POWER_22; // Device power, puede ser 22,17,13 o 10 dBm

  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
  config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION; //FT_TRANSPARENT_TRANSMISSION

  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing

  //Establecer la configuracion del modulo LoRa
  e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
  rc.close(); //Terminar la configuracion

  Serial.println("Configuracion completa");
}