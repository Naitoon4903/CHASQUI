// RECEPTOR

// Frecuencia base nodo LoRa
#define FREQUENCY_868

///***************************************************** LIBRERIAS
#include <LoRa_E220.h>
#include <HardwareSerial.h>

///**************************************************************** VARIABLES GENERALES
// ID de nodos LoRa
#define LORA_NODE1_ID  0x00F1

//GPIO ESP32 - LoRa
#define UART1_TX  26 // ESP32 TX -- E220 RX
#define UART1_RX  27 // ESP32 RX -- E220 TX
#define AUX_PIN   18 // AUX
#define M0_PIN    4  // M0
#define M1_PIN    5  // 

HardwareSerial LoRaSerial(1);
LoRa_E220 e220ttl(UART1_RX, UART1_TX, &LoRaSerial, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

////******************************************************************* FUNCIONES

void LoRa_Init(void);

//**********************************************************************************************Estructura para el manejo de datos
typedef struct
{
  uint16_t idNodo;
  uint16_t readingId;
  //int      numerito;
  float    loaddata_send[14];
} Message_t;

Message_t recvMsg  = {0};
uint16_t idMessage = 0;

///////////////*********************************************************************************** CODIGO GENERAL -- TENER CUIDADO
void setup() 
{
  //Puerto Serial
  Serial.begin(115200);
  // Configuracion de nodo LoRa
  LoRa_Init();
}

void loop() 
{
  if (e220ttl.available() > 1) 
  {
    //Serial.println("Mensaje Recibido");  //UTILIZAR PARA DEPURACION EN LA RECEPCION DE DATOS
    ResponseStructContainer rscRx = e220ttl.receiveMessage(sizeof(recvMsg));
    memcpy((uint8_t *)&recvMsg, (uint8_t *)rscRx.data, sizeof(recvMsg));

    // Verificar si la recepción fue exitosa
    if (rscRx.status.code == 1) 
    {
      //Serial.println("Datos Leidos"); ///UTILIZAR SI SE DESEA VERIFICAR LA RECEPCION DE DATOS
      idMessage = recvMsg.idNodo;

      if(idMessage == LORA_NODE1_ID)
      {
        //Serial.printf("Recibido, numerito: %i \r\n", recvMsg.numerito);
        Serial.print("Datos recibidos: ");
        for (int i = 0; i < 14; i++) {
            Serial.print(recvMsg.loaddata_send[i]);
            Serial.print(" ");
        }
        Serial.println();
      }
    } 
    else 
    {
      //Serial.println("Error al recibir datos\r\n"); //UTILIZAR SI SE DESEA VERIFICAR LA RECEPCION DE DATOS
    }
    rscRx.close();
  }
  //delay(500);
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
    Serial.printf("\r\nLectura de configuracion exitosa\r\n");
  }
  else {
    Serial.printf("\r\nError en la lectura de configuracion\r\n");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  //CONFIGURACION DE REGISTROS DEL MODULO LORA
  config.ADDH = 0x00; 
  config.ADDL = 0x1A;  

  config.CHAN = 67; // Communication channel

  config.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
  config.SPED.airDataRate  = AIR_DATA_RATE_000_24; // Air baud rate
  config.SPED.uartParity   = MODE_00_8N1; // Parity bit

  config.OPTION.subPacketSetting = SPS_200_00; // Packet size
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
  config.OPTION.transmissionPower = POWER_22; // Device power

  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
  config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION; //FT_TRANSPARENT_TRANSMISSION

  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing

  //Establecer la configuracion del modulo LoRa
  e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
  rc.close(); //Terminar la configuracion

  Serial.printf("Configuracion LoRa completa \r\n\r\n");
}