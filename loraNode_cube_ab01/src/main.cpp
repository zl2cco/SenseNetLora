/* Heltec Automation Receive communication test example
 *
 * Function:
 * 1. Receive the same frequency band lora signal program
 * 
 * 
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>               
#include "HT_SH1107Wire.h"
#include <CRC32.h>

#define NODE_ID 2


/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


//#define RX_TIMEOUT_VALUE                            1000
//#define BUFFER_SIZE                                 30 // Define the payload size here

//char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t rssi,rxSize;

//
// Lora Packet
//
struct obj_t {
    uint8_t   node_id;
    double    distance_m;
    double    vbat;
    double    temp;
    uint32_t  crc32;
};
obj_t obj;
CRC32 crc;
volatile bool txing = false;
int packet_count=0;

//
// Low Power
//
#define timetillsleep 500
#define timetillwakeup 30000
static TimerEvent_t sleep;
static TimerEvent_t wakeUp;
uint8_t lowpower=1;

//
// OLED
//
SH1107Wire  oled(0x3c, 500000, SDA, SCL ,GEOMETRY_128_64,GPIO10); // addr, freq, sda, scl, resolution, rst

//
// Functions
//
void VextON(void);
void VextOFF(void); //Vext default OFF;
//void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void radioBegin(); 
void radioStop();
void OnTxDone();
void OnTxTimeout();
void onSleep();
void onWakeUp();


/****************************************************************************[ setup ]
 * Main setup
 ****************************************************************************/
void setup() {
    Serial.begin(115200);

    //VextON();
    //delay(100);

    // Initialising the UI will init the oled too.
    //oled.init();
    //oled.setFont(ArialMT_Plain_24);

    rssi=0;
    obj.distance_m = 0.55;
	
    radioBegin();

  //TimerInit( &sleep, onSleep );
  TimerInit( &wakeUp, onWakeUp );
  onSleep();
}



/****************************************************************************[ loop ]
 * Main loop 
 ****************************************************************************/
void loop()
{
  if(lowpower){
    lowPowerHandler();
  }

  if (!txing) {
    obj.node_id = NODE_ID;
    obj.distance_m = obj.distance_m + 0.1;
    obj.vbat = (double) getBatteryVoltage() / 1000;
    obj.temp = 17.65;
    obj.crc32 = 0;

    if (obj.distance_m > 3.0) obj.distance_m = 0.55;

    crc.reset();
    crc.setPolynome(0xDEADBEEF);
    crc.add((uint8_t*) &obj, sizeof(obj));
    obj.crc32 = crc.getCRC();

    //turnOnRGB(COLOR_SEND,0);
    txing = true;	
    Radio.Send( (uint8_t *)&obj, sizeof(obj_t) ); //send the package out

    while (txing) ;
    if (packet_count++ > 10) {
      packet_count=0;
      onSleep();
    }
  }
}


/****************************************************************************
 * Vext control functions - drive OLED power
 ****************************************************************************/
void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}



/****************************************************************************
 * Radio begin and stop functions
 ****************************************************************************/
void radioBegin()
{
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 

}

void radioStop() 
{
  Radio.Sleep();
}



/****************************************************************************
 * Event handlers for transmit
 ****************************************************************************/
void OnTxDone( void )
{
//  Serial.print(".");
//  turnOnRGB(0,0);
  txing = false;	
}
void OnTxTimeout( void )
{
//    Serial.print("x");
//    turnOnRGB(0,0);
    txing = false;	
}




/****************************************************************************
 * Event handlers for sleep modes
 ****************************************************************************/
void onSleep()
{
//  Serial.printf("\r\nGoing into lowpower mode, %d ms later wake up.\r\n",timetillwakeup);
  lowpower=1;
//  turnOffRGB();
  radioStop();
  //timetillwakeup ms later wake up;
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );
}
void onWakeUp()
{
//  Serial.printf("Woke up, %d ms later into lowpower mode.\r\n",timetillsleep);
  lowpower=0;
  txing = false;	
  radioBegin();
  //timetillsleep ms later into lowpower mode;
  //TimerSetValue( &sleep, timetillsleep );
  //TimerStart( &sleep );
}





// void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
// {
//   obj_t obj;
  
//     rssi=rssi;
//     rxSize=size;
//     if (rxSize == sizeof(obj_t)) {    
//       memcpy(&obj, payload, size );
//       turnOnRGB(COLOR_RECEIVED,0);
//       Radio.Sleep( );
//       uint16_t voltage = getBatteryVoltage();
      
//       Serial.printf("\r\nreceived obj ");
//       Serial.print(voltage);
//       Serial.print(" Vint, ");
//       Serial.print(obj.distance_m);
//       Serial.print(" m, ");
//       Serial.print(obj.vbat);
//       Serial.printf(" V with rssi %d , length %d\r\n",rssi,rxSize);

//       oled.clear();
//       oled.setTextAlignment(TEXT_ALIGN_LEFT);
//       oled.drawString(0, 0, String(obj.distance_m) + "m");
//       oled.drawString(0, 30, String(obj.vbat) + "(" + String((float)voltage/1000) + ")V");
//       // write the buffer to the oled
//       oled.display();
//     }
//     else {
//       memcpy(rxpacket, payload, size );
//       rxpacket[size]='\0';
//       turnOnRGB(COLOR_RECEIVED,0);
//       Radio.Sleep( );
//      Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
//     }
// }
