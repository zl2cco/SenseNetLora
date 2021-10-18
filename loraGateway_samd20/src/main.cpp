#include <Arduino.h>

#include <LoRa.h>
#include <U8g2lib.h>
#include <CRC32.h>


//
// OLED
//
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


//
// Lora
//
const long frequency = 915E6;  // LoRa Frequency

const int csPin = 2;          // LoRa radio chip select
const int resetPin = 3;        // LoRa radio reset
const int irqPin = 8;          // change for your board; must be a hardware interrupt pin

bool packet_received=false;

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
CRC32 crc;

// //
// // Functions
// //
boolean runEvery(unsigned long interval);
void onTxDone();
void onReceive(int packetSize);
void LoRa_sendMessage(String message);
void LoRa_txMode();
void LoRa_rxMode();
void lora_begin();
void printTick (uint32_t sec);


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  SerialUSB.begin(115200);
  SerialUSB.println("Lora Node (samd21)");
  SerialUSB.println(__FILE__);

  u8g2.begin();
  lora_begin();

  digitalWrite(LED_BUILTIN, LOW);  
}

uint32_t tick = millis();
uint32_t tick_sec = 0;
void loop() {
  // put your main code here, to run repeatedly:
  // printDataPacket();
  // SerialUSB.print(".");
  // delay(2000);


  if (millis() - tick > 1000) {
    tick_sec++;    
    //printTick(tick_sec);
    tick = millis();
  }

}


void printTick (uint32_t sec) {
if (!packet_received) {
    String str = String(sec);
    u8g2.setFont(u8g2_font_ncenR10_tr);  // choose a suitable font
    u8g2.setDrawColor(0);
    u8g2.drawBox(0,0,40,16);    
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 15, str.c_str());    // write something to the internal memory
    u8g2.sendBuffer();                   // transfer internal memory to the display  
  }
}



void lora_begin() {
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    SerialUSB.println("LoRa failed to initialise. Check your connections.");
  }

  SerialUSB.println("LoRa init succeeded.");

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
    
  //LoRa.enableCrc();
  LoRa_rxMode();  
}



void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  obj_t obj;

  if (packetSize == sizeof(obj_t)) { 
    packet_received=true;
    LoRa.readBytes((uint8_t*) &obj, sizeof(obj_t));
    float snr = LoRa.packetSnr();

    if ((obj.node_id < 10) && 
        (obj.distance_m >= 0.0) && (obj.distance_m < 5.0) && 
        (obj.vbat >= 0.0)       && (obj.vbat < 5.0) && 
        (obj.temp >= -20.0)     && (obj.temp < 100.0)
        ) {

        char str[255];

        sprintf(str, "%d, %f, %f, %f, 0, %f, 1", obj.node_id, obj.distance_m, obj.vbat, obj.temp, snr);

        SerialUSB.println(str);
        // SerialUSB.print(obj.node_id);
        // SerialUSB.print(", ");
        // SerialUSB.print(obj.distance_m);
        // SerialUSB.print(", ");
        // SerialUSB.print(obj.vbat);
        // SerialUSB.print(", ");
        // SerialUSB.print(obj.temp);
        // SerialUSB.print(", 0, ");
        // SerialUSB.print(snr);
        // SerialUSB.print(", ");
        // SerialUSB.println("1");


        sprintf(str, "%.2fm, %.2fV", obj.distance_m, obj.vbat);

        //u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB10_tr);  // choose a suitable font
        if (obj.node_id == 1) {
          u8g2.setDrawColor(0);
          u8g2.drawBox(0, 0, 127, 15);
          u8g2.setDrawColor(1);
          u8g2.drawStr(0, 15, str);    // write something to the internal memory
        }
        else {
          u8g2.setDrawColor(0);
          u8g2.drawBox(0, 16, 127, 15);
          u8g2.setDrawColor(1);
          u8g2.drawStr(0, 30, str);    // write something to the internal memory
        } 
        u8g2.sendBuffer();                   // transfer internal memory to the display  
    }
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    tick_sec=0;    
    packet_received=false;
  }
  else {
    //Serial.println("Not a valid package");
  }

}

void onTxDone() {
  //Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}







// #include <LoRa.h>
// #include <U8g2lib.h>



// //
// // OLED
// //
// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif
// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif

// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


// //
// // Lora
// //
// const long frequency = 915E6;  // LoRa Frequency

// const int csPin = 2;          // LoRa radio chip select
// const int resetPin = 3;        // LoRa radio reset
// const int irqPin = 8;          // change for your board; must be a hardware interrupt pin

// bool packet_received=false;

// //
// // Lora Packet
// //
// struct obj_t {
//     uint8_t   diag;
//     uint8_t   errors;
//     uint8_t   preset;
//     uint16_t  tof;
//     uint8_t   width;
//     uint8_t   peak_amp;
//     double    distance_m;
//     double    vbat;
// };


// //
// // Functions
// //
// boolean runEvery(unsigned long interval);
// void onTxDone();
// void onReceive(int packetSize);
// void LoRa_sendMessage(String message);
// void LoRa_txMode();
// void LoRa_rxMode();
// void lora_begin();
// void printTick (uint32_t sec);


// void setup() {
//   // put your setup code here, to run once:
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);
  
//   SerialUSB.begin(115200);
//   SerialUSB.println("Lora Node (samd21)");
//   SerialUSB.println(__FILE__);

//   u8g2.begin();
//   lora_begin();

//   digitalWrite(LED_BUILTIN, LOW);  
// }

// uint32_t tick = millis();
// uint32_t tick_sec = 0;
// void loop() {
//   // put your main code here, to run repeatedly:
//   // printDataPacket();
//   // SerialUSB.print(".");
//   // delay(2000);


//   if (millis() - tick > 1000) {
//     tick_sec++;    
//     printTick(tick_sec);
//     tick = millis();
//   }

// }


// void printTick (uint32_t sec) {
// if (!packet_received) {
//     String str = String(sec);
//     u8g2.setFont(u8g2_font_ncenR10_tr);  // choose a suitable font
//     u8g2.setDrawColor(0);
//     u8g2.drawBox(0,0,40,16);    
//     u8g2.setDrawColor(1);
//     u8g2.drawStr(0, 15, str.c_str());    // write something to the internal memory
//     u8g2.sendBuffer();                   // transfer internal memory to the display  
//   }
// }



// void lora_begin() {
//   LoRa.setPins(csPin, resetPin, irqPin);

//   if (!LoRa.begin(frequency)) {
//     SerialUSB.println("LoRa failed to initialise. Check your connections.");
//   }

//   SerialUSB.println("LoRa init succeeded.");

//   LoRa.onReceive(onReceive);
//   LoRa.onTxDone(onTxDone);
//   LoRa_rxMode();  
// }



// void LoRa_rxMode(){
//   LoRa.disableInvertIQ();               // normal mode
//   LoRa.receive();                       // set receive mode
// }

// void LoRa_txMode(){
//   LoRa.idle();                          // set standby mode
//   LoRa.enableInvertIQ();                // active invert I and Q signals
// }

// void LoRa_sendMessage(String message) {
//   LoRa_txMode();                        // set tx mode
//   LoRa.beginPacket();                   // start packet
//   LoRa.print(message);                  // add payload
//   LoRa.endPacket(true);                 // finish packet and send it
// }

// void onReceive(int packetSize) {
//   obj_t obj;
//   static uint32_t time_stamp=0;

//   if (LoRa.available() == sizeof(obj_t)) { 
//     packet_received=true;
//     LoRa.readBytes((uint8_t*) &obj, sizeof(obj_t));
//     float snr = LoRa.packetSnr();

//     SerialUSB.print(obj.distance_m);
//     SerialUSB.print(", ");
//     SerialUSB.print(obj.vbat);
//     SerialUSB.print(", ");
//     SerialUSB.println(snr);

//     String str = String(obj.distance_m) + "m " + String(obj.vbat) + "V";
//     u8g2.clearBuffer();                  // clear the internal memory
//     u8g2.setFont(u8g2_font_ncenB12_tr);  // choose a suitable font
//     u8g2.drawStr(0, 30, str.c_str());    // write something to the internal memory
//     u8g2.sendBuffer();                   // transfer internal memory to the display  

//     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//     tick_sec=0;    
//     packet_received=false;
//   }
//   else {
//     //Serial.println("Not a valid package");
//   }

// }

// void onTxDone() {
//   //Serial.println("TxDone");
//   LoRa_rxMode();
// }

// boolean runEvery(unsigned long interval)
// {
//   static unsigned long previousMillis = 0;
//   unsigned long currentMillis = millis();
//   if (currentMillis - previousMillis >= interval)
//   {
//     previousMillis = currentMillis;
//     return true;
//   }
//   return false;
// }

