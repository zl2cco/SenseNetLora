/*
  LoRa Simple Gateway/Node Exemple

  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

  created 05 August 2018
  by Luiz H. Cassettari
*/

#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <pga460.h>
#include "ArduinoLowPower.h"
#include <CRC32.h>

#define NODE_ID 1

CRC32 crc;

void onReceive(int packetSize);
boolean runEvery(unsigned long interval);
void onTxDone();
void LoRa_sendMessage(String message);
void LoRa_sendObj(obj_t* obj);
void LoRa_txMode();
void LoRa_rxMode();
void wakeupISR();

const long frequency = 915E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 8;          // change for your board; must be a hardware interrupt pin

PGA460 pga460(2, 3, &Serial1);

//const uint32_t sleeptime = 10000;

bool dbg = false;
int txdone = 0;

#define dbg_println(_s) if (dbg) SerialUSB.println(_s);
#define dbg_print(_s)   if (dbg) SerialUSB.print(_s);
void run_config();

void sleep_gpio() {
  digitalWrite(csPin, HIGH);
  digitalWrite(resetPin, LOW);
  detachInterrupt(digitalPinToInterrupt(irqPin));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  analogReference(AR_DEFAULT);

  SerialUSB.begin(9600);                   // initialize SerialUSB
  
  unsigned long now = millis();
  while ((!SerialUSB) && (millis() - now < 10000)) ;

  if (!SerialUSB) {
    dbg = false;
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LED_BUILTIN, INPUT);
    SerialUSB.end();
  }
  else {
    dbg = true;
  }

  pga460.begin();
  if (pga460.get_status() != 0) {
    dbg_print("PGA460 error ");
    dbg_println(pga460.get_status());
  }
  else {
    dbg_println("PGA460 initialised ");
  }
  //pga460.set_tv_gain(3);
 
 bool done = false;
  if (dbg) {      // In debug mode, wait for command ('n' - normal, 'c' - config)
    while (!done) {
      SerialUSB.println("Select 'n' for normal mode, or 'c' for configuration mode");
      SerialUSB.print("Command (n/c): ");

      while (SerialUSB.available() == 0) ;

      switch (SerialUSB.read()) {
        case 'n':
        case 'N':
          done=true;
          break;

        case 'c':
        case 'C':
          run_config();
          break;
      }
    }
  }

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    dbg_println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  dbg_println("LoRa init succeeded.");
  dbg_println();
  dbg_println("LoRa Simple Node");
  dbg_println("Only receive messages from gateways");
  dbg_println("Tx: invertIQ disable");
  dbg_println("Rx: invertIQ enable");
  dbg_println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

uint32_t counter = 0;
void loop() {
  //if (runEvery(1000)) { // repeat every 1000 millis
    obj_t obj;

    pga460.get_distance(&obj);
    obj.node_id = NODE_ID;
    obj.vbat = 2.0 * (3.3 * ((double)analogRead(A0) / 1024.0));
    obj.temp = 15.65;
    obj.crc32 = 0;

    crc.reset();
    crc.setPolynome(0xDEADBEEF);
    crc.add((uint8_t *)&obj, sizeof(obj));
    obj.crc32 = crc.getCRC();


    txdone++;
    LoRa_sendObj(&obj);

    dbg_print("Send Message! " + String(obj.distance_m));
    dbg_print(" Vbat:" + String(obj.vbat));

    if ((!dbg)&&(counter > 10)) {
      digitalWrite(LED_BUILTIN, LOW);
      while (!txdone) ;
      pga460.end();
      LoRa.end();
      sleep_gpio();
      LowPower.deepSleep(60000);

      analogReference(AR_DEFAULT);
      
      pga460.begin();
      LoRa.setPins(csPin, resetPin, irqPin);
      LoRa.begin(frequency);
      LoRa.onReceive(onReceive);
      LoRa.onTxDone(onTxDone);
      LoRa_rxMode();

      counter=0;
    }
    else {
      counter++;
      while (!txdone) ;
      delay(200);
    }
  //}
}



void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void LoRa_sendObj(obj_t* obj) {
  LoRa_txMode();                              // set tx mode
  LoRa.beginPacket();                         // start packet
  LoRa.write((uint8_t*)obj, sizeof(obj_t));  // add payload
  LoRa.endPacket(true);                       // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  dbg_print("Node Receive: ");
  dbg_println(message);
}

void onTxDone() {
  dbg_println(" TxDone");
  LoRa_rxMode();
  txdone--;
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


void run_config() {
  char done=false;
  uint8_t diag, data[128];

  while (!done) {
    SerialUSB.println("CONFIG MENU:");
    SerialUSB.println("  't' - run no-object burst and listen profile, average over 100");
    SerialUSB.println("  'q' - quit diagnostic menu");
    SerialUSB.print("Command (t): ");

    while (SerialUSB.available() == 0) ;

    switch (SerialUSB.read()) {
      case 't':
      case 'T':
        pga460.getNOBALdata(data, &diag);
        break;
      case 'q':
      case 'Q':
        done = true;
        break;

    }
  }
    
}