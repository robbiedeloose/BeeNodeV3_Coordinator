#include <Arduino.h>

#define DEBUG
#define DEBUG2
#define DEBUGGPRS
#ifdef DEBUG
 #define debugPrint(x)  Serial.print (x)
 #define debugPrintLn(x)  Serial.println (x)
#else
 #define debugPrint(x)
 #define debugPrintLn(x)
#endif

#ifdef DEBUG2
 #define debug2Print(x)  Serial.print (x)
 #define debug2PrintLn(x)  Serial.println (x)
 #define debug2PrintHex(x)  Serial.println (x, HEX)
#else
 #define debug2Print(x)
 #define debug2PrintLn(x)
 #define debug2PrintHex(x)
#endif

#ifdef DEBUGGPRS
 #define debugGprsPrint(x)  Serial.print (x)
 #define debugGprsPrintLn(x)  Serial.println (x)
 #define debug2GprsPrintHex(x)  Serial.println (x, HEX)
#else
 #define debugGprsPrint(x)
 #define debugGprsPrintLn(x)
 #define debugGprsPrintHex(x)
#endif

////////////////////////////////////////////////////////////////////////////////
#define radioPin1 8 // RF24
#define radioPin2 9 // RF24

#define softSerialAtRx 4
#define softSerialAtTx 5
#define resetPin A2 // resetting esp/gprs module

#define MISO 11
#define MOSI 12
#define CLK 13

#define seedRef A3 // seed for random function

///////////////////////////////////////// GPRS /////////////////////////////////
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(softSerialAtRx, softSerialAtTx);
#define TINY_GSM_MODEM_SIM800
#include <PubSubClient.h>
#include <TinyGsmClient.h>
// Your GPRS credentials
const char apn[] = "hologram";
const char user[] = "";
const char pass[] = "";
// Server details
const char *broker = "m20.cloudmqtt.com";
const uint16_t mqttPort = 15913;
const char *mqttUser = "oseiokpx";
const char *mqttPswd = "31IMHCdWxVVt";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
char mqttClient[12] = "";
///////////////////////////////////// RADIO ////////////////////////////////////
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
RF24 radio(radioPin1, radioPin2); // start RF24 communication layer
RF24Network network(radio);       // start RF24 network layer
const uint16_t thisNode = 00;     // Coordinator address
///////////////////////////////////// Wire /////////////////////////////////////
#include <Wire.h>
///////////////////////////////////// RTC //////////////////////////////////////
#include "uRTCLib.h"
uRTCLib rtc(0x68, 0x57);
const uint8_t interval = 2;
uint8_t nextSend;
uint8_t coordId[4] = {0xAA,0xFF,0x00,0xFF};
//////////////////////////////// own libraries /////////////////////////////////
#include "MesureVoltageInternal.h" // Internal Voltage measurement
float iREF = 1.1;
MesureVoltageInternal battery(iREF);
///////////////////////////////////// STRUCTS & STRUCT ARAYS ///////////////////
struct Payload_t {
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t weight;
  uint16_t humidity;
  uint8_t alarm;
};

struct plBuffer_t {
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t humidity;
};

char buf[60];
char buf2[50];
char bufLocal[16] = "3412,6732,33333";
plBuffer_t plBuffer[7];
////////////////////////////////////////////////////////////////////////////////
#define numberOfSensors 6


void clearplBuffer() {
  for (int i = 0; i < 7; i++) {
    plBuffer[i].temp[0] = 0;
  }
}

//////////// init gprs, connect and disconnect from network ////////////////////
void gprsResetModem() {
  modem.restart();
  String modemInfo = modem.getModemInfo();
  debugGprsPrint(F(" Modem: "));
  debugGprsPrintLn(modemInfo);
}

void gprsConnectNetwork() {
  debugPrint(F(" Waiting for network..."));
  if (!modem.waitForNetwork()) {
    debugPrintLn(F(" fail"));
    delay(10000);
    return;
  }
  debugGprsPrintLn(F(" OK"));
  debugGprsPrint(F(" Connecting to "));
  debugGprsPrint(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    debugGprsPrintLn(F(" fail"));
    delay(20000);
    return;
  }
  debugGprsPrintLn(F(" OK"));
}

void gprsEnd() {
  modem.gprsDisconnect();
}

//////////// init nRf radio ////////////////////////////////////////////////////
void initRFRadio(uint8_t channel, uint16_t nodeAddress) { // clean
  debugPrint(F("Channel: "));
  debugPrint(channel);
  debugPrint(F(", Node: "));
  debugPrintLn(nodeAddress);

  SPI.begin();
  radio.begin();
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9);
}

//////////// Mqtt functions ////////////////////////////////////////////////////

void registerNodeMqtt() {
  gprsResetModem();
  gprsConnectNetwork();
  if (mqtt.connect(mqttClient, mqttUser, mqttPswd)) {
    debug2PrintLn("Mqtt connected");
    mqtt.publish("c/r", mqttClient);
  }
  gprsEnd();
}

void sendScaleMqtt(){
  gprsResetModem();
  gprsConnectNetwork();
  if (mqtt.connect(mqttClient, mqttUser, mqttPswd)) {
    debug2PrintLn("Mqtt connected");
    mqtt.publish("c/s", buf);
  }
  mqtt.disconnect();
  gprsEnd();
}

void sendDataMqtt() {
  gprsResetModem();
  gprsConnectNetwork();
  for (int b = 0; b < 1; b++) {
    if (plBuffer[b].temp[0] != 0) {
      debugPrint(F("Send buffer "));
      debugPrintLn(b);
      sprintf(buf2, "%02X%02X%02X%02X,%d,%d,%d,%d,%d,%d,%u,%u,%s", plBuffer[b].id[0], plBuffer[b].id[1], plBuffer[b].id[2], plBuffer[b].id[3], plBuffer[b].temp[0], plBuffer[b].temp[1], plBuffer[b].temp[2], plBuffer[b].temp[3], plBuffer[b].temp[4], plBuffer[b].temp[5],plBuffer[b].humidity,plBuffer[b].bat,bufLocal);
      //Serial.println(buf2);
      mqtt.connect(mqttClient, mqttUser, mqttPswd);
      mqtt.publish("h/d", buf2);
    }
  }
  mqtt.disconnect();
  gprsEnd();
}

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(9600);
  SerialAT.begin(9600);
  delay(1000);

  debugPrint(F("BeeNode v0.1"));
  //beeNodeId.getId(coordId);
  debugPrint(F(", Id: CO"));
  for (byte b : coordId)
    Serial.print(b, HEX);
  Serial.println();
  sprintf(mqttClient, "CO%02X%02X%02X%02X",coordId[0],coordId[1],coordId[2],coordId[3]);
  debugPrintLn(mqttClient);

  battery.setRefInternal();
  //myHumidity.begin();
  //lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  initRFRadio(90, thisNode); // start nRF24l radio
  mqtt.setServer(broker, mqttPort);
  registerNodeMqtt();

  rtc.refresh();
  nextSend = rtc.minute();
  clearplBuffer();
}

void getScaleData() {
  char sbuf[2][24];
  int i = 0;
  for (size_t a = 0; a < 2; a++) {
    i = 0;
    Wire.requestFrom(1, 24);
    //delay(100);
    while (Wire.available() && i < 23) {
      sbuf[a][i] = Wire.read();
      i++;
    }
    sbuf[a][i] = '\0';
  }
  sprintf(buf, "%s,%s,%s", mqttClient, sbuf[0], sbuf[1]);
  debug2PrintLn(buf);
}

void fillBufferArray(Payload_t *payloadAddress) {
  uint8_t bufferLocation = 0;
  for (int i = 0; i < 7; i++) {
    if (plBuffer[bufferLocation].temp[0] != 0)
      bufferLocation++;
  }
  debugPrint(F(" Array position "));
  debugPrintLn(bufferLocation); // print the buffer location that is used
  for (int i = 0; i < 4; i++)
    plBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    plBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  plBuffer[bufferLocation].humidity = payloadAddress->humidity;
  plBuffer[bufferLocation].bat = payloadAddress->bat;
}

void fillBufferArray(Payload_t *payloadAddress) {
  uint8_t bufferLocation = 0;
  for (int i = 0; i < 7; i++) {
    if (plBuffer[bufferLocation].temp[0] != 0)
      bufferLocation++;
  }
  debugPrint(F(" Array position "));
  debugPrintLn(bufferLocation); // print the buffer location that is used
  for (int i = 0; i < 4; i++)
    plBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    plBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  plBuffer[bufferLocation].humidity = payloadAddress->humidity;
  plBuffer[bufferLocation].bat = payloadAddress->bat;
}

void saveDataToEep(Payload_t *payloadAddress){
    uint8_t eepRomLocation = 1;
  // get next free buffer location
  for (int i = 1; i < BUFFERSIZE; i++) {
    char ad = 0;
    int b = 1;
    rtc.eeprom_read((i*25),ad,b);
    if (ad != 0)
      eepRomLocation++; //increase buffer location
    else
      break; // use current location
  }
  

//// Getting data //////////////////////////////////////////////////////////////
void checkForNetworkData() {
  network.update();
  RF24NetworkHeader header;
  Payload_t payload;

  while (network.available()) { // Any data on the network ready to read
    network.read(header, &payload, sizeof(payload));
    debug2Print(F(" Node ID: "));
    for (byte b : payload.id)
      debug2PrintHex(b);
    debug2PrintLn();
    fillBufferArray(&payload);
  }
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() {
  checkForNetworkData();
  rtc.refresh();
  if (nextSend <= rtc.minute()) {
    nextSend = rtc.minute() + interval;
    if (nextSend >= 60)
      nextSend = nextSend - 60;
    debugPrintLn(F("timer tripped"));
    getScaleData();
    sendScaleMqtt();
  }
  debugPrintLn(F("sleep"));
  Serial.write('S');
  delay(250); // give serial time to complete before node goes to sleep
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}
