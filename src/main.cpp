#include <Arduino.h>

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
///////////////////////////////////// MQTT specific ////////////////////////////
char mqttCl[12] = "";
///////////////////////////////////// RADIO ////////////////////////////////////
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
RF24 radio(radioPin1, radioPin2); // start RF24 communication layer
RF24Network network(radio);       // start RF24 network layer
const uint16_t thisNode = 00;     // Coordinator address
////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
///////////////////////////////////// HUMIDITY /////////////////////////////////
//#include "SparkFunHTU21D.h"
//HTU21D myHumidity;
///////////////////////////////////// LUX //////////////////////////////////////
//#include <BH1750.h>
//BH1750 lightMeter;
///////////////////////////////////// RTC //////////////////////////////////////
#include "uRTCLib.h"
uRTCLib rtc(0x68, 0x57);
const uint8_t interval = 2;
uint8_t nextSend;
///////////////////////////////////// EEPROM ///////////////////////////////////
// EEPROM address locations
#include <EEPROM.h>      //EEPROM
#define EEPRomDeviceId 1 // 1 byte for #, 4 bytes for ID
#define EEPRomOptions 6
uint8_t coordId[4];
//////////////////////////////// own libraries /////////////////////////////////
#include "RandomNodeId.h" // NodeId
RandomNodeId beeNodeId;
#include "MesureVoltageInternal.h" // Internal Voltage measurement
float iREF = 1.1;
MesureVoltageInternal battery(iREF);

///////////////////////////////////// STRUCTS & STRUCT ARAYS ///////////////////
// Structure of our payload coming from router and end devices
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

struct LocalData_t {
  uint8_t baseId[4];
  int16_t baseTemp;
  uint16_t baseHum;
  uint16_t baseLux;
  uint16_t baseBat;
  char scales[2][23];
};

// Globalstruct array to collect data before Sending
#define BUFFERSIZE 6
plBuffer_t plBuffer[BUFFERSIZE];
////////////////////////////////////////////////////////////////////////////////
#define numberOfSensors 6

////////////////////////// FUNCTION DECLARATIONS ///////////////////////////////
// getting data
void fillBufferArray(Payload_t *payloadAddress);
void getLocalData(LocalData_t *local);
void getScaleData(LocalData_t *local);
// gprs network functions
void gprsResetModem();
void gprsConnectNetwork();
void gprsEnd();
// sending data
void registerNodeMqtt();
// MQTT specific
void sendMqttData(LocalData_t *local);
// radio functions
void initRFRadio(uint8_t channel, uint16_t nodeAddress);
void checkForNetworkData();
// misc Functions
void clearplBuffer();

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(9600);
  SerialAT.begin(9600);
  delay(1000);

  Serial.print(F("BeeNode v0.1"));
  beeNodeId.getId(coordId);
  Serial.print(F(", Id: CO"));
  for (byte b : coordId)
    Serial.print(b, HEX);
  Serial.println();
  sprintf(mqttCl, "CO%02X%02X%02X%02X",coordId[0],coordId[1],coordId[2],coordId[3]);


  battery.setRefInternal();
  //myHumidity.begin();
  //lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  initRFRadio(90, thisNode); // start nRF24l radio

  mqtt.setServer(broker, mqttPort);

  registerNodeMqtt();

  clearplBuffer();
  rtc.refresh();
  nextSend = rtc.minute();
  //Serial.println(F("init complete"));
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() {
  checkForNetworkData();
  rtc.refresh();
  if (nextSend <= rtc.minute()) {
    nextSend = rtc.minute() + interval;
    if (nextSend >= 60)
      nextSend = nextSend - 60;
    Serial.println(F("timer tripped"));
    LocalData_t localData;
    getLocalData(&localData);
    getScaleData(&localData);
    sendMqttData(&localData);
    clearplBuffer();
  }
  Serial.println(F("sleep"));
  delay(500); // give serial time to complete before node goes to sleep
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}

//// Getting data //////////////////////////////////////////////////////////////
void checkForNetworkData() {
  network.update();
  RF24NetworkHeader header;
  Payload_t payload;

  while (network.available()) { // Any data on the network ready to read
    network.read(header, &payload, sizeof(payload));
    Serial.print(F(" Node ID: "));
    for (byte b : payload.id)
      Serial.print(b, HEX);
    Serial.println();
    fillBufferArray(&payload);
  }
}

void fillBufferArray(Payload_t *payloadAddress) {
  uint8_t bufferLocation = 0;
  // get next free buffer location
  for (int i = 0; i < BUFFERSIZE; i++) {
    if (plBuffer[bufferLocation].temp[0] != 0)
      bufferLocation++;
  }
  Serial.print(F(" Array position "));
  Serial.println(bufferLocation); // print the buffer location that is used
  for (int i = 0; i < 4; i++)
    plBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    plBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  plBuffer[bufferLocation].humidity = payloadAddress->humidity;
  plBuffer[bufferLocation].bat = payloadAddress->bat;
}

void getLocalData(LocalData_t *local) {
  for (uint8_t i = 0; i < 4; i++) // fill coordinatorId
    local->baseId[i] = coordId[i];
  local->baseTemp = 123;//myHumidity.readTemperature() * 100;
  local->baseHum = 123;//myHumidity.readHumidity() * 100;
  local->baseBat = battery.getVoltage() * 100; // Battery
  local->baseLux = 1234; //lightMeter.readLightLevel();
}

void getScaleData(LocalData_t *local) {
  for (int a = 0; a < 2; a++) {
    Wire.requestFrom(1, 25);
    delay(100);
    uint8_t i = 0;
    while (Wire.available()) {
      char c = Wire.read();
      if (c != '<') {
        local->scales[a][i] = c;
        i++;
      } else if (c == '>') {
        break;
      }
    }
  }
}

//////////// MQTT Code /////////////////////////////////////////////////////////
void registerNodeMqtt() {
  gprsResetModem();
  gprsConnectNetwork();
  if (mqtt.connect(mqttCl, mqttUser, mqttPswd)) {
    mqtt.publish("c/r", mqttCl);
  }
  gprsEnd();
}

void sendMqttData(LocalData_t *local) {
  gprsResetModem();
  gprsConnectNetwork();
  for (int b = 0; b < BUFFERSIZE; b++) {
    if (plBuffer[b].temp[0] != 0) {
      Serial.print(F("Send buffer "));
      Serial.println(b);
      if (mqtt.connect(mqttCl, mqttUser, mqttPswd)) {
        char buf[120] = "";
        sprintf(buf, "%02X%02X%02X%02X,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%d", plBuffer[b].id[0], plBuffer[b].id[1], plBuffer[b].id[2], plBuffer[b].id[3], plBuffer[b].temp[0], plBuffer[b].temp[1], plBuffer[b].temp[2], plBuffer[b].temp[3], plBuffer[b].temp[4], plBuffer[b].temp[5],plBuffer[b].humidity,plBuffer[b].bat,local->baseTemp,local->baseHum,local->baseLux);
        mqtt.publish("h/d", buf);
      }
      /*
      data = data + String(local->baseHum)
      data = data + String(local->baseLux)
      */
    }
  }
  mqtt.disconnect();
  gprsEnd();
}

//////////// init gprs, connect and disconnect from network ////////////////////
void gprsResetModem() {
  modem.restart();
  String modemInfo = modem.getModemInfo();
  Serial.print(F(" Modem: "));
  Serial.println(modemInfo);
}

void gprsConnectNetwork() {
  Serial.print(F(" Waiting for network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }
  Serial.println(F(" OK"));
  Serial.print(F(" Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(F(" fail"));
    delay(10000);
    return;
  }
  Serial.println(F(" OK"));
}

void gprsEnd() {
  modem.gprsDisconnect();
}

//////////// init nrf radio ////////////////////////////////////////////////////
void initRFRadio(uint8_t channel, uint16_t nodeAddress) { // clean
  Serial.print(F("Channel: "));
  Serial.print(channel);
  Serial.print(F(", Node: "));
  Serial.println(nodeAddress);

  SPI.begin();
  radio.begin();
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9);
}

//////////// Misc Functions ////////////////////////////////////////////////////
void clearplBuffer() {
  for (int i = 0; i < BUFFERSIZE; i++) {
    plBuffer[i].temp[0] = 0;
  }
}
