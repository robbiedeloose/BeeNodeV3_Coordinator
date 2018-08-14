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

struct payloadBuffer_t {
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
  char scales[2][24];
};

char s1[24];
char s2[24];
//char buf[120];
//char buf2[120] = "";
// Globalstruct array to collect data before Sending
#define BUFFERSIZE 6
payloadBuffer_t payloadBuffer[BUFFERSIZE];
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
void clearpayloadBuffer();

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(9600);
  SerialAT.begin(9600);
  delay(1000);

  Serial.print(F("BeeNode v0.1"));
  //beeNodeId.getId(coordId);
  Serial.print(F(", Id: CO"));
  for (byte b : coordId)
    Serial.print(b, HEX);
  Serial.println();
  sprintf(mqttClient, "CO%02X%02X%02X%02X",coordId[0],coordId[1],coordId[2],coordId[3]);
  Serial.println(mqttClient);

  battery.setRefInternal();
  //myHumidity.begin();
  //lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  initRFRadio(90, thisNode); // start nRF24l radio

  mqtt.setServer(broker, mqttPort);

  registerNodeMqtt();

  clearpayloadBuffer();
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
    clearpayloadBuffer();
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
    if (payloadBuffer[bufferLocation].temp[0] != 0)
      bufferLocation++;
  }
  Serial.print(F(" Array position "));
  Serial.println(bufferLocation); // print the buffer location that is used
  for (int i = 0; i < 4; i++)
    payloadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payloadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payloadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payloadBuffer[bufferLocation].bat = payloadAddress->bat;
}

void saveDataToEep(Payload_t *payloadAddress){
    uint8_t eepRomLocation = 1;
  // get next free buffer location
  for (int i = 1; i < BUFFERSIZE; i++) {
    char ad = 0;
    rtc.eeprom_read(i*25,ad,1);
    if (ad != 0)
      eepRomLocation++; //increase buffer location
    else
      break; // use current location
  }
  
  Serial.print(F(" Eeprom position "));
  Serial.println(eepRomLocation*25); // print the buffer location that is used
  for (int i = 0; i < 4; i++)
    rtc.eeprom_write((eepRomLocation*25)+i,payloadAddress->id[i]);
  for (int i = 0; i < numberOfSensors; i++)
    rtc.eeprom_write(30+(eepRomLocation*10)+(i*2),payloadAddress->temp[i]);
  rtc.eeprom_write(42+(eepRomLocation*10), payloadAddress->humidity);
  rtc.eeprom_write(44+(eepRomLocation*10), payloadAddress->bat);
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
  Wire.requestFrom(1, 24);
  delay(100);
  int i = 0;
  while (Wire.available()&& i < 22) {
    s1[i] = Wire.read();
    i++;
  }
  Wire.requestFrom(1, 24);
  delay(100);
  i = 0;
  while (Wire.available()&& i < 22) {
    s2[i] = Wire.read();
    i++;
  }
  s1[23] = 0;
  s2[23] = 0;

  for (size_t i = 0; i < 23; i++) {
  Serial.print(s1[i]);
  }
  Serial.println();
  Serial.println(s1);
  Serial.println(s2);
}

//////////// MQTT Code /////////////////////////////////////////////////////////
void registerNodeMqtt() {
  gprsResetModem();
  gprsConnectNetwork();
  if (mqtt.connect(mqttClient, mqttUser, mqttPswd)) {
    mqtt.publish("c/r", mqttClient);
  }
  gprsEnd();
}

void sendMqttData(LocalData_t *local) {
  gprsResetModem();
  gprsConnectNetwork();
  for (int b = 0; b < BUFFERSIZE; b++) {
    if (payloadBuffer[b].temp[0] != 0) {
      Serial.print(F("Send buffer "));
      Serial.println(b);
      if (mqtt.connect(mqttClient, mqttUser, mqttPswd)) {
        char buf[120] = ""; // als ik deze als globale variabele declareer werkt het ook niet
        sprintf(buf, "%02X%02X%02X%02X,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%d", payloadBuffer[b].id[0], payloadBuffer[b].id[1], payloadBuffer[b].id[2], payloadBuffer[b].id[3], payloadBuffer[b].temp[0], payloadBuffer[b].temp[1], payloadBuffer[b].temp[2], payloadBuffer[b].temp[3], payloadBuffer[b].temp[4], payloadBuffer[b].temp[5],payloadBuffer[b].humidity,payloadBuffer[b].bat,local->baseTemp,local->baseHum,local->baseLux);
        mqtt.publish("h/d", buf);
        // als ik deze lijnen uncomment werkt het ook niet
        //char buf2[80] = "";
        //sprintf(buf2, "%s,%s,%s", mqttClient,s1,s2);
        //mqtt.publish("c/s", buf2);
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
void clearpayloadBuffer() {
  for (int i = 0; i < BUFFERSIZE; i++) {
    payloadBuffer[i].temp[0] = 0;
  }
}
