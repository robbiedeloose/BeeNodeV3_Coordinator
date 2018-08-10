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
#include "SparkFunHTU21D.h"
HTU21D myHumidity;
///////////////////////////////////// LUX //////////////////////////////////////
#include <BH1750.h>
BH1750 lightMeter;
///////////////////////////////////// RTC //////////////////////////////////////
//#include "uRTCLib.h"
// uRTCLib rtc(0x68, 0x57);
#include "RTClib.h"
RTC_DS3231 rtc;
double epochCounter = 0L;
#define minuteInterval 2
DateTime now;
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

struct PayloadBuffer_t {
  double timestamp;
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t humidity;
  uint8_t alarm;
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
PayloadBuffer_t payLoadBuffer[BUFFERSIZE];
////////////////////////////////////////////////////////////////////////////////
#define numberOfSensors 6

////////////////////////// FUNCTION DECLARATIONS ///////////////////////////////
// getting data
void fillBufferArray(Payload_t *payloadAddress, double timestamp);
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
void checkForNetworkData(double timestamp);
// misc Functions
void clearPayloadBuffer();

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(9600);
  SerialAT.begin(9600);
  delay(1000);

  Serial.print(F("BeeNode v0.1"));
  beeNodeId.getId(coordId);
  Serial.print(", Id: CO");
  for (byte b : coordId)
    Serial.print(b, HEX);
  Serial.println();

  clearPayloadBuffer();

  battery.setRefInternal();
  myHumidity.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  initRFRadio(90, thisNode); // start nRF24l radio
  if (!rtc.begin()) {
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  mqtt.setServer(broker, mqttPort);

  //////
  //////

  registerNodeMqtt();

  Serial.println("init complete");
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() {
  now = rtc.now();
  double timestamp = now.unixtime();
  checkForNetworkData(timestamp);
  if (timestamp >= epochCounter) {
    epochCounter = timestamp + (minuteInterval * 60L);
    LocalData_t localData;
    getLocalData(&localData);
    getScaleData(&localData);
    Serial.println("time!");
    sendMqttData(&localData);
    clearPayloadBuffer();
  }
  Serial.println("sleep");
  delay(500); // give serial time to complete before node goes to sleep
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}

//// Getting data //////////////////////////////////////////////////////////////
void checkForNetworkData(double timestamp) {
  network.update();
  RF24NetworkHeader header;
  Payload_t payload;

  while (network.available()) { // Any data on the network ready to read
    network.read(header, &payload, sizeof(payload));
    Serial.print(" Node ID: ");
    for (byte b : payload.id)
      Serial.print(b, HEX);
    Serial.println();
    fillBufferArray(&payload, timestamp);
  }
}

void fillBufferArray(Payload_t *payloadAddress, double timestamp) {
  uint8_t bufferLocation = 0;
  // get next free buffer location
  for (int i = 0; i < BUFFERSIZE; i++) {
    if (payLoadBuffer[bufferLocation].timestamp != 0)
      bufferLocation++;
  }
  Serial.print(" Array position ");
  Serial.println(bufferLocation); // print the buffer location that is used
  payLoadBuffer[bufferLocation].timestamp = timestamp;
  // copy temp array to next free buffer location
  for (int i = 0; i < 4; i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    if (payloadAddress->temp[i] == -12700) {
      payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
    } else {
      payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
    }
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;
  payLoadBuffer[bufferLocation].alarm = payloadAddress->alarm;
}

void getLocalData(LocalData_t *local) {
  for (uint8_t i = 0; i < 4; i++) // fill coordinatorId
    local->baseId[i] = coordId[i];
  local->baseTemp = myHumidity.readTemperature() * 100;
  local->baseHum = myHumidity.readHumidity();
  local->baseBat = battery.getVoltage() * 100; // Battery
  local->baseLux = lightMeter.readLightLevel();
}

void getScaleData(LocalData_t *local) {
  for (int a = 0; a < 2; a++) {
    Wire.requestFrom(1, 25); // request 6 bytes from slave device #8
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
  now = rtc.now();
  String s = String("CO");
  for (uint8_t i = 0; i < 4; i++) {
    s = s + String(coordId[i], HEX);
  }
  s = s + "," + String(now.unixtime());
  char b[12];
  s.toCharArray(b, s.length() + 1);
  if (mqtt.connect(b, mqttUser, mqttPswd)) {
    mqtt.publish("co/reg", b);
  }
  gprsEnd();
}

void sendMqttData(LocalData_t *local) {
  for (int b = 0; b < BUFFERSIZE; b++) {
    if (payLoadBuffer[b].timestamp != 0) {
      Serial.println("Send Data");
      String id = "";
      for (uint8_t i = 0; i < 4; i++) {
        id = id + String(payLoadBuffer[b].id[i], HEX);
      }
      Serial.println(id);
      char cId[10];
      id.toCharArray(cId, id.length() + 1);

      char dest[8] = "";
      char buf1[8] = ""; // = "strg1";
      char buf2[8] = ""; // = "string2";
      char buf3[8] = "";
      char buf4[8] = "";
      char buf5[8] = "";
      char buf6[8] = "";

      itoa(payLoadBuffer[b].temp[0], buf1, 10);
      itoa(payLoadBuffer[b].temp[1], buf2, 10);
      itoa(payLoadBuffer[b].temp[2], buf3, 10);
      itoa(payLoadBuffer[b].temp[3], buf4, 10);
      itoa(payLoadBuffer[b].temp[4], buf5, 10);
      itoa(payLoadBuffer[b].temp[5], buf6, 10);

      strcpy(dest, buf1);
      strcat(dest, ",");
      strcat(dest, buf2);
      strcat(dest, ",");
      strcat(dest, buf3);
      strcat(dest, ",");
      strcat(dest, buf4);
      strcat(dest, ",");
      strcat(dest, buf5);
      strcat(dest, ",");
      strcat(dest, buf6);
      strcat(dest, ",");
      strcat(dest, cId);

      Serial.println(dest);

      //  data = data + String(payLoadBuffer[b].timestamp) + ",";
      /*data = data + String() + ",";
      data = data + String(local->baseHum) + ",";
      data = data + String(local->baseLux) + ",";
      Serial.println(data.length());
      Serial.println(data);*/
    }
  }
  // mqtt.disconnect();
  // gprsEnd();
}

//////////// init gprs, connect and disconnect from network ////////////////////
void gprsResetModem() {
  modem.restart();
  // String modemInfo = modem.getModemInfo();
  // Serial.print(F("  Modem: "));
  // Serial.println(modemInfo);
}

void gprsConnectNetwork() {
  Serial.print("  Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" OK");
  Serial.print(" Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(F(" OK"));
}

void gprsEnd() {
  modem.gprsDisconnect();
  // Serial.println(F(" GPRS disconnected"));
}

//////////// init nrf radio ////////////////////////////////////////////////////
void initRFRadio(uint8_t channel, uint16_t nodeAddress) { // clean
  Serial.print("Channel: ");
  Serial.print(channel);
  Serial.print(", Node:  ");
  Serial.println(nodeAddress);

  SPI.begin();
  radio.begin();
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9);
}

//////////// Misc Functions ////////////////////////////////////////////////////
void clearPayloadBuffer() {
  for (int i = 0; i < BUFFERSIZE; i++) {
    payLoadBuffer[i].timestamp = 0L;
  }
}
