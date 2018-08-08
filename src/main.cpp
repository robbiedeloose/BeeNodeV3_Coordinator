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

////////////////////////////////////////// Serials /////////////////////////////
#define SerialMon Serial
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(softSerialAtRx, softSerialAtTx);
///////////////////////////////////////// GPRS /////////////////////////////////
// Select your modem:
#define TINY_GSM_MODEM_SIM800
// Increase RX buffer if needed
//#define TINY_GSM_RX_BUFFER 512
#include <PubSubClient.h>
#include <TinyGsmClient.h>
// Your GPRS credentials
const char apn[] = "hologram";
const char user[] = "";
const char pass[] = "";
// Server details
const char *broker = "m20.cloudmqtt.com";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
///////////////////////////////////// MQTT specific ////////////////////////////
// define topics based on CO ID

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
#define minuteInterval 15
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
  // uint8_t containsData;
  double timestamp;
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t weight;
  uint16_t humidity;
  uint8_t alarm;
};

struct LocalData_t {
  uint8_t baseId[4];
  int16_t baseTemp;
  uint16_t baseHum;
  uint16_t baseLux;
  uint16_t baseBat;
  uint16_t baseWind;
  uint16_t baseRain;
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
void getLocalData(LocalData_t *localDataAddress);
void getScaleData(LocalData_t *localDataAddress);
// gprs network functions
void gprsInit();
void gprsConnectNetwork();
void gprsEnd();
// sending data
void registerNodeMqtt();
// MQTT specific

// radio functions
void initRFRadio(uint8_t channel, uint16_t nodeAddress);
void checkForNetworkData(double timestamp);
// misc Functions
void printCurrentDateTime();
void clearPayloadBuffer();

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() { // clean
  Wire.begin();
  SerialMon.begin(9600); // SerialMon Start
  SerialAT.begin(9600);  // Serial port for GPRS
  delay(1000);
  // print some coordinator node information
  SerialMon.print(F("BeeNode v0.1"));
  beeNodeId.getId(coordId);
  SerialMon.print(", Id: CO");
  for (byte b : coordId)
    SerialMon.print(b, HEX);
  SerialMon.println();
  clearPayloadBuffer();
  battery.setRefInternal();
  myHumidity.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  initRFRadio(90, thisNode); // start nRF24l radio
  if (!rtc.begin())
    if (rtc.lostPower()) {
      Serial.println("RTC lost power");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  printCurrentDateTime();
  registerNodeMqtt();
  SerialMon.println("init complete");
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() { // clean
  now = rtc.now();
  // display current epoch and next send time in epoch
  double timestamp = now.unixtime();
  checkForNetworkData(timestamp); // network data available?
  if (timestamp >= epochCounter) {
    epochCounter = timestamp + (minuteInterval * 60L);
    LocalData_t localData;
    getLocalData(&localData);
    getScaleData(&localData);
    Serial.println("sending");
    // send data
    clearPayloadBuffer();
  }
  SerialMon.println("sleep");
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
    SerialMon.print(" Node ID: ");
    for (byte b : payload.id)
      SerialMon.print(b, HEX);
    SerialMon.println();
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
  SerialMon.print(" Array position ");
  SerialMon.println(bufferLocation); // print the buffer location that is used
  payLoadBuffer[bufferLocation].timestamp = timestamp;
  // copy temp array to next free buffer location
  for (int i = 0; i < 4; i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;
  payLoadBuffer[bufferLocation].alarm = payloadAddress->alarm;
}

void getLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill coordinatorId
    localDataAddress->baseId[i] = coordId[i];
  localDataAddress->baseTemp = myHumidity.readTemperature() * 100;
  localDataAddress->baseHum = myHumidity.readHumidity();
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
  localDataAddress->baseLux = lightMeter.readLightLevel();
}

void getScaleData(LocalData_t *localDataAddress) {
  for (int a = 0; a < 2; a++) {
    Wire.requestFrom(1, 25); // request 6 bytes from slave device #8
    delay(100);
    uint8_t i = 0;
    while (Wire.available()) {
      char c = Wire.read();
      if (c != '<') {
        localDataAddress->scales[a][i] = c;
        i++;
      } else if (c == '>') {
        break;
      }
    }
    for (int i = 0; i < 23; i++) {
      Serial.print(localDataAddress->scales[a][i]);
    }
    Serial.println();
  }
}

//////////// MQTT Code /////////////////////////////////////////////////////////
void registerNodeMqtt() {
  SerialMon.println(F("Register to NodeRed"));
  if (mqtt.connect("m20.cloudmqtt.com", "oseiokpx", "31IMHCdWxVVt")) {
   mqtt.publish("CO/regsiter","CO12345678");
 }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  //boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" OK");
  mqtt.publish("topicInit", "GsmClientTest started");
  mqtt.subscribe("topicLed");
  return mqtt.connected();
}

//////////// init gprs, connect and disconnect from network ////////////////////
void gprsInit() {
  // SerialMon.println(F("Initializing modem..."));
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("  Modem: "));
  SerialMon.println(modemInfo);
}

void gprsConnectNetwork() {
  SerialMon.print(F("  Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" OK"));
  SerialMon.print(F(" Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" OK"));
}

void gprsEnd() {
  modem.gprsDisconnect();
  SerialMon.println(F(" GPRS disconnected"));
}

//////////// init nrf radio ////////////////////////////////////////////////////
void initRFRadio(uint8_t channel, uint16_t nodeAddress) { // clean
  SerialMon.print(F("Channel: "));
  SerialMon.print(channel);
  SerialMon.print(", Node:  ");
  SerialMon.println(nodeAddress);

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

void printCurrentDateTime() {
  now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  Serial.println(now.unixtime(), DEC);
}
