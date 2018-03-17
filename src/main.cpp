#include <Arduino.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// PINS
//#define ledPin 6 // might change to buzzerPin
#define radioPin1 7
#define radioPin2 8
#define softSerialRx A0
#define softSerialTx A1
#define resetPin A2
//#define alarmPin 3
#define seedRef A3

// wifi / ...
#include <SoftwareSerial.h>
SoftwareSerial esp8266Module(softSerialRx, softSerialTx); // RX, TX
// radio
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
RF24 radio(7, 8);             // start RF24 communication layer
RF24Network network(radio);   // start RF24 network layer
const uint16_t thisNode = 00; // Coordinator address

// humidity
#include "SparkFunHTU21D.h"
#include <Wire.h>
HTU21D myHumidity;

// EEPROM address locations
#include <EEPROM.h>      //EEPROM
#define EEPRomDeviceId 1 // 1 byte for #, 4 bytes for ID
#define EEPRomOptions 6
uint8_t nodeId[4];

// own libraries //
// NodeId
#include "RandomNodeId.h"
RandomNodeId beeNodeId;
// Internal Voltage
#include "MesureVoltageInternal.h"
float iREF = 1.1;
MesureVoltageInternal battery(iREF);

#define numberOfSensors 3

// Structure of our payload coming from router and end devices
struct Payload_t {
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
};

void postData(Payload_t *payloadAddress, LocalData_t *localDataAddress) {
  // Post Hive Data
  // WIFI VERSION

  Serial.println("Resetting ESP");
  digitalWrite(resetPin, LOW);
  delay(1000);
  digitalWrite(resetPin, HIGH);
  Serial.println("Waiting for ESP to start");
  delay(3000);
  Serial.println("Sending data to ESP");

  Serial.println("Posting Data");

  esp8266Module.print("http://192.168.10.192:1880/hiveonly");

  esp8266Module.print("?");
  esp8266Module.print("nodeId");
  esp8266Module.print("=");
  for (byte b : payloadAddress->id)
    esp8266Module.print(b, HEX);

  for (int i = 0; i < numberOfSensors; i++) {
    esp8266Module.print("&");
    esp8266Module.print("temp");
    esp8266Module.print(i + 1);
    esp8266Module.print("=");
    if (payloadAddress->temp[i] == -12700)
      esp8266Module.print("-");
    else
      esp8266Module.print(payloadAddress->temp[i]);
  }

  esp8266Module.print("&");
  esp8266Module.print("hum");
  esp8266Module.print("=");
  if (payloadAddress->humidity == 1500)
    esp8266Module.print("-");
  else
    esp8266Module.print(payloadAddress->humidity);

  esp8266Module.print("&");
  esp8266Module.print("bat");
  esp8266Module.print("=");
  esp8266Module.print(payloadAddress->bat);

  delay(1000);

  Serial.println("ESP going to sleep");
}



void addLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill nodeId
    localDataAddress->baseId[i] = nodeId[i];
  localDataAddress->baseTemp = 1234;
  localDataAddress->baseHum = 5678;
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
}

void startCustomESP() {
  // esp8266Module
  esp8266Module.begin(9600);
  delay(500); //------------
  Serial.println("Soft Serial started at 9600");
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
}

void startRFRadio(uint8_t channel, uint16_t nodeAddress) {
  Serial.print("Starting rf radio. ");
  Serial.print("Channel: ");
  Serial.print(channel);
  Serial.print(", NodeAddress:  ");
  Serial.println(nodeAddress);

  SPI.begin();
  radio.begin();
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9); // Sets the WDT to trigger every second
}

void checkForNetworkData() {
  // check network communication regularly
  network.update();

  RF24NetworkHeader header; // create header variable
  Payload_t payload;        // create payload variable
  // Any data on the network ready to read
  while (network.available()) {
    // If so, grab it and print it out
    network.read(header, &payload, sizeof(payload));

    // Add coordinator Data
    LocalData_t localData;
    addLocalData(&localData);

    // Display Node Data
    Serial.println("Remote Data ----------------------------------------------");
    Serial.print("The node this is from: ");
    Serial.println(header.from_node);
    Serial.print("Node ID: ");
    for (byte b : payload.id)
      Serial.print(b, HEX);
    Serial.println();
    for (int i = 0; i < numberOfSensors; i++) {
      Serial.print("Temperature");
      Serial.print(i + 1);
      Serial.print(": ");
      if (payload.temp[i] == -12700)
        Serial.println("-");
      else
        Serial.println(payload.temp[i], DEC);
    }
    Serial.print(" Battery status: ");
    Serial.println(payload.bat, DEC);

    // Display Local Data
    Serial.println("Local Data ----------------------------------------------");
    Serial.print("Base bat: ");
    Serial.println(localData.baseBat, DEC);
    Serial.print("Base temp: ");
    Serial.println(localData.baseTemp, DEC);
    Serial.print("Base hum: ");
    Serial.println(localData.baseHum, DEC);

    // sendDataToESP();
    postData(&payload, &localData);
  }
}

// setup functions
void initCoordinator(){
  battery.setRefInternal(); // Set voltage reference
  beeNodeId.getId(nodeId); // send array to fill as parameter
  for (byte b : nodeId)
    Serial.print(b, HEX);
    Serial.println();
}

void setup() {
  // put your setup code here, to run once:
  // Serial Start
  Serial.begin(9600);
  Serial.println("BeeNode Coordinator v0.1");
  initCoordinator();
  startCustomESP();
  startRFRadio(90, thisNode);
  delay(10000); // delay for reprogramming purposses

  beeNodeId.getId(nodeId); // send array to fill as parameter
  battery.setRefInternal();
}

void loop() {
  checkForNetworkData();
  Serial.println("Node going to sleep");
  delay(500);
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}
