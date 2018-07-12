//GPRS Branch

#include <Arduino.h>

#define DEBUG

// Change between Wifi and GPRS
//#define GPRS
#define WIFI

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

//LUX
#include <Wire.h>
#include <AS_BH1750.h>
AS_BH1750 sensor;

// own libraries //
// NodeId
#include "RandomNodeId.h"
RandomNodeId beeNodeId;
// Internal Voltage
#include "MesureVoltageInternal.h"
float iREF = 1.1;
MesureVoltageInternal battery(iREF);

#define numberOfSensors 6


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
  uint8_t containsData;
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

// Globalstruct array to collect data before Sending
#define BUFFERSIZE 12
PayloadBuffer_t payLoadBuffer[BUFFERSIZE];

// setup functions
void initCoordinator();
void startRFRadio(uint8_t channel, uint16_t nodeAddress);
void startCustomESP();

// getting data
void checkForNetworkData();
void fillBufferArray(Payload_t *payloadAddress);
void addLocalData(LocalData_t *localDataAddress);

// posting data
void postDataWifi(Payload_t *payloadAddress, LocalData_t *localDataAddress);

void setup() {
  // put your setup code here, to run once:
  initCoordinator();
  startCustomESP();
  startRFRadio(90, thisNode);
  Serial.println("Started RF Radio");
  delay(10000); // delay for reprogramming purposses
Serial.println("Get BeeNodeId");
  beeNodeId.getId(nodeId); // send array to fill as parameter
  Serial.println("Set voltage Ref");
  battery.setRefInternal();
  Serial.println("End Setup");
}

//// setup functions ///////////////////////////////////////////////////////////
void initCoordinator(){
  Serial.begin(9600); // Serial Start
  Serial.println("BeeNode Coordinator v0.1");
  battery.setRefInternal(); // Set voltage reference
  beeNodeId.getId(nodeId); // send array to fill as parameter
  myHumidity.begin(); // start humidity sensor
  // for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown) use: sensor.begin(RESOLUTION_NORMAL, false);
  // Initialize sensor. if sensor is not present, false is returned
  if(sensor.begin()) {
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not present");
  }
  Serial.print("Coordinator Id: CO");
  for (byte b : nodeId)
    Serial.print(b, HEX);
  Serial.println();
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
  Serial.println("SPI started");
  radio.begin();
  Serial.println("Radio started");
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  Serial.println("Network started");
  network.setup_watchdog(9); // Sets the WDT to trigger every second
  Serial.println("Watchdog set");
}

void loop() {
  Serial.println("Start Loop");
  checkForNetworkData();
  Serial.println("Node going to sleep");
  delay(500);
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}

//// Getting data //////////////////////////////////////////////////////////////
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
    Serial.println("REMOTE DATA");
    Serial.print(" The node this is from: ");
    Serial.println(header.from_node);
    Serial.print(" Node ID: ");
    for (byte b : payload.id)
      Serial.print(b, HEX);
    Serial.println();
    for (int i = 0; i < numberOfSensors; i++) {
      Serial.print(" Temperature");
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
    Serial.println("LOCAL DATA");
    Serial.print(" Base bat: ");
    Serial.println(localData.baseBat, DEC);
    Serial.print(" Base temp: ");
    Serial.println(float(localData.baseTemp)/100, DEC);
    Serial.print(" Base hum: ");
    Serial.println(localData.baseHum, DEC);
    Serial.print(" Base lux: ");
    Serial.println(localData.baseLux, DEC);

    // fillArray
    fillBufferArray(&payload);


    // sendDataToESP();
    #ifdef WIFI
      postDataWifi(&payload, &localData);
    #endif
    #ifdef GPRS
      postDataGPRS(&payload, &localData);
    #endif
  }
}

void fillBufferArray(Payload_t *payloadAddress){
  uint8_t bufferLocation = 0;
  // get next free buffer location
  for (int i = 0; i < BUFFERSIZE; i++){
    if (payLoadBuffer[bufferLocation].containsData != 0)
      bufferLocation++;
  }
  // copy temp array to next free buffer location
  for (int i = 0; i<4;i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;

  // temp code
  Serial.println("buffer data");
  Serial.print(" The node this is from: ");
  Serial.print(" Node ID: ");
  for (byte b : payLoadBuffer[bufferLocation].id)
    Serial.print(b, HEX);
  Serial.println();
  for (int i = 0; i < numberOfSensors; i++) {
    Serial.print(" Temperature");
    Serial.print(i + 1);
    Serial.print(": ");
    if (payLoadBuffer[bufferLocation].temp[i] == -12700)
      Serial.println("-");
    else
      Serial.println(payLoadBuffer[bufferLocation].temp[i], DEC);
  }
  Serial.print(" Battery status: ");
  Serial.println(payLoadBuffer[bufferLocation].bat, DEC);
  //end temp code
}

void addLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill nodeId
    localDataAddress->baseId[i] = nodeId[i];
  localDataAddress->baseTemp = myHumidity.readTemperature()*100;
  localDataAddress->baseHum = myHumidity.readHumidity();
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
  localDataAddress->baseLux = sensor.readLightLevel();
//  localDataAddress->baseLux = lightMeter.readLightLevel();
}

//// Posting Data //////////////////////////////////////////////////////////////
void postDataWifi(Payload_t *payloadAddress, LocalData_t *localDataAddress) {
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

  esp8266Module.print("http://beelog.dynu.net:1880/hiveonly");

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

// coorddinator data
  esp8266Module.print("?");
  esp8266Module.print("coordId");
  esp8266Module.print("=CO");
  for (byte b : localDataAddress->baseId)
    esp8266Module.print(b, HEX);

  esp8266Module.print("&");
  esp8266Module.print("baseBat");
  esp8266Module.print("=");
  esp8266Module.print(localDataAddress->baseBat);

  esp8266Module.print("&");
  esp8266Module.print("baseHum");
  esp8266Module.print("=");
  esp8266Module.print(localDataAddress->baseHum);

  esp8266Module.print("&");
  esp8266Module.print("baseBat");
  esp8266Module.print("=");
  esp8266Module.print(localDataAddress->baseTemp);

  esp8266Module.print("&");
  esp8266Module.print("baseLux");
  esp8266Module.print("=");
  esp8266Module.print(localDataAddress->baseLux);

  delay(1000);

  Serial.println("ESP going to sleep");
}
