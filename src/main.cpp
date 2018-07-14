//GPRS Branch
#include <Arduino.h>

#define DEBUG

/*
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
*/

// Change between Wifi and GPRS
#define GPRS
//#define WIFI

///////////////////////////////////////// HX711 ////////////////////////////////
#define radioPin1 7 // RF24
#define radioPin2 8 // RF24

#define softSerialRx 3
#define softSerialTx 4
#define resetPin A2 // resetting esp module

#define seedRef A3 // seed for random function

///////////////////////////////////////// HX711 ////////////////////////////////
#include "HX711-multi.h"

///////////////////////////////////////// GPRS /////////////////////////////////
/**************************************************************
 *
 * This sketch connects to a website and downloads a page.
 * It can be used to perform HTTP/RESTful API calls.
 *
 * TinyGSM Getting Started guide:
 *   http://tiny.cc/tiny-gsm-readme
 *
 **************************************************************/

// Select your modem:
#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_ESP8266

// Increase RX buffer if needed
//#define TINY_GSM_RX_BUFFER 512

#include <TinyGsmClient.h>

// Uncomment this if you want to see all AT commands
//#define DUMP_AT_COMMANDS

// Uncomment this if you want to use SSL
//#define USE_SSL

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Use Hardware Serial on Mega, Leonardo, Micro
//#define SerialAT Serial1

// or Software Serial on Uno, Nano
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(3, 4); // RX, TX


// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "telenetwap.be";
const char user[] = "";
const char pass[] = "";

// Server details
const char server[] = "beelog.dynu.net";
const char resource[] = "/hiveonly?d=1";

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#ifdef USE_SSL
  TinyGsmClientSecure client(modem);
  const int  port = 443;
#else
  TinyGsmClient client(modem);
  const int  port = 1880;
#endif


///////////////////////////////////// WIFI /////////////////////////////////////
/*#ifdef WIFI

  #include <SoftwareSerial.h>
  SoftwareSerial esp8266Module(softSerialRx, softSerialTx); // RX, TX

#endif*/

///////////////////////////////////// RADIO ////////////////////////////////////
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
RF24 radio(radioPin1, radioPin2);             // start RF24 communication layer
RF24Network network(radio);   // start RF24 network layer
const uint16_t thisNode = 00; // Coordinator address

///////////////////////////////////// HUMIDITY /////////////////////////////////
#include "SparkFunHTU21D.h"
#include <Wire.h>
HTU21D myHumidity; // humidity + temperature

///////////////////////////////////// EEPROM ///////////////////////////////////
// EEPROM address locations
#include <EEPROM.h>      //EEPROM
#define EEPRomDeviceId 1 // 1 byte for #, 4 bytes for ID
#define EEPRomOptions 6
uint8_t nodeId[4];

//////////////////////////////// own libraries /////////////////////////////////

// NodeId
#include "RandomNodeId.h"
RandomNodeId beeNodeId;
// Internal Voltage
#include "MesureVoltageInternal.h"
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
  uint8_t containsData;
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t weight;
  uint16_t humidity;
  uint8_t alarm;
};

//
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

////////////////////////////////////////////////////////////////////////////////

#define SerialMon Serial
#define numberOfSensors 6
uint8_t sendCounter = 0;

////////////////////////// FUNCTION DECLARATIONS ///////////////////////////////
// setup functions
//void initCoordinator();
void startRFRadio(uint8_t channel, uint16_t nodeAddress);
/*#ifdef WIFI
  void startCustomESP();
#endif*/
#ifdef GPRS
  void initGprs();
#endif

// getting data
void checkForNetworkData();
void fillBufferArray(Payload_t *payloadAddress);
void addLocalData(LocalData_t *localDataAddress);
void addScaleData();

// posting data
void sendArrayContent();
void sendScaleData();

void sendGprsData(uint8_t gprsMode);

/*#ifdef WIFI
  void postDataWifi(Payload_t *payloadAddress, LocalData_t *localDataAddress);
#endif*/

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  SerialMon.begin(9600); // SerialMon Start
  delay(1000);
  SerialMon.println("SerialMon Started");
  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(1000);
  SerialMon.println("SerialAT Started");
  //initCoordinator();

  SerialMon.println("BeeNode Coordinator v0.1");
  battery.setRefInternal(); // Set voltage reference
  beeNodeId.getId(nodeId); // send array to fill as parameter
  myHumidity.begin(); // start humidity sensor

  SerialMon.print("Coordinator Id: CO");
  for (byte b : nodeId)
    SerialMon.print(b, HEX);
  SerialMon.println();

  SerialMon.println("First GPRS POST");
  sendGprsData(1);
  /*#ifdef WIFI
    startCustomESP();
  #endif*/

  startRFRadio(90, thisNode);
  SerialMon.println("Started RF Radio");
  SerialMon.println("Get BeeNodeId");
  beeNodeId.getId(nodeId); // send array to fill as parameter
  SerialMon.println("Set voltage Ref");
  battery.setRefInternal();
  SerialMon.println("End Setup");

}

//// setup functions ///////////////////////////////////////////////////////////
/*void initCoordinator(){
  SerialMon.println("BeeNode Coordinator v0.1");
  battery.setRefInternal(); // Set voltage reference
  beeNodeId.getId(nodeId); // send array to fill as parameter
  myHumidity.begin(); // start humidity sensor

  SerialMon.print("Coordinator Id: CO");
  for (byte b : nodeId)
    SerialMon.print(b, HEX);
  SerialMon.println();
}*/

/*#ifdef WIFI
void startCustomESP() {
  // esp8266Module
  esp8266Module.begin(9600);
  delay(500); //------------
  SerialMon.println("Soft Serial started at 9600");
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
}
#endif*/

#ifdef GPRS
void initGprs(){
  SerialMon.println(F("Initializing modem..."));
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("Modem: "));
  SerialMon.println(modemInfo);
}
#endif

void startRFRadio(uint8_t channel, uint16_t nodeAddress) {
  SerialMon.print("Starting rf radio. ");
  SerialMon.print("Channel: ");
  SerialMon.print(channel);
  SerialMon.print(", NodeAddress:  ");
  SerialMon.println(nodeAddress);

  SPI.begin();
  SerialMon.println("SPI started");
  radio.begin();
  SerialMon.println("Radio started");
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  SerialMon.println("Network started");
  network.setup_watchdog(9); // Sets the WDT to trigger every second
  SerialMon.println("Watchdog set");
}



void loop() {
  SerialMon.println("Start Loop");
  sendGprsData(2);
  checkForNetworkData();
  sendCounter++;
  if (sendCounter == 15){
    sendArrayContent();
    sendScaleData();
  }
SerialMon.println("GPRS----------------");
  //sendGprsData();
  SerialMon.println("Node going to sleep");
  delay(500);
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
  */
}

//// Getting data //////////////////////////////////////////////////////////////
void checkForNetworkData() {
  network.update(); // check network communication regularly
  RF24NetworkHeader header; // create header variable
  Payload_t payload;        // create payload variable

  while (network.available()) { // Any data on the network ready to read
        network.read(header, &payload, sizeof(payload)); // If so, grab it and print it out
    // Add coordinator Data
    //LocalData_t localData;
    //addLocalData(&localData);

    // Display Node Data
    SerialMon.println(F("REMOTE DATA"));
    SerialMon.print(" The node this is from: ");
    SerialMon.println(header.from_node);
    SerialMon.print(" Node ID: ");
    for (byte b : payload.id)
      SerialMon.print(b, HEX);
    SerialMon.println();
    for (int i = 0; i < numberOfSensors; i++) {
      SerialMon.print(" Temperature");
      SerialMon.print(i + 1);
      SerialMon.print(": ");
      if (payload.temp[i] == -12700)
        SerialMon.println("-");
      else
        SerialMon.println(payload.temp[i], DEC);
    }
    SerialMon.print(" Battery status: ");
    SerialMon.println(payload.bat, DEC);

    // Display Local Data
    /*
    SerialMon.println("LOCAL DATA");
    SerialMon.print(" Base bat: ");
    SerialMon.println(localData.baseBat, DEC);
    SerialMon.print(" Base temp: ");
    SerialMon.println(float(localData.baseTemp)/100, DEC);
    SerialMon.print(" Base hum: ");
    SerialMon.println(localData.baseHum, DEC);
    SerialMon.print(" Base lux: ");
    SerialMon.println(localData.baseLux, DEC);
    */

    fillBufferArray(&payload); // fill buffer array


    /*
    #ifdef WIFI
      postDataWifi(&payload, &localData);
    #endif
    #ifdef GPRS
      postDataGPRS(&payload, &localData);
    #endif
    */
  }
}

void fillBufferArray(Payload_t *payloadAddress){
  uint8_t bufferLocation = 0;
  // get next free buffer location
  for (int i = 0; i < BUFFERSIZE; i++){
    if (payLoadBuffer[bufferLocation].containsData != 0)
      bufferLocation++;
  }
  SerialMon.print("Filling array position");
  SerialMon.println(bufferLocation);
  payLoadBuffer[bufferLocation].containsData = 1;
  // copy temp array to next free buffer location
  for (int i = 0; i<4;i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;

  // temp code
  SerialMon.println("buffer data");
  SerialMon.print(" The node this is from: ");
  SerialMon.print(" Node ID: ");
  for (byte b : payLoadBuffer[bufferLocation].id)
    SerialMon.print(b, HEX);
  SerialMon.println();
  for (int i = 0; i < numberOfSensors; i++) {
    SerialMon.print(" Temperature");
    SerialMon.print(i + 1);
    SerialMon.print(": ");
    if (payLoadBuffer[bufferLocation].temp[i] == -12700)
      SerialMon.println("-");
    else
      SerialMon.println(payLoadBuffer[bufferLocation].temp[i], DEC);
  }
  SerialMon.print(" Battery status: ");
  SerialMon.println(payLoadBuffer[bufferLocation].bat, DEC);
  //end temp code
}

void sendArrayContent(){
  //we check each array position, get the variables and send them to gprs functions
  // or should we send them from here?
}

void addLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill nodeId
    localDataAddress->baseId[i] = nodeId[i];
  localDataAddress->baseTemp = myHumidity.readTemperature()*100;
  localDataAddress->baseHum = myHumidity.readHumidity();
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
  //localDataAddress->baseLux = sensor.readLightLevel();
//  localDataAddress->baseLux = lightMeter.readLightLevel();
}

void sendScaleData(){

}
void addScaleData(){
  //read scales and add data to
}


//// Posting Data //////////////////////////////////////////////////////////////
/*#ifdef WIFI
void postDataWifi(Payload_t *payloadAddress, LocalData_t *localDataAddress) {
  // Post Hive Data
  // WIFI VERSION

  SerialMon.println("Resetting ESP");
  digitalWrite(resetPin, LOW);
  delay(1000);
  digitalWrite(resetPin, HIGH);
  SerialMon.println("Waiting for ESP to start");
  delay(3000);
  SerialMon.println("Sending data to ESP");

  SerialMon.println("Posting Data");

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

  SerialMon.println("ESP going to sleep");
}
#endif*/

void sendGprsData(uint8_t gprsMode){

  //initGprs();
  SerialMon.println(F("SEND DATA THROUGH GPRS"));
  SerialMon.print(F("Mode:"));
  SerialMon.println(gprsMode);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println(F("   Initializing modem..."));
  modem.restart();
delay(3000);
  String modemInfo = modem.getModemInfo();
delay(1000);
  SerialMon.print(F("   Modem: "));
  SerialMon.println(modemInfo);

  //////////////////////////////////////////////

  SerialMon.print(F("   Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  SerialMon.print(F("   Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  SerialMon.print(F("   Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");



  // Make a HTTP GET request:


  switch (gprsMode) {
  case 1:
    client.print(String("GET ") + "/register?coordinatorId=");
    for (byte b : nodeId)
      client.print(b, HEX);
    client.print(" HTTP/1.0\r\n");
    break;
  case 2:
    client.print(String("GET ") + resource + "&e=2" + " HTTP/1.0\r\n");
    break;
  }

  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      SerialMon.print(c);
      timeout = millis();
      if(c == '#')
        SerialMon.print("!");
    }
  }
  SerialMon.println();

  client.stop();
  SerialMon.println(F("   Server disconnected"));

  modem.gprsDisconnect();
  SerialMon.println(F("   GPRS disconnected"));

SerialMon.println(F("End GPRS"));
}
