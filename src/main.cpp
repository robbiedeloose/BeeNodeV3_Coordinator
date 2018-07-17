#include <Arduino.h>

#define DEBUG

// Change between Wifi and GPRS
#define GPRS
//#define WIFI

///////////////////////////////////////// HX711 ////////////////////////////////
#define radioPin1 7 // RF24
#define radioPin2 8 // RF24

#define softSerialRx 3
#define softSerialTx 4
#define resetPin A2 // resetting esp module

#define MISO 11
#define MOSI 12
#define CLK 13

#define seedRef A3 // seed for random function

///////////////////////////////////////// HX711 ////////////////////////////////
#include "HX711-multi.h"

///////////////////////////////////////// GPRS /////////////////////////////////
// Select your modem:
#define TINY_GSM_MODEM_SIM800
// Increase RX buffer if needed
//#define TINY_GSM_RX_BUFFER 512
#include <TinyGsmClient.h>
// Uncomment this if you want to see all AT commands
//#define DUMP_AT_COMMANDS
// Uncomment this if you want to use SSL
//#define USE_SSL

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
const char resource[] = "/hiveonly";

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
////////////////////////////////////////////////////////////////////////////////
#define SerialMon Serial
#define numberOfSensors 6
uint8_t sendCounter = 0;
////////////////////////// FUNCTION DECLARATIONS ///////////////////////////////
// setup functions
void initRFRadio(uint8_t channel, uint16_t nodeAddress);
void initGprs();
// getting data
void checkForNetworkData();
void fillBufferArray(Payload_t *payloadAddress);
void addLocalData(LocalData_t *localDataAddress);
void addScaleData();
// posting data
void sendArrayContent();
void sendScaleData();
void sendGprsData(uint8_t gprsMode);
////////////////////////////////////////////////////////////////////////////////

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  delay(2000);
  SerialMon.begin(9600); // SerialMon Start
  delay(1000);
  SerialMon.println("Initialising...");
  // Set GSM module baud rate - make sure to put mon baud rate at same level
  SerialAT.begin(9600);
  delay(1000);
  SerialMon.println("SerialAT Started");
  // print some coordinator node information
  SerialMon.println("BeeNode Coordinator v0.1");
  beeNodeId.getId(nodeId); // send array to fill as parameter
  SerialMon.print("Coordinator Id: CO");
  for (byte b : nodeId)
    SerialMon.print(b, HEX);
  SerialMon.println();

  // initiate sensors
  SerialMon.println("Set voltage Ref");
  battery.setRefInternal(); // Set voltage reference
  SerialMon.println("Start HTU21D");
  myHumidity.begin(); // start humidity sensor

  // register ccordinator to NodeRed
  SerialMon.println("Register Coordinator to NodeRed");
  sendGprsData(1);

  // start GPRS & nRF24l radio
  initGprs();
  initRFRadio(90, thisNode);
  SerialMon.println("initialisation complete");
  SerialMon.println("--------------------------------------------------------");
}

void initGprs(){
  SerialMon.println(F("Initializing modem..."));
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("   Modem: "));
  SerialMon.println(modemInfo);
}

void initRFRadio(uint8_t channel, uint16_t nodeAddress) {
  SerialMon.print("Starting rf radio. ");
  SerialMon.print("Channel: ");
  SerialMon.print(channel);
  SerialMon.print(", NodeAddress:  ");
  SerialMon.println(nodeAddress);

  SPI.begin();
  SerialMon.println("   SPI started");
  radio.begin();
  SerialMon.println("   Radio started");
  // radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  SerialMon.println("   Network started");
  network.setup_watchdog(9); // Sets the WDT to trigger every second
  SerialMon.println(   "Watchdog set");
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() {
  SerialMon.println("Start Loop");
  checkForNetworkData(); // network data available?

  // If a given time threshold is breached, read the buffer array and send the data over gprs
  // Timer yet to be implemented. RTC/counter..
  // Send scale data
	sendArrayContent();
	sendScaleData();

  SerialMon.println("Node going to sleep");
  delay(500); // give serial time to complete before node goes to sleep
  network.sleepNode(15, 0); // 15 cycles of 4 seconds
}

//// Getting data //////////////////////////////////////////////////////////////
void checkForNetworkData() {
  network.update(); // check network communication regularly
  RF24NetworkHeader header; // create header variable
  Payload_t payload;        // create payload variable

  while (network.available()) { // Any data on the network ready to read
    network.read(header, &payload, sizeof(payload)); // If so, grab it and print it out
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

    fillBufferArray(&payload); // fill buffer array
  }
}

void fillBufferArray(Payload_t *payloadAddress){
  uint8_t bufferLocation = 0;
  // get next free buffer location
  for (int i = 0; i < BUFFERSIZE; i++){
    if (payLoadBuffer[bufferLocation].containsData != 0)
      bufferLocation++;
  }
  SerialMon.print("Filling array position ");
  SerialMon.println(bufferLocation); // print the buffer location that is used
  payLoadBuffer[bufferLocation].containsData = 1;
  // copy temp array to next free buffer location
  for (int i = 0; i<4;i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;

  /*
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
  */
}

void sendArrayContent(){
  // we check each array position, get the variables and send them to gprs functions
  // include the local data from the coordinator (temp, humidity, light, ...)
  // addLocalData(LocalData_t *localDataAddress)
  // clear array after sending
}

void addLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill nodeId
    localDataAddress->baseId[i] = nodeId[i];
  localDataAddress->baseTemp = myHumidity.readTemperature()*100;
  localDataAddress->baseHum = myHumidity.readHumidity();
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
  //localDataAddress->baseLux = sensor.readLightLevel();
  //localDataAddress->baseLux = lightMeter.readLightLevel();
}

void sendScaleData(){
  // This data will have to come from second arduino
}

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

  client.print(String("GET ") + resource + "?e=2" + " HTTP/1.0\r\n");
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
        SerialMon.print("   - Send OK");
    }
  }
  SerialMon.println();

  client.stop();
  SerialMon.println(F("   Server disconnected"));

  modem.gprsDisconnect();
  SerialMon.println(F("   GPRS disconnected"));

SerialMon.println(F("End GPRS"));
}
