#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////
#define radioPin1 8 // RF24
#define radioPin2 9 // RF24

#define softSerialAtRx 4
#define softSerialAtTx 5
#define resetPin A2 // resetting esp/gprs module
#define softSerialScaleRx 6
#define softSerialScaleTx 7

#define MISO 11
#define MOSI 12
#define CLK 13

#define seedRef A3 // seed for random function

////////////////////////////////////////// Serials /////////////////////////////
#define SerialMon Serial
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(softSerialAtRx, softSerialAtTx);
SoftwareSerial SerialScale(softSerialScaleRx, softSerialScaleTx);
///////////////////////////////////////// GPRS /////////////////////////////////
// Select your modem:
#define TINY_GSM_MODEM_SIM800
// Increase RX buffer if needed
//#define TINY_GSM_RX_BUFFER 512
#include <TinyGsmClient.h>
// Uncomment this if you want to see all AT commands
//#define DUMP_AT_COMMANDS

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "hologram";
const char user[] = "";
const char pass[] = "";
// Server details
const char server[] = "beelog.dynu.net";
const char server2[] = "beelog2.dynu.net";
const char resource[] = "/hiveonly";
boolean GprsAvtice = false;

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
RF24 radio(radioPin1, radioPin2); // start RF24 communication layer
RF24Network network(radio);       // start RF24 network layer
const uint16_t thisNode = 00;     // Coordinator address
///////////////////////////////////// HUMIDITY /////////////////////////////////
#include "SparkFunHTU21D.h"
#include <Wire.h>
HTU21D myHumidity;           // humidity + temperature
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
#define BUFFERSIZE 6
PayloadBuffer_t payLoadBuffer[BUFFERSIZE];
////////////////////////////////////////////////////////////////////////////////
#define numberOfSensors 6
uint8_t sendCounter = 0;
////////////////////////// FUNCTION DECLARATIONS ///////////////////////////////
// setup functions
void initRFRadio(uint8_t channel, uint16_t nodeAddress);
// getting data
void checkForNetworkData();
void fillBufferArray(Payload_t *payloadAddress);
void addLocalData(LocalData_t *localDataAddress);
void addScaleData();
// gprs functions
void gprsInit();
void gprsConnectNetwork();
void gprsConnectHost();
void gprsSendHiveData();
void gprsRegisterNode();
void gprsDisconnectHost();
void gprsEnd();
void getGprsResponse();
// posting data
void gprsSendScaleData();
////////////////////////////////////////////////////////////////////////////////

void clearPayloadBuffer(){
  for (int i = 0; i < BUFFERSIZE; i++){
    payLoadBuffer[i].containsData = 0;
  }
}

/////////////// SETUP //////////////////////////////////////////////////////////
void setup() { //clean
  SerialMon.begin(9600); // SerialMon Start
  SerialAT.begin(9600); // Serial port for GPRS
  //SerialScale.begin(38400); //Serial port for scale module
  delay(1000);
  // print some coordinator node information
  SerialMon.print(F("BeeNode v0.1"));
  beeNodeId.getId(coordId); // send array to fill as parameter
  SerialMon.print(", Id: CO");
  for (byte b : coordId)
    SerialMon.print(b, HEX);
  SerialMon.println();
  clearPayloadBuffer();   // clear payloadbuffer
  battery.setRefInternal(); // Set voltage reference
  myHumidity.begin(); // start humidity sensor
  initRFRadio(90, thisNode); // start nRF24l radio

  SerialMon.println(F("Register to NodeRed"));
  delay(1000);
  gprsInit();
  gprsConnectNetwork();
  gprsRegisterNode();
  gprsEnd();

  SerialMon.println("init complete");
}

void initRFRadio(uint8_t channel, uint16_t nodeAddress) { //clean
  SerialMon.print(F("Rf Channel: "));
  SerialMon.print(channel);
  SerialMon.print(", NodeAddress:  ");
  SerialMon.println(nodeAddress);

  SPI.begin();
  radio.begin();
  //radio.setPALevel(HIGH);
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9); // Sets the WDT to trigger every second
}

/////////////// LOOP ///////////////////////////////////////////////////////////
void loop() { //clean
  SerialMon.println();
  SerialMon.println();
  SerialMon.println("Loop");
  checkForNetworkData(); // network data available?

  // If a given time threshold is breached, read the buffer array and send the data over gprs
  // Timer yet to be implemented. RTC/counter..
  // Send scale data
  if (payLoadBuffer[1].containsData == 1){
    Serial.println("sending");
    gprsInit();
    gprsConnectNetwork();
    gprsSendHiveData();
    //gprsSendScaleData();
    gprsEnd();
  }

  SerialMon.println("sleep");
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
    SerialMon.print(" Node ID: ");
    for (byte b : payload.id)
      SerialMon.print(b, HEX);
    SerialMon.println();
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
  SerialMon.print("Array position ");
  SerialMon.println(bufferLocation); // print the buffer location that is used
  payLoadBuffer[bufferLocation].containsData = 1;
  // copy temp array to next free buffer location
  for (int i = 0; i<4;i++)
    payLoadBuffer[bufferLocation].id[i] = payloadAddress->id[i];
  for (int i = 0; i < numberOfSensors; i++)
    payLoadBuffer[bufferLocation].temp[i] = payloadAddress->temp[i];
  payLoadBuffer[bufferLocation].humidity = payloadAddress->humidity;
  payLoadBuffer[bufferLocation].bat = payloadAddress->bat;
  payLoadBuffer[bufferLocation].alarm = payloadAddress->alarm;

}

void addLocalData(LocalData_t *localDataAddress) {
  for (uint8_t i = 0; i < 4; i++) // fill coordinatorId
    localDataAddress->baseId[i] = coordId[i];
  localDataAddress->baseTemp = myHumidity.readTemperature()*100;
  localDataAddress->baseHum = myHumidity.readHumidity();
  localDataAddress->baseBat = battery.getVoltage() * 100; // Battery
  //localDataAddress->baseLux = sensor.readLightLevel();
  //localDataAddress->baseLux = lightMeter.readLightLevel();
}

void gprsInit(){ //clean
  //SerialMon.println(F("Initializing modem..."));
  delay(1000);
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("  Modem: "));
  SerialMon.println(modemInfo);
}

void gprsConnectNetwork(){
  SerialMon.print(F("  Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" OK"));

  SerialMon.print(F("  Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" OK"));
}

void gprsConnectHost(){
  SerialMon.print(F("  Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" OK"));
}

void gprsDisconnectHost(){
  client.stop();
  SerialMon.println(F("  Server disconnected"));
}

void gprsRegisterNode(){
  gprsConnectHost();
  client.print("GET /register?coordinator=CO");
  for (byte b : coordId)
    client.print(b, HEX);
  client.print(" HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");
  getGprsResponse();
  gprsDisconnectHost();
}

void gprsSendHiveData(){
  for (int i = 0; i < BUFFERSIZE; i++){
    if (payLoadBuffer[i].containsData != 0){
      gprsConnectHost();
      client.print("GET /hivedata?nodeId=");
      for (byte b : payLoadBuffer[i].id)
        client.print(b, HEX);
      for (int a = 0; a < numberOfSensors; a++) {
        client.print("&temp" + String(a+1) + "=");
        if (payLoadBuffer[i].temp[a] == -12700)
          client.print("-");
        else
          client.print(payLoadBuffer[i].temp[a], DEC);
      }
      client.print("&hum=");
      client.print(payLoadBuffer[i].humidity);
      client.print("&bat=");
      client.print(payLoadBuffer[i].bat);
      client.print(" HTTP/1.0\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.print("Connection: close\r\n\r\n");
      getGprsResponse();
      gprsDisconnectHost();
    }
  }
}

void gprsSendScaleData(){
  gprsConnectHost();
  client.print("GET /scaledata?scale=2");
  //
  client.print(" HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");
  getGprsResponse();
  gprsDisconnectHost();
}

void getGprsResponse(){
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      SerialMon.print(c);
      timeout = millis();
      //if(c == '#')
        //SerialMon.print("   - Send OK");
    }
  }
  SerialMon.println();
}

void gprsEnd(){
  modem.gprsDisconnect();
  SerialMon.println(F("  GPRS disconnected"));
}
