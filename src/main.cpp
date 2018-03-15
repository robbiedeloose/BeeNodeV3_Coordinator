#include <Arduino.h>
#include <SoftwareSerial.h>

#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>

//#include <EEPROM.h>
// OBJECTS
SoftwareSerial esp8266Module(A0, A1); // RX, TX
uint8_t resetPin = A2;

///// RF24 DECLARATIONS ////////////////////////////////////////////////////////
// start RF24 communication layer
RF24 radio(7, 8);
// start RF24 network layer
RF24Network network(radio);
// Coordinator address
const uint16_t thisNode = 00;
// Structure of our payload coming from router and end devices

struct Payload2_t {
  uint8_t id[4];
  int16_t temp[6];
  uint16_t bat;
  uint16_t weight;
  uint16_t humidity;
  uint8_t alarm;
};

void postData(Payload2_t *payloadAddress) {
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

  for (int i = 0; i < 3; i++) {
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

void startCustomESP() {
  // esp8266Module
  esp8266Module.begin(9600);
  delay(500); //------------
  Serial.println("Soft Serial started at 9600");
  // start rf radio

  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
}

void startRFRadio(uint8_t channel, uint16_t nodeAddress) {
  Serial.println("Starting rf radio");
  SPI.begin();
  radio.begin();
  network.begin(channel, nodeAddress);
  network.setup_watchdog(9); // Sets the WDT to trigger every second
}

void checkForNetworkData() {
  // check network communication regularly
  network.update();

  RF24NetworkHeader header; // create header variable
  Payload2_t payload;       // create payload variable
  // Any data on the network ready to read
  while (network.available()) {
    // If so, grab it and print it out
    network.read(header, &payload, sizeof(payload));
    Serial.print("The node this is from: ");
    Serial.println(header.from_node);
    Serial.print("Node ID: ");
    for (byte b : payload.id)
      Serial.print(b, HEX);
    Serial.println();
    Serial.print("Temperature1: ");
    Serial.println(payload.temp[0], DEC);
    Serial.print("Temperature2: ");
    Serial.println(payload.temp[1], DEC);
    Serial.print("Temperature3: ");
    Serial.println(payload.temp[2], DEC);
    Serial.print(" Battery status: ");
    Serial.println(payload.bat, DEC);
    // sendDataToESP();
    postData(&payload);
  }
}

void setup() {
  // put your setup code here, to run once:
  // Serial Start
  Serial.begin(9600);
  Serial.println("BeeNode Coordinator v0.1");
  startCustomESP();
  startRFRadio(90, thisNode);
  delay(10000);
}

void loop() {

  checkForNetworkData();

  Serial.println("Node going to sleep");
  delay(500);
  if (network.sleepNode(12, 0)) {
    Serial.println("Timer met");
  } else {
    Serial.println("Interrupt");
  }
}
