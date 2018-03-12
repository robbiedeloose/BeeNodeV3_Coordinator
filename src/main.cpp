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

void sendDataToESP() {
  // esp8266Module
  Serial.println("Resetting ESP");
  digitalWrite(resetPin, LOW);
  delay(1000);
  digitalWrite(resetPin, HIGH);
  Serial.println("Waiting for ESP to start");
  delay(3000);
  Serial.println("Sending data to ESP");
  esp8266Module.print(
      F("http://192.168.10.191:1880/hiveonly?fffff=dddddddddd"));
  delay(1000);

  Serial.println("ESP going to sleep");
}

void setup() {
  // put your setup code here, to run once:
  // Serial Start
  Serial.begin(9600);
  Serial.println("Serial started");
  // esp8266Module
  esp8266Module.begin(9600);
  delay(1000);
  Serial.println("Soft Serial started");
  // Serial end

  // start rf radio
  Serial.println("Starting rf radio");
  SPI.begin();
  radio.begin();
  network.begin(90, thisNode);

  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:

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
    sendDataToESP();
    // postDataToSparkFun(&payload);
  }
}
