#include <Arduino.h>
#include <SoftwareSerial.h>
//#include <EEPROM.h>
// OBJECTS
SoftwareSerial esp8266Module(A0, A1); // RX, TX
uint8_t resetPin = A2;

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
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // esp8266Module
  Serial.println("Resetting");
  digitalWrite(resetPin, LOW);
  delay(1000);
  digitalWrite(resetPin, HIGH);
  Serial.println("Waiting for ESP to start");
  delay(3000);
  Serial.println("Sending data");
  esp8266Module.print(
      F("http://192.168.10.191:1880/hiveonly?fffff=dddddddddd"));
  delay(1000);

  while (Serial.available()) {
    String a = Serial.readString(); // read the incoming data as string
    Serial.println(a);
  }
  Serial.println("going to sleep");
  delay(60000);
  /*if (esp8266Module.find("OK"))
  {
 Serial.println("OK");
  }
  else
  {
 Serial.println("NOK");
}*/
}
