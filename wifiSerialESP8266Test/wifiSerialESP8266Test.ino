/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-webserial-library/
  
  This sketch is based on the WebSerial library example: ESP8266_Demo
  https://github.com/ayushsharma82/WebSerial
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define LED 2

AsyncWebServer server(80);

const char* ssid = "Big monkey on da bed";          // Your WiFi SSID
const char* password = "lil monkey gettin groovy";  // Your WiFi Password

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "ON"){
    digitalWrite(LED, LOW);
  }
  if (d=="OFF"){
    digitalWrite(LED, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Began serial");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Serial.println("Setting wifi mode");
  WiFi.mode(WIFI_STA);
  Serial.println("Beginning wifi connection");
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
}

void loop() {
  WebSerial.println("Hello!");
  Serial.println("Testing 123");
  delay(2000);
}