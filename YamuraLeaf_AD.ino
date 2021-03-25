/*
   Yamura Leaf - A2D and Digital IO
   BBS 3/2021
   send 16 digital I/O and 4 analog to digital values to hub

   for ESP8266
 */
#define DEBUG_PRINT
#define TARGET_INTERVAL 10000
#define TIMESTAMP_REQUEST_INTERVAL 5000000
#define STATUS_LED 8
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
// 4 channel A2D
#include <SparkFun_ADS1015_Arduino_Library.h>
// 16 channel digial IO
#include <SparkFunSX1509.h>
// define data logger structures/unions for transmitting/receiving data
#include "DataStructures.h"

// timestamp (H and T types)
TimeStampPacket timeStamp;
// digital/A2D data (I type)
IOPacket leafData;

unsigned long lastSampleTime;
unsigned long currentSampleTime;
unsigned long targetInterval = TARGET_INTERVAL;
unsigned long lastSampleInterval = 0;
// offset to hub time for timestamp sync
unsigned long timestampAdjust = 0;
bool isLogging = false;
unsigned long sentCount = 0;
unsigned long errorCount = 0;
int blinkState = LOW;
// hub MAC addres
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};

// ADS1015 sensor
ADS1015 adcSensor;
// SX1509 sensor
const byte SX1509_ADDRESS = 0x3E;
SX1509 io;
//
//
//
void setup()
{
  Serial.begin(115200);
 
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  // This is the mac address of this device
  #ifdef DEBUG_PRINT
  Serial.print("YamuaraLeaf digital/A2D at ");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  #endif
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // register for receive callback for data send response
  esp_now_register_recv_cb(OnDataRecv);
  // send message callback
  esp_now_register_send_cb(OnDataSent);
  // add hub as a peer
  esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  // leaf type
  leafData.packet.leafType[0] = 'I';
  leafData.packet.leafType[1] = 0;
  leafData.packet.leafType[2] = 0;
  leafData.packet.leafType[3] = 0;

  // start I2C and set up sensors
  Wire.begin();
  Wire.setClock(400000);
  // start sensors, blink if not found
  while (!adcSensor.begin())
  {
    #ifdef DEBUG_PRINT
    Serial.println("ADS1015 not found. Try again...");
    #endif
    lastSampleTime = millis();
    while(millis() - lastSampleTime < 5000000)
    {
      digitalWrite(STATUS_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
    }
  }
  digitalWrite(STATUS_LED, LOW);
  // start sx1509 with default settings
  while (!io.begin(SX1509_ADDRESS))
  {
    #ifdef DEBUG_PRINT
    Serial.println("SX1509 not found. Try again...");
    #endif
    lastSampleTime = millis();
    while(millis() - lastSampleTime < 5000000)
    {
      digitalWrite(STATUS_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
    }
  }
  digitalWrite(STATUS_LED, LOW);
  // set all digital io for input
  for(int idx = 0; idx < 16; idx++)
  {
    io.pinMode(idx, INPUT_PULLUP);
  }
  // ready indication
  for(int cnt = 0; cnt < 30; cnt++)
  {
      digitalWrite(STATUS_LED, blinkState);
      delay(100);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
  }
  digitalWrite(STATUS_LED, LOW);
  Serial.println();
  // set last and current sample 
  currentSampleTime = micros();
  lastSampleTime = currentSampleTime;
}
//
// read sensors, push data to hub
//
void loop() 
{
  currentSampleTime = micros();
  // logging - check interval, send sensor updates
  if(isLogging == true)
  {
    lastSampleInterval = currentSampleTime - lastSampleTime; 
    if(lastSampleInterval >= targetInterval)
    {
      // adjusted time
      leafData.packet.timeStamp = currentSampleTime - timestampAdjust;;
      for(int idx = 0; idx < 4; idx++) 
      {
        leafData.packet.a2dValues[idx] = adcSensor.getSingleEnded(idx);
        //Serial.println(leafData.packet.a2dValues[idx]);
      }
      leafData.packet.digitalValue = 0;
      for(int idx = 0; idx < 16; idx++)
      {
        leafData.packet.digitalValue |= (io.digitalRead(idx) << idx);
      }
      // send data to hub
      sendData();
      lastSampleTime = currentSampleTime;
    }
  }
  // not logging - every 10000000 micros (10 sec) send heartbeat
  // triggers a timestamp send from hub  else
  {
    lastSampleInterval = currentSampleTime - lastSampleTime; 
    if(lastSampleInterval >= 10000000)
    {
      timeStamp.packet.timeStamp = micros() - timestampAdjust;
      SendHeartBeat();
      lastSampleTime = currentSampleTime;
    }
  }
}
//
// send data
//
void sendData()
{
  // was disconnected from hub, try to reconnect
  if(!esp_now_is_peer_exist(hub_addr))
  {
    #ifdef DEBUG_PRINT
    Serial.println("Reconnectting");
    #endif
    uint8_t addStatus = esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }
  uint8_t result = esp_now_send(hub_addr, &leafData.dataBytes[0], sizeof(leafData));
  sentCount++;
  #ifdef DEBUG_PRINT
  if(result != 0)
  {
    errorCount++;
    Serial.print("Send error ");
    Serial.print(errorCount);
    Serial.print(" of ");
    Serial.print(sentCount);
    Serial.println(" send attempts");
  }
  #endif
}
//
// send heartbeat
//
void SendHeartBeat()
{
  // was disconnected from hub, try to reconnect
  if(!esp_now_is_peer_exist(hub_addr))
  {
    #ifdef DEBUG_PRINT
    Serial.println("Reconnectting");
    #endif
    uint8_t addStatus = esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }
  timeStamp.packet.msgType[0] = 'H';
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  #ifdef DEBUG_PRINT
  Serial.print(micros());
  Serial.print(" To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(timeStamp.packet.msgType);
  Serial.print(" Time ");
  Serial.print(timeStamp.packet.timeStamp);
  Serial.print(" bytes ");
  Serial.print(timeStamp.dataBytes[0], HEX);
  for(int idx = 1; idx < 8; idx++)
  {
    Serial.print(" ");
    Serial.print(timeStamp.dataBytes[idx], HEX);
  }
  Serial.print("\n");
  #endif
}
//
// request timestamp from hub
//
void requestTimestamp()
{
  timeStamp.packet.msgType[0] = 'T';
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  
  #ifdef DEBUG_PRINT
  Serial.print(micros());
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.println((char)timeStamp.packet.msgType[0]);
  if(result != 0)
  {
    Serial.print("message send error ");
  }
  #endif
}
//
// Init ESP Now with fallback
//
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) 
  {
    #ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Success");
    #endif
  }
  else 
  {
    #ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Failed");
    #endif
    // restart
    ESP.restart();
  }
}
//
// callback when data is sent
//
void OnDataSent(uint8_t *mac_addr, uint8_t status) 
{
  if(status != 0)
  {
    timestampAdjust = 0;
    #ifdef DEBUG_PRINT
    char macStr[18];
    Serial.print("Last Packet Sent to: ");
    for(int idx = 0; idx < 6; idx++)
    {
      Serial.print(mac_addr[idx], HEX); Serial.print(" ");
    }
    Serial.print(" Failed: ");
    Serial.println(status);
    #endif
  }
}
//
// callback when data is received
//
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{
  if(data[0] == 'T')
  {
    memcpy(&timeStamp.dataBytes, data, sizeof(timeStamp));
    timestampAdjust =  micros() - timeStamp.packet.timeStamp;
    #ifdef DEBUG_PRINT
    Serial.print("Recv ");Serial.print(data_len);Serial.print(" bytes from: ");
    for(int idx = 0; idx < 6; idx++)
    {
      Serial.print(mac_addr[idx], HEX);Serial.print(":");
    }
    unsigned long localTs = micros();
    Serial.print(" local timestamp ");
    Serial.print (localTs);
    Serial.print(" HUB timestamp ");
    Serial.print (timeStamp.packet.timeStamp);
    Serial.print(" adjustment ");
    Serial.print (timestampAdjust);
    Serial.print(" corrected ");
    Serial.println(localTs - timestampAdjust);
    #endif
  }
  // change state of logging
  else if(data[0] == 'B')
  {
    #ifdef DEBUG_PRINT
    Serial.print("Recv ");Serial.print(data[0]);Serial.println(" START logging");
    isLogging = true;
    #endif
  }
  else if(data[0] == 'E')
  {
    #ifdef DEBUG_PRINT
    Serial.print("Recv ");Serial.print(data[0]);Serial.println(" END logging");
    isLogging = false;
    #endif
  }
}
