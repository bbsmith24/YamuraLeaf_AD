/*
   Yamura Leaf - A2D and Digital IO
   BBS 3/2021
   send 16 digital I/O and 4 analog to digital values to hub

   for ESP8266
 */
#define PRINT_DEBUG
#define TARGET_INTERVAL 10000
#define STATUS_LED 8
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <queue.h>

#include <Wire.h>
// 4 channel A2D
#include <SparkFun_ADS1015_Arduino_Library.h>
// 16 channel digial IO
#include <SparkFunSX1509.h>
// define data logger structures/unions for transmitting/receiving data
#include "DataStructures.h"

// timestamp (H, T, B, E types)
TimeStampPacket timeStamp;
// digital/A2D data (I type)
CombinedIOPacket combinedPacket;

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

uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0x30, 0x54, 0xCC};

// ADS1015 sensor
ADS1015 adcSensor;
// SX1509 sensor
const byte SX1509_ADDRESS = 0x3E;
SX1509 io;
//
//
//
xQueueHandle  sensorSendQueue;
xQueueHandle  timeStampSendQueue;
xQueueHandle  timeStampReceiveQueue;
//
//
//
void setup()
{
  Serial.begin(115200);
 
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  // This is the mac address of this device
  Serial.println();
  Serial.print("YamuaraLeaf Combined Digital/A2D ");Serial.println(WiFi.macAddress());
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
  combinedPacket.packet.leafType = COMBINEDIO_LEAFTYPE;

  // start I2C and set up sensors
  Wire.begin();
  Wire.setClock(400000);
  // start sensors, blink if not found
  while (!adcSensor.begin())
  {
    #ifdef PRINT_DEBUG
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
  adcSensor.setMode(0);
  adcSensor.setGain(ADS1015_CONFIG_PGA_1);
  adcSensor.setSampleRate(ADS1015_CONFIG_RATE_3300HZ);
  
  #ifdef PRINT_DEBUG
  Serial.println("ADS1015 ready");
  #endif
  digitalWrite(STATUS_LED, LOW);
  // start sx1509 with default settings
  while (!io.begin(SX1509_ADDRESS))
  {
    #ifdef PRINT_DEBUG
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
  #ifdef PRINT_DEBUG
  Serial.println("SX1509 ready");
  #endif
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

  // create a queue for sensor readings and timestamps to send
  // send function will pull from here to send to hub
  sensorSendQueue = xQueueCreate(100, sizeof(CombinedIOPacket));
  timeStampSendQueue = xQueueCreate(100, sizeof(TimeStampPacket));
  // create a queue for timestamps received
  // receive function put messages received from hub here
  timeStampReceiveQueue = xQueueCreate(100, sizeof(TimeStampPacket));
  
  Serial.println("Running, Idle");
}
//
// read sensors, push data to hub
//
void loop() 
{
  currentSampleTime = micros();
  // logging - check interval, send sensor updates
  if(isLogging)
  {
    lastSampleInterval = currentSampleTime - lastSampleTime; 
    if(lastSampleInterval >= targetInterval)
    {
      currentSampleTime = micros();
      combinedPacket.packet.timeStamp = currentSampleTime - timestampAdjust;;
      for(uint8_t idx = 0; idx < 4; idx++) 
      {
        combinedPacket.packet.a2dValues[idx] = (int16_t)adcSensor.getSingleEnded(idx);
      }
      combinedPacket.packet.digitalValue = 0;
      for(uint8_t idx = 0; idx < 16; idx++)
      {
        combinedPacket.packet.digitalValue |= (io.digitalRead(idx) << idx);
      }
      // put data in queue to send to hub
      xQueueSendToBack( sensorSendQueue, &combinedPacket, portMAX_DELAY );
      //sendData();
      lastSampleTime = currentSampleTime;
    }
  }
  // not logging - every 10000000 micros (10 sec) send heartbeat
  // triggers a timestamp send from hub
  else
  {
    lastSampleInterval = currentSampleTime - lastSampleTime; 
    if(lastSampleInterval >= 10000000)
    {
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
  uint8_t result = esp_now_send(hub_addr, &combinedPacket.dataBytes[0], sizeof(CombinedIOPacket));
  sentCount++;
  #ifdef PRINT_DEBUG
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(combinedPacket.packet.leafType);
  Serial.print(" Time ");
  Serial.print(combinedPacket.packet.timeStamp);
  Serial.print(" ACCEL Values ");
  Serial.print(combinedPacket.packet.a2dValues[0]); Serial.print(" ");
  Serial.print(combinedPacket.packet.a2dValues[1]); Serial.print(" ");
  Serial.print(combinedPacket.packet.a2dValues[2]); Serial.print(" ");
  Serial.print(combinedPacket.packet.a2dValues[3]); Serial.print(" ");
  Serial.println(combinedPacket.packet.digitalValue, HEX);
  #endif
  if(result != 0)
  {
    errorCount++;
    Serial.print("Send error ");
    Serial.print(errorCount);
    Serial.print(" of ");
    Serial.print(sentCount);
    Serial.println(" send attempts");
  }
}
//
// send heartbeat
//
void SendHeartBeat()
{
  // was disconnected from hub, try to reconnect
  if(!esp_now_is_peer_exist(hub_addr))
  {
    #ifdef PRINT_DEBUG
    Serial.println("Reconnecting");
    #endif
    uint8_t addStatus = esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }
  timeStamp.packet.msgType = 'H';
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  #ifdef PRINT_DEBUG
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
  Serial.print(" To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.println();
  #endif
}
//
// request timestamp from hub
//
void requestTimestamp()
{
  timeStamp.packet.msgType = TIMESTAMP_TYPE;
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  #ifdef PRINT_DEBUG
  Serial.print(timeStamp.packet.timeStamp);
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.println((char)timeStamp.packet.msgType);
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
    #ifdef PRINT_DEBUG
    Serial.println("ESPNow Init Success");
    #endif
  }
  else 
  {
    #ifdef PRINT_DEBUG
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
    #ifdef PRINT_DEBUG
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
  if(data[0] == TIMESTAMP_TYPE)
  {
    memcpy(&timeStamp.dataBytes, data, sizeof(timeStamp));
    timestampAdjust =  micros() - timeStamp.packet.timeStamp;
    #ifdef PRINT_DEBUG
    Serial.print(mac_addr[0], HEX);
    for(int idx = 1; idx < 6; idx++)
    {
      Serial.print(":");
      Serial.print(mac_addr[idx], HEX);
    }
    unsigned long localTs = micros();
    Serial.print(" local timestamp ");
    Serial.print (localTs);
    Serial.print(" HUB timestamp ");
    Serial.print (timeStamp.packet.timeStamp);
    Serial.print(" adjustment ");
    Serial.print ((long)timestampAdjust);
    Serial.print(" corrected ");
    Serial.println(localTs - timestampAdjust);
    #endif
  }
  // change state of logging
  else if(data[0] == LOGGING_BEGIN)
  {
    Serial.println("Logging");
    isLogging = true;
  }
  else if(data[0] == LOGGING_END)
  {
    Serial.println("Idle");
    isLogging = false;
  }
}
