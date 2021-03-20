/*
   Yamura Leaf - A2D and Digital IO
   BBS 3/2021
   send 16 digital I/O and 4 analog to digital values to hub

   for ESP8266
 */
//#define DEBUG_PRINT
#define TARGET_INTERVAL 50000
#define TIMESTAMP_REQUEST_INTERVAL 5000000

#include <ESP8266WiFi.h>
#include <espnow.h>

#include <Wire.h>
// 4 channel A2D
#include <SparkFun_ADS1015_Arduino_Library.h>
// 16 channel digial IO
#include <SparkFunSX1509.h>

// data package structure
#define MESSAGE_LEN 14
struct LeafData
{
  char leafType;            // 1 byte  - type, in this case 'I' for IO - might need to make this 2 characters...
  unsigned long timeStamp;  // 4 bytes - micros() value of sample
  int16_t a2dValues[4];     // 8 bytes, 2 per a2d channel
  int16_t digitalValue;     // 1 byte, 1 bit per digital channel
};
// data package union to do conversion to bytes
union DataToSend
{
  struct LeafData leafData;
  uint8_t dataBytes[MESSAGE_LEN];
} da2dToSend;

struct HubTimeStamp
{
  char msgType;
  unsigned long timeStamp;  // 4 bytes - micros() value of sample
} hubTimestamp;
// data packet for received hub timestamp
union HearbeatToSend
{
  HubTimeStamp heartbeat;
  uint8_t dataBytes[5];
} heartBeatToSend;


unsigned long lastTime;
unsigned long curTime;
unsigned long targetInterval = TARGET_INTERVAL;
unsigned long sampleInterval = TARGET_INTERVAL;
unsigned long lastInterval = 0;
unsigned long lastTimestampRequest = 0;
unsigned long timestampAdjust = 0;
bool startLogging = false;
// hub MAC addres
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};

// ADS1015 sensor
ADS1015 adcSensor;
// SX1509 sensor
const byte SX1509_ADDRESS = 0x3E;
SX1509 io;

#define ESP8266_LED 5
//
//
//
void setup()
{
  Serial.begin(115200);
  int blinkState = HIGH;
  pinMode(ESP8266_LED, OUTPUT); // built in LED
 
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

  // set last and current sample 
  lastTime = micros();
  curTime = micros();
  // leaf type
  da2dToSend.leafData.leafType = 'I';

  // start I2C and set up sensors
  Wire.begin();
  Wire.setClock(400000);
  // start sensors, blink if not found
  while (!adcSensor.begin())
  {
    #ifdef DEBUG_PRINT
    Serial.println("ADS1015 not found. Try again...");
    #endif
    lastTime = millis();
    while(millis() - lastTime < 5000000)
    {
      digitalWrite(ESP8266_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
    }
  }
  digitalWrite(ESP8266_LED, LOW);
  // start sx1509 with default settings
  while (!io.begin(SX1509_ADDRESS))
  {
    #ifdef DEBUG_PRINT
    Serial.println("SX1509 not found. Try again...");
    #endif
    lastTime = millis();
    while(millis() - lastTime < 5000000)
    {
      digitalWrite(ESP8266_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
    }
  }
  digitalWrite(ESP8266_LED, LOW);
  // set all digital io for input
  for(int idx = 0; idx < 16; idx++)
  {
    io.pinMode(idx, INPUT_PULLUP);
  }
  // ready indication
  for(int cnt = 0; cnt < 30; cnt++)
  {
      digitalWrite(ESP8266_LED, blinkState);
      delay(100);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
  }
  digitalWrite(ESP8266_LED, LOW);
  Serial.println();
  curTime = micros();
  lastTime = curTime;
}
//
// read sensors, push data to hub
//
void loop() 
{
  curTime = micros();
  // logging - check interval, send sensor updates
  if(startLogging)
  {
    lastInterval = curTime - lastTime; 
    if(lastInterval >= sampleInterval)
    {
      if(lastInterval > targetInterval)
      {
        sampleInterval = targetInterval - (lastInterval - targetInterval);
        #ifdef DEBUG_PRINT
        Serial.print("Target interval ");Serial.println(targetInterval);
        Serial.print("Last interval ");Serial.println(lastInterval);
        Serial.print("Delta ");Serial.println((lastInterval - targetInterval));
        Serial.print("New sample interval ");Serial.println(sampleInterval); 
        #endif
      }
      da2dToSend.leafData.timeStamp = curTime - timestampAdjust;;
      for(int idx = 0; idx < 4; idx++) 
      {
        da2dToSend.leafData.a2dValues[idx] = adcSensor.getSingleEnded(idx);
      }
      da2dToSend.leafData.digitalValue = 0;
      for(int idx = 0; idx < 16; idx++)
      {
        da2dToSend.leafData.digitalValue |= (io.digitalRead(idx) << idx);
      }
      sendData();
      lastTime = curTime;
    }
  }
  // not logging - every 10000000 micros (10 sec) send heartbeat
  // triggers a timestamp send from hub  else
  {
    lastInterval = curTime - lastTime; 
    if(lastInterval >= 10000000)
    {
      heartBeatToSend.heartbeat.timeStamp = micros() - timestampAdjust;
      SendHeartBeat();
      lastTime = curTime;
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
  uint8_t result = esp_now_send(hub_addr, &da2dToSend.dataBytes[0], sizeof(da2dToSend));
  #ifdef DEBUG_PRINT
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(da2dToSend.leafData.leafType);
  Serial.print(" Time ");
  Serial.print(da2dToSend.leafData.timeStamp);
  Serial.print(" A2D Values ");
  for(int idx = 0; idx < 4; idx++)
  {
    Serial.print(da2dToSend.leafData.a2dValues[idx]); Serial.print(" ");
  }
  Serial.print(" DIGITAL Values ");
  Serial.print(da2dToSend.leafData.digitalValue, BIN); Serial.print(" ");
  Serial.print(" bytes ");
  Serial.print(da2dToSend.dataBytes[0], HEX);
  for(int idx = 1; idx < MESSAGE_LEN; idx++)
  {
    Serial.print(" ");
    Serial.print(da2dToSend.dataBytes[idx], HEX);
  }
  Serial.print("\n");
  if(result != 0)
  {
    Serial.println("send error");
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
  heartBeatToSend.heartbeat.msgType = 'H';
  heartBeatToSend.heartbeat.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &heartBeatToSend.dataBytes[0], sizeof(heartBeatToSend));
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
  Serial.print(heartBeatToSend.heartbeat.msgType);
  Serial.print(" Time ");
  Serial.print(heartBeatToSend.heartbeat.timeStamp);
  Serial.print(" bytes ");
  Serial.print(heartBeatToSend.dataBytes[0], HEX);
  for(int idx = 1; idx < 5; idx++)
  {
    Serial.print(" ");
    Serial.print(heartBeatToSend.dataBytes[idx], HEX);
  }
  Serial.print("\n");
  #endif
}
//
// request timestamp from hub
//
void requestTimestamp()
{
  uint8_t msgType = 'T';
  lastTimestampRequest = micros();
  uint8_t result = esp_now_send(hub_addr, &msgType, sizeof(msgType));
  
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
  Serial.println((char)msgType);
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
// first good send request timestamp adjustment
  if((status == 0) && 
     (timestampAdjust == 0) &&
     (micros() - lastTimestampRequest > TIMESTAMP_REQUEST_INTERVAL))
  {
    #ifdef DEBUG_PRINT
    Serial.println("Request timestamp from hub on first succesful send - OnDataSent");
    #endif
    requestTimestamp();
  }
  else if(status != 0)
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
    memcpy(&hubTimestamp, data, sizeof(hubTimestamp));
    timestampAdjust =  micros() - hubTimestamp.timeStamp;
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
    Serial.print (hubTimestamp.timeStamp);
    Serial.print(" adjustment ");
    Serial.print (timestampAdjust);
    Serial.print(" corrected ");
    Serial.println(localTs - timestampAdjust);
    #endif
  }
  // change state of logging
  else if(data[0] == 'B')
  {
    startLogging = true;
  }
  else if(data[0] == 'E')
  {
    startLogging = false;
  }
}
