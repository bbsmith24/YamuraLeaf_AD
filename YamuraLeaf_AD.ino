/*
   Yamura Leaf - A2D and Digital IO
   BBS 3/2021
   send 16 digital I/O and 4 analog to digital values to hub

   for ESP8266
 */
//#define DEBUG_PRINT
#define TARGET_INTERVAL 50000

#include <ESP8266WiFi.h>
#include <espnow.h>

#include <Wire.h>
//#include <SPI.h>
// 4 channel A2D
#include <SparkFun_ADS1015_Arduino_Library.h>
// 16 channel digial IO
#include <SparkFunSX1509.h>

#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define MESSAGE_LEN 14
struct LeafData
{
  char leafType;            // 1 byte  - type, in this case 'I' for IO - might need to make this 2 characters...
  unsigned long timeStamp;  // 4 bytes - micros() value of sample
  int16_t a2dValues[4];     // 8 bytes, 2 per a2d channel
  int16_t digitalValue;     // 1 byte, 1 bit per digital channel
};
union DataToSend
{
  struct LeafData leafData;
  uint8_t dataBytes[MESSAGE_LEN];
} toSend;

struct HubTimeStamp
{
  char msgType;
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
};

unsigned long lastTime;
unsigned long curTime;
unsigned long targetInterval = TARGET_INTERVAL;  // (sample at 20Hz)
unsigned long sampleInterval = TARGET_INTERVAL;
unsigned long lastInterval;
unsigned long timestampAdjust = 0;

uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};

// ADS1015 sensor
ADS1015 adcSensor;

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

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
  Serial.print("ESP-NOW digital/A2D leaf at ");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  #endif
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // register for receive callback for data send response
  esp_now_register_recv_cb(OnDataRecv);
  //
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  // set last and current sample 
  lastTime = micros();
  curTime = micros();
  toSend.leafData.leafType = 'I';

  Wire.begin();
  // start ads1015 with default settings
  while (!adcSensor.begin())
  {
    #ifdef DEBUG_PRINT
    Serial.println("ADS1015 not found. Try again...");
    #endif
    curTime = millis();
    lastTime = curTime;
    while(curTime - lastTime < 5000000)
    {
      digitalWrite(ESP8266_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
      lastTime = millis();
    }
  }
  digitalWrite(ESP8266_LED, LOW);
  // start sx1509 with default settings
  while (!io.begin(SX1509_ADDRESS))
  {
    #ifdef DEBUG_PRINT
    Serial.println("SX1509 not found. Try again...");
    #endif
    curTime = millis();
    lastTime = curTime;
    while(curTime - lastTime < 5000000)
    {
      digitalWrite(ESP8266_LED, blinkState);
      delay(500);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
      lastTime = millis();
    }
  }
  digitalWrite(ESP8266_LED, LOW);
  // set all digital io for input
  for(int idx = 0; idx < 16; idx++)
  {
    io.pinMode(idx, INPUT_PULLUP);
  }
  curTime = micros();
  lastTime = curTime;
}
//
// read sensors, push data to hub
//
void loop() 
{
  curTime = micros();
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
    toSend.leafData.timeStamp = curTime - timestampAdjust;;
    lastTime = curTime;
    for(int idx = 0; idx < 4; idx++) 
    {
      toSend.leafData.a2dValues[idx] = adcSensor.getSingleEnded(idx);
    }
    toSend.leafData.digitalValue = 0;
    for(int idx = 0; idx < 16; idx++)
    {
      toSend.leafData.digitalValue |= (io.digitalRead(idx) << idx);
    }
    sendData();
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
  #ifdef DEBUG_PRINT
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(toSend.leafData.leafType);
  Serial.print(" Time ");
  Serial.print(toSend.leafData.timeStamp);
  Serial.print(" A2D Values ");
  for(int idx = 0; idx < 4; idx++)
  {
    Serial.print(toSend.leafData.a2dValues[idx]); Serial.print(" ");
  }
  Serial.print(toSend.leafData.digitalValue, BIN); Serial.print(" ");
  Serial.print(" bytes ");
  Serial.print(toSend.dataBytes[0], HEX);
  for(int idx = 1; idx < MESSAGE_LEN; idx++)
  {
    Serial.print(" ");
    Serial.print(toSend.dataBytes[idx], HEX);
  }
  Serial.print("\n");
  #endif
  uint8_t result = esp_now_send(hub_addr, &toSend.dataBytes[0], sizeof(LeafData));
  switch(result)
  {
    case 0:
      break;
    /* 
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("\nESPNOW not Init");
      break;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("\nInvalid Argument");
      break;
    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("\nInternal Error");
      break;
    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("\nESP_ERR_ESPNOW_NO_MEM");
      break;
    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("\nPeer not found.");
      break;
    */
    default:
      #ifdef DEBUG_PRINT
      Serial.println("Unknown error");
      #endif
      break;
  }
}
//
// send data
//
void requestTimestamp()
{
  uint8_t msgType = 'T';
  #ifdef DEBUG_PRINT
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.println((char)msgType);
  #endif
  uint8_t result = esp_now_send(hub_addr, &msgType, sizeof(msgType));
  #ifdef DEBUG_PRINT
  switch(result)
  {
    case 0:
      break;
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("\nESPNOW not initialized Error");
      break;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("\nInvalid Argument Error");
      break;
    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("\nOut of memory Error");
      break;
    case ESP_ERR_ESPNOW_FULL:
      Serial.println("\nPeer list full Error");
      break;
    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("\nPeer not found Error");
      break;
    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("\nInternal Error");
      break;
    case ESP_ERR_ESPNOW_EXIST:
      Serial.println("\nPeer has existed Error");
      break;
    case ESP_ERR_ESPNOW_IF:
      Serial.println("\nInterface error Error");
      break;
    default:
      Serial.print("Other message send error ");
      Serial.print(result);
      Serial.print(" error base ");
      Serial.println(ESP_ERR_ESPNOW_BASE);
      break;
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
// callback when data is sent from Master to Slave
//
void OnDataSent(uint8_t *mac_addr, uint8_t status) 
{
// first good send request timestamp adjustment
  if((status == 0) && (timestampAdjust == 0))
  {
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
  if(data[0] != 'T')
  {
    return;
  }
  #ifdef DEBUG_PRINT
  Serial.print("Recv ");Serial.print(data_len);Serial.print(" bytes from: ");
  for(int idx = 0; idx < 6; idx++)
  {
    Serial.print(mac_addr[idx], HEX);Serial.print(":");
  }
  #endif
  HubTimeStamp hubTimestamp;
  memcpy(&hubTimestamp, data, sizeof(hubTimestamp));
  #ifndef DEBUG_PRINT
  timestampAdjust =  micros() - hubTimestamp.timeStamp;
  #endif
  #ifdef DEBUG_PRINT
  unsigned long localTs = micros();
  timestampAdjust =  localTs - hubTimestamp.timeStamp;
  Serial.print(" local timestamp ");
  Serial.print (localTs);
  Serial.print (" ");
  Serial.print (localTs,HEX);
  Serial.print(" HUB timestamp ");
  Serial.print (hubTimestamp.timeStamp);
  Serial.print (" ");
  Serial.print (hubTimestamp.timeStamp, HEX);
  Serial.print(" adjustment ");
  Serial.print (timestampAdjust);
  Serial.print (" ");
  Serial.println (timestampAdjust, HEX);
  #endif
}
