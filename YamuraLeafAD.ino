/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `hub` to the SSID of hub for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `hub`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

//#include <esp_now.h>
//#include <WiFi.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define MESSAGE_LEN 11
struct LeafData
{
  char leafType;            // 1 byte  - type, in this case 'A' for accelerometer
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
  int16_t values[3];        // 6 bytes - X,Y,Z acceleration values
};
union DataToSend
{
  struct LeafData leafData;
  uint8_t dataBytes[11];
} toSend;

unsigned long lastTime;
unsigned long curTime;
unsigned long sampleInterval = 50;
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};
//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;
//
//
//
void setup()
{
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  // This is the mac address of this device
  Serial.print("ESP-NOW accelerometer leaf at ");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  // set last and current sample 
  lastTime = millis();
  curTime = millis();
  toSend.leafData.leafType = 'A';

  Wire.begin();

  while (!imu.begin()) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    delay(5000);
  }
}
//
// read accelerometer, push data to hub
//
void loop() 
{
  curTime = millis();
  if((long)(curTime - lastTime) > sampleInterval)
  {
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
      toSend.leafData.timeStamp = curTime;
      toSend.leafData.values[0] = imu.ax;
      toSend.leafData.values[1] = imu.ay;
      toSend.leafData.values[2] = imu.az;
      sendData();
      lastTime = millis();
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
    Serial.println("Reconnectting");
    uint8_t addStatus = esp_now_add_peer(hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }
  //const uint8_t *hub_addr = hub.hub_addr;
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
  Serial.print(" Values ");
  Serial.print(toSend.leafData.values[0]); Serial.print(" ");
  Serial.print(toSend.leafData.values[1]); Serial.print(" ");
  Serial.print(toSend.leafData.values[2]); Serial.print(" ");

  Serial.print(" bytes ");
  Serial.print(toSend.dataBytes[0], HEX);
  for(int idx = 1; idx < MESSAGE_LEN; idx++)
  {
    Serial.print(" ");
    Serial.print(toSend.dataBytes[idx], HEX);
  }
  Serial.print("\n");
  uint8_t result = esp_now_send(hub_addr, &toSend.dataBytes[0], sizeof(LeafData));
  switch(result)
  {
    case 0:
      //Serial.println("Success");
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
      Serial.println("Unknown error");
      break;
  }
}
//
// Init ESP Now with fallback
//
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
//
// callback when data is sent from Master to Slave
//
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
  if(status != 0)
  {
    char macStr[18];
    Serial.print("Last Packet Sent to: ");
    for(int idx = 0; idx < 6; idx++)
    {
      Serial.print(mac_addr[idx], HEX); Serial.print(" ");
    }
    Serial.print(" Failed: ");
    Serial.println(status);
  }
}
