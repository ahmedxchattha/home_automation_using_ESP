#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
//#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include "settingsPZEM.h"

#include <SoftwareSerial.h>  //  ( NODEMCU ESP8266 )
SoftwareSerial pzem(D7,D6);  // (RX,TX) connect to TX,RX of PZEM for NodeMCU
//SoftwareSerial pzem(D7,D8);  // (RX,TX) connect to TX,RX of PZEM
#include <ModbusMaster.h>
ModbusMaster node;
SimpleTimer timer;

//WiFi data
char ssid[] = "admin";                    //WiFi Credential
char pass[] = "12345678";              //WiFi Password
char server[] = "Put your Blynk local server IP address here";         //Blynk local server IP address
int port = 8080;                        //Blynk local port
//#define USE_LOCAL_SERVER                //Use local Blynk Server - comment-out if use Blynk hosted cloud service
#define AUTH    "O_gImc2_Yg1W5Hi_p2ntVRPiuI_XLfBy"    //PZEM-004v3 Auth code for Blynk Local Server

//////////////////////control part //////////////////
// define the GPIO connected with Relays and switches
#define RelayPin1 2  //D4
#define RelayPin2 4  //D2
#define RelayPin3 5 //D1
#define RelayPin4 14 //D5

#define SwitchPin1 10  //SD3
#define SwitchPin2 0   //D3 
#define SwitchPin3 1  //TX
#define SwitchPin4 15 //D8

#define wifiLed   16   //D0

#define VPIN_BUTTON_1    V1 
#define VPIN_BUTTON_2    V2
#define VPIN_BUTTON_3    V3 
#define VPIN_BUTTON_4    V4

int toggleState_1 = 1; //Define integer to remember the toggle state for relay 1
int toggleState_2 = 1; //Define integer to remember the toggle state for relay 2
int toggleState_3 = 1; //Define integer to remember the toggle state for relay 3
int toggleState_4 = 1; //Define integer to remember the toggle state for relay 4

int wifiFlag = 0;


BlynkTimer timer1;


void relayOnOff(int relay){

    switch(relay){
      case 1: 
             if(toggleState_1 == 1){
              digitalWrite(RelayPin1, LOW); // turn on relay 1
              toggleState_1 = 0;
              Serial.println("Device1 ON");
              }
             else{
              digitalWrite(RelayPin1, HIGH); // turn off relay 1
              toggleState_1 = 1;
              Serial.println("Device1 OFF");
              }
             delay(100);
      break;
      case 2: 
             if(toggleState_2 == 1){
              digitalWrite(RelayPin2, LOW); // turn on relay 2
              toggleState_2 = 0;
              Serial.println("Device2 ON");
              }
             else{
              digitalWrite(RelayPin2, HIGH); // turn off relay 2
              toggleState_2 = 1;
              Serial.println("Device2 OFF");
              }
             delay(100);
      break;
      case 3: 
             if(toggleState_3 == 1){
              digitalWrite(RelayPin3, LOW); // turn on relay 3
              toggleState_3 = 0;
              Serial.println("Device3 ON");
              }
             else{
              digitalWrite(RelayPin3, HIGH); // turn off relay 3
              toggleState_3 = 1;
              Serial.println("Device3 OFF");
              }
             delay(100);
      break;
      case 4: 
             if(toggleState_4 == 1){
              digitalWrite(RelayPin4, LOW); // turn on relay 4
              toggleState_4 = 0;
              Serial.println("Device4 ON");
              }
             else{
              digitalWrite(RelayPin4, HIGH); // turn off relay 4
              toggleState_4 = 1;
              Serial.println("Device4 OFF");
              }
             delay(100);
      break;
      default : break;      
      }
  
}

void with_internet(){
    //Manual Switch Control
    if (digitalRead(SwitchPin1) == LOW){
      delay(200);
      relayOnOff(1); 
      Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);   // Update Button Widget  
    }
    else if (digitalRead(SwitchPin2) == LOW){
      delay(200);
      relayOnOff(2);      
      Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);   // Update Button Widget
    }
    else if (digitalRead(SwitchPin3) == LOW){
      delay(200);
      relayOnOff(3);
      Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);   // Update Button Widget
    }
    else if (digitalRead(SwitchPin4) == LOW){
      delay(200);
      relayOnOff(4);
      Blynk.virtualWrite(VPIN_BUTTON_4, toggleState_4);   // Update Button Widget
    }
}
void without_internet(){
    //Manual Switch Control
    if (digitalRead(SwitchPin1) == LOW){
      delay(200);
      relayOnOff(1);      
    }
    else if (digitalRead(SwitchPin2) == LOW){
      delay(200);
      relayOnOff(2);
    }
    else if (digitalRead(SwitchPin3) == LOW){
      delay(200);
      relayOnOff(3);
    }
    else if (digitalRead(SwitchPin4) == LOW){
      delay(200);
      relayOnOff(4);
    }
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
  Blynk.syncVirtual(VPIN_BUTTON_4);
}

// When App button is pushed - switch the state

BLYNK_WRITE(VPIN_BUTTON_1) {
  toggleState_1 = param.asInt();
  digitalWrite(RelayPin1, toggleState_1);
}

BLYNK_WRITE(VPIN_BUTTON_2) {
  toggleState_2 = param.asInt();
  digitalWrite(RelayPin2, toggleState_2);
}

BLYNK_WRITE(VPIN_BUTTON_3) {
  toggleState_3 = param.asInt();
  digitalWrite(RelayPin3, toggleState_3);
}

BLYNK_WRITE(VPIN_BUTTON_4) {
  toggleState_4 = param.asInt();
  digitalWrite(RelayPin4, toggleState_4);
}


void checkBlynkStatus() { // called every 3 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false) {
    wifiFlag = 1;
    digitalWrite(wifiLed, HIGH); //Turn off WiFi LED
  }
  if (isconnected == true) {
    wifiFlag = 0;
    digitalWrite(wifiLed, LOW); //Turn on WiFi LED
  }
}
///////////// control data end ////////////////

int timerTask1;
double U_PR, I_PR,  P_PR,  PPR, PR_F, PR_PF, PR_alarm;
uint8_t result;  uint16_t data[6];

/////////////// energy reset //////////
  BLYNK_WRITE(V49)                                       // Virtual push button is defined as V4 of Blynk App. When the button is pushed, it will activate the codes  
        {
          if(param.asInt()==1)
          { 
            uint16_t u16CRC = 0xFFFF;                         /* declare CRC check 16 bits*/
            static uint8_t resetCommand = 0x42;               /* reset command code*/
            uint8_t slaveAddr =0X01;
            u16CRC = crc16_update(u16CRC, slaveAddr);
            u16CRC = crc16_update(u16CRC, resetCommand);
            pzem.write(slaveAddr);
            pzem.write(resetCommand);
            pzem.write(lowByte(u16CRC));
            pzem.write(highByte(u16CRC));
            delay(100);
          }
        }
/////////////energy reset part end/ ////////////////

void setup(){
 
  Serial.begin(115200); Serial.println("Start serial"); pzem.begin(9600); Serial.println("Start PZEM serial");
node.begin(1, pzem);  Serial.println("Start PZEM"); // 1 = ID MODBUS
 /////////// setup part for controlling the LEDS //////////////////
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);

  pinMode(wifiLed, OUTPUT);

  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);
  pinMode(SwitchPin4, INPUT_PULLUP);

  //During Starting all Relays should TURN OFF
  digitalWrite(RelayPin1, toggleState_1);
  digitalWrite(RelayPin2, toggleState_2);
  digitalWrite(RelayPin3, toggleState_3);
  digitalWrite(RelayPin4, toggleState_4);

  /////////// end ( setup for controlling leds 0) /////////////////////
WiFi.mode(WIFI_STA);
#if defined(USE_LOCAL_SERVER)
  WiFi.begin(ssid, pass);
  Blynk.config(AUTH, server, port);
#else
  Blynk.begin(AUTH, ssid, pass);
#endif
  while (Blynk.connect() == false) {}
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

//  timerTask1 = timer.setInterval(1000, updateBlynk);

}

void updateBlynk() {
  Blynk.virtualWrite(vPIN_VOLTAGE,               U_PR/*String(U_PR, 1) + " V"*/);
  Blynk.virtualWrite(vPIN_CURRENT_USAGE,         I_PR/*String(I_PR, 1) + " Amps"*/);
  Blynk.virtualWrite(vPIN_ACTIVE_POWER,          String(P_PR, 1) + " Watts");
  Blynk.virtualWrite(vPIN_ACTIVE_ENERGY,         String(PPR*1000, 1) + " Wh");
  Blynk.virtualWrite(vPIN_FREQUENCY,             PR_F);
  Blynk.virtualWrite(vPIN_POWER_FACTOR,          PR_PF);
  Blynk.virtualWrite(vPIN_OVER_POWER_ALARM,      PR_alarm);
  BLYNK_WRITE(V49);

}

void loop(){
Blynk.run();
//ArduinoOTA.handle();
//timer.run();
///////////loop part for control //////////////
 timer1.run(); // Initiates SimpleTimer
  if (wifiFlag == 0)
    with_internet();
  else
    without_internet();
 //////////looop part for control ////////////
result = node.readInputRegisters(0x0000, 10);
  if (result == node.ku8MBSuccess)  {
U_PR      = (node.getResponseBuffer(0x00)/10.0f);
I_PR      = (node.getResponseBuffer(0x01)/1000.000f);
P_PR      = (node.getResponseBuffer(0x03)/10.0f);
PPR       = (node.getResponseBuffer(0x05)/1000.0f);
PR_F      = (node.getResponseBuffer(0x07)/10.0f);
PR_PF     = (node.getResponseBuffer(0x08)/100.0f);
PR_alarm  = (node.getResponseBuffer(0x09));  
 } 

    Serial.print("U_PR:     "); Serial.println(U_PR);   // V
    Serial.print("I_PR:     "); Serial.println(I_PR,3);   //  A
    Serial.print("P_PR:     "); Serial.println(P_PR);   //  W 
    Serial.print("PPR:      "); Serial.println(PPR,3);   // kWh
    Serial.print("PR_F:     "); Serial.println(PR_F);    // Hz
    Serial.print("PR_PF:    "); Serial.println(PR_PF);  
    Serial.print("PR_alarm: "); Serial.println(PR_alarm,0);
Serial.println("====================================================");

updateBlynk();
//  delay(1000);
}
