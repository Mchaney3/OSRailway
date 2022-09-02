/*
******************************    About     *******************************
    
  BLE control for OS Railway using an ESP32-S3 WROOM1 and Android or iPhone

******************************    PINS USED     *******************************
   
  //  Motors
  int motorForwardPin =  5; //GPIO5
  int motorBackwardPin = 6; //GPIO6
  
  //  Voltage Sensor
  const int batteryVOLTAGE = 7; // Use GPIO number. See ESP32 board pinouts
  
  //  NFC
  Chip Select       8
  
  //  Software SPI Pins
  SPID = MOSI = GPIO11 = data out
  SPIQ = MISO = GPIO13 = data in
  CLK = GPIO12
  CS = GPIO10

  //  Ws2812B
  int ledPIN = 48;

******************************    TODO     *******************************

  Send values back to app to display rather than displaying static values sent from the phone app
  

******************************************************************************/

#include <stdio.h>
#include <Arduino.h>

/*
//Timer
#define _TIMERINTERRUPT_LOGLEVEL_     4
#include "ESP32TimerInterrupt.h"
#define PIN_D2              4         // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2/LED_BUILTIN of ESP32
#define PIN_D3              5         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
*/
//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#define SERVICE_UUID           "ffbfa26f-69b2-405b-83fc-5708c9f48a30" // UART service UUID
#define CHARACTERISTIC_UUID_RX "ffbfa26f-69b2-405b-83fc-5708c9f48a31"
#define CHARACTERISTIC_UUID_TX "ffbfa26f-69b2-405b-83fc-5708c9f48a32"
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool sendResponse = false;
bool autoPilot = false;
String rxString = "";    // string to hold input
String responseValue = "";
const char* rx_LED_Speed = "SPD,";
const char* rx_LED_Brightness = "BRT,";
const char* rx_LED_Mode = "MDE,";
const char* rx_LED_Color = "RGB,";
const char* rx_motorSpeed = "MSP,";
const char* rx_MD = "MFD,";
const char* rx_ESTOP = "EST,";
const char* rx_AP = "AP";
/*
const char* tx_Speed = "";
const char* tx_Brightness = "";
const char* tx_Mode = "";
String tx_Value = "";
*/

//WiFi
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char* ssid = "WiFi Network Name";
const char* password = "WiFi Network Password";
const char deviceName[] = "OS Railway";

//Voltage Sensor
const int batteryVOLTAGE = 4; // //GPIO7

//Motor
int motorForwardPin = 5; //GPIO5
int motorBackwardPin = 6; //GPIO6
int motorSpeed = 0;  //PWM value to initialize motors with
String rx_MotorDirection = "";
String motorForward = "true";

void setMotorSpeed(int ms)  
{
  if (motorForward)  
  {
    analogWrite(motorForwardPin, ms);
    analogWrite(motorBackwardPin, 0);
  }
  else  
  {
    analogWrite(motorForwardPin, 0);
    analogWrite(motorBackwardPin, ms);
  }
}

//Neopixel
#include <WS2812FX.h>
#define ledPin    48  // digital pin used to drive the LED strip
#define ledCount 1  // number of LEDs on the strip
int led_BRIGHTNESS = 64;
int led_SPEED = 1000;
int led_MODE = 1;
String led_COLOR = "BLUE";
WS2812FX ws2812fx = WS2812FX(ledCount, ledPin, NEO_GRB + NEO_KHZ800);

//SD Libraries
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"
#define SD_FAT_TYPE 1
#define SCK 12  //  Hardware System Clock Pin
#define MISO 13  //  Hardware MISO Pin
#define MOSI 11  //  Hardware MOSI Pin
#define SS 10  //  Hardware Chip Select Pin

/*
  SPID = MOSI = GPIO11 = data out
  SPIQ = MISO = GPIO13 = data in
  CLK = GPIO12
  CS = GPIO10
*/

// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 10;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 13;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN  = 12;
// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(40))
SdFat32 sd;
File32 file;

//  BLE Callbacks
class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
      deviceConnected = true;
      Serial.println("Device Connected");
    };

    void onDisconnect(BLEServer* pServer) 
    {
      deviceConnected = false;
      if (autoPilot)
      {
        Serial.println("Device Disonnected with AutoPilot engaged");
        pServer->getAdvertising()->start();
        Serial.println("Awaiting client connection...");
      }
      else
      {
        analogWrite(motorForwardPin, 0);  //  Set motor to half speed using PWM
        analogWrite(motorBackwardPin, 0);  //  Set motor to half speed using PWM 
        Serial.println("Device Disonnected without AutoPilot. Stopping motors");
        pServer->getAdvertising()->start();
        Serial.println("Awaiting client connection...");
      }
    }
};

class MyCallbacks: public BLECharacteristicCallbacks 
{

  void onRead(BLECharacteristic *pCharacteristic) 
  {
    pCharacteristic->notify();
  }

  //  Called anytime a value is changed in the app
  void onWrite(BLECharacteristic *pCharacteristic) 
  {
    String rxString = pCharacteristic->getValue().c_str();

    if (rxString.length() > 0) 
    {
      Serial.println("*********");
      Serial.print("\r\nReceived Value: ");

      for (int i = 0; i < rxString.length(); i++) 
      {
        Serial.print(rxString[i]);
      }
       Serial.println();
       
/************************     E-STOP      ************************/
      if (rxString.indexOf(rx_ESTOP) != -1) 
      {
        Serial.println("EMERGENCY STOPPING!");
        analogWrite(motorForwardPin, motorSpeed);
        analogWrite(motorBackwardPin, motorSpeed);
        Serial.println("MOTORS STOPPED!");
        pCharacteristic->setValue("Motor's E-STOPPED");
        pCharacteristic->notify();
      }
      
/************************     Motor Direction      ************************/
      else if (rxString.indexOf(rx_MD) != -1) 
      {
        rx_MotorDirection = rxString.substring(4);
        if (rx_MotorDirection == "true")
        {
          Serial.print("Chugaluggin forward at "); Serial.print(motorSpeed); Serial.println(" PWM's");
          analogWrite(motorBackwardPin, 0);
          analogWrite(motorForwardPin, motorSpeed);
          motorForward = true;
          pCharacteristic->setValue("Chugaluggin forward!");
        }
        else
        {
          Serial.print("Chugaluggin backurds at "); Serial.print(motorSpeed); Serial.println(" PWM's");
          analogWrite(motorForwardPin, 0);  //  Set motor to half speed using PWM
          analogWrite(motorBackwardPin, motorSpeed);  //  Set motor to half speed using PWM
          motorForward = false;
          pCharacteristic->setValue("Chugaluggin Backurds!"); // Send the value to the app!
        }
      }

/************************     Autopilot      ************************/
      else if (rxString.indexOf(rx_AP) != -1) 
      {
        autoPilot = !autoPilot;
      }
      
      else if (rxString.indexOf(rx_motorSpeed) != -1) 
      {
        motorSpeed = rxString.substring(4).toInt();
        motorSpeed = map(motorSpeed, 0, 100, 0, 255);
        if (motorForward)
          {
            Serial.print("Setting FORWARD motor speed to: ");  Serial.println(rxString.substring(4));
            analogWrite(motorForwardPin, motorSpeed);
          }
          else
          {
            Serial.print("Setting BACKWARD motor speed to: ");  Serial.println(rxString.substring(4));
            analogWrite(motorBackwardPin, motorSpeed);
          }
        pCharacteristic->setValue("Motor Speed Set!"); // Send the value to the app!
      }

/************************     LED Speed      ************************/
      else if (rxString.indexOf(rx_LED_Speed) != -1) 
      {
        Serial.print("Current LED speed: "); Serial.println(ws2812fx.getSpeed());
        Serial.print("Setting LED speed: ");  Serial.println(rxString.substring(4));
        led_SPEED = rxString.substring(4).toInt();
        led_SPEED = map(led_SPEED, 0, 100, 4975, 50);
        ws2812fx.setSpeed(led_SPEED);
        ws2812fx.service();
        Serial.println(led_SPEED);
        Serial.print("Current LED speed: "); Serial.println(ws2812fx.getSpeed());
        pCharacteristic->setValue("LED Speed Set!"); // Send the value to the app!
      }

/************************     LED Brightness      ************************/
      else if (rxString.indexOf(rx_LED_Brightness) != -1) 
      {
        Serial.print("Current LED Brightness: "); Serial.println(ws2812fx.getBrightness());
        Serial.print("Setting LED brightness: ");  Serial.println(rxString.substring(4));
        led_BRIGHTNESS = rxString.substring(4).toInt();
        led_BRIGHTNESS = map(led_BRIGHTNESS, 0, 100, 0, 255);
        Serial.println(led_BRIGHTNESS);
        ws2812fx.setBrightness(led_BRIGHTNESS);
        ws2812fx.service();
        Serial.print("Current LED Brightness: "); Serial.println(ws2812fx.getBrightness());
        pCharacteristic->setValue("LED Brightness Set!"); // Send the value to the app!
      }    

/************************     LED Mode      ************************/
      else if (rxString.indexOf(rx_LED_Mode) != -1) 
      {
        led_MODE = rxString.substring(4).toInt();
        Serial.print("Current LED mode: "); Serial.println(ws2812fx.getModeName(ws2812fx.getMode()));
        Serial.print("Setting LED Mode: ");  Serial.println(ws2812fx.getModeName(led_MODE));
        ws2812fx.setMode(led_MODE); // segment 0 is leds 0 - 9
        Serial.print("Current LED mode: "); Serial.println(ws2812fx.getModeName(ws2812fx.getMode()));
        pCharacteristic->setValue("LED Mode Set"); // Send the value to the app!
      }   

/************************     LED Color      ************************/
      else if (rxString.indexOf(rx_LED_Color) != -1) 
      {
        const byte numChars = 12;
        char receivedChars[numChars];                   // temporary array for use when parsing
        // variables to hold the parsed data
        int intLedColor = 0;
        led_COLOR = rxString.substring(4);
        led_COLOR.toCharArray(receivedChars, 12);
        char * strtokIndx; // this is used by strtok() as an index
        strtokIndx = strtok(receivedChars,",");      // get the first part - the string
        intLedColor = atoi(strtokIndx);     // convert red part to an integer
        int r = intLedColor;
        //  Red value mapped. Now we're looking at the green value in the array
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        intLedColor = atoi(strtokIndx);     // convert this part to an integer
        int g = intLedColor;
        //  Blue's Turn
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        intLedColor = atoi(strtokIndx);     // convert this part to an integer
        int b = intLedColor;
        Serial.print("r: "); Serial.println(r);  Serial.print("g: "); Serial.println(g);  Serial.print("b: ");  Serial.println(b);
        ws2812fx.setColor(r,g,b);
        pCharacteristic->setValue("LED Color Set"); // Send the value to the app!
      }
      else
      {
        Serial.print("Received unmapped value: "); Serial.println(rxString);
        pCharacteristic->setValue("Received unmapped value"); // Send the value to the app!
      }
      Serial.println();
      Serial.println("*********"); 
    }
  }
};

// Display useful info at startup
void display_Running_Sketch (void)
{
  int cpuSpeed = getCpuFrequencyMhz();
  String the_path = __FILE__;
  int slash_loc = the_path.lastIndexOf('/');
  String the_cpp_name = the_path.substring(slash_loc+1);
  int dot_loc = the_cpp_name.lastIndexOf('.');
  String the_sketchname = the_cpp_name.substring(0, dot_loc);

  Serial.print("\nCPU initialized at " + String(cpuSpeed) + "MHz");
  Serial.print("\nBooting ");
  Serial.print(the_sketchname);
  Serial.print("\nCompiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.print("\nESP-IDF SDK: "); 
  Serial.println(ESP.getSdkVersion());
  Serial.println();
}

void setup() 
{

  //  Begin Serial Debugging
  delay(500);
  Serial.setDebugOutput(true);
  Serial.begin(115200);
  delay(500);

  // detailed information, see the MemoryAndCpuUsage example
  display_Running_Sketch();

  if (!sd.begin(SD_CONFIG)) 
  {
        Serial.println("Failed to initialize SD card.");  // Use a bool to continue booting and disable SD read/write in sketch
        //while(true);                  // end program        
  } else
  {
    Serial.println("SD card initialized");
  }

  pinMode(motorForwardPin, OUTPUT);
  pinMode(motorBackwardPin, OUTPUT);
  analogWrite(motorForwardPin, motorSpeed);
  analogWrite(motorBackwardPin, motorSpeed);
  Serial.println("Motors initialized");

  ws2812fx.init();
  ws2812fx.setBrightness(led_BRIGHTNESS);
  // parameters: index, start, stop, mode, color, speed, reverse
  //ws2812fx.setSegment(0,  0,  9, FX_MODE_BLINK, 0xFF0000, 1000, false); // segment 0 is leds 0 - 9
  //ws2812fx.setSegment(1, 10, 19, FX_MODE_SCAN,  0x00FF00, 1000, false); // segment 1 is leds 10 - 19
  //ws2812fx.setSegment(2, 20, 29, FX_MODE_COMET, 0x0000FF, 1000, true);  // segment 2 is leds 20 - 29
  ws2812fx.setSegment(0,  0,  ledCount-1, FX_MODE_STATIC, BLUE, 1000, false); // segment 0 is led 0 a.k.a. Status LED
  ws2812fx.start();
  Serial.println("Neopixels initialized");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  Serial.println("WiFi initialized");
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPasswordHash("0228eeeaf825a0c3cab212950b92de6b");

  ArduinoOTA
    .onStart([]()
    {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      ws2812fx.setSegment(0,  0,  1, FX_MODE_BREATH, ORANGE, 1000, false); // segment 0 is led 0 a.k.a. Status LED
      ws2812fx.service();
      Serial.println("Start updating " + type);
    })
    .onEnd([]() 
    {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) 
    {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) 
    {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("OTA initialized");

  // Create the BLE Device
  BLEDevice::init(deviceName); // Give it a name
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("BLE initialized");
  
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  Serial.println("\r\nSystem initialized and ready. Awaiting client connection to begin BLE advertising...");
  ws2812fx.service();
}

void loop() {

  ws2812fx.service();
  ArduinoOTA.handle();

}
