/**
  This program is designed to read wind speed and wind direction data
  from an Ecowitt WS90 wind sensor via Modbus RTU 485 communication.
  The read data is published via MQTT.
  The data is stored on an SD card with a timestamp when the internet is unavailable.
  Once the internet is back, the stored data is then published again via the broker.
  
  The program uses the ArduinoModbus library and ArduinoRS485 library to communicate with the wind sensor,
  the MKRNB library and ArduinoMqttClient library to publish the data via MQTT, 
  and SD library and RTCZero library to save the data with timestamp on the SD card.

  Circuit:
  - MKR NB 1500
  - Ecowitt WS90 Wind Sensor
  - MKR SD Proto Shield
  - MKR 485 Shield
    - Y connected with A (Green) of the Modbus RTU sensor
    - Z connected with B (White) of the Modbus RTU sensor
    - Jumper positions
       - A \/\/ B set to OFF
       - FULL set to OFF
       - Z \/\/ Y set to ON

  @file
  @date Created on: 09.07.2023
  @author C.Feng
  
*/

// Include libraries
#include <ArduinoModbus.h>
#include <ArduinoRS485.h> 
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <SD.h>
#include <RTCZero.h>

// Set time (24-hour format)
#define HOURS 18
#define MINUTES 30
#define SECONDS 0

// Modbus configuration
#define SLAVE_ID 0x090 // Device Address of the Ecowitt WS90 Wind Sensor
#define WIND_SPEED_REGISTER 0x0169 // Register address of the wind speed
#define WIND_DIRECTION_REGISTER 0x016B // Register address of the wind direction
#define BATTERY_EMPTY_VALUE 65535 // Battery-empty state
#define TIME_OUT -1

// Initialize library instance
NBClient client;
GPRS gprs;
NB nbAccess;
MqttClient mqttClient(client);
RTCZero rtc; 

// MQTT configuration
const char broker[] = "test.mosquitto.org";
const uint16_t port = 1883;
const uint8_t QoS = 2;
const char topic_windData[] = "Sensor_KN/Winddata";
const char topic_errorMessage[] = "Sensor_KN/Error";

void setup() {
  Serial.begin(9600);
  while(!Serial){
    // Baud rate of the Modbus device (9600 bps), 8 Data bits, No Parity, 1 Stop
    if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
      Serial.println("Failed to start Modbus RTU Client!");
      while (1);
    }
  }

  // Connect to NB IoT
  if (!nbAccess.begin()) {
    Serial.println("Failed to start NB IoT!");
    while (1);
  }

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("SD card failed, or not present");
    while (1);
  } else {
    Serial.println(" SD card initialized.");
  }

  // Connect to MQTT
  if (!mqttClient.connect(broker, port)) {
    Serial.print("Failed to connect to MQTT broker! Erroe code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker");

  // Initialize RTC
  rtc.begin();  
  rtc.setHours(HOURS);
  rtc.setMinutes(MINUTES);
  rtc.setSeconds(SECONDS);
}

void loop() {
  // Current time
  byte hours = rtc.getHours();
  byte minutes = rtc.getMinutes();
  byte seconds = rtc.getSeconds();
  String timestamp = String(hours) + ":" + String(minutes) + ":" + String(seconds);

  // Print the current time
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(seconds);

  if (mqttClient.connected()) {
    mqttPublish(timestamp);
    readAndSendDataFromSDCard();
  } else {
    saveDataToSDCard(timestamp);
  }
  delay(5000); // 5 seconds
}

void mqttPublish(String timestamp) {
  // Read wind speed register and wind direction register
  uint16_t windSpeedRaw = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_SPEED_REGISTER);
  uint16_t windDirection = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_DIRECTION_REGISTER);
  float windSpeed;

  if (windSpeedRaw == BATTERY_EMPTY_VALUE || windDirection == BATTERY_EMPTY_VALUE) {
    // Both wind speed and wind direction have the battery-empty value, publish an error message
    Serial.println("Error: Battery is empty.");
    mqttClient.beginMessage(topic_errorMessage);
    mqttClient.print("Error: Battery is empty.");
    mqttClient.endMessage();
  } else if (windSpeedRaw != TIME_OUT && windDirection != TIME_OUT) {
    windSpeed = windSpeedRaw * 0.1; // Convert raw wind speed to actual wind speed
    Serial.print("Wind Speed: ");
    Serial.println(windSpeed, 1);
    Serial.print("Wind Direction: ");
    Serial.println(windDirection);
    String dataString = timestamp + "," + String(windSpeed, 1) + "," + String(windDirection); // format in string

    // Publish wind data via MQTT
    mqttClient.beginMessage(topic_windData, false, QoS, false);
    mqttClient.print(dataString);
    mqttClient.endMessage();
  } else {
    Serial.println(ModbusRTUClient.lastError());
    // Publish error message via MQTT
    mqttClient.beginMessage(topic_errorMessage);
    mqttClient.print("Error: Time Out.");
    mqttClient.endMessage();
  }
}

void saveDataToSDCard(String timestamp) {
  // Read wind speed register and wind direction register
  uint16_t windSpeedRaw = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_SPEED_REGISTER);
  uint16_t windDirection = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_DIRECTION_REGISTER);
  float windSpeed;

  if (windSpeedRaw != BATTERY_EMPTY_VALUE && windDirection != BATTERY_EMPTY_VALUE) {
    if (windSpeedRaw != TIME_OUT && windDirection != TIME_OUT) {
      windSpeed = windSpeedRaw * 0.1;
      String dataString = timestamp + "," + String(windSpeed, 1) + "," + String(windDirection); // format in string
      File dataFile = SD.open("winddata.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        Serial.print(dataString);
        Serial.println(" ...saved");
        dataFile.close();
      } else {
        Serial.println("Error opening winddata.txt");
      }
    } else {
      Serial.println(ModbusRTUClient.lastError());
    }
  } else {
    Serial.println("Error: Battery is empty.");
  }
}

void readAndSendDataFromSDCard() {
  if (SD.exists("winddata.txt")) {
    File dataFile = SD.open("winddata.txt");
    if (dataFile) {
      String dataString;
      while (dataFile.available()) {
        dataString = dataFile.readStringUntil('\n');
        mqttClient.beginMessage(topic_windData, false, QoS, false);
        mqttClient.print(dataString);
        mqttClient.endMessage();
      }
      dataFile.close();
      SD.remove("winddata.txt");
    } else {
      Serial.println("Error opening winddata.txt");
    }
  } 
}
