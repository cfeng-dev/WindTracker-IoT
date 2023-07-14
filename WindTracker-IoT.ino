/**
  This program is designed to read wind speed and wind direction data
  from an Ecowitt WS90 wind sensor via Modbus RTU 485 communication.
  The read data is published via MQTT.
  The data is stored on an SD card with a timestamp when the internet is unavailable.
  Once the internet is back, the stored data is then published again via the broker.
  
  The program uses the ArduinoModbus library and ArduinoRS485 library to communicate with the wind sensor,
  the MKRNB library and ArduinoMqttClient library to publish the data via MQTT, 
  and SD library and RTCZero library to save the data with timestamp on the SD card.

  The implementation has been enhanced with the use of the FreeRTOS_SAMD21 library, enabling multitasking capabilities
  to improve the overall performance and reliability of the program. Tasks in the program include reading from the sensor,
  handling data, and monitoring tasks, each running independently on separate cores.

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
#include <FreeRTOS_SAMD21.h>

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

// Define a structure to hold wind data
struct WindData {
  uint16_t windSpeedRaw;
  uint16_t windDirection;
  float windSpeed;
};
QueueHandle_t queueWindDataMqtt; // Handle to the queue storing data destined for MQTT communication
QueueHandle_t queueWindDataMonitor; // Handle to the queue storing data destined for system monitoring

//*****************************************************************************
//
// Function to handle sensor data collection and distribution.
//
//*****************************************************************************
static void vTaskSensor(void *pvParameters) {
  while(1) {
    WindData data;
    // Read wind speed register and wind direction register
    data.windSpeedRaw = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_SPEED_REGISTER);
    data.windDirection = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_DIRECTION_REGISTER);
    data.windSpeed = data.windSpeedRaw * 0.1; // Convert raw wind speed to actual wind speed

    // Send data to the queues, block for max. 100 ms if the queue is full
    if (xQueueSend(queueWindDataMqtt, &data, pdMS_TO_TICKS(100)) != pdPASS ||
      xQueueSend(queueWindDataMonitor, &data, pdMS_TO_TICKS(100)) != pdPASS) {
      // handle error...
      Serial.println("sensorTask Failed! \n");
    }
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
  }
}

//*****************************************************************************
//
// Function to handle the processing of wind data:
//
// 1. If connected to the MQTT broker, validates the wind data and sends the formatted data to the broker.
// 2. If not connected to the MQTT broker, saves wind data to SD card.
// 3. If data exists on the SD card (i.e., when MQTT connection was previously unavailable), sends this data to the MQTT broker.
//
//*****************************************************************************
static void vTaskDataHandler(void *pvParameters) {
  while(1) {
    // Current time
    byte hours = rtc.getHours();
    byte minutes = rtc.getMinutes();
    byte seconds = rtc.getSeconds();
    String timestamp = String(hours) + ":" + String(minutes) + ":" + String(seconds);
    WindData data;

    if (xQueueReceive(queueWindDataMqtt, &data, pdMS_TO_TICKS(100)) == pdPASS) {
      if (mqttClient.connected()) {
        // Connection to MQTT broker
        if (data.windSpeedRaw == BATTERY_EMPTY_VALUE || data.windDirection == BATTERY_EMPTY_VALUE) {
          // Both wind speed and wind direction have the battery-empty value, publish an error message
          Serial.println("Error: Battery is empty.");
          mqttClient.beginMessage(topic_errorMessage);
          mqttClient.print("Error: Battery is empty.");
          mqttClient.endMessage();
        } else if (data.windSpeedRaw != TIME_OUT && data.windDirection != TIME_OUT) {
          String dataString = timestamp + "," + String(data.windSpeed, 1) + "," + String(data.windDirection); // format in string

          // Publish wind data via MQTT
          mqttClient.beginMessage(topic_windData, false, QoS, false);
          mqttClient.print(dataString);
          mqttClient.endMessage();

          // Read and send data from SD Card
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
        } else {
          Serial.println(ModbusRTUClient.lastError());
          // Publish error message via MQTT
          mqttClient.beginMessage(topic_errorMessage);
          mqttClient.print("Error: Time Out.");
          mqttClient.endMessage();
        }
      } else {
        // Not connected to the MQTT broker, saves wind data to SD card
        if (data.windSpeedRaw != BATTERY_EMPTY_VALUE && data.windDirection != BATTERY_EMPTY_VALUE) {
          if (data.windSpeedRaw != TIME_OUT && data.windDirection != TIME_OUT) {
            data.windSpeed = data.windSpeedRaw * 0.1;
            String dataString = timestamp + "," + String(data.windSpeed, 1) + "," + String(data.windDirection); // format in string
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
    
    } else {
      // handle error...
      Serial.println("mqttTask Failed!");
    }
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
  }  
}

//*****************************************************************************
//
// Function to monitor wind data.
//
//*****************************************************************************
static void vTaskMonitor(void *pvParameters) {
  while(1) {
    WindData data;
    if (xQueueReceive(queueWindDataMonitor, &data, pdMS_TO_TICKS(100)) == pdPASS) {
      if (mqttClient.connected()) {
        if (data.windSpeedRaw == BATTERY_EMPTY_VALUE || data.windDirection == BATTERY_EMPTY_VALUE) {
          // Both wind speed and wind direction have the battery-empty value, publish an error message
          Serial.println("Error: Battery is empty.");
        } else if (data.windSpeedRaw != TIME_OUT && data.windDirection != TIME_OUT) {
          // Monitor data
          Serial.print("Wind Speed: ");
          Serial.println(data.windSpeed, 1);
          Serial.print("Wind Direction: ");
          Serial.println(data.windDirection);
        } else {
          Serial.println(ModbusRTUClient.lastError());
        }
      }
    } else {
      // handle error...
      Serial.println("monitorTask Failed!");
    }  
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
  }
}

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

  // Create the queues
  queueWindDataMqtt = xQueueCreate(10, sizeof(WindData));  
  queueWindDataMonitor = xQueueCreate(10, sizeof(WindData));
  
  // Check if the queues were created successfully
  if (queueWindDataMqtt == NULL || queueWindDataMonitor == NULL) {
    Serial.println("Error: Failed to create one or more queues.");

    // Stop execution
    while(1) {}
  }

  // Create tasks
  xTaskCreate(vTaskSensor, "Sensor data collection",                  256, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(vTaskDataHandler, "Data processing and transmission",   512, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(vTaskMonitor, "System monitoring",                      128, NULL, tskIDLE_PRIORITY + 1, NULL);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never get here
  while (1) {
	  Serial.println("Scheduler Failed!");
	  Serial.flush();
	  delay(1000);
  }
}

void loop() {
  // If execution reaches here, then there might be insufficient heap memory for creating the idle task
}
