/**
  This program is designed to read wind speed and wind direction data
  from an Ecowitt WS90 wind sensor via Modbus RTU 485 communication.
  The read data is published via MQTT.
  The data is stored on an SD card with a timestamp when the internet is unavailable.
  Once the internet is back, the stored data is then published again via the broker.
  
  The program uses the ArduinoModbus library and ArduinoRS485 library to communicate with the wind sensor,
  the MKRNB library and ArduinoMqttClient library to publish the data via MQTT, and SD library and RTCZero library 
  to save the data with timestamp on the SD card, and the wdt_samd21 library to enable and manage the Watchdog Timer functionality.

  The implementation has been enhanced with the use of the FreeRTOS_SAMD21 library, enabling multitasking capabilities
  to improve the overall performance and reliability of the program. Tasks in the program include reading from the sensor,
  handling data, and monitoring tasks, each running independently on separate cores.

  The watchdog task, implemented using the wdt_samd21 library, continuously monitors the execution of the other tasks.
  If there are no signals within a specified period of time, it initiates a system reset to prevent failures.

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
#include <wdt_samd21.h>
#include <ArduinoJson.h>
#include "SIM_Card_Secrets.h"

// Modbus configuration
#define SLAVE_ID 0x090 // Device Address of the Ecowitt WS90 Wind Sensor
#define WIND_SPEED_REGISTER 0x0169 // Register address of the wind speed
#define WIND_DIRECTION_REGISTER 0x016B // Register address of the wind direction
#define TIME_OUT -1

// Initialize library instance
NBClient client;
GPRS gprs;
NB nbAccess;
MqttClient mqttClient(client);
RTCZero rtc;

// If your SIM card has a PIN number, please enter it in the "SIM_Card_Secrets.h"
const char PINNUMBER[] = SECRET_PINNUMBER;

// MQTT configuration
const char broker[] = "test.mosquitto.org";
const uint16_t port = 1883;
const uint8_t QoS = 2;
const char topic_windData[] = "Sensor_KN/Winddata";
const char topic_errorMessage[] = "Sensor_KN/Error";

// Timestamp configuration
long timeOffset = 3600 * 2; // Timezone offset in seconds (here: Summertime GMT+2)

// Connection counter configuration
const int BASE_FAILURES = 10;  // Base value for failed attempts
const int MAX_FAILURES = 800;  // Maximum number of failed attempts before reconnecting
static int failureCount = 0; // Counter for consecutive connection errors
int maxFailures = BASE_FAILURES; // Maximum number of connection errors
const int BACKOFF_FACTOR = 2;

// Define a structure to hold wind data
struct WindData {
  uint windSpeedRaw;
  uint windDirection;
  float windSpeed;
};

// Define a structure to hold wind heartbeat
struct Heartbeat {
  bool sensor;
  bool dataHandler;
};

QueueHandle_t queueWindDataMqtt; // Handle to the queue storing data destined for MQTT communication
QueueHandle_t queueHeartbeatSensor; // Handle to the queue storing vTaskSensor heartbeat destined for system monitoring
QueueHandle_t queueHeartbeatDataHandler; // Handle to the queue storing vTaskDataHandler heartbeat destined for system monitoring

//*****************************************************************************
//
// Function to read stored wind data from the SD card and publish it via MQTT.
//
//*****************************************************************************
void readAndSendDataFromSDCard() {
  if (SD.exists("winddata.txt")) {
    // now disable WDT
    wdt_disable();

    File dataFile = SD.open("winddata.txt");
    if (dataFile) {
      String dataString;
      Serial.print("Sending data... ");
      while (dataFile.available()) {
        dataString = dataFile.readStringUntil('\n');
        mqttClient.beginMessage(topic_windData, false, QoS, false);
        mqttClient.print(dataString);
        mqttClient.endMessage();
      }
      dataFile.close();
      SD.remove("winddata.txt");
      Serial.println("Finish");

      // now enable WDT again
      wdt_reEnable();
    } else {
      Serial.println("Error opening winddata.txt");
    }
  }
}

//*****************************************************************************
//
// Function to save wind speed and wind direction with the current timestamp on the SD card.
//
//*****************************************************************************
void saveDataToSDCard(String dataString) {
  File dataFile = SD.open("winddata.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    Serial.print(dataString);
    Serial.println(" ...saved");
    dataFile.close();
  } else {
    Serial.println("Error opening winddata.txt");
  }
}

//*****************************************************************************
//
// Function to reset the counters related to connection failures.
//
//*****************************************************************************
void resetFailureCounters() {
  failureCount = 0; // Reset the failed attempt counter since there is a successful connection
  maxFailures = BASE_FAILURES; // Reset maxFailures to the base value
}

//*****************************************************************************
//
// Function to check the connectivity status of the cellular network (NB IoT).
//
//*****************************************************************************
bool isConnectedToCellular() {
  // Returns true if connected, false otherwise.
  return (nbAccess.status() == NB_READY);
}

//*****************************************************************************
//
// Function to handle reconnection attempts to the MQTT broker.
//
//*****************************************************************************
void handleMQTTReconnection() {
  failureCount++;
  Serial.println("Failed to connect to MQTT broker");
  if (failureCount >= maxFailures) {
    if (mqttClient.connect(broker, port)) {
      Serial.println("Reconnected to MQTT broker");
      resetFailureCounters();
    } else {
      Serial.println("Reconnection attempt failed");
      maxFailures *= BACKOFF_FACTOR; // Increase the number of failures before next reconnection attempt
      if (maxFailures > MAX_FAILURES) {
        maxFailures = MAX_FAILURES; // Cap it to a maximum value
      }
    }
  }
}

//*****************************************************************************
//
// Function to manage reconnection attempts to the cellular network (NB IoT).
//
//*****************************************************************************
void handleCellularReconnection() {
  failureCount++;
  Serial.println("Failed to connect to cellular network (NB IoT)");
  if (failureCount >= maxFailures) {
    if (nbAccess.begin(PINNUMBER) == NB_READY) {
      Serial.println("Reconnected to cellular network (NB IoT)");
      resetFailureCounters();
    } else {
      Serial.println("Reconnection attempt failed");
      maxFailures *= BACKOFF_FACTOR; // Increase the number of failures before next reconnection attempt
      if (maxFailures > MAX_FAILURES) {
        maxFailures = MAX_FAILURES; // Cap it to a maximum value
      }
    }
  }
}

//*****************************************************************************
//
// Thread to handle sensor data collection and distribution.
//
//*****************************************************************************
static void vTaskSensor(void *pvParameters) {
  TaskStatus_t xTaskDetails; // structure to hold the task's details

  while (1) {
    WindData data;
    Heartbeat sensorHeartbeat;
    // Read wind speed register and wind direction register
    data.windSpeedRaw = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_SPEED_REGISTER);
    data.windDirection = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_DIRECTION_REGISTER);
    data.windSpeed = data.windSpeedRaw * 0.1; // Convert raw wind speed to actual wind speed
    sensorHeartbeat.sensor = true;
    sensorHeartbeat.dataHandler = false;

    // Send data to the queues, block for max. 100 ms if the queue is full
    if (xQueueSend(queueWindDataMqtt, &data, pdMS_TO_TICKS(100)) != pdPASS) {
      // handle error...
      Serial.println("Failed to send data to MQTT queue!");
      // Retry sending data to the queue
      vTaskDelay(pdMS_TO_TICKS(200));  // delay for 200 ms before retry
      xQueueSend(queueWindDataMqtt, &data, portMAX_DELAY);  // try sending again with indefinite blocking
    }
    if (xQueueSend(queueHeartbeatSensor, &sensorHeartbeat, pdMS_TO_TICKS(100)) != pdPASS) {
      // handle error...
      Serial.println("Failed to send heartbeat from Sensor task!");
    }

    // Check the task's stack
    vTaskGetInfo(NULL, &xTaskDetails, pdTRUE, eInvalid);
    Serial.print("vTaskSensor free stack space: ");
    Serial.println(xTaskDetails.usStackHighWaterMark);

    vTaskDelay(pdMS_TO_TICKS(3000));  // delay for 3000 ms
  }
}

//*****************************************************************************
//
// Thread to handle the processing of wind data:
//
// 1. If connected to the MQTT broker, validates the wind data and sends the formatted data to the broker.
// 2. If not connected to the MQTT broker, saves wind data to SD card.
// 3. If data exists on the SD card (i.e., when MQTT connection was previously unavailable), sends this data to the MQTT broker.
//
//*****************************************************************************
static void vTaskDataHandler(void *pvParameters) {
  TaskStatus_t xTaskDetails; // structure to hold the task's details

  while (1) {
    // Current date
    byte year = rtc.getYear();
    byte month = rtc.getMonth();
    byte day = rtc.getDay();

    // Current time
    byte hours = rtc.getHours();
    byte minutes = rtc.getMinutes();
    byte seconds = rtc.getSeconds();

    String timestamp = String(year) + "-" + String(month) + "-" + String(day) + ", " + String(hours) + ":" + String(minutes) + ":" + String(seconds);
    WindData data;
    Heartbeat dataHandlerHeartbeat;
    String dataString;
    DynamicJsonDocument doc(128);

    // Create object "Received"
    JsonObject received = doc.createNestedObject("Received");

    if (xQueueReceive(queueWindDataMqtt, &data, pdMS_TO_TICKS(100)) == pdPASS) {
      dataHandlerHeartbeat.sensor = false;
      dataHandlerHeartbeat.dataHandler = true;

      if (data.windSpeedRaw != TIME_OUT && data.windDirection != TIME_OUT) {
        // Add values to the "Received" object
        received["windSpeed"] = String(data.windSpeed, 1);
        received["windDirection"] = String(data.windDirection);
        received["timestamp"] = timestamp;
        serializeJson(doc, dataString); // format in JSON

        // Monitor data
        Serial.println("******************************");
        Serial.print("Wind Speed: ");
        Serial.println(data.windSpeed, 1);
        Serial.print("Wind Direction: ");
        Serial.println(data.windDirection);
        Serial.print("Timestamp: ");
        Serial.println(timestamp);
        Serial.println("******************************");

        // Check if the connection to the cellular network (NB IoT) is ready
        if (!isConnectedToCellular()) {
          // The connection to the cellular network is not ready
          // Save wind data to SD card
          saveDataToSDCard(dataString);

          // Connection with cellular network failed, handle the error
          handleCellularReconnection();

          // Check if the MQTT client is successfully connected to the broker
        } else if (!mqttClient.connected()) {
          // The MQTT client is not connected to the broker
          // Save wind data to SD card
          saveDataToSDCard(dataString);

          // Connection with MQTT broker failed, handle the error
          handleMQTTReconnection();

          // Both connections are successful
        } else {
          // Handle the main logic here
          // Publish wind data via MQTT
          mqttClient.beginMessage(topic_windData, false, QoS, false);
          mqttClient.print(dataString);
          mqttClient.endMessage();

          // Read and send data from SD Card
          readAndSendDataFromSDCard();

          // Reset the failed attempt counter
          resetFailureCounters();
        }
      } else {
        Serial.println(ModbusRTUClient.lastError());

        // Check if the connection to the cellular network (NB IoT) is ready
        if (!isConnectedToCellular()) {
          // Connection with cellular network failed, handle the error
          handleCellularReconnection();

          // Check if the MQTT client is successfully connected to the broker
        } else if (!mqttClient.connected()) {
          // Connection with MQTT broker failed, handle the error
          handleMQTTReconnection();

          // Both connections are successful
        } else {
          // Publish error message via MQTT
          mqttClient.beginMessage(topic_errorMessage);
          mqttClient.print(ModbusRTUClient.lastError());
          mqttClient.endMessage();
        }
      }
    } else {
      // handle error...
      Serial.println("Failed to receive data from the MQTT queue!");
      // clear the queue
      xQueueReset(queueWindDataMqtt);
    }

    if (xQueueSend(queueHeartbeatDataHandler, &dataHandlerHeartbeat, pdMS_TO_TICKS(100)) != pdPASS) {
      // handle error...
      Serial.println("Failed to send heartbeat from DataHandler task!");
    }

    // Check the task's stack
    vTaskGetInfo(NULL, &xTaskDetails, pdTRUE, eInvalid);
    Serial.print("vTaskDataHandler free stack space: ");
    Serial.println(xTaskDetails.usStackHighWaterMark);

    vTaskDelay(pdMS_TO_TICKS(3000));  // delay for 3000 ms
  }  
}

//*****************************************************************************
//
// Thread to monitor task execution.
//
//*****************************************************************************
static void vTaskWatchdog(void *pvParameters) {
  TaskStatus_t xTaskDetails; // structure to hold the task's details

  while (1) {
    bool receivedSensorHeartbeat = false;
    bool receivedDataHandlerHeartbeat = false;
    Heartbeat sensorHeartbeat;
    Heartbeat dataHandlerHeartbeat;

    if (xQueueReceive(queueHeartbeatSensor, &sensorHeartbeat, pdMS_TO_TICKS(100)) == pdPASS && sensorHeartbeat.sensor == true) {
      // Sign of life received from vTaskSensor
      receivedSensorHeartbeat = true;
    } else {
      Serial.println("No heartbeat received from Sensor task!");
    }

    if (xQueueReceive(queueHeartbeatDataHandler, &dataHandlerHeartbeat, pdMS_TO_TICKS(100)) == pdPASS && dataHandlerHeartbeat.dataHandler == true) {
      // Sign of life received from vTaskDataHandler
      receivedDataHandlerHeartbeat = true;
    } else {
      Serial.println("No heartbeat received from DataHandler task!");
    }

    if (receivedSensorHeartbeat && receivedDataHandlerHeartbeat) {
      Serial.println("Received heartbeats from both tasks");
      wdt_reset();
      receivedSensorHeartbeat = false;
      receivedDataHandlerHeartbeat = false;
    }

    // Check the task's stack
    vTaskGetInfo(NULL, &xTaskDetails, pdTRUE, eInvalid);
    Serial.print("vTaskWatchdog free stack space: ");
    Serial.println(xTaskDetails.usStackHighWaterMark);

    vTaskDelay(pdMS_TO_TICKS(3000)); // delay for 3000 ms
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial){
    // Wait for serial port to connect. Needed for native USB port only
  }
  for(int i = 0; i < 30; i++) {
  Serial.println();
  }

  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");

  // Connect to NB IoT
  Serial.print("Initializing NB IoT... ");
  if (nbAccess.begin(PINNUMBER) != NB_READY) {
    Serial.println("Failed to start NB IoT!");
    while (1);
  }
  Serial.println("NB IoT initialized.");

  // Initialize SD card
  Serial.print("Initializing SD card... ");
  if (!SD.begin()) {
    Serial.println("SD card failed, or not present");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Initialize Modbus RTU Client
  Serial.print("Initializing Modbus RTU Client... ");
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    // Baud rate of the Modbus device (9600 bps), 8 Data bits, No Parity, 1 Stop
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
  Serial.println("Modbus RTU Client initialized.");

  // Connect to MQTT
  Serial.print("Connecting to MQTT broker... ");
  if (!mqttClient.connect(broker, port)) {
    Serial.print("Failed to connect to MQTT broker! Erroe code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker.");

  // Initialize RTC
  Serial.print("Initializing time... ");
  rtc.begin();
  rtc.setEpoch(nbAccess.getLocalTime() + timeOffset); // Get current epoch time from the cellular module
  Serial.println("Time initialized.");

  // Current date and time
  byte year = rtc.getYear();
  byte month = rtc.getMonth();
  byte day = rtc.getDay();
  byte hours = rtc.getHours();
  byte minutes = rtc.getMinutes();
  byte seconds = rtc.getSeconds();

  Serial.print("Current time -> ");
  Serial.print(year);
  Serial.print("-");
  Serial.print(month);
  Serial.print("-");
  Serial.print(day);
  Serial.print(", ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(seconds);

  // Initialize watchdog with 16 seconds (timeout)
  Serial.print("Initializing Watchdog timer (WDT)... ");
  wdt_init(WDT_CONFIG_PER_16K);
  Serial.println("Watchdog enabled for 16 seconds.");

  // Create the queues
  queueWindDataMqtt = xQueueCreate(10, sizeof(WindData));
  queueHeartbeatSensor = xQueueCreate(5, sizeof(Heartbeat));
  queueHeartbeatDataHandler = xQueueCreate(5, sizeof(Heartbeat));
  
  // Check if the queues were created successfully
  if (queueWindDataMqtt == NULL || queueHeartbeatSensor == NULL || queueHeartbeatDataHandler == NULL) {
    Serial.println("Error: Failed to create one or more queues.");

    // Stop execution
    while (1);
  }

  // Create tasks
  xTaskCreate(vTaskSensor, "Sensor data collection",                  256, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(vTaskDataHandler, "Data processing and transmission",   650, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(vTaskWatchdog, "Watchdog timer",                        128, NULL, tskIDLE_PRIORITY + 1, NULL);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never get here
  while (1) {
    // If execution reaches here, then there might be insufficient heap memory for creating the idle task
    Serial.println("Scheduler Failed!");
    delay(1000);
  }
}

void loop() {}