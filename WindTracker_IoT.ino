/**
  This program is designed to read wind speed and wind direction data
  from an Ecowitt WS90 wind sensor via Modbus RTU 485 communication.
  The read data is published via MQTT.
  The data is stored on an SD card with a timestamp when the internet is unavailable.
  Once the internet is back, the stored data is then published again via the broker.
  
  The program uses the ArduinoModbus library and ArduinoRS485 library to communicate with the wind sensor,
  the MKRNB library and ArduinoMqttClient library to publish the data via MQTT, and SD library, NTPClient library and RTCZero library 
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
#include <NTPClient.h>
#include <RTCZero.h>
#include <FreeRTOS_SAMD21.h>
#include <wdt_samd21.h>

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
NBUDP ntpUDP;

// MQTT configuration
const char broker[] = "test.mosquitto.org";
const uint16_t port = 1883;
const uint8_t QoS = 2;
const char topic_windData[] = "Sensor_KN/Winddata";
const char topic_errorMessage[] = "Sensor_KN/Error";

// NTP Server Konfiguration
long timeOffset = 3600 * 2; // Timezone offset in seconds (here: GMT+2)
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", timeOffset);

// timeClient initializes to 14:30:00 if it does not receive an NTP packet 
uint8_t HOUR = 14; // 24 hour format
uint8_t MINUTE = 30;
uint8_t SECOND = 0;

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
QueueHandle_t queueHeartbeatSensor; // Handle to the queue storing data destined for system monitoring
QueueHandle_t queueHeartbeatDataHandler;

//*****************************************************************************
//
// Thread to handle sensor data collection and distribution.
//
//*****************************************************************************
static void vTaskSensor(void *pvParameters) {
  while(1) {
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
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
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
  while(1) {
    // Current time
    byte hours = rtc.getHours();
    byte minutes = rtc.getMinutes();
    byte seconds = rtc.getSeconds();
    Serial.print("Current time -> ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.println(seconds);

    String timestamp = String(hours) + ":" + String(minutes) + ":" + String(seconds);
    WindData data;
    Heartbeat dataHandlerHeartbeat;

    if (xQueueReceive(queueWindDataMqtt, &data, pdMS_TO_TICKS(100)) == pdPASS) {
      dataHandlerHeartbeat.sensor = false;
      dataHandlerHeartbeat.dataHandler = true;

      if (data.windSpeedRaw != TIME_OUT && data.windDirection != TIME_OUT) {
        String dataString = timestamp + "," + String(data.windSpeed, 1) + "," + String(data.windDirection); // format in string

        // Monitor data
        Serial.print("Wind Speed: ");
        Serial.println(data.windSpeed, 1);
        Serial.print("Wind Direction: ");
        Serial.println(data.windDirection);

        if (mqttClient.connected()) {
          // Publish wind data via MQTT
          mqttClient.beginMessage(topic_windData, false, QoS, false);
          mqttClient.print(dataString);
          mqttClient.endMessage();

          // Read and send data from SD Card
          readAndSendDataFromSDCard();
        } else {
          // Try to reconnect to the MQTT broker
          int connectionResult = mqttClient.connect(broker, port);
          // Check if the reconnection was successful
          if (connectionResult == MQTT_SUCCESS) {
            // Connection is successful
            Serial.println("Reconnection is successful");
            mqttClient.beginMessage(topic_windData, false, QoS, false);
            mqttClient.print(dataString);
            mqttClient.endMessage();

            // Read and send data from SD Card
            readAndSendDataFromSDCard();
          } else {
            // Connection failed, handle the error
            Serial.print("Failed to connect to MQTT broker, error code: ");
            Serial.println(connectionResult);

            // Save wind data to SD card
            saveDataToSDCard(dataString);
          }
        }
      } else {
        Serial.println(ModbusRTUClient.lastError());
        if (mqttClient.connected()) {
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
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
  }  
}

//*****************************************************************************
//
// Thread to monitor task execution.
//
//*****************************************************************************
static void vTaskWatchdog(void *pvParameters) {
  while (1) {
    bool receivedSensorHeartbeat = false;
    bool receivedDataHandlerHeartbeat = false;
    Heartbeat sensorHeartbeat;
    Heartbeat dataHandlerHeartbeat;

    if (xQueueReceive(queueHeartbeatSensor, &sensorHeartbeat, pdMS_TO_TICKS(100)) == pdPASS && sensorHeartbeat.sensor == true) {
      // Sign of life received from vTaskSensor
      Serial.println("Received heartbeat from vTaskSensor");
      receivedSensorHeartbeat = true;
    } else {
      Serial.println("No heartbeat received from Sensor task!");
    }

    if (xQueueReceive(queueHeartbeatDataHandler, &dataHandlerHeartbeat, pdMS_TO_TICKS(100)) == pdPASS && dataHandlerHeartbeat.dataHandler == true) {
      // Sign of life received from vTaskDataHandler
      Serial.println("Received heartbeat from vTaskDataHandler");
      receivedDataHandlerHeartbeat = true;
    } else {
      Serial.println("No heartbeat received from DataHandler task!");
    }

    if (receivedSensorHeartbeat && receivedDataHandlerHeartbeat) {
      Serial.print("Received heartbeats from both tasks. ");
      wdt_reset();
      Serial.println("Reset WDT");
      receivedSensorHeartbeat = false;
      receivedDataHandlerHeartbeat = false;
    }
    vTaskDelay(pdMS_TO_TICKS(4500));  // delay for 4500 ms
  }
}

//*****************************************************************************
//
// Function to read stored wind data from the SD card and publish it via MQTT.
//
//*****************************************************************************
void readAndSendDataFromSDCard() {
  if (SD.exists("winddata.txt")) {
    // now disable WDT
    wdt_disable();
    Serial.println("WDT disabled...");

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

      // now enable WDT again
      wdt_reEnable();
      Serial.println("WDT reEnabled...");
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

void setup() {
  Serial.begin(9600);
  while(!Serial){
    // Baud rate of the Modbus device (9600 bps), 8 Data bits, No Parity, 1 Stop
    if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
      Serial.println("Failed to start Modbus RTU Client!");
      while (1);
    }
  }
  for(int i = 0; i < 30; i++) {
  Serial.println();
  }

  // Connect to NB IoT
  Serial.print("Initializing NB IoT... ");
  if (!nbAccess.begin()) {
    Serial.println("Failed to start NB IoT!");
    while (1);
  }
  Serial.println("NB IoT initialized.");

  // Initialize SD card
  Serial.print("Initializing SD card... ");
  if (!SD.begin()) {
    Serial.println("SD card failed, or not present");
    while (1);
  } else {
    Serial.println("SD card initialized.");
  }

  // Connect to MQTT
  Serial.print("Connecting to MQTT broker... ");
  if (!mqttClient.connect(broker, port)) {
    Serial.print("Failed to connect to MQTT broker! Erroe code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker");

  // Initialize time
  timeClient.begin();
  timeClient.forceUpdate();
  Serial.print("Initializing time... ");
  if (!timeClient.isTimeSet()) {
    Serial.print("Failed to receive NTP packet and set time! Retry... "); 
    delay(3000);
    timeClient.forceUpdate();
    if (!timeClient.isTimeSet()) {
      Serial.print("Failed to receive NTP packet and set time... ");
      Serial.println("Set time to 14:30:00 (default)");
    } else {
      HOUR = timeClient.getHours() + timeOffset;
      MINUTE = timeClient.getMinutes();
      SECOND = timeClient.getSeconds();
      Serial.println("Time initialized.");
    }
  } else {
    HOUR = timeClient.getHours() + timeOffset;
    MINUTE = timeClient.getMinutes();
    SECOND = timeClient.getSeconds();
    Serial.println("Time initialized.");
  }
  timeClient.end();

  // Initialize RTC
  rtc.begin();
  rtc.setHours(HOUR);
  rtc.setMinutes(MINUTE);
  rtc.setSeconds(SECOND);

  // Initialize watchdog with 8 seconds (timeout)
  Serial.print("Initializing Watchdog timer (WDT)... ");
  wdt_init (WDT_CONFIG_PER_8K);
  Serial.println("Watchdog enabled for: 8 s");

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
  xTaskCreate(vTaskDataHandler, "Data processing and transmission",   512, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(vTaskWatchdog, "Watchdog timer",                        256, NULL, tskIDLE_PRIORITY + 1, NULL);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never get here
  while (1) {
	  Serial.println("Scheduler Failed!");
	  delay(1000);
  }
}

void loop() {
  // If execution reaches here, then there might be insufficient heap memory for creating the idle task
}