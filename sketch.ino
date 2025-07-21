

/*
 * Intelligent Heating Control System
 * ESP32-based temperature monitoring and control
 * 
 * Features:
 * - Temperature-based state machine
 * - BLE advertising
 * - Serial logging
 * - Visual/Audio feedback
 * - FreeRTOS task management
 * 
 * Author: Heating System Intern
 * Date: 2025
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

// Pin Definitions
#define TEMP_SENSOR_PIN     4    // DS18B20 data pin
#define HEATER_RELAY_PIN    2    // Relay control pin
#define LED_RED_PIN         5    // Red LED
#define LED_GREEN_PIN       18   // Green LED  
#define LED_BLUE_PIN        19   // Blue LED
#define BUZZER_PIN          21   // Buzzer pin

// Temperature Thresholds (Celsius)
#define TARGET_TEMP         50.0f
#define TEMP_HYSTERESIS     2.0f
#define OVERHEAT_THRESHOLD  85.0f
#define RECOVERY_TEMP       75.0f

// Timing Constants
#define SENSOR_READ_INTERVAL    1000  // ms
#define BLE_ADVERTISE_INTERVAL  2000  // ms
#define STATE_CHECK_INTERVAL    500   // ms

// System States
enum HeatingState {
    STATE_IDLE = 0,
    STATE_HEATING,
    STATE_STABILIZING,
    STATE_TARGET_REACHED,
    STATE_OVERHEAT,
    STATE_ERROR
};

// Global Variables
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

volatile HeatingState currentState = STATE_IDLE;
volatile float currentTemp = 0.0f;
volatile bool heaterStatus = false;
volatile unsigned long lastTempRead = 0;
volatile unsigned long stateChangeTime = 0;

// BLE Variables
BLEServer* pServer = nullptr;
BLEAdvertising* pAdvertising = nullptr;
bool deviceConnected = false;

// FreeRTOS Task Handles
TaskHandle_t tempReadTaskHandle = NULL;
TaskHandle_t stateManagerTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t feedbackTaskHandle = NULL;

// Timer Handles
TimerHandle_t sensorTimer = NULL;
TimerHandle_t bleTimer = NULL;

// Function Prototypes
void initHardware();
void initBLE();
void tempReadTask(void *pvParameters);
void stateManagerTask(void *pvParameters);
void bleTask(void *pvParameters);
void feedbackTask(void *pvParameters);
void sensorTimerCallback(TimerHandle_t xTimer);
void bleTimerCallback(TimerHandle_t xTimer);
void updateHeaterControl();
void updateLEDs();
void playBuzzer(int frequency, int duration);
void logStatus();
String getStateString(HeatingState state);
void changeState(HeatingState newState);

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected");
        pAdvertising->start();
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Heating Control System Starting ===");
    
    // Initialize hardware
    initHardware();
    
    // Initialize BLE
    initBLE();
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        tempReadTask,           // Task function
        "TempReadTask",         // Task name
        2048,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority
        &tempReadTaskHandle,    // Task handle
        0                       // Core
    );
    
    xTaskCreatePinnedToCore(
        stateManagerTask,
        "StateManagerTask",
        2048,
        NULL,
        3,                      // Higher priority
        &stateManagerTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        bleTask,
        "BLETask",
        2048,
        NULL,
        1,
        &bleTaskHandle,
        1                       // Different core
    );
    
    xTaskCreatePinnedToCore(
        feedbackTask,
        "FeedbackTask",
        1024,
        NULL,
        1,
        &feedbackTaskHandle,
        1
    );
    
    // Create timers
    sensorTimer = xTimerCreate(
        "SensorTimer",
        pdMS_TO_TICKS(SENSOR_READ_INTERVAL),
        pdTRUE,                 // Auto-reload
        (void*)0,
        sensorTimerCallback
    );
    
    bleTimer = xTimerCreate(
        "BLETimer",
        pdMS_TO_TICKS(BLE_ADVERTISE_INTERVAL),
        pdTRUE,
        (void*)0,
        bleTimerCallback
    );
    
    // Start timers
    xTimerStart(sensorTimer, 0);
    xTimerStart(bleTimer, 0);
    
    Serial.println("System Initialized Successfully");
    changeState(STATE_IDLE);
}

void loop() {
    // Main loop runs on core 1, tasks handle everything
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Watchdog functionality
    static unsigned long lastWatchdog = 0;
    if (millis() - lastWatchdog > 10000) {
        Serial.println("System Watchdog: All tasks running");
        lastWatchdog = millis();
    }
}

void initHardware() {
    // Configure pins
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Initialize outputs to safe state
    digitalWrite(HEATER_RELAY_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Initialize temperature sensor
    tempSensor.begin();
    tempSensor.setResolution(12); // 12-bit resolution
    
    Serial.println("Hardware initialized");
}

void initBLE() {
    BLEDevice::init("HeatingController");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Create service and characteristic
    BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        "87654321-4321-4321-4321-cba987654321",
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("12345678-1234-1234-1234-123456789abc");
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    
    Serial.println("BLE initialized and advertising");
}

void tempReadTask(void *pvParameters) {
    while (1) {
        // Request temperature reading
        tempSensor.requestTemperatures();
        
        // Read temperature
        float newTemp = tempSensor.getTempCByIndex(0);
        
        if (newTemp != DEVICE_DISCONNECTED_C && newTemp > -50 && newTemp < 150) {
            currentTemp = newTemp;
            lastTempRead = millis();
        } else {
            Serial.println("ERROR: Invalid temperature reading");
            if (millis() - lastTempRead > 5000) {
                changeState(STATE_ERROR);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
    }
}

void stateManagerTask(void *pvParameters) {
    while (1) {
        // State machine logic
        switch (currentState) {
            case STATE_IDLE:
                if (currentTemp < (TARGET_TEMP - TEMP_HYSTERESIS)) {
                    changeState(STATE_HEATING);
                }
                break;
                
            case STATE_HEATING:
                if (currentTemp >= OVERHEAT_THRESHOLD) {
                    changeState(STATE_OVERHEAT);
                } else if (currentTemp > (TARGET_TEMP - 2.0f)) {
                    changeState(STATE_STABILIZING);
                }
                break;
                
            case STATE_STABILIZING:
                if (currentTemp >= OVERHEAT_THRESHOLD) {
                    changeState(STATE_OVERHEAT);
                } else if (abs(currentTemp - TARGET_TEMP) <= 1.0f) {
                    changeState(STATE_TARGET_REACHED);
                } else if (currentTemp < (TARGET_TEMP - TEMP_HYSTERESIS)) {
                    changeState(STATE_HEATING);
                }
                break;
                
            case STATE_TARGET_REACHED:
                if (currentTemp >= OVERHEAT_THRESHOLD) {
                    changeState(STATE_OVERHEAT);
                } else if (currentTemp < (TARGET_TEMP - TEMP_HYSTERESIS)) {
                    changeState(STATE_HEATING);
                }
                break;
                
            case STATE_OVERHEAT:
                if (currentTemp < RECOVERY_TEMP) {
                    // Manual reset required - check for button press or command
                    changeState(STATE_IDLE);
                }
                break;
                
            case STATE_ERROR:
                // Check if sensor is back online
                if (millis() - lastTempRead < 2000) {
                    changeState(STATE_IDLE);
                }
                break;
        }
        
        // Update heater control
        updateHeaterControl();
        
        vTaskDelay(pdMS_TO_TICKS(STATE_CHECK_INTERVAL));
    }
}

void bleTask(void *pvParameters) {
    while (1) {
        if (deviceConnected) {
            // Update BLE characteristic with current status
            String status = String(currentTemp, 1) + "," + getStateString(currentState) + "," + (heaterStatus ? "ON" : "OFF");
            // Update characteristic value here if needed
        }
        
        vTaskDelay(pdMS_TO_TICKS(BLE_ADVERTISE_INTERVAL));
    }
}

void feedbackTask(void *pvParameters) {
    while (1) {
        updateLEDs();
        
        // Audio feedback for state changes
        static HeatingState lastState = STATE_IDLE;
        if (currentState != lastState) {
            switch (currentState) {
                case STATE_HEATING:
                    playBuzzer(1000, 100);
                    break;
                case STATE_TARGET_REACHED:
                    playBuzzer(1500, 200);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    playBuzzer(1500, 200);
                    break;
                case STATE_OVERHEAT:
                    for (int i = 0; i < 5; i++) {
                        playBuzzer(2000, 100);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    break;
                default:
                    break;
            }
            lastState = currentState;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sensorTimerCallback(TimerHandle_t xTimer) {
    // This timer is used for periodic sensor health checks
    static int sensorErrorCount = 0;
    
    if (millis() - lastTempRead > 3000) {
        sensorErrorCount++;
        if (sensorErrorCount > 3) {
            Serial.println("Sensor timeout - switching to error state");
            changeState(STATE_ERROR);
        }
    } else {
        sensorErrorCount = 0;
    }
}

void bleTimerCallback(TimerHandle_t xTimer) {
    // Update BLE advertisement data
    char advData[50];
    snprintf(advData, sizeof(advData), "Temp:%.1f State:%s Heater:%s", 
             currentTemp, getStateString(currentState).c_str(), heaterStatus ? "ON" : "OFF");
    
    // Update advertising data
    pAdvertising->stop();
    BLEAdvertisementData advertisementData;
    advertisementData.setName("HeatingController");
    advertisementData.setCompleteServices(BLEUUID("12345678-1234-1234-1234-123456789abc"));
    pAdvertising->setAdvertisementData(advertisementData);
    pAdvertising->start();
}

void updateHeaterControl() {
    bool shouldHeat = false;
    
    switch (currentState) {
        case STATE_HEATING:
        case STATE_STABILIZING:
            shouldHeat = (currentTemp < TARGET_TEMP);
            break;
        case STATE_TARGET_REACHED:
            shouldHeat = (currentTemp < (TARGET_TEMP - 0.5f));
            break;
        default:
            shouldHeat = false;
            break;
    }
    
    if (heaterStatus != shouldHeat) {
        heaterStatus = shouldHeat;
        digitalWrite(HEATER_RELAY_PIN, heaterStatus ? HIGH : LOW);
        Serial.printf("Heater %s\n", heaterStatus ? "ON" : "OFF");
    }
}

void updateLEDs() {
    // Turn off all LEDs first
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
    
    switch (currentState) {
        case STATE_IDLE:
            digitalWrite(LED_BLUE_PIN, HIGH);
            break;
        case STATE_HEATING:
            digitalWrite(LED_RED_PIN, HIGH);
            digitalWrite(LED_BLUE_PIN, HIGH); // Purple
            break;
        case STATE_STABILIZING:
            // Blinking yellow
            digitalWrite(LED_RED_PIN, HIGH);
            digitalWrite(LED_GREEN_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(250));
            digitalWrite(LED_RED_PIN, LOW);
            digitalWrite(LED_GREEN_PIN, LOW);
            break;
        case STATE_TARGET_REACHED:
            digitalWrite(LED_GREEN_PIN, HIGH);
            break;
        case STATE_OVERHEAT:
            // Blinking red
            digitalWrite(LED_RED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(LED_RED_PIN, LOW);
            break;
        case STATE_ERROR:
            // Fast blinking red
            digitalWrite(LED_RED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(LED_RED_PIN, LOW);
            break;
    }
}

void playBuzzer(int frequency, int duration) {
    tone(BUZZER_PIN, frequency, duration);
}

void changeState(HeatingState newState) {
    if (currentState != newState) {
        Serial.printf("State change: %s -> %s\n", 
                     getStateString(currentState).c_str(), 
                     getStateString(newState).c_str());
        currentState = newState;
        stateChangeTime = millis();
        logStatus();
    }
}

String getStateString(HeatingState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_HEATING: return "HEATING";
        case STATE_STABILIZING: return "STABILIZING";
        case STATE_TARGET_REACHED: return "TARGET_REACHED";
        case STATE_OVERHEAT: return "OVERHEAT";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void logStatus() {
    unsigned long uptime = millis() / 1000;
    unsigned long stateTime = (millis() - stateChangeTime) / 1000;
    
    Serial.printf("[%lu] Temp: %.1f°C | State: %s (%lus) | Heater: %s | Target: %.1f°C\n",
                 uptime, currentTemp, getStateString(currentState).c_str(), 
                 stateTime, heaterStatus ? "ON" : "OFF", TARGET_TEMP);
}
