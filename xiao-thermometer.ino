#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2901.h> 
#include <esp_sleep.h>
#include <esp_pm.h> 

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CURR_TEMP_UUID "4fafc202-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MAX_TEMP_UUID  "4fafc203-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MIN_TEMP_UUID  "4fafc204-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_RESET_UUID     "4fafc205-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_BATTERY_UUID   "4fafc206-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TERM_UUID      "4fafc207-1fb5-459e-8fcc-c5c9c331914b" 
#define CHAR_EXTEND_UUID    "4fafc208-1fb5-459e-8fcc-c5c9c331914b" 
#define CHAR_PAUSE_UUID     "4fafc209-1fb5-459e-8fcc-c5c9c331914b" 

#define REED_PIN  1  
#define VBAT_PIN  0  
#define GATE_PIN  22 
#define POWER_PIN 23 
#define DATA_PIN  21 

OneWire oneWire(DATA_PIN);
DallasTemperature sensors(&oneWire);

float currentTemp = 0.0;
float maxTemp = -999.0;
float minTemp = 999.0;
float batteryVoltage = 0.0;

BLECharacteristic *pCurrChar;
BLECharacteristic *pMaxChar;
BLECharacteristic *pMinChar;
BLECharacteristic *pBatChar;

bool resetRequested = false;
bool terminateSession = false; 
bool extendRequested = false; 
bool deviceConnected = false; 
bool initialHandshakeCompleted = false; 

// Non-Blocking Sensor State Machine variables
enum TempState { IDLE_STATE, REQUESTED_STATE };
TempState currentTempState = IDLE_STATE;
unsigned long tempRequestTimestamp = 0;

bool timerSuspended = false; 
unsigned long sessionStartTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastBatterySampleTime = 0; 
unsigned long lastSuspendUpdate = 0; 
const unsigned long SAMPLE_INTERVAL = 10000; 
const unsigned long BATTERY_INTERVAL = 30000; 
unsigned long sessionDuration = 180000; 
class DeviceServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    pServer->updateConnParams(pServer->getConnId(), 80, 160, 1, 600);
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    initialHandshakeCompleted = false;
    BLEDevice::startAdvertising(); 
  }
};

class ControlCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String uuid = pCharacteristic->getUUID().toString().c_str();
    String value = pCharacteristic->getValue();
    
    if (value.length() > 0) {
      if ((value == "1" || value.charAt(0) == 0x01)) { 
        if (uuid == CHAR_RESET_UUID) resetRequested = true;
        else if (uuid == CHAR_TERM_UUID) terminateSession = true;
        else if (uuid == CHAR_EXTEND_UUID) extendRequested = true;
        else if (uuid == CHAR_PAUSE_UUID) timerSuspended = true; 
      }
      else if ((value == "0" || value.charAt(0) == 0x00)) {
        if (uuid == CHAR_PAUSE_UUID) timerSuspended = false; 
      }
    }
  }
};

void addLabel(BLECharacteristic* pChar, const char* labelText) {
  BLE2901* pDescriptor = new BLE2901();
  pDescriptor->setDescription(labelText);
  pChar->addDescriptor(pDescriptor);
}

void resetTracking() {
  maxTemp = -999.0; minTemp = 999.0; resetRequested = false;
  
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  delay(10);
  
  sensors.requestTemperatures();
  delay(750); // Kept strictly here for manual triggers where blocking is acceptable
  
  float newTemp = sensors.getTempCByIndex(0);
  if (newTemp != DEVICE_DISCONNECTED_C && newTemp > -50.0) {
    currentTemp = newTemp; maxTemp = currentTemp; minTemp = currentTemp;
    pCurrChar->setValue(String(currentTemp, 2).c_str());
    pMaxChar->setValue(String(maxTemp, 2).c_str()); 
    pMinChar->setValue(String(minTemp, 2).c_str()); 
    if (deviceConnected) { pCurrChar->notify(); pMaxChar->notify(); pMinChar->notify(); }
  }
}

void enterUltraLowPowerSleep() {
  BLEDevice::getAdvertising()->stop();
  unsigned long clearWait = millis();
  while(millis() - clearWait < 600) { delay(10); }
  
  digitalWrite(GATE_PIN, LOW); 
  digitalWrite(POWER_PIN, LOW);
  pinMode(DATA_PIN, INPUT); 
  pinMode(VBAT_PIN, INPUT); 
  esp_deep_sleep_enable_gpio_wakeup(1ULL << GPIO_NUM_1, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

float readBatteryPhysical() {
  digitalWrite(GATE_PIN, HIGH); 
  delay(5); 
  
  uint32_t adcTotal = 0;
  for(int i = 0; i < 3; i++) {
    adcTotal += analogReadMilliVolts(VBAT_PIN);
    delay(2);
  }
  
  digitalWrite(GATE_PIN, LOW); 
  return ((adcTotal / 3.0) / 1000.0) * 2.0;
}
void setup() {
  const uint8_t unusedPins[] = {2, 19, 20, 9, 8, 3};
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(unusedPins[i], INPUT_PULLUP);
  }

  pinMode(REED_PIN, INPUT_PULLUP);
  delay(50); 
  
  pinMode(GATE_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(GATE_PIN, LOW); 
  digitalWrite(POWER_PIN, HIGH);
  delay(100); 
  
  sensors.begin();
  sensors.setWaitForConversion(false); // CRITICAL: Stop sensor from blocking the CPU execution thread
  analogReadResolution(12);
  batteryVoltage = readBatteryPhysical();
  
  BLEDevice::init("XIAO_C6_Thermometer");
  BLEServer *pServer = BLEDevice::createServer(); 
  pServer->setCallbacks(new DeviceServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCurrChar = pService->createCharacteristic(CHAR_CURR_TEMP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pMaxChar = pService->createCharacteristic(CHAR_MAX_TEMP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pMinChar = pService->createCharacteristic(CHAR_MIN_TEMP_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBatChar = pService->createCharacteristic(CHAR_BATTERY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  
  BLECharacteristic *pResetChar = pService->createCharacteristic(CHAR_RESET_UUID, BLECharacteristic::PROPERTY_WRITE);
  BLECharacteristic *pTermChar = pService->createCharacteristic(CHAR_TERM_UUID, BLECharacteristic::PROPERTY_WRITE);
  BLECharacteristic *pExtendChar = pService->createCharacteristic(CHAR_EXTEND_UUID, BLECharacteristic::PROPERTY_WRITE); 
  BLECharacteristic *pPauseChar = pService->createCharacteristic(CHAR_PAUSE_UUID, BLECharacteristic::PROPERTY_WRITE); 
  
  ControlCallbacks* callbacks = new ControlCallbacks();
  pResetChar->setCallbacks(callbacks); pTermChar->setCallbacks(callbacks);
  pExtendChar->setCallbacks(callbacks); pPauseChar->setCallbacks(callbacks); 
  
  addLabel(pCurrChar, "Current Temperature (C)"); addLabel(pMaxChar, "Maximum Temperature (C)");
  addLabel(pMinChar, "Minimum Temperature (C)"); addLabel(pBatChar, "Battery Voltage (V)");
  addLabel(pResetChar, "Write 1 to Reset Metrics"); addLabel(pTermChar, "Write 1 to Sleep Device");
  addLabel(pExtendChar, "Write 1 to Add 2 Mins to Session"); addLabel(pPauseChar, "Write 1 to Pause Timer, 0 to Resume");
  
  // Set accurate baseline on startup before running BLE loops
  digitalWrite(POWER_PIN, HIGH);
  delay(10);
  sensors.requestTemperatures();
  delay(750); // Visual alignment buffer acceptable only inside setup()
  float initTemp = sensors.getTempCByIndex(0);
  if (initTemp != DEVICE_DISCONNECTED_C && initTemp > -50.0) {
    currentTemp = initTemp; maxTemp = initTemp; minTemp = initTemp;
    pCurrChar->setValue(String(currentTemp, 2).c_str());
    pMaxChar->setValue(String(maxTemp, 2).c_str());
    pMinChar->setValue(String(minTemp, 2).c_str());
  }

  pBatChar->setValue(String(batteryVoltage, 2).c_str());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  sessionStartTime = millis();
  lastSampleTime = millis();
  lastBatterySampleTime = millis(); 
  lastSuspendUpdate = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long connectionTime = 0;
  static bool pmConfigured = false;

  if (deviceConnected && connectionTime == 0) {
    connectionTime = currentMillis;
  }

  // Complete initial handshake 5 seconds after connection
  if (deviceConnected && !initialHandshakeCompleted && (currentMillis - connectionTime >= 5000)) {
    initialHandshakeCompleted = true; 
  }

  // Engage power management safely only after initial discovery is 100% complete
  if (deviceConnected && initialHandshakeCompleted && !pmConfigured) {
    pmConfigured = true;
    esp_pm_config_t pm_config = { .max_freq_mhz = 160, .min_freq_mhz = 40, .light_sleep_enable = true };
    esp_pm_configure(&pm_config);
  }
  
  if (!deviceConnected) {
    connectionTime = 0;
    initialHandshakeCompleted = false;
    if (pmConfigured) {
      pmConfigured = false;
      esp_pm_config_t pm_disable = { .max_freq_mhz = 160, .min_freq_mhz = 160, .light_sleep_enable = false };
      esp_pm_configure(&pm_disable);
    }
  }

  if (digitalRead(REED_PIN) == LOW) {
    delay(50); 
    if (digitalRead(REED_PIN) == LOW) enterUltraLowPowerSleep();
  }
  if (terminateSession) enterUltraLowPowerSleep();
  if (extendRequested) { sessionDuration += 120000; extendRequested = false; }
  
  if (timerSuspended) {
    unsigned long timeElapsed = currentMillis - lastSuspendUpdate;
    sessionStartTime += timeElapsed; 
  }
  lastSuspendUpdate = currentMillis; 

  if (currentMillis - sessionStartTime >= sessionDuration) enterUltraLowPowerSleep();
  if (resetRequested) resetTracking();

  // 1. Non-Blocking Temperature Processing State Machine
  if (currentTempState == IDLE_STATE && (currentMillis - lastSampleTime >= SAMPLE_INTERVAL)) {
    lastSampleTime = currentMillis;
    pinMode(DATA_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    delay(5);
    
    sensors.requestTemperatures(); // Starts conversion immediately without waiting
    tempRequestTimestamp = currentMillis;
    currentTempState = REQUESTED_STATE;
  }

  if (currentTempState == REQUESTED_STATE && (currentMillis - tempRequestTimestamp >= 750)) {
    float newTemp = sensors.getTempCByIndex(0);
    currentTempState = IDLE_STATE; // Reset state machine structure
    
    if (newTemp != DEVICE_DISCONNECTED_C && newTemp > -50.0) {
      if (newTemp != currentTemp) {
        currentTemp = newTemp;
        pCurrChar->setValue(String(currentTemp, 2).c_str());
        if (deviceConnected) pCurrChar->notify();
      }
      if (currentTemp > maxTemp) {
        maxTemp = currentTemp;
        pMaxChar->setValue(String(maxTemp, 2).c_str());
        if (deviceConnected) pMaxChar->notify();
      }
      if (currentTemp < minTemp || minTemp == 999.0) {
        minTemp = currentTemp;
        pMinChar->setValue(String(minTemp, 2).c_str());
        if (deviceConnected) pMinChar->notify();
      }
    }
  }

  // 2. Battery Processing Loop (Runs every 30 seconds)
  if (currentMillis - lastBatterySampleTime >= BATTERY_INTERVAL) {
    lastBatterySampleTime = currentMillis;
    batteryVoltage = readBatteryPhysical();
    pBatChar->setValue(String(batteryVoltage, 2).c_str());
    if (deviceConnected) pBatChar->notify(); 
  }

  delay(15); 
}
