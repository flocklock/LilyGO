//#define DEBUG
#include <Arduino.h>
#include <acc.hpp>
#include <utils.hpp>
#include <location.hpp>
#include <mqtt.hpp>
#include <fotaHeaders.h>

bool update = false;
bool readAccFlag = false;
bool readGnssFlag = true;
hw_timer_t *accTimer = NULL;
hw_timer_t *gnssTimer = NULL;
unsigned long lastGnssCheck = 0;
unsigned long lastFotaCheck = 0;
unsigned long gnssTimeout = 0;
Accelerometer accel = Accelerometer(12345);
float battery_voltage = 0;
String deviceID = "xxxxx";
String versionStr = "1";
#define gnssInterval 7200
#define accInterval 30
#define fotaInterval 72000
#define  activitySize 2 * gnssInterval / accInterval

int activityPointer = 0;
float lastActivity[activitySize];
int activityCounter[ACTIVITY::COUNT];
float totalActivity = 0;
int lastSuccesfulSend = 0;


void setup()
{
  uint64_t sleepDurationMicros = (uint64_t)24 * 60 * 60 * uS_TO_S_FACTOR;
  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  // Put the ESP32 into deep sleep for two days
  esp_sleep_enable_timer_wakeup(sleepDurationMicros);
  esp_light_sleep_start();
  lastSuccesfulSend = millis();

  esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); // Add the default loop task to the list of tasks watched by the WDT
  battery_voltage = readBattery();
  lastFotaCheck = 0;
  lowBatteryCheck(battery_voltage);
  
  deviceID = fota.getDeviceID();
  deviceID = deviceID.substring(deviceID.length() - 5, deviceID.length());

  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  D(SerialMon.begin(115200);)
  D(delay(100);)
  D(SerialMon.print("Current version: ");) 
  D(SerialMon.println(boardCurrentVersion);)
  D(SerialMon.print("Device id: ");) 
  D(SerialMon.println(deviceID);)
  D(SerialMon.flush();)
  delay(1000);

 if(accel.begin()) {D(SerialMon.println("acc sensor found");)}
  setupGSM();
  delay(3000);
  D(modem.sendAT("+CMEE=2");)
  setupFOTAGSM();
  D(SerialMon.println("modem set");)
  delay(1000);
  
 
  mqtt.setServer(broker, mqttPort);
  mqtt.setKeepAlive(7200);

 if(modem.testAT())
    {D(SerialMon.println("modem ok");)}

  else
  {
    D(SerialMon.println("modem not ok");)}
  
  

  delay(100);
  digitalWrite(PIN_DTR, LOW);
  digitalWrite(LED_PIN, HIGH);

  modem.sendAT("+CNETLIGHT=0");
  D(SerialMon.begin(115200);)
 
  delay(100);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00000000);
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_12_5_HZ);
  accel.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0b00000000);

  
  delay(1000);
  setCpuFrequencyMhz(80);
  delay(1000);
  disableGPS();
}

void loop()
{
  esp_task_wdt_reset();
  battery_voltage = readBattery();
  lowBatteryCheck(battery_voltage);
  
  for (int i = 0; i < 32; i++) {
    e[i] = accel.readAccData(ADXL345_REG_DATAX0);
  }
  if(activityPointer >= activitySize)
    activityPointer = 0;
  lastActivity[activityPointer] = stdDev(e).stdDevX;
  activityPointer++;
  activityCounter[evaluate(e)]++;

  

  if( !lastFotaCheck || millis() - lastFotaCheck > fotaInterval * mS_TO_S_FACTOR) {
    D(SerialMon.println("Checking server");)
    digitalWrite(PIN_DTR, LOW);
    delay(1000);
    bool updatedNeeded = fota.execHTTPcheck();
  if (updatedNeeded)
  {
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    esp_task_wdt_init(2000, true); // Enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); 
    D(SerialMon.println("Got new update");)
    modem.sendAT("+CSCLK=0");
    fota.execOTA();
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); // Add the default loop task to the list of tasks watched by the WDT
  }
  else
  {    
    D(SerialMon.println("Already up to date. No need to update");)
  }
    lastFotaCheck = millis();
  }

  if(!lastGnssCheck || millis() - lastGnssCheck > gnssInterval * mS_TO_S_FACTOR) {
    digitalWrite(PIN_DTR, LOW);
    delay(10);
    //enableGPS();

    
    for(int i = 0; i < activityPointer; i++) {
      totalActivity += lastActivity[i];
    }
    
    float lat = 0,  lon = 0;
    String evaluated;
    for(int i = 0; i < ACTIVITY::COUNT; i++) {
      evaluated += String(activityCounter[i]) + '-';
      activityCounter[i] = 0;
    }
    String message = String(name) + ',' + versionStr + "," + deviceID
     + "," + String(millis()) + "," + String(lat, 1) + "," + String(lon, 1) 
     + "," + String(battery_voltage, 2) + "," + String(totalActivity / (activityPointer+1), 3)
     + "," + evaluated;
    D(SerialMon.println(message);)
    delay(2000);
    if(mqtt.loop()) {
      D(SerialMon.println("mqtt loop");)
      if(mqtt.publish(topic, message.c_str())) {
        lastSuccesfulSend = millis();
      }
      D(SerialMon.println("mqtt published");)
    } else {
      mqttConnect(deviceID);
      D(SerialMon.println("mqtt reconnected");)
     if(mqtt.publish(topic, message.c_str())) {
        lastSuccesfulSend = millis();
      }
    }
    if (millis() - lastSuccesfulSend >  gnssInterval * 3 * mS_TO_S_FACTOR) {
      modemPowerOff();
      delay(1000);
      ESP.restart();
    }
    modem.sendAT("+CSCLK=1");
    activityPointer = 0;
    totalActivity = 0;
    lastGnssCheck = millis();
    delay(1000);
    }
    
  delay(1);
  digitalWrite(PIN_DTR, HIGH);
  esp_sleep_enable_timer_wakeup(accInterval * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();
}
