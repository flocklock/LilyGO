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
unsigned long lastFotaCheck = 1;
unsigned long gnssTimeout = 0;
Accelerometer accel = Accelerometer(12345);
float battery_voltage = 0;
String deviceID = "xxxxx";
String versionStr = "1";
#define gnssInterval 120
#define accInterval 10
#define  activitySize 2 * gnssInterval / accInterval

int activityPointer = 0;
float lastActivity[activitySize];
int activityCounter[ACTIVITY::COUNT];
float totalActivity = 0;


void setup()
{
  battery_voltage = readBattery();
  lowBatteryCheck(battery_voltage);
  
  deviceID = fota.getDeviceID();
  deviceID = deviceID.substring(deviceID.length() - 5, deviceID.length());

  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  D(SerialMon.begin(115200);)
  D(delay(10);)
  
  D(SerialMon.print("Current version: ");) 
  D(SerialMon.println(boardCurrentVersion);)
  D(SerialMon.print("Device id: ");) 
  D(SerialMon.println(deviceID);)

  D(SerialMon.println(String("battery: ") + String(readBattery()));)
 if(accel.begin()) {D(SerialMon.println("acc sensor found");)}
  
  setupGSM();
  delay(3000);
  setupFOTAGSM();
  SerialMon.println("modem set");
  delay(1000);
  


 
  mqtt.setServer(broker, mqttPort);
  mqtt.setKeepAlive(7200);

 if(modem.testAT())
    SerialMon.println("modem ok");
  else
    SerialMon.println("modem not ok");
  modem.sendAT("+CNETLIGHT=0");
  D(modem.sendAT("+CMEE=2");)

  delay(100);
  digitalWrite(PIN_DTR, LOW);
  digitalWrite(LED_PIN, HIGH);

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
}

void loop()
{
  for (int i = 0; i < 32; i++) {
    e[i] = accel.readAccData(ADXL345_REG_DATAX0);
  }
  if(activityPointer >= activitySize)
    activityPointer = 0;
  lastActivity[activityPointer] = stdDev(e).stdDevX;
  activityPointer++;
  activityCounter[evaluate(e)]++;

  battery_voltage = readBattery();
  lowBatteryCheck(battery_voltage);

  if( !lastFotaCheck || millis() - lastFotaCheck > 3600 * mS_TO_S_FACTOR) {
    D(SerialMon.println("Checking server");)
    digitalWrite(PIN_DTR, LOW);
    delay(1000);
    bool updatedNeeded = fota.execHTTPcheck();
    modem.sendAT("+CSCLK=1");
  if (updatedNeeded)
  {
    D(SerialMon.println("Got new update");)
    fota.execOTA();
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
    enableGPS();

    
    for(int i = 0; i < activityPointer; i++) {
      totalActivity += lastActivity[i];
    }
    
    float lat = 0,  lon = 0;
    for(int i = 0; i < 80; i++) {
      if (modem.getGPS(&lat, &lon)) {
        D(Serial.println("The location has been locked");)
        D(Serial.print("latitude:"); Serial.println(lat);)
        D(Serial.print("longitude:"); Serial.println(lon);)
        break;
      }
      D(Serial.println("No fix");)
      delay(1000);
    }
    String evaluated;
    for(int i = 0; i < ACTIVITY::COUNT; i++) {
      evaluated += String(activityCounter[i]) + '-';
      activityCounter[i] = 0;
    }
    String message = String(name) + ',' + versionStr + "," + deviceID
     + "," + String(millis()) + "," + String(lat,5) + "," + String(lon,5) 
     + "," + String(battery_voltage,0) + "," + String(totalActivity / (activityPointer+1), 3)
     + "," + evaluated;
    D(SerialMon.println(message);)
    if(mqtt.loop()) {
      D(SerialMon.println("mqtt loop");)
      mqtt.publish(topic, message.c_str());
      D(SerialMon.println("mqtt published");)
    } else {
      mqttConnect(deviceID);
      D(SerialMon.println("mqtt reconnected");)
      mqtt.publish(topic, message.c_str());
    }
    //disableGPS();
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(1000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
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
