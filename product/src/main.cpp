#include <Arduino.h>
#include <acc.hpp>
#include <utils.hpp>
#include <location.hpp>
#include <mqtt.hpp>
#include <fotaHeaders.h>

int deviceId = 1;
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
String deviceID = "xxx";
String versionStr = "2";
int accCounter = 0;


int sleep_period = 10;



void setup()
{
  if(readBattery() > 0.1 && readBattery() < 2.7) {
    modemPowerOff();
    delay(10);
    esp_sleep_enable_timer_wakeup(43200 * uS_TO_S_FACTOR); // sleep half a day
    D(SerialMon.flush();)
    esp_deep_sleep_start();
  }
  

  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  D(SerialMon.begin(115200);)
  D(delay(10);)
  
  D(SerialMon.print("Current version: ");) 
  D(SerialMon.println(boardCurrentVersion);)
  D(SerialMon.print("Device id: ");) 
  D(SerialMon.println(deviceId);)

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


  SerialMon.begin(115200);
  //D(Serial.begin(115200);)
 
  delay(1000);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00000000);
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_12_5_HZ);
  accel.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0b00000000);

  deviceID = fota.getDeviceID();
  delay(1000);
  setCpuFrequencyMhz(80);
  delay(1000);
}

void loop()
{
  for (int i = 0; i < 32; i++) {
    e[i] = accel.readAccData(ADXL345_REG_DATAX0);
    accCounter++;
  }

  battery_voltage = readBattery();
  if(battery_voltage > 0.1 && battery_voltage < 2.7) {
    esp_sleep_enable_timer_wakeup(43200 * uS_TO_S_FACTOR); // sleep half a day
    D(SerialMon.flush();)
    esp_deep_sleep_start();
  }

  if( !lastFotaCheck || millis() - lastFotaCheck > 3600 * mS_TO_S_FACTOR) {
    D(SerialMon.println("Checking server");)
    digitalWrite(PIN_DTR, LOW);
    delay(1000);
    bool updatedNeeded = fota.execHTTPcheck();
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

  if(!lastGnssCheck || millis() - lastGnssCheck > 1200 * mS_TO_S_FACTOR) {
    digitalWrite(PIN_DTR, LOW);
    delay(10);
    enableGPS();

    float lat = 0,  lon = 0;
    for(int i = 0; i < 80; i++) {
      if (modem.getGPS(&lat, &lon)) {
        D(Serial.println("The location has been locked, the latitude and longitude are:");)
        D(Serial.print("latitude:"); Serial.println(lat);)
        D(Serial.print("longitude:"); Serial.println(lon);)
        break;
      }
      D(Serial.println("No fix");)
      delay(1000);
    }
    String message = "luk," + versionStr + "," 
    + deviceID.substring(deviceID.length() - 5, deviceID.length())
     + "," + String(millis()) + "," + String(lat,5) + "," + String(lon,5) 
     + "," + String(battery_voltage,0) + "," + String(accCounter);
    D(SerialMon.println(message);)
    if(mqtt.loop()) {
      D(SerialMon.println("mqtt loop");)
      mqtt.publish(topicLukas, message.c_str());
      D(SerialMon.println("mqtt published");)
    } else {
      mqttConnect();
      D(SerialMon.println("mqtt reconnected");)
      mqtt.publish(topicLukas, message.c_str());
    }
    disableGPS();
    digitalWrite(PIN_DTR, HIGH);
    lastGnssCheck = millis();
    }
    

  modem.sendAT("+CSCLK=1");
  delay(10);
  digitalWrite(PIN_DTR, HIGH);
  esp_sleep_enable_timer_wakeup(1200 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();
}