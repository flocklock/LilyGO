#include <Arduino.h>
#include <acc.hpp>
#include <utils.hpp>
#include <location.hpp>
#include <mqtt.hpp>

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
String devID;


int sleep_period = 10;
/*
void setup() {
  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  SerialMon.begin(115200);
  //D(Serial.begin(115200);)
  if(accel.begin()) {SerialMon.println("acc sensor found");}

  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00000000);
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_12_5_HZ);
  accel.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0b00000000);

  //attachInterrupt(ACC_INT1, dummyFNC(), HIGH);

    delay(10);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    modemPowerOn();

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("");
    Serial.println("To initialize the network test, please make sure your GPS");
    Serial.println("antenna has been connected to the GPS port on the board.");
    Serial.println("\n\n");

    delay(10000);
}

void loop() {
  if (!modem.testAT()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
        modemRestart();
        return;
    }

    Serial.println("Start positioning . Make sure to locate outdoors.");
    Serial.println("The blue indicator light flashes to indicate positioning.");

    enableGPS();

    float lat,  lon;
    while (1) {
        if (modem.getGPS(&lat, &lon)) {
            Serial.println("The location has been locked, the latitude and longitude are:");
            Serial.print("latitude:"); Serial.println(lat);
            Serial.print("longitude:"); Serial.println(lon);
            break;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(2000);
    }

    if (modem.testAT()) {
        Serial.println("Modem ok");
    }
    modem.sendAT("+CSCLK=1");
    digitalWrite(PIN_DTR, HIGH);
    delay(3000);
    digitalWrite(PIN_DTR, LOW);



    disableGPS();
    delay(1000);
    enableGPS();
    while (1) {
        if (modem.getGPS(&lat, &lon)) {
            Serial.println("The second location has been locked, the latitude and longitude are:");
            Serial.print("latitude:"); Serial.println(lat);
            Serial.print("longitude:"); Serial.println(lon);
            break;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(2000);
    }

    Serial.println("");
    Serial.println("After the network test is complete, please enter the  ");
    Serial.println("AT command in the serial terminal.");
    Serial.println("\n\n");

    while (1) {
        while (SerialAT.available()) {
            SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
            SerialAT.write(SerialMon.read());
        }
    }


  esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  esp_light_sleep_start();
  for (int i = 0; i < 31; i++) {
    e[i] = accel.readAccData(ADXL345_REG_DATAX0);
    D(Serial.println(String("reading data: ") + String(e[i].X, 2));)
  }
  
}
*/

#include <fotaHeaders.h>

void IRAM_ATTR onAccTimer() {
  readAccFlag = true;
}
void IRAM_ATTR onGnssTimer() {
  readGnssFlag = true;
}

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

  devID = iccid.substring(iccid.length() - 5, iccid.length());
 
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
/*
  accTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(accTimer, onAccTimer, true);
  timerAlarmWrite(accTimer, 10 * uS_TO_S_FACTOR, true);
  timerAlarmEnable(accTimer);

  gnssTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(gnssTimer, onGnssTimer, true);
  timerAlarmWrite(gnssTimer, 20 * uS_TO_S_FACTOR, true);
  timerAlarmEnable(gnssTimer);
*/
  delay(1000);
  setCpuFrequencyMhz(80);
  delay(1000);
}

void loop()
{

  if(!mqtt.loop()) {
    SerialMon.println("client not connected");
  } else {
    SerialMon.println("client loop");
    mqtt.publish(topicTest, "test");
  }
  
  
  battery_voltage = readBattery();
  if(battery_voltage > 0.1 && battery_voltage < 2.7) {
    esp_sleep_enable_timer_wakeup(43200 * uS_TO_S_FACTOR); // sleep half a day
    D(SerialMon.flush();)
    esp_deep_sleep_start();
  }

  if( !lastFotaCheck || millis() - lastFotaCheck > 600 * mS_TO_S_FACTOR) {
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

  if(!lastGnssCheck || millis() - lastGnssCheck > 30 * mS_TO_S_FACTOR) {
    digitalWrite(PIN_DTR, LOW);
        delay(10);
        enableGPS();

        float lat = 0,  lon = 0;
        for(int i = 0; i < 60; i++) {
          if (modem.getGPS(&lat, &lon)) {
            D(Serial.println("The location has been locked, the latitude and longitude are:");)
            D(Serial.print("latitude:"); Serial.println(lat);)
            D(Serial.print("longitude:"); Serial.println(lon);)
            break;
        }
        D(Serial.println("No fix");)
        delay(1000);
      }
      String message = devID + "," + String(millis()) + ",lat:" + String(lat) + ",lon:" + String(lon);
      if(mqtt.loop()) {
      mqtt.publish(topicGnss, message.c_str());
      } else {
        mqttConnect();
        mqtt.publish(topicGnss, message.c_str());
      }

      disableGPS();
      

    digitalWrite(PIN_DTR, HIGH);
    lastGnssCheck = millis();
    }

  modem.sendAT("+CSCLK=1");
  delay(10);
  digitalWrite(PIN_DTR, HIGH);
  esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();
  
}