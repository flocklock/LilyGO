#include <Arduino.h>
#include <acc.hpp>
#include <utils.hpp>

int deviceId = 1;
bool update = false;
bool readAccFlag = false;
bool readGnssFlag = true;
hw_timer_t *accTimer = NULL;
hw_timer_t *gnssTimer = NULL;
unsigned long lastGnssCheck = 0;
unsigned long lastFotaCheck = 1;


//#include <location.hpp>

Accelerometer accel = Accelerometer(12345);
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
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);
  delay(10000); 
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

  /*
  setupGSM();
  delay(3000);
  setupFOTAGSM();
 SerialMon.println("modem set");
 */
 
 
digitalWrite(PIN_DTR, HIGH);
delay(500);
digitalWrite(PIN_DTR, LOW);
delay(500);
 SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
 modem.init();
 if(modem.testAT())
    SerialMon.println("modem ok");
  else
    SerialMon.println("modem not ok");
  //modem.sendAT("+CNETLIGHT=0");
  delay(100);
  digitalWrite(PIN_DTR, LOW);
  digitalWrite(LED_PIN, HIGH);
  //esp_sleep_enable_timer_wakeup(20 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  SerialAT.flush();
  //esp_light_sleep_start();
  digitalWrite(LED_PIN, LOW);


  SerialMon.begin(115200);
  //D(Serial.begin(115200);)
  if(accel.begin()) {D(SerialMon.println("acc sensor found");)}

      


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
  /*
  while (true) {
    if (SerialAT.available()) {
      Serial.write(SerialAT.read());
    }
    if (Serial.available()) {
      SerialAT.write(Serial.read());
    }
    delay(1);
  }
  Serial.println("Failed");
  setup();
  
  digitalWrite(PIN_DTR, LOW);
  delay(10000);
  SerialMon.println("loop");
  
  D(SerialMon.println(String("acc reading"));)
    for (int i = 0; i < 32; i++) {
    e[i] = accel.readAccData(ADXL345_REG_DATAX0);
    }
    D(SerialMon.println(String("time: ") + String(millis()) );)
    for(int i = 0; i < 32; i++) {
      D(SerialMon.println(String("acc data: ") + String(e[i].X, 2));)
    }
//if(false) {
  if(!lastGnssCheck || millis() - lastGnssCheck > 20000) {
    digitalWrite(PIN_DTR, LOW);
    if (modem.testAT()) {
        D(Serial.println("Modem ok");)
        modem.sendAT("+CSCLK=0");
        if (modem.waitResponse() != 1) D(Serial.println("cant slow clock in gps function");)

        delay(10);
        modem.sendAT("+SGPIO=0,4,1,1");
        if (modem.waitResponse() != 1) D(Serial.println("cant set gpio high");)
        delay(1000);
        //modem.sendAT(GF("+CGNSPWR=1"));
    //if (modem.waitResponse() != 1) D(Serial.println("cant power on");)

        float lat,  lon;
        int i = 0;
        while (1) {
          //modem.sendAT(GF("+CGNSINF"));
    
          //String res = stream.readStringUntil('\n');
          modem.waitResponse();
          //res.trim();
          if (modem.getGPS(&lat, &lon)) {
            D(Serial.println("The location has been locked, the latitude and longitude are:");)
            D(Serial.print("latitude:"); Serial.println(lat);)
            D(Serial.print("longitude:"); Serial.println(lon);)
            break;
        }
        D(Serial.println("No fix");)
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
        i++;
        if(i >= 5)
          break;
      }
      modem.sendAT("+SGPIO=0,4,1,0");
        if (modem.waitResponse() != 1) D(Serial.println("cant set gpio low");)
        delay(1000);
        //modem.sendAT(GF("+CGNSPWR=0"));
    //if (modem.waitResponse() != 1) D(Serial.println("cant power off");)
      delay(100);
      digitalWrite(PIN_DTR, HIGH);
    }
    lastGnssCheck = millis();
    digitalWrite(PIN_DTR, HIGH);
    }
  
  if(!lastFotaCheck || millis() - lastFotaCheck > 3600000) {
    digitalWrite(PIN_DTR, LOW);
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
  modem.sendAT("+CSCLK=1");
  delay(10);
  digitalWrite(PIN_DTR, HIGH);
  digitalWrite(LED_PIN, LOW);
  esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();
  digitalWrite(LED_PIN, HIGH);
  delay(10);
*/


digitalWrite(PIN_DTR, LOW);
modem.sendAT("+SGPIO=0,4,1,1");
delay(1000);

modem.sendAT("+CGNSPWR=1");
delay(1000);

for(int i = 0; i < 5; i++) {
  modem.sendAT("+CGNSINF");
  blink(50, 200);
  delay(1000);
}

digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(4 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();

modem.sendAT("+CGNSPWR=0");
blink(1000, 3000);

modem.sendAT("+SGPIO=0,4,1,0");
blink(500, 3000);

modem.sendAT("+CSCLK=1"); 
digitalWrite(PIN_DTR, HIGH);
digitalWrite(LED_PIN, HIGH);
delay(2000);

digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(2 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();


  digitalWrite(PIN_DTR, LOW);
SerialMon.println("starting loop");
blink(100, 7000);

modem.sendAT("+CSCLK=1"); 
digitalWrite(PIN_DTR, HIGH);
digitalWrite(LED_PIN, HIGH);
delay(12000);

digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();
/*
digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();

digitalWrite(PIN_DTR, LOW);
SerialMon.println("starting loop");
blink(100, 7000);

modem.sendAT("+CSCLK=1");
digitalWrite(PIN_DTR, HIGH);
digitalWrite(LED_PIN, HIGH);
delay(12000);

digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();

modem.sendAT("+CSCLK=1");
blink(1000, 14000);

digitalWrite(LED_PIN, LOW);
digitalWrite(PIN_DTR, HIGH);
esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
  D(SerialMon.flush();)
  esp_light_sleep_start();

modem.sendAT("+CSCLK=1");

delay(100);
digitalWrite(PIN_DTR, HIGH);
D(SerialMon.flush();)
digitalWrite(LED_PIN, HIGH);
delay(12000);

digitalWrite(LED_PIN, LOW);
esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
D(SerialMon.flush();)
esp_light_sleep_start();
*/
}