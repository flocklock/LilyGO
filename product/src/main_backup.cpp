
/*
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
unsigned long lastFotaCheck = 0;


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
  
}

#include <fotaHeaders.h>

void IRAM_ATTR onAccTimer() {
  readAccFlag = true;
}
void IRAM_ATTR onGnssTimer() {
  readGnssFlag = true;
}

void setup()
{
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
  
  
  setupGSM();
  delay(3000);
  setupFOTAGSM();
  modem.sendAT("+CSCLK=1");
  modem.sendAT("+CNETLIGHT=0");

  SerialMon.begin(115200);
  //D(Serial.begin(115200);)
  if(accel.begin()) {D(SerialMon.println("acc sensor found");)}

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
  

  delay(1000);
  setCpuFrequencyMhz(80);
  delay(1000);
  enableGPS();
}
void blink(int interval, int duration){
  int time = millis();
  while(millis() - time < duration) {
  digitalWrite(LED_PIN, LOW);
  delay(interval);
  digitalWrite(LED_PIN, HIGH);
  delay(interval);
  }
}
void loop()
{
  disableGPS();
  digitalWrite(PIN_DTR, LOW);
  blink(2000, 10000);
  //modem.sendAT("+CSCLK=1");
  delay(10);
  //modem.sendAT("+CSCLK=?");
  delay(10);
  digitalWrite(PIN_DTR, HIGH);
  //modem.sendAT("+CSCLK=?");
  blink(1000, 10000);
  //digitalWrite(PIN_DTR, LOW);

  esp_sleep_enable_timer_wakeup(20 * uS_TO_S_FACTOR);
  digitalWrite(LED_PIN, LOW);
  esp_light_sleep_start();
  disableGPS();
/*
  esp_sleep_enable_timer_wakeup(20 * uS_TO_S_FACTOR);
  gpio_hold_en(GPIO_NUM_25);
  gpio_hold_en(GPIO_NUM_12);
  delay(1);
  D(SerialMon.flush();)
  esp_light_sleep_start();
  gpio_hold_dis(GPIO_NUM_25);
  gpio_hold_dis(GPIO_NUM_12);
  digitalWrite(LED_PIN, HIGH);
  delay(10);
  

}
*/