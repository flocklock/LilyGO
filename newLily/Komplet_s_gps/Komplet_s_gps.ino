
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#define TINY_GSM_MODEM_SIM7000
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#define BLYNK_TEMPLATE_ID "TMPLLLCphTNF"
#define BLYNK_DEVICE_NAME "Obojek"
//#define BLYNK_AUTH_TOKEN "-_EYozVWqPndSjPL90boPIQm0SiVqVOG"
#define BLYNK_AUTH_TOKEN "KdBLJR2utmFgW0PB1qEXrnw1b-oNwjDg"
#define BLYNK_HEARTBEAT 600

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

#include <BlynkSimpleTinyGSM.h>
#include <TinyGsmClient.h>

#include <StreamDebugger.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1

#define uS_TO_S_FACTOR      1000000ULL
#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4
#define LED_PIN             12

#define LED_PIN             12
#define butBlue             32
#define butYellow           33
#define butGreen            34
#define butRed              35
#define butWhite            2

#include "blynk.h"

// Your GPRS credentials, if any
const char apn[]  = "lpwa.vodafone.com";
const char user[] = "";
const char pass[] = "";

#ifdef DUMP_AT_COMMANDS
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

BlynkTimer timer;
bool reply = false;
//WidgetMap myMap(V10);

uint8_t dtr = LOW;
uint8_t led = HIGH;
bool blynk_status = false;
int csclk = 0;

int readBut(int but) {
  if(digitalRead(but)) {
   delay(25);
   if(digitalRead(but)) {
    delay(200);
    return true;
   }
   
  }
  return 0;
}


void enableGPS()
{
    // Set SIM7000G GPIO4 LOW ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    if(modem.enableGPS())
      SerialMon.println("GPS enabled");
}
void disableGPS()
{
    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
    if(modem.disableGPS())
      SerialMon.println("GPS disabled");

}
void sendAcc()
{
  sensors_event_t event; 
  accel.getEvent(&event);
  //float acc = [[1658575323000, (fabs(event.acceleration.x) + fabs(event.acceleration.y), + fabs(event.acceleration.z))]];
  //Blynk.virtualWrite(V4, [[1658575323000, (fabs(event.acceleration.x) + fabs(event.acceleration.y), + fabs(event.acceleration.z))]]);
  Blynk.virtualWrite(V4, fabs(event.acceleration.x) + fabs(event.acceleration.y) + fabs(event.acceleration.z));
  //SerialMon.print("\nAcc: ");
  //SerialMon.print("\nAcc: ");
  //SerialMon.print(acc);
}


void sendLocation() {
  //myMap.clear();
  
  //enableGPS();
  float lat = 50;
  float lon = 20;
  for(int i = 0; i < 10; i++) {
    SerialMon.println("Getting GPS data");
    if (modem.getGPS(&lat, &lon)) {
      SerialMon.println("sending GPS data to a map");
      //myMap.location(1, lat, lon, "value");
      Blynk.virtualWrite(V10, double(lon), double(lat));
      Blynk.virtualWrite(V7, double(lon));
      Blynk.virtualWrite(V8, double(lat));
      SerialMon.print("Lon: ");
      SerialMon.println(lon);
      SerialMon.print(double(lon));

      SerialMon.print("\nLat: ");
      SerialMon.print(lat);

      break;
    }
    delay(500);
    
  }
  if(lat == 50) {
  Blynk.virtualWrite(V10, double(60.1), double(20.2));
      Blynk.virtualWrite(V7, double(20.2));
      Blynk.virtualWrite(V8, double(12.1));
  }
  //disableGPS();
}

void modemSleep() {
  /*
  modem.sendAT("+CSCLK?");
  delay(300);
  if (SerialAT.available()) {
      String r = SerialAT.readString();
      SerialMon.println(r);
  }
  */
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  modem.sendAT("+CSCLK=1");
  digitalWrite(PIN_DTR, HIGH);
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
  esp_light_sleep_start();
  SerialMon.println("waking up\n");
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(PIN_DTR, LOW);
  delay(1000);
    
  sendAcc();
  sendLocation();
  delay(200);
}

void setup()
{
  pinMode(PIN_DTR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
  digitalWrite(LED_PIN, HIGH);
  
    
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(500);
  SerialMon.println("\nStarting Up Modem...");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);
  delay(10000);                 //Wait for the SIM7000 communication to be normal, and will quit when receiving OK

  int i = 10;
  SerialMon.println("\nTesting Modem Response...\n");
  SerialMon.println("****");
  while (i) {
    SerialAT.println("AT");
    delay(500);
    if (SerialAT.available()) {
      String r = SerialAT.readString();
      SerialMon.println(r);
      if ( r.indexOf("OK") >= 0 ) {
        reply = true;
        break;;
      }
    }
    delay(500);
    i--;
  }
  SerialMon.println("****\n");

  if (reply) {

    bool ret = modem.factoryDefault();

    SerialMon.println(F("\n***********************************************************"));
    SerialMon.print  (F(" Reset settings to Factory Default: "));
    SerialMon.println((ret) ? "OK" : "FAIL");
    SerialMon.println(F("***********************************************************"));
  } else {
    SerialMon.println(F("***********************************************************"));
    SerialMon.println(F(" Failed to connect to the modem! Check the baud and try again."));
    SerialMon.println(F("***********************************************************\n"));
  }

  SerialMon.println("Initializing modem...");
  modem.init();


  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  Blynk.begin(BLYNK_AUTH_TOKEN, modem, apn, user, pass);
  Wire.begin();
  SerialMon.println("Accelerometer Test"); Serial.println("");
  if(!accel.begin())
  {
    SerialMon.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  modem.sendAT("+CSCLK=1");
  delay(1000);
  displaySensorDetails();

  //timer.setInterval(1000L, sendAcc);
  //timer.setInterval(60000L, sendLocation);
  timer.setInterval(15000L, modemSleep);

}

void loop()
{
  Blynk.run();
  timer.run();
  
}
