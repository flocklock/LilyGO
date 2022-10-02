// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "lpwa.vodafone.com";     //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#include <BlynkSimpleTinyGSM.h>

#include <TinyGsmClient.h>
#include <SPI.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

BlynkTimer timer;

#define uS_TO_S_FACTOR      1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP       60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4


#define LED_PIN             12
#define butBlue             32
#define butYellow           33
#define butGreen            34
#define butRed              35

#define BLYNK_TEMPLATE_ID "TMPLLLCphTNF"
#define BLYNK_DEVICE_NAME "Obojek"
#define BLYNK_AUTH_TOKEN "KdBLJR2utmFgW0PB1qEXrnw1b-oNwjDg"
const char auth[] = BLYNK_AUTH_TOKEN;

uint8_t dtr = LOW;
uint8_t led = HIGH;
bool blynk_status = false;

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
void enableGPS(void)
{
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,1 false ");
    }
}

void disableGPS(void)
{
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
}

void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1000);    //Datasheet Ton mintues = 1S
    digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1500);    //Datasheet Ton mintues = 1.2S
    digitalWrite(PWR_PIN, HIGH);
}


void modemRestart()
{
    modemPowerOff();
    delay(1000);
    modemPowerOn();
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
void setup()
{
  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
    // Set console baud rate
    SerialMon.begin(115200);
    pinMode(butBlue, INPUT);
    pinMode(butYellow, INPUT);
    pinMode(butGreen, INPUT);
    pinMode(butRed, INPUT);
    delay(10);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, led);

    modemPowerOn();

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);


    Serial.println("/**********************************************************/");
    Serial.println("To initialize the network test, please make sure your LET ");
    Serial.println("antenna has been connected to the SIM interface on the board.");
    Serial.println("/**********************************************************/\n\n");

    delay(10000);
    String res;

    Serial.println("========INIT========");

    if (!modem.init()) {
        modemRestart();
        delay(2000);
        Serial.println("Failed to restart modem, attempting to continue without restarting");
        return;
    }

    Serial.println("========SIMCOMATI======");
    modem.sendAT("+SIMCOMATI");
    modem.waitResponse(1000L, res);
    res.replace(GSM_NL "OK" GSM_NL, "");
    Serial.println(res);
    res = "";
    Serial.println("=======================");

    Serial.println("=====Preferred mode selection=====");
    modem.sendAT("+CNMP?");
    if (modem.waitResponse(1000L, res) == 1) {
        res.replace(GSM_NL "OK" GSM_NL, "");
        Serial.println(res);
    }
    res = "";
    Serial.println("=======================");


    Serial.println("=====Preferred selection between CAT-M and NB-IoT=====");
    modem.sendAT("+CMNB?");
    if (modem.waitResponse(1000L, res) == 1) {
        res.replace(GSM_NL "OK" GSM_NL, "");
        Serial.println(res);
    }
    res = "";
    Serial.println("=======================");


    String name = modem.getModemName();
    Serial.println("Modem Name: " + name);

    String modemInfo = modem.getModemInfo();
    Serial.println("Modem Info: " + modemInfo);

    for (int i = 0; i <= 4; i++) {
        uint8_t network[] = {
            2,  /*Automatic*/
            13, /*GSM only*/
            38, /*LTE only*/
            51  /*GSM and LTE only*/
        };
        Serial.printf("Try %d method\n", network[i]);
        modem.setNetworkMode(network[i]);
        delay(3000);
        bool isConnected = false;
        int tryCount = 60;
        while (tryCount--) {
            int16_t signal =  modem.getSignalQuality();
            Serial.print("Signal: ");
            Serial.print(signal);
            Serial.print(" ");
            Serial.print("isNetworkConnected: ");
            isConnected = modem.isNetworkConnected();
            Serial.println( isConnected ? "CONNECT" : "NO CONNECT");
            if (isConnected) {
                break;
            }
            delay(1000);
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        if (isConnected) {
            break;
        }
    }
    digitalWrite(LED_PIN, HIGH);

    Serial.println();
    Serial.println("Device is connected .");
    Serial.println();

    Serial.println("=====Inquiring UE system information=====");
    modem.sendAT("+CPSI?");
    if (modem.waitResponse(1000L, res) == 1) {
        res.replace(GSM_NL "OK" GSM_NL, "");
        Serial.println(res);
    }
    modem.sendAT("+CSCLK=0");
    //Blynk.begin(BLYNK_AUTH_TOKEN, modem, apn, "", "");
    //Blynk.config(modem, BLYNK_AUTH_TOKEN);
    Serial.println("/**********************************************************/");
    Serial.println("After the network test is complete, please enter the  ");
    Serial.println("AT command in the serial terminal.");
    Serial.println("/**********************************************************/\n\n");
}

void loop()
{

      
      String str;
        while (SerialAT.available()) {
          SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
          str = SerialMon.readString();
          SerialMon.println(str);
          
          if (!strcmp(str.c_str(), "dtr\r\n")) {
              
              dtr = !dtr;
              SerialMon.println((dtr) ? "sent DTR=true" : "sent DTR=false");
              if (dtr == HIGH)
                SerialMon.println("HHHHIIIIGGGGHHHH");
              digitalWrite(PIN_DTR, dtr);
          }
          else if(!strcmp(str.c_str(), "slp\r\n")) {
              SerialMon.println("sent SLEEP");
              modem.sendAT("+CSCLK=1");
              esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
              esp_light_sleep_start();
          }
          else if(!strcmp(str.c_str(), "rst\r\n")) {
            SerialMon.println("restarting");
            modem.factoryDefault();
          }
          else if(!strcmp(str.c_str(), "blynk\r\n")) {
            SerialMon.println("Blynk switch");
            blynk_status = !blynk_status;
            //Blynk.begin(auth, modem, apn, "", "");
            //Blynk.connect();
          }
          else {
            SerialAT.print(str);
          }
        }
        if (blynk_status) {
          Blynk.run();
        }
        if(readBut(butBlue)) {
          led = !led;
          digitalWrite(LED_PIN, led);
        }
        if(readBut(butYellow)) {
          led = !led;
          digitalWrite(LED_PIN, led);
          sendAcc();
        }
        if(readBut(butGreen)) {
          led = !led;
          digitalWrite(LED_PIN, led);
          dtr = !dtr;
          SerialMon.println((dtr) ? "sent DTR=true" : "sent DTR=false");
          if (dtr == HIGH)
            SerialMon.println("HHHHIIIIGGGGHHHH");
          digitalWrite(PIN_DTR, dtr);
        }
        if(readBut(butRed)) {
          led = !led;
          digitalWrite(LED_PIN, led);
          esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
          esp_light_sleep_start();
        }
      //Blynk.run();
        
   }
