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

#include <BlynkSimpleTinyGSM.h>

#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <driver/rtc_io.h>

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

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12


uint8_t dtr = HIGH;
uint8_t led = HIGH;


void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
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

void setup()
{
  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);
    // Set console baud rate
    SerialMon.begin(115200);

    delay(10);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    modemPowerOn();

    Serial.println("========SDCard Detect.======");
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);
    if (!SD.begin(SD_CS)) {
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
    }
    Serial.println("===========================");

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);


    Serial.println("/**********************************************************/");
    Serial.println("To initialize the network test, please make sure your LET ");
    Serial.println("antenna has been connected to the SIM interface on the board.");
    Serial.println("/**********************************************************/\n\n");

    delay(5000);

    //Blynk.begin(auth, modem, apn, "", "");

    //timer.setInterval(10000L, myTimerEvent);

}

void loop()
{
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

    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
    }


    for (int i = 0; i <= 4; i++) {
        uint8_t network[] = {
            // 2,  /*Automatic*/
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

    Serial.println("ATB");
    modem.sendAT("B");
    if (modem.waitResponse(1000L, res) == 1) {
        res.replace(GSM_NL "OK" GSM_NL, "");
        Serial.println(res);
    }

    

    Serial.println("/**********************************************************/");
    Serial.println("After the network test is complete, please enter the  ");
    Serial.println("AT command in the serial terminal.");
    Serial.println("/**********************************************************/\n\n");

    /*
    Serial.println("Putting the module to sleep");
    modem.sendAT("+CSCLK=1");
    if (modem.waitResponse(1000L, res) == 1) {
        Serial.println(res);
    }
    Serial.println("here");
    pinMode(PIN_DTR, OUTPUT);
    digitalWrite(PIN_DTR, HIGH);
    delay(5000);
    Serial.println("sleeping");
    modem.sendAT("");
    if (modem.waitResponse(1000L, res) == 1) {
        Serial.println(res);
    }
    delay(5000);
    digitalWrite(PIN_DTR, LOW);
    delay(1000);
    Serial.println("waking up");
    if (modem.waitResponse(1000L, res) == 1) {
        Serial.println(res);
    }
    Serial.println("here2");
    modem.sendAT("");
    if (modem.waitResponse(1000L, res) == 1) {
        Serial.println(res);
    }
    Serial.println("here3");
    */

  //Blynk.run();
  //timer.run();
    gpio_num_t pin = (gpio_num_t) 12;  
    while (1) {
      String str;
        while (SerialAT.available()) {
          SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
          str = SerialMon.readString();
          SerialMon.println(str);
          
          if (!strcmp(str.c_str(), "DTR\r\n")) {
              SerialMon.println("sent DTR");
              dtr = !dtr;
              digitalWrite(PIN_DTR, dtr);
          } else if (!strcmp(str.c_str(), "LED\r\n")) {
              SerialMon.println("sent LED");
              led = !led;
              //digitalWrite(LED_PIN, led);
              gpio_pullup_en(GPIO_NUM_12);
              if (gpio_set_level(GPIO_NUM_12, 1) == ESP_OK)
                SerialMon.println("good");
              delay(1000);
          } else if (!strcmp(str.c_str(), "sleep\r\n")) {
              SerialMon.println("sent sleep");

              
              
              rtc_gpio_set_direction_in_sleep(pin, RTC_GPIO_MODE_OUTPUT_ONLY);
              rtc_gpio_pullup_en(pin);
              delay(1000);
              
              esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, 
                    ESP_PD_OPTION_ON);
              gpio_deep_sleep_hold_en();   
              esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
              gpio_sleep_set_pull_mode(pin, GPIO_PULLUP_PULLDOWN); 
              esp_deep_sleep_start();
          }
          else {
            SerialAT.print(str);
          }
        }
       }
        
    
}
