/**************************************************************
 *
 * For this example, you need to install Blynk library:
 *   https://github.com/blynkkk/blynk-library/releases/latest
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 **************************************************************
 *
 * Blynk is a platform with iOS and Android apps to control
 * Arduino, Raspberry Pi and the likes over the Internet.
 * You can easily build graphic interfaces for all your
 * projects by simply dragging and dropping widgets.
 *
 * Blynk supports many development boards with WiFi, Ethernet,
 * GSM, Bluetooth, BLE, USB/Serial connection methods.
 * See more in Blynk library examples and community forum.
 *
 *                http://www.blynk.io/
 *
 * Change GPRS apm, user, pass, and Blynk auth token to run :)
 **************************************************************/

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#define BLYNK_TEMPLATE_ID "TMPLLLCphTNF"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "-_EYozVWqPndSjPL90boPIQm0SiVqVOG"

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
// #define BLYNK_HEARTBEAT 30

// Select your modem:
// #define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_SIM900
#define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_SIM7000SSL
// #define TINY_GSM_MODEM_SIM7080
// #define TINY_GSM_MODEM_SIM5360
// #define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_SARAR4
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE
// #define TINY_GSM_MODEM_SEQUANS_MONARCH

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Hardware Serial on Mega, Leonardo, Micro
#ifndef __AVR_ATmega328P__
#define SerialAT Serial1

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3);  // RX, TX
#endif


#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4


// Your GPRS credentials, if any
const char apn[]  = "lpwa.vodafone.com";
const char user[] = "";
const char pass[] = "";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).

TinyGsm modem(SerialAT);

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Blynk.begin(BLYNK_AUTH_TOKEN, modem, apn, user, pass);
}

void loop()
{
  Blynk.run();
}
