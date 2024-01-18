// References
// https://github.com/vshymanskyy/TinyGSM
// https://github.com/chrisjoyce911/esp32FOTA
// https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.h
// https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.cpp
// https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.cpp#L221
// https://github.com/vshymanskyy/TinyGSM/blob/master/examples/BlynkClient/BlynkClient.ino
// https://github.com/chrisjoyce911/esp32FOTA/issues/54

// ============== FOTA =============
#include <esp32fotagsm.h>

#ifndef MODEM_HEADER
#include <modemHeader.hpp>
#endif

const char *boardModel = "flocklock";
int boardCurrentVersion = 1; // The firmware version

// To define firmware type and version
esp32FOTAGSM fota(boardModel, boardCurrentVersion);

// To define link to check update json
#define esp32FOTAGSM_checkHOST      "147.251.115.100"         // TO CHANGE
#define esp32FOTAGSM_checkPORT      8000                 // TO CHANGE, HTTP ONLY
#define esp32FOTAGSM_checkRESOURCE  "/otice.json" // TO CHANGE

// ============== GSM ===============
#if (!defined(SRC_TINYGSMCLIENT_H_))
#define TINY_GSM_MODEM_SIM7000
#include <TinyGsmClient.h>
#endif  // SRC_TINYGSMCLIENT_H_

// To CHANGE to fit your board

// Your GPRS credentials, if any
const char apn[]  = "lpwa.vodafone.com";    // TO CHANGE
const char user[] = "";      // TO CHANGE
const char pass[] = "";      // TO CHANGE
bool gprs_connected = false;
String iccid = "";

bool reply = false;

void setupGSM()
{
  // =============== GSM ================
  // Set GSM module baud rate

  SerialAT.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(500);
  SerialMon.println("\nStarting Up Modem...");
  pinMode(PWR_PIN, OUTPUT);
  pinMode(PIN_DTR, OUTPUT);
  digitalWrite(PIN_DTR, LOW);

  modemPowerOn();  
  SerialMon.println("\nStarted");
  delay(6000);
  SerialMon.println("Initializing modem...");
  modem.restart();
  //SerialAT.println("AT+IPR=115200");
  //SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  int i = 10;
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
  
  modem.sendAT("+CCID");
  modem.waitResponse(10000L, iccid);

  modem.init();


  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  SerialAT.println("AT+CSQ");

  SerialAT.println("AT+IPR?");


  // To register to GSM
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) 
  {
    SerialMon.println("Network connected"); 
  }

  // =============== GPRS ================
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) 
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  gprs_connected = true;
  SerialMon.println(" success");
  SerialMon.println(modem.localIP());  
}

void setupFOTAGSM()
{
  fota.checkHOST = esp32FOTAGSM_checkHOST;
  fota.checkPORT = esp32FOTAGSM_checkPORT;
  fota.checkRESOURCE = esp32FOTAGSM_checkRESOURCE;  
  fota.setModem(modem); 
}