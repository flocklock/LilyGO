#define TINY_GSM_MODEM_SIM7000
#define PWR_PIN             4

#define SerialMon Serial
#define SerialAT Serial1
#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>
#include <StreamDebugger.h>

#ifdef DUMP_AT_COMMANDS
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
char replybuffer[255];

uint8_t readline(uint16_t timeout, bool multiline) {
  uint16_t replyidx = 0;

  while (timeout--) {
    if (replyidx >= 254) {
      // DEBUG_PRINTLN(F("SPACE"));
      break;
    }

    while (SerialAT.available()) {
      char c = SerialAT.read();
      if (c == '\r')
        continue;
      if (c == 0xA) {
        if (replyidx == 0) // the first 0x0A is ignored
          continue;

        if (!multiline) {
          timeout = 0; // the second 0x0A is the end of the line
          break;
        }
      }
      replybuffer[replyidx] = c;
      // DEBUG_PRINT(c, HEX); DEBUG_PRINT("#"); DEBUG_PRINTLN(c);
      replyidx++;
    }

    if (timeout == 0) {
      // DEBUG_PRINTLN(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  replybuffer[replyidx] = 0; // null term
  return replyidx;
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
    delay(1000);
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
    delay(1000);
    if(modem.disableGPS())
      SerialMon.println("GPS disabled");

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

String readAT(int timeout) {
  int time = millis();
  String res = SerialAT.readStringUntil('\n');
  while (SerialMon.available() && millis() - time > timeout) {
    //res.push_back(SerialMon.read());
    SerialAT.write(SerialMon.read());
  }
}