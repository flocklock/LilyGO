#define MODEM_HEADER
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





/*
String readAT(int timeout) {
  int time = millis();
  String res = SerialAT.readStringUntil('\n');
  while (SerialMon.available() && millis() - time > timeout) {
    //res.push_back(SerialMon.read());
    SerialAT.write(SerialMon.read());
  }
}

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
*/