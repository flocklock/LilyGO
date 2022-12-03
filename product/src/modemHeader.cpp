#include <modemHeader.hpp>

#ifdef DUMP_AT_COMMANDS
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient clientFOTA(modem, 0);
TinyGsmClient clientMQTT(modem, 1);