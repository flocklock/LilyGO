#include <PubSubClient.h>
#ifndef MODEM_HEADER
#include <modemHeader.hpp>
#endif

#ifdef DEBUG
#  define D(x) x
#else
#  define D(x)
#endif // DEBUG

#define topicTest   "fl/test"
#define topicTest2  "fl/test2"
#define topicGnss   "fl/gnss"
#define topicLukas  "fl/lukas"
#define topicTou    "fl/tou"
#define topicApp    "fl/app"
#define topicOvce   "fl/ovce"
#define topicLaz    "fl/laz"
#define topicItaly  "fl/it"
#define topicHurta  "fl/hur"
#define topicHub    "fl/hub"
#define topicSedlar "fl/sed"
#define topicKone   "fl/kone"

char* topic = topicTest;
char* name = "test";

char* broker = "147.251.115.100";
uint16_t mqttPort = 1883;



PubSubClient  mqtt(clientMQTT);

uint32_t lastReconnectAttempt = 0;


boolean mqttConnect(String _deviceID) {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect(_deviceID.c_str(), NULL, NULL, 0, 0, 0, 0, false);

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    D(SerialMon.println("mqtt connect fail");)
    return false;
  }
  D(SerialMon.println(" success");)
  String connectMess = _deviceID + String(" client connected");
  mqtt.publish(topic, connectMess.c_str());
  return mqtt.connected();
}