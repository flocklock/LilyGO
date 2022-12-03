#include <PubSubClient.h>
#ifndef MODEM_HEADER
#include <modemHeader.hpp>
#endif

const char* broker = "147.251.115.100";
uint16_t mqttPort = 1883;

const char* topicTest       = "fl/test";
const char* topicGnss      = "fl/gnss";
const char* topicLukas = "fl/lukas";

PubSubClient  mqtt(clientMQTT);

uint32_t lastReconnectAttempt = 0;


boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("flocklockID");

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println("mqtt connect fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.publish(topicTest, "GsmClientTest started");
  return mqtt.connected();
}