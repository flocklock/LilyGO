


#include "header.hpp"

Accelerometer::Accelerometer(int32_t sensorID) {
    int32_t _sensorID;
    range_t _range;
}

uint8_t Accelerometer::readRegister(uint8_t reg) {
  uint8_t buffer[1] = {i2c_dev ? reg : reg | 0x80};
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 1);
  return buffer[0];
}

uint8_t Accelerometer::getDeviceID(void) {
  return readRegister(ADXL345_REG_DEVID);
}

void Accelerometer::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  i2c_dev->write(buffer, 2);
}

event Accelerometer::readAccData(uint8_t reg) {
  event e;
  uint8_t buffer[6] = {i2c_dev ? reg : reg | 0x80 | 0x40, 0};
  i2c_dev->write(buffer, 1);
  i2c_dev->read(buffer, 6);
  e.X = int16_t(uint16_t(buffer[1]) << 8 | uint16_t(buffer[0])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  e.Y = int16_t(uint16_t(buffer[3]) << 8 | uint16_t(buffer[2])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  e.Z = int16_t(uint16_t(buffer[5]) << 8 | uint16_t(buffer[4])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  return e;
}
bool Accelerometer::begin(uint8_t i2caddr) {
    if (i2c_dev)
      delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice(i2caddr, &Wire);
    if (!i2c_dev->begin())
      return false;


  /* Check connection */
  uint8_t deviceid = Accelerometer::getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    return false;
  }

  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);

  return true;
}

void Accelerometer::setRange(range_t range) {
  /* Read the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}
void Accelerometer::setDataRate(dataRate_t dataRate) {
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}


// Replace the next variables with your SSID/Password combination
const char* ssid = "flock";
const char* password = "flocklock";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.108.179";
String server_ip;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
Accelerometer accel = Accelerometer(12345);
event e;
std::string message;

void setup() {
  Serial.begin(115200);
    /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_12_5_HZ);
  //uint8_t fifoReg = accel.readRegister(ADXL345_REG_FIFO_CTL);
  //fifoReg &= 0b00011111;
  //fifoReg |= 0b10000000;
  accel.writeRegister(ADXL345_REG_FIFO_CTL, 0b00011110);
  //accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00000010);
  //accel.writeRegister(ADXL345_REG_INT_MAP, 0b00000000);

  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  
  setup_wifi();

  Serial.println("Napiš IP adresu počítače:");
  while(Serial.available() == 0) {}
  server_ip = Serial.readString();
  Serial.println("Adresa je " + server_ip);
  client.setServer(server_ip.c_str(), 1883);
  

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 120) {
    lastMsg = now;
    
    // Temperature in Celsius
    e = accel.readAccData(ADXL345_REG_DATAX0);
    message = (std::to_string(e.X).substr(0,5) + "," + std::to_string(e.Y).substr(0,5) + "," + std::to_string(e.Z).substr(0,5) + "\n");  
    
    //Serial.print("Time: ");
    //Serial.println(message.c_str());
    client.publish("esp32/acc", message.c_str());
  }
}
