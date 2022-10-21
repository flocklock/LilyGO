/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12
#define BAT_ADC     35

#define uS_TO_S_FACTOR      1000000ULL

#define ADXL345_REG_DEVID (0x00)        ///< Device ID
#define ADXL345_REG_THRESH_TAP (0x1D)   ///< Tap threshold
#define ADXL345_REG_OFSX (0x1E)         ///< X-axis offset
#define ADXL345_REG_OFSY (0x1F)         ///< Y-axis offset
#define ADXL345_REG_OFSZ (0x20)         ///< Z-axis offset
#define ADXL345_REG_DUR (0x21)          ///< Tap duration
#define ADXL345_REG_LATENT (0x22)       ///< Tap latency
#define ADXL345_REG_WINDOW (0x23)       ///< Tap window
#define ADXL345_REG_THRESH_ACT (0x24)   ///< Activity threshold
#define ADXL345_REG_THRESH_INACT (0x25) ///< Inactivity threshold
#define ADXL345_REG_TIME_INACT (0x26)   ///< Inactivity time
#define ADXL345_REG_ACT_INACT_CTL                                              \
  (0x27) ///< Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF (0x28) ///< Free-fall threshold
#define ADXL345_REG_TIME_FF (0x29)   ///< Free-fall time
#define ADXL345_REG_TAP_AXES (0x2A)  ///< Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS (0x2B) ///< Source for single/double tap
#define ADXL345_REG_BW_RATE (0x2C)        ///< Data rate and power mode control
#define ADXL345_REG_POWER_CTL (0x2D)      ///< Power-saving features control
#define ADXL345_REG_INT_ENABLE (0x2E)     ///< Interrupt enable control
#define ADXL345_REG_INT_MAP (0x2F)        ///< Interrupt mapping control
#define ADXL345_REG_INT_SOURCE (0x30)     ///< Source of interrupts
#define ADXL345_REG_DATA_FORMAT (0x31)    ///< Data format control
#define ADXL345_REG_DATAX0 (0x32)         ///< X-axis data 0
#define ADXL345_REG_DATAX1 (0x33)         ///< X-axis data 1
#define ADXL345_REG_DATAY0 (0x34)         ///< Y-axis data 0
#define ADXL345_REG_DATAY1 (0x35)         ///< Y-axis data 1
#define ADXL345_REG_DATAZ0 (0x36)         ///< Z-axis data 0
#define ADXL345_REG_DATAZ1 (0x37)         ///< Z-axis data 1
#define ADXL345_REG_FIFO_CTL (0x38)       ///< FIFO control
#define ADXL345_REG_FIFO_STATUS (0x39)

#define ADXL345_MG2G_MULTIPLIER (0.004)
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  struct event {
    float X;
    float Y;
    float Z;
  };
class Accelerometer {
  public:
Accelerometer(int32_t sensorID = -1) {
    int32_t _sensorID;
    range_t _range;
}


uint8_t readRegister(uint8_t reg) {
  uint8_t buffer[1] = {i2c_dev ? reg : reg | 0x80};
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 1);
  return buffer[0];
}

uint8_t getDeviceID(void) {
  return readRegister(ADXL345_REG_DEVID);
}

void writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  i2c_dev->write(buffer, 2);
}

event readAccData(uint8_t reg) {
  event e;
  uint8_t buffer[6] = {i2c_dev ? reg : reg | 0x80 | 0x40, 0};
  i2c_dev->write(buffer, 1);
  i2c_dev->read(buffer, 6);
  e.X = int16_t(uint16_t(buffer[1]) << 8 | uint16_t(buffer[0])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  e.Y = int16_t(uint16_t(buffer[3]) << 8 | uint16_t(buffer[2])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  e.Z = int16_t(uint16_t(buffer[5]) << 8 | uint16_t(buffer[4])) * ADXL345_MG2G_MULTIPLIER * (9.80665F);
  return e;
}
bool begin(uint8_t i2caddr = ADXL345_DEFAULT_ADDRESS) {
    if (i2c_dev)
      delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice(i2caddr, &Wire);
    if (!i2c_dev->begin())
      return false;


  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    return false;
  }

  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);

  return true;
}

void setRange(range_t range) {
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
void setDataRate(dataRate_t dataRate) {
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

  private:
     Adafruit_I2CDevice *i2c_dev = NULL;
     int32_t _sensorID;
     range_t _range;
};


// Replace the next variables with your SSID/Password combination
const char* ssid = "flock";
const char* password = "flocklock";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.3.179";

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
  client.setServer(mqtt_server, 1883);;

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
