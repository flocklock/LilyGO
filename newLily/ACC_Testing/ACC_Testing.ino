#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12
#define BAT_ADC     35

#define uS_TO_S_FACTOR      1000000ULL


#define SerialMon Serial
#define SerialAT Serial1

#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SimpleTimer.h>

#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


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

Accelerometer accel = Accelerometer(12345);
event e;
std::string message;

void setup() {
  Serial.begin(115200);
  delay(10);
  setCpuFrequencyMhz(80);
    delay(10);


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
  accel.writeRegister(ADXL345_REG_FIFO_CTL, 0b10011110);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0b00000010);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0b00000000);


}

void loop() {
  esp_sleep_enable_timer_wakeup(3 * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, HIGH);
  Serial.flush();
  message = "";
  esp_light_sleep_start();
  print_wakeup_reason();

  for (int i = 0; i < 31; i++) {
    e = accel.readAccData(ADXL345_REG_DATAX0);
    message.append(std::to_string(millis()) + "," + std::to_string(e.X).substr(0,6) + "," + std::to_string(e.Y).substr(0,5) + "," + std::to_string(e.Z).substr(0,5) + "\n");
  }
  Serial.println(message.c_str());
  //event e = accel.readAccData(ADXL345_REG_DATAX0);
  //accel.getEvent(&event);
  
  //std::string message = std::to_string(millis()) + "," + std::to_string(e.X).substr(0,6) + "," + std::to_string(e.Y).substr(0,5) + "," + std::to_string(e.Z).substr(0,5) + "\n";
  //Serial.println(message.c_str());

}
