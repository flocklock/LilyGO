#include <acc.hpp>

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