/**************************************************************************/
/*!
    @file     Adafruit_ADXL345_U.cpp
    @author   K.Townsend (Adafruit Industries)

    LOCAL COPY — patched to use SPI_MODE3 (CPOL=1, CPHA=1) as required
    by the ADXL345 datasheet (clock idle HIGH).  The upstream library
    (v1.3.4) incorrectly uses SPI_MODE1 which causes getDeviceID() to
    return garbage and begin() to fail with "sensor not found".
*/
/**************************************************************************/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <limits.h>

#include "Adafruit_ADXL345_U.h"

Adafruit_ADXL345_Unified::~Adafruit_ADXL345_Unified() {
  if (i2c_dev)
    delete i2c_dev;
  if (spi_dev)
    delete spi_dev;
}

void Adafruit_ADXL345_Unified::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (i2c_dev) {
    i2c_dev->write(buffer, 2);
  } else {
    spi_dev->write(buffer, 2);
  }
}

uint8_t Adafruit_ADXL345_Unified::readRegister(uint8_t reg) {
  uint8_t buffer[1] = {i2c_dev ? reg : (uint8_t)(reg | 0x80)};
  if (i2c_dev) {
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 1);
  } else {
    spi_dev->write_then_read(buffer, 1, buffer, 1);
  }
  return buffer[0];
}

int16_t Adafruit_ADXL345_Unified::read16(uint8_t reg) {
  uint8_t buffer[2] = {i2c_dev ? reg : (uint8_t)(reg | 0x80 | 0x40), 0};
  if (i2c_dev) {
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 2);
  } else {
    spi_dev->write_then_read(buffer, 1, buffer, 2);
  }
  return uint16_t(buffer[1]) << 8 | uint16_t(buffer[0]);
}

uint8_t Adafruit_ADXL345_Unified::getDeviceID(void) {
  return readRegister(ADXL345_REG_DEVID);
}

int16_t Adafruit_ADXL345_Unified::getX(void) {
  return read16(ADXL345_REG_DATAX0);
}

int16_t Adafruit_ADXL345_Unified::getY(void) {
  return read16(ADXL345_REG_DATAY0);
}

int16_t Adafruit_ADXL345_Unified::getZ(void) {
  return read16(ADXL345_REG_DATAZ0);
}

Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
}

Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(uint8_t clock, uint8_t miso,
                                                   uint8_t mosi, uint8_t cs,
                                                   int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  // FIX: ADXL345 requires SPI_MODE3 (CPOL=1, CPHA=1 — clock idle HIGH).
  // Upstream v1.3.4 uses SPI_MODE1 which inverts the clock polarity and
  // causes the device-ID read to return 0x00 instead of 0xE5.
  spi_dev = new Adafruit_SPIDevice(cs, clock, miso, mosi, 1000000,
                                   SPI_BITORDER_MSBFIRST, SPI_MODE3);
}

bool Adafruit_ADXL345_Unified::begin(uint8_t i2caddr) {
  if (spi_dev == NULL) {
    if (i2c_dev)
      delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice(i2caddr, &Wire);
    if (!i2c_dev->begin())
      return false;
  } else {
    if (!spi_dev->begin())
      return false;
  }

  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    return false;
  }

  writeRegister(ADXL345_REG_POWER_CTL, 0x08);
  return true;
}

void Adafruit_ADXL345_Unified::setRange(range_t range) {
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);
  format &= ~0x0F;
  format |= range;
  format |= 0x08;
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
  _range = range;
}

range_t Adafruit_ADXL345_Unified::getRange(void) {
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

void Adafruit_ADXL345_Unified::setDataRate(dataRate_t dataRate) {
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

dataRate_t Adafruit_ADXL345_Unified::getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

bool Adafruit_ADXL345_Unified::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x =
      getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y =
      getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z =
      getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  return true;
}

void Adafruit_ADXL345_Unified::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = -156.9064F;
  sensor->min_value = 156.9064F;
  sensor->resolution = 0.03923F;
}
