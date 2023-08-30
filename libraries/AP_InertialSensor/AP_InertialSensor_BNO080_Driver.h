// Pared down version to retrive raw data

/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.
*/

#pragma once

// #include "Arduino.h"
// #include <Wire.h>
// #include <SPI.h>
#include <stddef.h>
#include <stdint.h>

#include <AP_HAL/AP_HAL.h>

#define USE_AP_HAL_DEVICE 1

// The default I2C address for the BNO080 on the SparkX breakout is 0x4B.
// 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B

// Define the size of the I2C buffer
#define I2C_BUFFER_LENGTH 32

//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

// All the ways we can configure or talk to the BNO080, figure 34,
// page 36 reference manual
// These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

// Packets can be up to 32k but we don't have that much RAM.
#define MAX_PACKET_SIZE 128
// This is in words. There can be many but we mostly only care about
// the first 9 (Qs, range, etc)
#define MAX_METADATA_SIZE 9

#if !USE_AP_HAL_DEVICE
// stub for TwoWire
class TwoWire
{
public:
    void beginTransmission(uint8_t address) {
    }

    uint8_t endTransmission(bool stop = true) {
        return 0;
    }

    uint8_t requestFrom(uint8_t aaddress, size_t quantity, bool stop = true) {
        return 0;
    }

    size_t write(uint8_t) {
        return 0;
    }

    size_t available() {
        return 0;
    }

    size_t read() {
        return 0;
    }
};
#endif

class BNO080
{
public:
  // By default use the default I2C addres, and use Wire port,
  // and don't declare an INT pin
#if !USE_AP_HAL_DEVICE
  bool begin(uint8_t deviceAddress = BNO080_DEFAULT_ADDRESS,
      TwoWire *wirePort = nullptr, uint8_t intPin = 255);
#else
  bool begin(AP_HAL::Device *dev = nullptr);
#endif

  // Try to reset the IMU via software
  void softReset();

#if !USE_AP_HAL_DEVICE
  // Delay based polling for I2C traffic
  bool waitForI2C();
#endif

  bool receivePacket(void);
  // Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
  bool getData(uint16_t bytesRemaining);
  bool sendPacket(uint8_t channelNumber, uint8_t dataLength);

  void enableAccelerometer(uint16_t timeBetweenReports);
  void enableGyro(uint16_t timeBetweenReports);
  void enableMagnetometer(uint16_t timeBetweenReports);
  void enableRawAccelerometer(uint16_t timeBetweenReports);
  void enableRawGyro(uint16_t timeBetweenReports);
  void enableRawMagnetometer(uint16_t timeBetweenReports);

  bool dataAvailable(void);
  uint16_t getReadings(void);
  // Parse sensor readings out of report
  uint16_t parseInputReport(void);
  // Parse command responses out of report
  uint16_t parseCommandReport(void);

  int16_t getRawAccelX();
  int16_t getRawAccelY();
  int16_t getRawAccelZ();

  int16_t getRawGyroX();
  int16_t getRawGyroY();
  int16_t getRawGyroZ();

  int16_t getRawMagX();
  int16_t getRawMagY();
  int16_t getRawMagZ();

  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports,
      uint32_t specificConfig);
  void sendCommand(uint8_t command);

  // debugging
  static void print_packet(uint8_t *pkt, uint8_t pkt_len);
  static void print_header(uint8_t *pkt);

  //Global Variables
  uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
  uint8_t shtpData[MAX_PACKET_SIZE];
  // There are 6 com channels. Each channel has its own seqnum
  uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0};
  // Commands have a seqNum as well. These are inside command packet,
  // the header uses its own seqNum per channel
  uint8_t commandSequenceNumber = 0;
  // There is more than 10 words in a metadata record
  // but we'll stop at Q point 3
  uint32_t metaData[MAX_METADATA_SIZE];

private:
#if !USE_AP_HAL_DEVICE
  // The generic connection to user's chosen I2C hardware
  TwoWire *_i2cPort;
  // Keeps track of I2C address. setI2CAddress changes this.
  uint8_t _deviceAddress;
  uint8_t _int;
#else
  AP_HAL::Device *_dev;
#endif

  // Keeps track of any Reset Complete packets we receive. 
  bool _hasReset = false;
  uint32_t timeStamp;

  //Raw readings from MEMS sensor
  uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; 
  uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;
  uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;
};
