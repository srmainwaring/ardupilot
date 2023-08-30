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
  Arduino IDE 1.8.5

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.
*/

#include "AP_InertialSensor_BNO080_Driver.h"

#include <math.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Attempt communication with the device
// Return true if we got a 'Polo' back from Marco
#if !USE_AP_HAL_DEVICE
bool BNO080::begin(uint8_t deviceAddress, TwoWire *wirePort, uint8_t intPin)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  _i2cPort = wirePort;    // Grab which port the user wants us to use
  _int = intPin;          // Get the pin that the user wants to use for interrupts.
  if (_int != 255)        // By default, it's 255 and we'll not use it in dataAvailable() function.
  {
    //! @todo(srmainwaring) implement
    // pinMode(_int, INPUT_PULLUP);
  }
#else
bool BNO080::begin(AP_HAL::Device *dev)
{
  _dev = dev;
#endif

  // We expect caller to begin their I2C port, with the speed of their choice
  // external to the library
  // But if they forget, we start the hardware here.
  // _i2cPort->begin();

  // Begin by resetting the IMU
  softReset();

  // Check communication with device
  shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
  shtpData[1] = 0;                // Reserved

  // Transmit packet on channel 2, 2 bytes
  sendPacket(CHANNEL_CONTROL, 2);

  // Now we wait for response
  if (receivePacket() == true)
  {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
    {



      return (true);
    }
  }

  return (false); // Something went wrong
}

// Updates the latest variables if possible
// Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
  return (getReadings() != 0);
}

uint16_t BNO080::getReadings(void)
{
#if !USE_AP_HAL_DEVICE
  // If we have an interrupt pin connection available, check if data is available.
  // If int pin is not set, then we'll rely on receivePacket() to timeout
  // See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
  if (_int != 255)
  {
    //! @todo(srmainwaring) implement
    // if (digitalRead(_int) == HIGH)
    //   return 0;
  }
#endif

  if (receivePacket() == true)
  {
    // Check to see if this packet is a sensor reporting its data to us
    if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
    {
      return parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
    }
    else if (shtpHeader[2] == CHANNEL_CONTROL)
    {
      return parseCommandReport(); // This will update responses to commands, calibrationStatus, etc.
    }
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      return parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
    }
  }
  return 0;
}

// This function pulls the data from the command response report

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0]: The Report ID
// shtpData[1]: Sequence number (See 6.5.18.2)
// shtpData[2]: Command
// shtpData[3]: Command Sequence Number
// shtpData[4]: Response Sequence Number
// shtpData[5 + 0]: R0
// shtpData[5 + 1]: R1
// shtpData[5 + 2]: R2
// shtpData[5 + 3]: R3
// shtpData[5 + 4]: R4
// shtpData[5 + 5]: R5
// shtpData[5 + 6]: R6
// shtpData[5 + 7]: R7
// shtpData[5 + 8]: R8
uint16_t BNO080::parseCommandReport(void)
{
  if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
  {
    return shtpData[0];
  }
  else
  {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
  }

  // TODO additional feature reports may be strung together. Parse them all.
  return 0;
}

// This function pulls the data from the input report
// The input reports vary in length so this function stores the various
// 16-bit values as globals

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
// shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
// shtpData[5 + 1]: Sequence number (See 6.5.18.2)
// shtpData[5 + 2]: Status
// shtpData[3]: Delay
// shtpData[4:5]: i/accel x/gyro x/etc
// shtpData[6:7]: j/accel y/gyro y/etc
// shtpData[8:9]: k/accel z/gyro z/etc
// shtpData[10:11]: real/gyro temp/etc
// shtpData[12:13]: Accuracy estimate
uint16_t BNO080::parseInputReport(void)
{
  // Calculate the number of data bytes in this packet
  int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
  dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this
  // package is a continuation of the last.
  // Ignore it for now. TODO catch this as an error and exit

  dataLength -= 4; // Remove the header bytes from the data count

  timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) |
      ((uint32_t)shtpData[3] << (8 * 2)) |
      ((uint32_t)shtpData[2] << (8 * 1)) |
      ((uint32_t)shtpData[1] << (8 * 0));

  // uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
  uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
  uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
  uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
  // uint16_t data4 = 0;
  // uint16_t data5 = 0; // We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
  // uint16_t data6 = 0;

  // if (dataLength - 5 > 9)
  // {
  //   data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
  // }
  // if (dataLength - 5 > 11)
  // {
  //   data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
  // }
  // if (dataLength - 5 > 13)
  // {
  //   data6 = (uint16_t)shtpData[5 + 15] << 8 | shtpData[5 + 14];
  // }

  // Store these generic values to their proper global variable
  if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
  {
    memsRawAccelX = data1;
    memsRawAccelY = data2;
    memsRawAccelZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
  {
    memsRawGyroX = data1;
    memsRawGyroY = data2;
    memsRawGyroZ = data3;
  }
  else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
  {
    memsRawMagX = data1;
    memsRawMagY = data2;
    memsRawMagZ = data3;
  }
  else
  {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
    return 0;
  }

  // TODO additional feature reports may be strung together. Parse them all.
  return shtpData[5];
}

// Return raw mems value for the accel
int16_t BNO080::getRawAccelX()
{
  return (memsRawAccelX);
}

// Return raw mems value for the accel
int16_t BNO080::getRawAccelY()
{
  return (memsRawAccelY);
}

// Return raw mems value for the accel
int16_t BNO080::getRawAccelZ()
{
  return (memsRawAccelZ);
}

// Return raw mems value for the gyro
int16_t BNO080::getRawGyroX()
{
  return (memsRawGyroX);
}

int16_t BNO080::getRawGyroY()
{
  return (memsRawGyroY);
}

int16_t BNO080::getRawGyroZ()
{
  return (memsRawGyroZ);
}

//Return raw mems value for the mag
int16_t BNO080::getRawMagX()
{
  return (memsRawMagX);
}

int16_t BNO080::getRawMagY()
{
  return (memsRawMagY);
}

int16_t BNO080::getRawMagZ()
{
  return (memsRawMagZ);
}

// Send command to reset IC
// Read all advertisement packets from sensor
// The sensor has been seen to reset twice if we attempt too much too quickly.
// This seems to work reliably.
void BNO080::softReset(void)
{
  shtpData[0] = 1; //Reset

  // Attempt to start communication with sensor
  sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

  // Read all incoming data and flush it
  // delay(50);
  while (receivePacket() == true)
    ; // delay(1);
  // delay(50);
  while (receivePacket() == true)
    ; // delay(1);
}

// Sends the packet to enable the accelerometer
void BNO080::enableAccelerometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

// Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

// Sends the packet to enable the magnetometer
void BNO080::enableMagnetometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

// Sends the packet to enable the raw accel readings
// Note you must enable basic reporting on the sensor as well
void BNO080::enableRawAccelerometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

// Sends the packet to enable the raw accel readings
// Note you must enable basic reporting on the sensor as well
void BNO080::enableRawGyro(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

// Sends the packet to enable the raw accel readings
// Note you must enable basic reporting on the sensor as well
void BNO080::enableRawMagnetometer(uint16_t timeBetweenReports)
{
  setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

// Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
  setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

// Given a sensor's report ID, this tells the BNO080 to begin reporting the values
// Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
  long microsBetweenReports = (long)timeBetweenReports * 1000L;

  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;   //Set feature command. Reference page 55
  shtpData[1] = reportID;                 //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0;                   //Feature flags
  shtpData[3] = 0;                   //Change sensitivity (LSB)
  shtpData[4] = 0;                   //Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
  shtpData[9] = 0;                   //Batch Interval (LSB)
  shtpData[10] = 0;                   //Batch Interval
  shtpData[11] = 0;                   //Batch Interval
  shtpData[12] = 0;                   //Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF;     //Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF;     //Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF;    //Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF;    //Sensor-specific config (MSB)

  //Transmit packet on channel 2, 17 bytes
  sendPacket(CHANNEL_CONTROL, 17);
}

// Tell the sensor to do a command
// See 6.3.8 page 41, Command request
// The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
  shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
  shtpData[1] = commandSequenceNumber++;   //Increments automatically each function call
  shtpData[2] = command;             //Command

  //Caller must set these
  /*shtpData[3] = 0; //P0
  shtpData[4] = 0; //P1
  shtpData[5] = 0; //P2
  shtpData[6] = 0;
  shtpData[7] = 0;
  shtpData[8] = 0;
  shtpData[9] = 0;
  shtpData[10] = 0;
  shtpData[11] = 0;*/

  //Transmit packet on channel 2, 12 bytes
  sendPacket(CHANNEL_CONTROL, 12);
}

#if !USE_AP_HAL_DEVICE
// Wait a certain time for incoming I2C bytes before giving up
// Returns false if failed
bool BNO080::waitForI2C()
{
  for (uint8_t counter = 0; counter < 100; counter++) //Don't got more than 255
  {
    if (_i2cPort->available() > 0)
      return (true);
    delay(1);
  }
  return (false);
}
#endif

// Check to see if there is any new data available
// Read the contents of the incoming packet into the shtpData array
bool BNO080::receivePacket(void)
{  
  {
#if !USE_AP_HAL_DEVICE
    // Ask for four bytes to find out how much data we need to read
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)4);
    if (waitForI2C() == false)
      return (false); //Error

    // Get the first four bytes, aka the packet header
    uint8_t packetLSB = _i2cPort->read();
    uint8_t packetMSB = _i2cPort->read();
    uint8_t channelNumber = _i2cPort->read();
    uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not

    // Store the header info.
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber;
#else
    bool ret = _dev->read(shtpHeader, 4);
    if (!ret) {
        hal.console->printf("BNO080: failed to recv packet\n");
        return false; // Error
    }

    uint8_t packetLSB = shtpHeader[0];
    uint8_t packetMSB = shtpHeader[1];
    // uint8_t channelNumber = shtpHeader[2];
    // uint8_t sequenceNumber = shtpHeader[3];
#endif  // USE_AP_HAL_DEVICE

    // Calculate the number of data bytes in this packet
    uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
    dataLength &= ~(1 << 15); //Clear the MSbit.
    // This bit indicates if this package is a continuation of the last. Ignore it for now.
    // TODO catch this as an error and exit

    hal.console->printf("BNO080: recv packet\n");
    hal.console->printf("BNO080: data length: %d\n", dataLength);
    print_header(shtpHeader);

    if (dataLength == 0)
    {
      // Packet is empty
      return (false); //All done
    }
    dataLength -= 4; //Remove the header bytes from the data count

    getData(dataLength);
  }

  // Quickly check for reset complete packet. No need for a seperate parser.
  // This function is also called after soft reset, so we need to catch this
  // packet here otherwise we need to check for the reset packet in multiple
  // places.
  if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == EXECUTABLE_RESET_COMPLETE) 
  {
    _hasReset = true;
  } 

  return (true); //We're done!
}

// Sends multiple requests to sensor until all data bytes are received from sensor
// The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
// Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool BNO080::getData(uint16_t bytesRemaining)
{
  uint16_t dataSpot = 0; //Start at the beginning of shtpData array

  // Setup a series of chunked 32 byte reads
  while (bytesRemaining > 0)
  {
    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
      numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

#if !USE_AP_HAL_DEVICE
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)(numberOfBytesToRead + 4));
    if (waitForI2C() == false)
      return (0); // Error

    // The first four bytes are header bytes and are throw away
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();
    _i2cPort->read();

    for (uint8_t x = 0; x < numberOfBytesToRead; x++)
    {
      uint8_t incoming = _i2cPort->read();
      if (dataSpot < MAX_PACKET_SIZE)
      {
        shtpData[dataSpot++] = incoming; // Store data into the shtpData array
      }
      else
      {
        // Do nothing with the data
      }
    }
#else
    uint8_t recv[I2C_BUFFER_LENGTH];
    bool ret = _dev->read(recv, numberOfBytesToRead + 4);
    if (!ret) {
        hal.console->printf("BNO080: failed to get data\n");
        return 0; // Error
    }

    for (uint8_t x = 0; x < numberOfBytesToRead; x++)
    {
      uint8_t incoming = recv[x + 4];
      if (dataSpot < MAX_PACKET_SIZE)
      {
        shtpData[dataSpot++] = incoming; // Store data into the shtpData array
      }
      else
      {
        // Do nothing with the data
      }
    }
    print_packet(recv, numberOfBytesToRead + 4);
#endif

    bytesRemaining -= numberOfBytesToRead;
  }
  return (true); //Done!
}

// Given the data packet, send the header then the data
// Returns false if sensor does not ACK
// TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
  uint8_t packetLength = dataLength + 4; // Add four bytes for the header

#if !USE_AP_HAL_DEVICE
  {
    // You are trying to send too much. Break into smaller packets.
    // if(packetLength > I2C_BUFFER_LENGTH) return(false);
    _i2cPort->beginTransmission(_deviceAddress);

    //Send the 4 byte packet header
    _i2cPort->write(packetLength & 0xFF);        //Packet length LSB
    _i2cPort->write(packetLength >> 8);          //Packet length MSB
    _i2cPort->write(channelNumber);            //Channel number
    // Send the sequence number, increments with each packet sent, different counter for each channel
    _i2cPort->write(sequenceNumber[channelNumber]++);

    //Send the user's data packet
    for (uint8_t i = 0; i < dataLength; i++)
    {
      _i2cPort->write(shtpData[i]);
    }
    uint8_t i2cResult = _i2cPort->endTransmission();
    if (i2cResult != 0)
    {
      return (false);
    }
  }
  return (true);
#else
    uint8_t send[I2C_BUFFER_LENGTH];
    send[0] = packetLength & 0xFF;
    send[1] = packetLength >> 8;
    send[2] = channelNumber;
    send[3] = sequenceNumber[channelNumber]++;
    for (uint8_t i = 0; i < dataLength; i++)
    {
      send[i + 4] = shtpData[i];
    }
    bool i2cResult = _dev->transfer(send, packetLength, nullptr, 0);

    hal.console->printf("BNO080: send packet\n");
    hal.console->printf("BNO080: i2cResult: %d\n", i2cResult);
    print_packet(send, packetLength);

    return i2cResult;
#endif

}

void test()
{
    BNO080 imu;

    imu.begin();
    imu.enableAccelerometer(50);
    imu.enableRawAccelerometer(50);
    imu.enableGyro(50);
    imu.enableRawGyro(50);
    imu.enableMagnetometer(50);
    imu.enableRawMagnetometer(50);

    //Look for reports from the IMU
    if (imu.dataAvailable() == true)
    {
        imu.getRawAccelX();
        imu.getRawAccelY();
        imu.getRawAccelZ();

        imu.getRawGyroX();
        imu.getRawGyroY();
        imu.getRawGyroZ();

        imu.getRawMagX();
        imu.getRawMagY();
        imu.getRawMagZ();
    }
}

void BNO080::print_packet(uint8_t *pkt, uint8_t pkt_len)
{
    uint16_t packetLength = (uint16_t)pkt[1] << 8 | pkt[0];

    // Print the four byte header
    hal.console->printf("Header:");
    for (uint8_t x = 0; x < 4; x++)
    {
        hal.console->printf(" 0x%02hhx", pkt[x]);
    }
    hal.console->printf("\n");

    // Print data
    uint8_t printLength = packetLength - 4;
    if (printLength > 40)
        printLength = 40; //Artificial limit. We don't want the phone book.

    hal.console->printf("Body:  ");
    for (uint8_t x = 0; x < printLength; x++)
    {
        hal.console->printf(" 0x%02hhx", pkt[x + 4]);
    }
    hal.console->printf("\n");

    if (packetLength & 1 << 15)
    {
        hal.console->printf("[Continued packet]\n");
        packetLength &= ~(1 << 15);
    }

    hal.console->printf("Length: ");
    hal.console->printf("%u\n", packetLength);

    hal.console->printf("Channel: ");
    if (pkt[2] == 0)
        hal.console->printf("Command");
    else if (pkt[2] == 1)
        hal.console->printf("Executable");
    else if (pkt[2] == 2)
        hal.console->printf("Control");
    else if (pkt[2] == 3)
        hal.console->printf("Sensor-report");
    else if (pkt[2] == 4)
        hal.console->printf("Wake-report");
    else if (pkt[2] == 5)
        hal.console->printf("Gyro-vector");
    else
        hal.console->printf("0x%02hhx", pkt[2]);

    hal.console->printf("\n");
}

void BNO080::print_header(uint8_t *pkt)
{
    hal.console->printf("Header:");
    for (uint8_t x = 0; x < 4; x++)
    {
        hal.console->printf(" 0x%02hhx", pkt[x]);
    }
    hal.console->printf("\n");
}

