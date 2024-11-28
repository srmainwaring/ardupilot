/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define HAL_ESP32_BOARD_NAME "esp32rover"

#define TRUE  1
#define FALSE 0

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))

//------------------------------------

//Protocols
// list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
// default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
// ESP32 serials:    AP_HAL_ESP32/HAL_ESP32_Class.cpp

//#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_MAVLink2   //A  UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD            AP_SERIALMANAGER_CONSOLE_BAUD/1000  //115200

//#define DEFAULT_SERIAL1_PROTOCOL        SerialProtocol_MAVLink2   //C  WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_MAVLink2   //D  UART2
#define DEFAULT_SERIAL2_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_GPS        //B  UART1: GPS1
#define DEFAULT_SERIAL3_BAUD            AP_SERIALMANAGER_GPS_BAUD/1000    //38400, Can not define default baudrate here (by config only)
//#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_None       //B
//#define DEFAULT_SERIAL3_BAUD            (115200/1000)

#define DEFAULT_SERIAL4_PROTOCOL        SerialProtocol_None       //E
#define DEFAULT_SERIAL5_BAUD            (115200/1000)

#define DEFAULT_SERIAL5_PROTOCOL        SerialProtocol_None       //F
#define DEFAULT_SERIAL5_BAUD            (115200/1000)

#define DEFAULT_SERIAL6_PROTOCOL        SerialProtocol_None       //G
#define DEFAULT_SERIAL6_BAUD            (115200/1000)

#define DEFAULT_SERIAL7_PROTOCOL        SerialProtocol_None       //H
#define DEFAULT_SERIAL7_BAUD            (115200/1000)

#define DEFAULT_SERIAL8_PROTOCOL        SerialProtocol_None       //I
#define DEFAULT_SERIAL8_BAUD            (115200/1000)

#define DEFAULT_SERIAL9_PROTOCOL        SerialProtocol_None       //J
#define DEFAULT_SERIAL9_BAUD            (115200/1000)

//I2C Buses
#define HAL_ESP32_I2C_BUSES \
    {.port=I2C_NUM_0, .sda=GPIO_NUM_21, .scl=GPIO_NUM_22, .speed=400*KHZ, .internal=TRUE, .soft=TRUE}

//SPI Buses
#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

//SPI Devices
#define HAL_ESP32_SPI_DEVICES \
    {.name= "mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_5, .mode=0, .lspeed=2*MHZ, .hspeed=8*MHZ}

//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER 4
//RCIN pin number - NOTE: disabled due to issue with legacy rmt library.
// #define HAL_ESP32_RCIN GPIO_NUM_36

//RCOUT
#define HAL_ESP32_RCOUT {GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32}

//AIRSPEED
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

// MPU-9250 is a SIP that combines two chips:
// MPU-6500 3-axis gyro, 3-axis accel
// AK8963   3-axis magnetometer
// https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/

//IMU
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)

//COMPASS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963_MPU9250
#define AP_COMPASS_AK8963_ENABLED TRUE
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1
#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_NONE))

//See boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//WIFI
#define HAL_ESP32_WIFI 1  //1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID "ardupilot-esp32"
#define WIFI_PWD "ardupilot-esp32"

//UARTs
#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 },\
    {.port=UART_NUM_1, .rx=GPIO_NUM_34, .tx=GPIO_NUM_18},\
    {.port=UART_NUM_2, .rx=GPIO_NUM_16, .tx=GPIO_NUM_26}

//ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

//LED
#define DEFAULT_NTF_LED_TYPES Notify_LED_None

//SD CARD
// Do u want to use mmc or spi mode for the sd card, this is board specific,
// as mmc uses specific pins but is quicker,
// and spi is more flexible pinouts....
// dont forget vspi/hspi should be selected to NOT conflict with HAL_ESP32_SPI_BUSES

#define HAL_ESP32_SDCARD 1
#define HAL_ESP32_SDSPI \
    {.host=HSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_13, .miso=GPIO_NUM_15, .sclk=GPIO_NUM_14, .cs=GPIO_NUM_4}

#define HAL_LOGGING_FILESYSTEM_ENABLED 1
#define HAL_LOGGING_DATAFLASH_ENABLED 0   // enabled when board is SITL
#define HAL_LOGGING_MAVLINK_ENABLED 1

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define SCRIPTING_DIRECTORY "/SDCARD/APM/SCRIPTS"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

//Set default for param LOG_FILE_BUFSIZE
#define HAL_LOGGING_FILE_BUFSIZE 8

//Scripting
#define MAX_HEAPS 1 // allow 1 heap (default is 10)

// uncommenting one or more of these will give more console debug in certain areas.. ... 
// ...however all the extra printf's use a lot of stack, so best to limit yourself to only uncommenting one at a time
//#define STORAGEDEBUG 1
//#define SCHEDDEBUG 1
//#define FSDEBUG 1
//#define BUSDEBUG 1 //ok
//#define WIFIDEBUG 1 //uses a lot?
//#define INS_TIMING_DEBUG 1 //define this to see all the imu-resets and temp resets and imu timing info on the console.

// Avoidance and fences. Enable to resolve fence breach message
// #define AP_AVOIDANCE_ENABLED 1
// #define AP_FENCE_ENABLED 1

// RCIN
#define AP_RCPROTOCOL_ENABLED 0
