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

/*
 * This board that does not contain any sensors (but pins are active), it is a great help for a novice user,
 * by flashing an empty board, you can connect via Mavlink (Mission Planner - MP) and gradually add sensors.
 * If you had some sensor configured and it doesn't work then the MP connection does not work and then you may not know what to do next.
*/

#pragma once

#define HAL_ESP32_BOARD_NAME "esp32-rhys"

#define TRUE  1
#define FALSE 0

//Protocols
// list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
// default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
// ESP32 serials:    AP_HAL_ESP32/HAL_ESP32_Class.cpp

#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_Console   //A  UART0
//#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_MAVLink2   //A  UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD            AP_SERIALMANAGER_CONSOLE_BAUD/1000  //115200

//#define DEFAULT_SERIAL1_PROTOCOL        SerialProtocol_MAVLink2   //C  WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

//#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_MAVLink2   //D  UART2
//#define DEFAULT_SERIAL2_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600
#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_DDS_XRCE   //D  UART2
#define DEFAULT_SERIAL2_BAUD            (115200/1000)
//#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_MSP        //D  UART2
//#define DEFAULT_SERIAL2_BAUD            (115200/1000)
//#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_None       //D  UART2
//#define DEFAULT_SERIAL2_BAUD            (115200/1000)

//#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_GPS        //B  UART1
//#define DEFAULT_SERIAL3_BAUD            AP_SERIALMANAGER_GPS_BAUD/1000
//#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_MAVLink2   //B  UART1
//#define DEFAULT_SERIAL3_BAUD            (AP_SERIALMANAGER_MAVLINK_BAUD/1000)
#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_MSP        //B  UART1
#define DEFAULT_SERIAL3_BAUD            (115200/1000)
//#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_None       //B  UART1
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

//Probe for sensors
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this, GET_I2C_DEVICE(bus, addr), ##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))

//INS
// 1. No INS
//#define HAL_INS_DEFAULT HAL_INS_NONE
//#define HAL_INS_DEFAULT AP_FEATURE_BOARD_DETECT
// 2. BNO080
//#define HAL_INS_DEFAULT HAL_INS_BNO080_I2C
//#define HAL_INS_BNO080_NAME "bno080"
//#define HAL_INS_BNO080_BUS 0
//#define HAL_INS_BNO080_ADDRESS (0x4B)
//#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(BNO080, HAL_INS_BNO080_BUS, HAL_INS_BNO080_ADDRESS, ROTATION_NONE)
// 3. MPU9250
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)

//I2C Buses
//#define HAL_ESP32_I2C_BUSES {}
#define HAL_ESP32_I2C_BUSES \
    {.port=I2C_NUM_0, .sda=GPIO_NUM_22, .scl=GPIO_NUM_21, .speed=400*KHZ, .internal=true, .soft=true}

//SPI Buses
// SPI BUS setup, including gpio, dma, etc
// note... we use 'vspi' for the bmp280 and mpu9250
// tip:  VSPI_HOST  is an alternative name for esp's SPI3
//#define HAL_ESP32_SPI_BUSES {}
#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

//SPI Devices
// SPI per-device setup, including speeds, etc.
//#define HAL_ESP32_SPI_DEVICES {}
//#define HAL_ESP32_SPI_DEVICES
//     {.name= "bmp280", .bus=0, .device=0, .cs=GPIO_NUM_26, .mode = 3, .lspeed=1*MHZ, .hspeed=1*MHZ},
//     {.name="mpu9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}
#define HAL_ESP32_SPI_DEVICES \
    {.name="mpu9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}

//RCIN
#define HAL_ESP32_RCIN GPIO_NUM_4

//RCOUT
#define HAL_ESP32_RCOUT {GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32}

//AIRSPEED
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

//BAROMETER
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

//IMU
//#define AP_INERTIALSENSOR_ENABLED 1
//#define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

//COMPASS
#define AP_COMPASS_ENABLE_DEFAULT 1
#define ALLOW_ARM_NO_COMPASS

//See boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//WIFI
#define HAL_ESP32_WIFI 1  // 1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_STATION 1    // 0-AP, 1-STA
#define WIFI_HOSTNAME "ardupilot-esp32"
#define WIFI_SSID "ardupilot-esp32"     // SSID for AP mode
#define WIFI_SSID_STATION "ssid"        // SSID for STA mode
#define WIFI_PWD "password"
#define WIFI_CHANNEL 7

//UARTs
// UART_NUM_0 and UART_NUM_2 are configured to use defaults
// UART_NUM_0 => SERIAL0
// UART_NUM_1 => SERIAL3
// UART_NUM_2 => SERIAL2
#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 },\
    {.port=UART_NUM_1, .rx=GPIO_NUM_14, .tx=GPIO_NUM_15},\
    {.port=UART_NUM_2, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17}

//ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

//LED
#define DEFAULT_NTF_LED_TYPES Notify_LED_None

//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER 4

//SD CARD
// Do u want to use mmc or spi mode for the sd card, this is board specific,
// as mmc uses specific pins but is quicker,
// and spi is more flexible pinouts....
// dont forget vspi/hspi should be selected to NOT conflict with HAL_ESP32_SPI_BUSES

//#define HAL_ESP32_SDCARD //after enabled, uncomment one of below
//#define HAL_ESP32_SDMMC
//#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_2, .miso=GPIO_NUM_15, .sclk=GPIO_NUM_26, .cs=GPIO_NUM_21}

#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

// uncommenting one or more of these will give more console debug in certain areas.. ... 
// ...however all teh extra printf's use a lot of stack, so best to limit yourself to only uncommenting one at a time
//#define STORAGEDEBUG 1
//#define SCHEDDEBUG 1
//#define FSDEBUG 1
//#define BUSDEBUG 1 //ok
//#define WIFIDEBUG 1 //uses a lot?
//#define INS_TIMING_DEBUG 1 //define this to see all the imu-resets and temp resets and imu timing info on the console.

// Avoid and fences. Enable to resolve fench breach message
#define AC_AVOID_ENABLED 1
#define AP_FENCE_ENABLED 1

//MSP sensors for Mateksys M8Q-CAN
#define HAL_MSP_ENABLED 1
#define HAL_MSP_SENSORS_ENABLED 1

//MSP GPS
#define HAL_MSP_GPS_ENABLED 1
#define HAL_GPS_TYPE_DEFAULT 19

//MSP Compass
#define AP_COMPASS_MSP_ENABLED 1
#define AP_COMPASS_ENABLE_DEFAULT 1

//MSP Baro
// #define HAL_BARO_PROBE_EXT_DEFAULT 4096
// #define AP_BARO_MSP_ENABLED 1
