#include "AP_UROS_Client.h"

#include <uxr/client/transport.h>
#include <rmw_microros/custom_transport.h>

#include <errno.h>

#include <driver/uart.h>
#include <driver/gpio.h>
#include <soc/uart_channel.h>

// Using IO_MUX (direct) vs GPIO Matrix (routed)
//
// UART_NUM_0 and UART_NUM_2 use the IO_MUX
// UART_NUM_1 use the GPIO Matrix
//
// For further details refer to the ESP32 API Reference:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#gpio-lookup-macros
//

// UART_NUM_0
#define UART_TXD_0  UART_NUM_0_TXD_DIRECT_GPIO_NUM  // GPIO1
#define UART_RXD_0  UART_NUM_0_RXD_DIRECT_GPIO_NUM  // GPIO3
#define UART_RTS_0  UART_NUM_0_RTS_DIRECT_GPIO_NUM  // GPIO22
#define UART_CTS_0  UART_NUM_0_CTS_DIRECT_GPIO_NUM  // GPIO19

// UART_NUM_1
#define UART_TXD_1  18  // GPIO18
#define UART_RXD_1  34  // GPIO34
#define UART_RTS_1  UART_PIN_NO_CHANGE
#define UART_CTS_1  UART_PIN_NO_CHANGE

// UART_NUM_2
#define UART_TXD_2  UART_NUM_2_TXD_DIRECT_GPIO_NUM  // GPIO17
#define UART_RXD_2  UART_NUM_2_RXD_DIRECT_GPIO_NUM  // GPIO16
#define UART_RTS_2  UART_NUM_2_RTS_DIRECT_GPIO_NUM  // GPIO7
#define UART_CTS_2  UART_NUM_2_CTS_DIRECT_GPIO_NUM  // GPIO8
// #define UART_TXD_2  19  // GPIO19
// #define UART_RXD_2  35  // GPIO35
// #define UART_RTS_2  UART_PIN_NO_CHANGE  // GPIO7
// #define UART_CTS_2  UART_PIN_NO_CHANGE  // GPIO8

// --- micro-ROS Transports ---
#define UART_BUFFER_SIZE (512)

bool AP_UROS_Client::serial_transport_open(struct uxrCustomTransport * transport) {
    size_t * uart_port = (size_t*) transport->args;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL) {
        return false;
    }

    int tx_io_num = UART_PIN_NO_CHANGE;
    int rx_io_num = UART_PIN_NO_CHANGE;
    int rts_io_num = UART_PIN_NO_CHANGE;
    int cts_io_num = UART_PIN_NO_CHANGE;

    switch (*uart_port) {
      case 0:
          tx_io_num = UART_TXD_0;
          rx_io_num = UART_RXD_0;
          break;
      case 1:
          tx_io_num = UART_TXD_1;
          rx_io_num = UART_RXD_1;
          break;
      case 2:
          tx_io_num = UART_TXD_2;
          rx_io_num = UART_RXD_2;
          break;
      default:
        break;
     }

    if (uart_set_pin(*uart_port, tx_io_num, rx_io_num, rts_io_num, cts_io_num) == ESP_FAIL) {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;
}

bool AP_UROS_Client::serial_transport_close(struct uxrCustomTransport * transport) {
    size_t * uart_port = (size_t*) transport->args;

    return uart_driver_delete(*uart_port) == ESP_OK;
}

size_t AP_UROS_Client::serial_transport_write(struct uxrCustomTransport* transport,
    const uint8_t * buf, size_t len, uint8_t * err) {
    size_t * uart_port = (size_t*) transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char*) buf, len);
    return txBytes;
}

size_t AP_UROS_Client::serial_transport_read(struct uxrCustomTransport* transport,
    uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    size_t * uart_port = (size_t*) transport->args;
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rxBytes;
}

/*
  initialise serial connection
 */
bool AP_UROS_Client::urosSerialInit()
{
    //! @todo set from param
    uart_port = UART_NUM_2;

    // setup a framed transport for serial
    rmw_ret_t rcl_ret = rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        serial_transport_open,
        serial_transport_close,
        serial_transport_write,
        serial_transport_read
    );

    // // setup a non-framed transport for UDP
    // rmw_ret_t rcl_ret = rmw_uros_set_custom_transport(
    //         false,
    //         (void*)this,
    //         udp_transport_open,
    //         udp_transport_close,
    //         udp_transport_write,
    //         udp_transport_read);

    return (rcl_ret == RCL_RET_OK);
}
