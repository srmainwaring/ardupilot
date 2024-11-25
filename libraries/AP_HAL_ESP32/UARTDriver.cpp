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

#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>

#include "esp_log.h"

extern const AP_HAL::HAL& hal;

namespace ESP32
{

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

void UARTDriver::vprintf(const char *fmt, va_list ap)
{

    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    // hal.console->printf("%s:%d UART num:%d\n", __PRETTY_FUNCTION__, __LINE__, uart_desc[uart_num].port);

    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
        if (!_initialized) {
            // report error status
            esp_err_t ret;

            // set communication params
            uart_config_t config = {
                .baud_rate = (int)b,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };
            ret = uart_param_config(p, &config);
            if (ret != ESP_OK) {
                hal.console->printf("UART num:%d: uart_param_config failed: %d\n",
                    uart_desc[uart_num].port, ret);
            }

            uint16_t options = _last_options;
            hal.console->printf("UART num:%d: options: %d\n", uart_desc[uart_num].port, options);

            // half-duplex
            if (options & OPTION_HDPLEX) {
                hal.console->printf("UART num:%d: half-duplex\n", uart_desc[uart_num].port);
                half_duplex = true;
            }

            // invert levels
            uint16_t inverse_mask = UART_SIGNAL_INV_DISABLE;

            // if we are half-duplex then treat either inversion option as
            // both being enabled. This is easier to understand for users, who
            // can be confused as to which pin is the one that needs inversion
            if ((options & OPTION_HDPLEX)
                && (options & (OPTION_TXINV|OPTION_RXINV)) != 0) {
                options |= OPTION_TXINV|OPTION_RXINV;
            }

            if (options & OPTION_RXINV) {
                hal.console->printf("UART num:%d: invert RX\n", uart_desc[uart_num].port);
                inverse_mask |= UART_SIGNAL_RXD_INV;
            }
            if (options & OPTION_TXINV) {
                hal.console->printf("UART num:%d: invert TX\n", uart_desc[uart_num].port);
                inverse_mask |= UART_SIGNAL_TXD_INV;
            }
            hal.console->printf("UART num:%d: inverse_mask: %d\n", uart_desc[uart_num].port, inverse_mask);
            ret = uart_set_line_inverse(p, inverse_mask);
            if (ret != ESP_OK) {
                hal.console->printf("UART num:%d: uart_set_line_inverse failed: %d\n",
                    uart_desc[uart_num].port, ret);
            }

            // set communication pins
            if (half_duplex) {
                // esp32 uses the RTS pin for half-duplex communication
                ret = uart_set_pin(p,
                    UART_PIN_NO_CHANGE,
                    UART_PIN_NO_CHANGE,
                    uart_desc[uart_num].tx,
                    UART_PIN_NO_CHANGE);
            } else {
                ret = uart_set_pin(p,
                    uart_desc[uart_num].tx,
                    uart_desc[uart_num].rx,
                    UART_PIN_NO_CHANGE,
                    UART_PIN_NO_CHANGE);
            }

            if (ret != ESP_OK) {
                hal.console->printf("UART num:%d: uart_set_pin failed: %d\n",
                    uart_desc[uart_num].port, ret);
            }

            // install drivers
            //uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            ret = uart_driver_install(p, 2*UART_HW_FIFO_LEN(p), 0, 0, nullptr, 0);
            if (ret != ESP_OK) {
                hal.console->printf("UART num:%d: uart_driver_install failed: %d\n",
                    uart_desc[uart_num].port, ret);
            }

            // support half-duplex - must be called after driver install
            if (half_duplex) {
                uart_mode_t mode = UART_MODE_RS485_HALF_DUPLEX;
                ret = uart_set_mode(p, mode);
                if (ret != ESP_OK) {
                    hal.console->printf("UART num:%d: uart_set_mode failed: %d\n",
                        uart_desc[uart_num].port, ret);
                }
            }

            // checks
            {
                uint32_t chk_baudrate;
                uart_get_baudrate(p, &chk_baudrate);
                hal.console->printf("UART num:%d: baudrate: %ld\n",
                    uart_desc[uart_num].port, chk_baudrate);

                // uint16_t chk_mode;
                // uart_get_mode(p, &chk_mode);
                // hal.console->printf("UART num:%d: mode: %d\n",
                //     uart_desc[uart_num].port, chk_mode);
            }

            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);

            _initialized = true;
        } else {
            flush();
            uart_set_baudrate(p, b);

        }
    }
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        uart_driver_delete(uart_desc[uart_num].port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = uart_desc[uart_num].port;
    uart_flush(p);
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t UARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

bool UARTDriver::set_options(uint16_t options)
{
    bool ret = true;
    _last_options = options;
    return ret;
}

uint16_t UARTDriver::get_options(void) const
{
    return _last_options;
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    do {
        count = uart_read_bytes(p, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = uart_tx_chars(p, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::_discard_input()
{
    //uart_port_t p = uart_desc[uart_num].port;
    //return uart_flush_input(p) == ESP_OK;
    return false;
}

// record timestamp of new incoming data
void IRAM_ATTR UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}


/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

}
