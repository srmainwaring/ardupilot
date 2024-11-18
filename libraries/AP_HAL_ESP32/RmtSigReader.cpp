#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

//#include <stdio.h>

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

void RmtSigReader::init()
{
    rx_chan = NULL;

    // Resource allocation
    rmt_rx_channel_config_t rx_chan_config;
    rx_chan_config.gpio_num = HAL_ESP32_RCIN;       // GPIO number
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;   // select source clock
    rx_chan_config.resolution_hz = 1 * 1000 * 1000; // 1 MHz tick resolution, i.e., 1 tick = 1 µs
    rx_chan_config.mem_block_symbols = 64;          // memory block size, 64 * 4 = 256 Bytes 
    rx_chan_config.intr_priority = 0;               // default interrupt priority
    rx_chan_config.flags.invert_in = 0;             // do not invert input signal
    rx_chan_config.flags.with_dma = 0;              // do not need DMA backend
    rx_chan_config.flags.io_loop_back = 0;          // do not enable loopback
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    // Carrier demodulation
    rmt_carrier_config_t rx_carrier_config;
    rx_carrier_config.frequency_hz = 0;               // carrier, should be smaller than the transmitter's carrier frequency
    rx_carrier_config.duty_cycle = 100;               // carrier wave duty cycle (0~100%)
    rx_carrier_config.flags.polarity_active_low = 1;  // polarity of carrier, by default it's modulated to base signal's high level
    rx_carrier_config.flags.always_on = 1;            // carrier can always exist even there's not transfer undergoing
    ESP_ERROR_CHECK(rmt_apply_carrier(rx_chan, &rx_carrier_config));

    // Callbacks
    // rmt_rx_event_callbacks_t::on_recv_done
    // rmt_rx_register_event_callbacks()
    // rmt_rx_done_event_data_t::received_symbols
    // rmt_rx_done_event_data_t::num_symbols
    // rmt_rx_done_event_data_t::is_last
    // rmt_receive_config_t::extra_flags::en_partial_rx

    // rmt_receive(buffer_size)


    // Enable channel
    // rmt_enable(rx_chan)
    // rmt_disenable(rx_chan)

    // Initiate rx transaction
    // const uint32_t tick_ns = 1e9 / rx_chan_config.resolution_hz;
    const uint32_t signal_range_min_ns = 8* 1000; // 8 * tick_ns;
    const uint32_t signal_range_max_ns = 3000 * 1000; //idle_threshold * 1000;
    rmt_receive_config_t rx_receive_config;
    rx_receive_config.signal_range_min_ns = signal_range_min_ns;  // A pulse whose width is smaller than this threshold will be treated as glitch and ignored 
    rx_receive_config.signal_range_max_ns = signal_range_max_ns;  // RMT will stop receiving if one symbol level has kept more than `signal_range_max_ns`
    rx_receive_config.flags.en_partial_rx = 1;                    // Set this flag if the incoming data is very long.

    rmt_symbol_word_t raw_symbols[64];
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rx_receive_config));
    // rmt_rx_done_event_data_t rx_data;

    // static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
    // {
    //     BaseType_t high_task_wakeup = pdFALSE;
    //     QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    //     // send the received RMT symbols to the parser task
    //     xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    //     // return whether any task is woken up
    //     return high_task_wakeup == pdTRUE;
    // }

    // QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    // xQueueReceive(receive_queue, &rx_data, portMAX_DELAY);

    // Invoked multiple times per callback
    // rmt_rx_event_callbacks_t::on_recv_done
    // rmt_symbol_word_t

    // Power management

    // IRAM safety

    // Thread safety

#if 0
    // rmt_rx_channel_config_t config;
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_RX;
    config.channel = RMT_CHANNEL_4; // On S3, Channel 0 ~ 3 (TX channel) are dedicated to sending signals. Channel 4 ~ 7 (RX channel) are dedicated to receiving signals, so this pin choice is compatible with both.
    config.clk_div = 80;   //80MHZ APB clock to the 1MHZ target frequency
    config.gpio_num = HAL_ESP32_RCIN;
    config.mem_block_num = 4; //each block could store 64 pulses
    config.flags = 0;
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 8;
    config.rx_config.idle_threshold = idle_threshold;

    rmt_config(&config);
    rmt_driver_install(config.channel, max_pulses * 8, 0);
    rmt_get_ringbuf_handle(config.channel, &handle);
    rmt_rx_start(config.channel, true);
#endif
}

bool RmtSigReader::add_item(uint32_t duration, bool level)
{
#if 0
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
#endif
    return false;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
#if 0
    if (item == nullptr) {
        item = (rmt_item32_t*) xRingbufferReceive(handle, &item_size, 0);
        item_size /= 4;
        current_item = 0;
    }
    if (item == nullptr) {
        return false;
    }
    bool buffer_empty = (current_item == item_size);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration0, item[current_item].level0);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration1, item[current_item].level1);
    current_item++;
    if (buffer_empty) {
        vRingbufferReturnItem(handle, (void*) item);
        item = nullptr;
    }
    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
#endif
    return false;
}

#endif
