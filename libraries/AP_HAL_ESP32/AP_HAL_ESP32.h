#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

#include "HAL_ESP32_Class.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
    #undef SOC_RMT_SUPPORT_RX_PINGPONG
    #define SOC_RMT_SUPPORT_RX_PINGPONG 0
#endif
