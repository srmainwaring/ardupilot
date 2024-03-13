// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for "normal" builds - typically vehicle binaries but
// also examples.

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED (HAL_PWM_COUNT>0)
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

// a similar define is present in AP_HAL_Boards.h:
#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef AP_GEOID_AVAILABLE
// enable geoid only if there's an SD card available:
#define AP_GEOID_AVAILABLE HAL_OS_FATFS_IO
#endif

#if AP_GEOID_AVAILABLE
#ifndef HAL_BOARD_GEOID_DIRECTORY
#define HAL_BOARD_GEOID_DIRECTORY "/APM/GEOID"
#endif
#endif  // AP_GEOID_AVAILABLE

#ifndef AP_TERRAIN_AVAILABLE
// enable terrain only if there's an SD card available:
#define AP_TERRAIN_AVAILABLE HAL_OS_FATFS_IO
#endif

#if AP_TERRAIN_AVAILABLE
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif
#endif  // AP_TERRAIN_AVAILABLE
