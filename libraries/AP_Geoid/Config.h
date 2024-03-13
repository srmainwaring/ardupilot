#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

// These are macros which affect the building of the library
#define GEOGRAPHICLIB_WORDS_BIGENDIAN 0

// 1 float, 2 double
#if HAL_HAVE_HARDWARE_DOUBLE
#define GEOGRAPHICLIB_PRECISION 2
#else
#define GEOGRAPHICLIB_PRECISION 1 
#endif

// Disable caching in Geoid
#define GEOGRAPHICLIB_GEOID_ENABLE_CACHE 0
