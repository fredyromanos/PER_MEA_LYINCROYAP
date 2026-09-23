#pragma once

// Minimal debug macros for the GPS test (subset of the firmware DebugConfig.h).
#define DEBUG_ENABLED  1
#define DEBUG_GPS      1
#define DEBUG_AXP      1

#if DEBUG_ENABLED && DEBUG_GPS
#  define DBG_GPS(fmt, ...)   Serial.printf("[GPS]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_GPS(fmt, ...)   ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_AXP
#  define DBG_AXP(fmt, ...)   Serial.printf("[AXP]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_AXP(fmt, ...)   ((void)0)
#endif
