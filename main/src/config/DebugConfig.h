#pragma once

// ═══════════════════════════════════════════════════════════════════════
//  MASTER SWITCH
//  0 → silencia todo el debug. Poner a 0 antes del deploy en agua.
// ═══════════════════════════════════════════════════════════════════════
#define DEBUG_ENABLED  1

// ═══════════════════════════════════════════════════════════════════════
//  SECCIONES INDIVIDUALES  (solo tienen efecto si DEBUG_ENABLED == 1)
// ═══════════════════════════════════════════════════════════════════════
#define DEBUG_RADIO    0   // LoRa TX/RX, heartbeat, comandos recibidos  ← muy verboso
#define DEBUG_GPS      1   // posición, satélites, HDOP, NMEA
#define DEBUG_RC       1   // lectura canales RC
#define DEBUG_ACT      0   // servos y ESC (MCPWM write)
#define DEBUG_CTRL     1   // loop de control manual
#define DEBUG_MISN     0   // misión: carga, waypoints, estado
#define DEBUG_APP      1   // arranque y transiciones de modo
#define DEBUG_AXP      1   // init AXP192
#define DEBUG_BAT      1   // lectura ADC batería

// ═══════════════════════════════════════════════════════════════════════
//  MACROS  — eliminación en tiempo de compilación cuando están off
// ═══════════════════════════════════════════════════════════════════════
#if DEBUG_ENABLED && DEBUG_RADIO
#  define DBG_RADIO(fmt, ...) Serial.printf("[RADIO] " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_RADIO(fmt, ...) ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_GPS
#  define DBG_GPS(fmt, ...)   Serial.printf("[GPS]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_GPS(fmt, ...)   ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_RC
#  define DBG_RC(fmt, ...)    Serial.printf("[RC]    " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_RC(fmt, ...)    ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_ACT
#  define DBG_ACT(fmt, ...)   Serial.printf("[ACT]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_ACT(fmt, ...)   ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_CTRL
#  define DBG_CTRL(fmt, ...)  Serial.printf("[CTRL]  " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_CTRL(fmt, ...)  ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_MISN
#  define DBG_MISN(fmt, ...)  Serial.printf("[MISN]  " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_MISN(fmt, ...)  ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_APP
#  define DBG_APP(fmt, ...)   Serial.printf("[APP]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_APP(fmt, ...)   ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_AXP
#  define DBG_AXP(fmt, ...)   Serial.printf("[AXP]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_AXP(fmt, ...)   ((void)0)
#endif

#if DEBUG_ENABLED && DEBUG_BAT
#  define DBG_BAT(fmt, ...)   Serial.printf("[BAT]   " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG_BAT(fmt, ...)   ((void)0)
#endif
