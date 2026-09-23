/*
  SeaDrone Ground Station — Post-Claude
  ======================================
  Runs on a second LilyGO T-Beam V1.1 connected to a PC via USB.
  Bridges USB serial (115200 baud) <-> LoRa 433 MHz.

  RX: forwards drone heartbeat JSON to serial as a single clean JSON line.
      No prefix — serial_link.py can parse it directly.

  TX: accepts raw JSON from IHM (serial_link.py) and transmits via LoRa.
      Also accepts shorthand commands for manual testing in the serial monitor.

  Shorthand commands (serial monitor only):
    navigate                → start autonomous mission
    stop                    → stop mission
    restart                 → reboot the drone
    wind-obs                → start wind observation (Phase 5)
    wind <0-359>            → set wind direction manually (degrees)
    home <lat,lon>          → set home / return point
    wpt <lat,lon,lat,lon,…> → load waypoints in Linear mode
    {…raw JSON…}            → pass through unchanged (used by IHM)

  Note on wpt: LoRa packets are max 255 bytes. With the JSON wrapper (~80 chars)
  you have ~170 chars for coordinates — roughly 8 waypoints at full precision.
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <axp20x.h>

// T-Beam V1.1 LoRa SPI pins
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_IRQ   26
#define LORA_RST   23      // GPIO23 — present on T-Beam V1.1 PCB
#define LORA_BAND  433E6

// T-Beam V1.1 I2C bus (AXP192 power management)
#define I2C_SDA    21
#define I2C_SCL    22

static AXP20X_Class axp;

static void sendJson(const char* json);
static void handleInput(String& line);

// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // 1. AXP192: enable LoRa power rail (LDO2) before touching SPI.
  //    On T-Beam V1.1, SX1276 is powered by LDO2 — without this step
  //    LoRa.begin() will fail.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_PASS) {
    axp.setLDO2Voltage(3300);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LoRa SX1276
    axp.setDCDC1Voltage(3300);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // 3.3V board rail
    Serial.println("[AXP]  OK");
  } else {
    Serial.println("[AXP]  WARN: init failed — LoRa may not power up");
  }

  // 2. LoRa radio
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (LoRa.begin(LORA_BAND) != 1) {
    Serial.println("[LORA] ERROR: init failed — check SPI wiring / AXP192 LDO2");
    for (;;);
  }
  Serial.println("[LORA] OK — 433 MHz, listening");
}

// ==========================================
void loop() {
  // --- RX: LoRa packet → Serial ---
  // On injecte le RSSI (mesuré ici, au sol) dans l'objet "message" du JSON,
  // juste avant les deux accolades finales "}}". serial_link.py parse ensuite
  // la ligne normalement. Le RSSI ne transite pas par LoRa : il ne compte donc
  // pas dans la limite des 255 octets du paquet.
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String msg = LoRa.readString();
    int rssi = LoRa.packetRssi();
    msg.trim();

    int end = msg.lastIndexOf("}}");
    if (msg.startsWith("{") && end > 0) {
      Serial.println(msg.substring(0, end) + ",\"rssi\":" + String(rssi) + "}}");
    } else {
      Serial.println(msg);   // non-JSON : transmis tel quel
    }
  }

  // --- TX: Serial line → LoRa ---
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    handleInput(line);
  }
}

// ==========================================
static void sendJson(const char* json) {
  LoRa.beginPacket();
  LoRa.print(json);
  LoRa.endPacket();
}

// ==========================================
static void handleInput(String& line) {

  // Raw JSON pass-through — used by IHM serial_link.py
  if (line.startsWith("{")) {
    char buf[256];
    line.toCharArray(buf, sizeof(buf));
    sendJson(buf);
    return;
  }

  // Shorthand commands for manual testing via serial monitor

  if (line.equalsIgnoreCase("navigate")) {
    sendJson("{\"origin\":\"server\",\"type\":\"command\",\"message\":\"navigate\"}");

  } else if (line.equalsIgnoreCase("stop")) {
    sendJson("{\"origin\":\"server\",\"type\":\"command\",\"message\":\"stop\"}");

  } else if (line.equalsIgnoreCase("restart")) {
    sendJson("{\"origin\":\"server\",\"type\":\"command\",\"message\":\"restart\"}");

  } else if (line.equalsIgnoreCase("wind-obs") ||
             line.equalsIgnoreCase("wind-observation")) {
    sendJson("{\"origin\":\"server\",\"type\":\"command\",\"message\":\"wind-observation\"}");

  } else if (line.startsWith("wind ") || line.startsWith("wind\t")) {
    int deg = line.substring(5).toInt();
    if (deg < 0 || deg > 359) {
      Serial.println("[ERR] wind: value must be 0-359");
      return;
    }
    char buf[96];
    snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"wind-command\":{\"value\":%d}}}",
      deg);
    sendJson(buf);

  } else if (line.startsWith("home ") || line.startsWith("home\t")) {
    String args = line.substring(5);
    args.trim();
    int comma = args.indexOf(',');
    if (comma < 0) {
      Serial.println("[ERR] home: format is  home <lat,lon>");
      return;
    }
    double lat = args.substring(0, comma).toDouble();
    double lon = args.substring(comma + 1).toDouble();
    char buf[128];
    snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"home\":{\"lat\":%.6f,\"lon\":%.6f}}}",
      lat, lon);
    sendJson(buf);

  } else if (line.startsWith("wpt ") || line.startsWith("wpt\t")) {
    String pts = line.substring(4);
    pts.trim();
    if (pts.length() == 0) {
      Serial.println("[ERR] wpt: no coordinates given");
      return;
    }
    int commas = 0;
    for (size_t i = 0; i < pts.length(); i++) if (pts[i] == ',') commas++;
    int n = (commas + 1) / 2;
    if (n < 1) {
      Serial.println("[ERR] wpt: need at least one lat,lon pair");
      return;
    }
    char buf[256];
    int written = snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"waypoints\":{\"number\":%d,\"points\":\"%s\"}}}",
      n, pts.c_str());
    if (written >= (int)sizeof(buf)) {
      Serial.println("[ERR] wpt: command too long for LoRa packet (max ~8 waypoints at full precision)");
      return;
    }
    sendJson(buf);

  } else {
    Serial.print("[ERR] unknown command: ");
    Serial.println(line);
    Serial.println("      try: navigate | stop | restart | wind-obs | wind <deg>");
    Serial.println("           home <lat,lon> | wpt <lat,lon,...> | {raw JSON}");
  }
}
