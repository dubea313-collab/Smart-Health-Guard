// ESP32 User Device - LoRa SOS Sender
// Requires: LoRa (Sandeep Mistry), TinyGPSPlus
// Packet format:
// [DeviceID(4B)][MsgType(1B)=0x01][Lat(4B)][Lon(4B)][Alt(2B)][Batt%(1B)][Seq(1B)][CRC16(2B)]

// --------- CONFIGURATION ---------
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

#define LORA_SS    18  // NSS / CS
#define LORA_SCK    5
#define LORA_MOSI  27
#define LORA_MISO  19
#define LORA_RST   14
#define LORA_DIO0  26

#define LORA_FREQ  868E6 // use 868E6 for India; change if needed

// GPS UART (HardwareSerial 2)
#define GPS_RX_PIN 16  // GPS TX -> ESP32 RX2 (GPIO16)
#define GPS_TX_PIN 17  // GPS RX -> ESP32 TX2 (GPIO17) - not required for NMEA receive

// SOS button and indicator
#define SOS_PIN    4    // Button wired to GND; INPUT_PULLUP, pressed = LOW
#define LED_PIN    2    // Status LED

// Battery sense (optional) - connect through divider to keep <= 3.3V
#define BAT_PIN    35   // ADC1 channel
#define BAT_MAX_VOLTAGE 4.2  // max battery voltage (LiPo)
#define BAT_MIN_VOLTAGE 3.3  // min usable (approx)

// Device ID (4 bytes). Change this to your unique device id.
const uint32_t DEVICE_ID = 0xA1B2C3D4UL;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

volatile bool sosPressed = false;
uint8_t seqNum = 0;
int lastFixAge = 99999;
double lastLat = 0.0, lastLon = 0.0;
int32_t lastAlt = 0;

// --------- UTILS: CRC16-CCITT ---------
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// --------- INTERRUPT HANDLER ---------
void IRAM_ATTR handleSOS() {
  sosPressed = true;
}

// --------- BUILD PACKET (binary) ---------
void buildPacket(uint8_t *buf, size_t &outLen, uint8_t msgType, double lat, double lon, int32_t altM, uint8_t battPct, uint8_t seq) {
  // Pack fields into buffer as described
  // DeviceID 4B (big-endian)
  buf[0] = (DEVICE_ID >> 24) & 0xFF;
  buf[1] = (DEVICE_ID >> 16) & 0xFF;
  buf[2] = (DEVICE_ID >> 8) & 0xFF;
  buf[3] = (DEVICE_ID) & 0xFF;
  buf[4] = msgType;

  // lat, lon -> int32 microdegrees (deg * 1e7)
  int32_t lat_i = (int32_t)round(lat * 1e7);
  int32_t lon_i = (int32_t)round(lon * 1e7);

  buf[5] = (lat_i >> 24) & 0xFF;
  buf[6] = (lat_i >> 16) & 0xFF;
  buf[7] = (lat_i >> 8) & 0xFF;
  buf[8] = (lat_i) & 0xFF;

  buf[9] = (lon_i >> 24) & 0xFF;
  buf[10] = (lon_i >> 16) & 0xFF;
  buf[11] = (lon_i >> 8) & 0xFF;
  buf[12] = (lon_i) & 0xFF;

  // Altitude int16 meters
  int16_t alt_s = (int16_t)altM;
  buf[13] = (alt_s >> 8) & 0xFF;
  buf[14] = (alt_s) & 0xFF;

  // Battery percent and seq
  buf[15] = battPct;
  buf[16] = seq;

  // Compute CRC on first 17 bytes
  uint16_t crc = crc16_ccitt(buf, 17);
  buf[17] = (crc >> 8) & 0xFF;
  buf[18] = (crc) & 0xFF;

  outLen = 20; // total bytes
}

// --------- READ BATTERY (approx %) ---------
uint8_t readBatteryPercent() {
  // Read ADC raw (0 - 4095) on ESP32 ADC1
  uint32_t raw = analogRead(BAT_PIN);
  // Convert raw to voltage: V = raw/4095 * Vref (approx 3.3)
  float v = (raw / 4095.0f) * 3.3f;
  // If using voltage divider, multiply accordingly (e.g., factor = (R1+R2)/R2)
  // Adjust this factor to match your divider. For now assume direct reading of battery <=3.3 (for single-cell through divider)
  // Example assume divider scaling factor:
  const float DIVIDER_FACTOR = 2.0f; // change to your actual divider ratio
  float battV = v * DIVIDER_FACTOR;
  if (battV > BAT_MAX_VOLTAGE) battV = BAT_MAX_VOLTAGE;
  if (battV < 0) battV = 0;
  // Map to percent
  int pct = (int)round( (battV - BAT_MIN_VOLTAGE) / (BAT_MAX_VOLTAGE - BAT_MIN_VOLTAGE) * 100.0f );
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(SOS_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // attach interrupt
  attachInterrupt(digitalPinToInterrupt(SOS_PIN), handleSOS, FALLING);

  // GPS serial
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // LoRa init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS); // SCK, MISO, MOSI, SS
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin((long)LORA_FREQ)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (1) { digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200); }
  }
  Serial.println("LoRa init OK");
  LoRa.setSpreadingFactor(10); // tune: 7..12 (higher SF => longer range, lower rate)
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(17); // dBm (0..20) - check module limits

  Serial.println("Setup complete.");
}

unsigned long lastGpsPrint = 0;
void loop() {
  // feed GPS parser
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Optional: indicate GPS lock
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    lastAlt = gps.altitude.meters();
    lastFixAge = gps.location.age();
  }

  if (sosPressed) {
    // debounce / clear
    sosPressed = false;
    digitalWrite(LED_PIN, HIGH);

    // Try to get a fresh fix for up to 10 seconds
    unsigned long t0 = millis();
    unsigned long timeout = 10000;
    bool gotFix = false;
    while (millis() - t0 < timeout) {
      while (gpsSerial.available()) gps.encode(gpsSerial.read());
      if (gps.location.isValid() && gps.location.age() < 2000) { // fresh fix
        lastLat = gps.location.lat();
        lastLon = gps.location.lng();
        lastAlt = (int32_t)gps.altitude.meters();
        gotFix = true;
        break;
      }
      delay(200);
    }

    // Use last known if no fresh fix
    if (!gotFix) {
      Serial.println("No fresh GPS fix; using last known (if any).");
    } else {
      Serial.println("GPS fix obtained.");
    }

    // read battery
    uint8_t batt = readBatteryPercent();

    // build packet
    uint8_t pkt[32];
    size_t pktLen = 0;
    buildPacket(pkt, pktLen, 0x01, lastLat, lastLon, lastAlt, batt, seqNum);

    // transmit with retries
    const int MAX_RETRIES = 3;
    bool sentOK = false;
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
      Serial.printf("Sending SOS attempt %d: seq=%u\n", attempt+1, seqNum);
      LoRa.beginPacket();
      LoRa.write(pkt, pktLen);
      LoRa.endPacket(true); // async? true waits for TX done
      // Wait a short time for RSSI etc.
      delay(500);

      // Optionally listen for ACK (not implemented) - here we just assume send succeeded
      // You might implement ACK by waiting for a small RX window and checking incoming packets.
      sentOK = true;
      if (sentOK) break;
      delay(500 + random(0, 500)*(attempt+1));
    }

    if (sentOK) {
      Serial.println("SOS sent.");
      seqNum++;
      // indicate success via LED flash
      for (int i=0;i<3;i++) { digitalWrite(LED_PIN, LOW); delay(150); digitalWrite(LED_PIN, HIGH); delay(150); }
      digitalWrite(LED_PIN, LOW);
    } else {
      Serial.println("SOS failed after retries.");
      // indicate failure via long blink
      for (int i=0;i<6;i++) { digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200); }
    }
  }

  // small idle delay
  delay(50);
}
