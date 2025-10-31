/*
  ESP32 LoRa (E32 TTL-1W) Telemetry Node - Compact Binary Payload (≤50B)
  seq + timestamp for PDR/latency; fixed TX rate; FreeRTOS tasks.

  Author: Andres Perez (refactored)
  Date: 2024-09-07
*/

#include <Arduino.h>
#include "LoRa_E32.h"
#include <TinyGPSPlus.h>

// ----------------------- Pins & Serials -----------------------
static const int GPS_RX = 15;  // ESP32 RX (to GPS TX)
static const int GPS_TX =  2;  // ESP32 TX (to GPS RX)
HardwareSerial SerialGPS(1);   // UART1 for GPS

// LoRa E32 on UART2 (as you had)
LoRa_E32 e32ttl(&Serial2, 15, 21, 19); // RX AUX M0 M1 (keep your wiring)

// ----------------------- Sensors & ADC ------------------------
#define ADC_VREF_mV      3300.0f
#define ADC_RESOLUTION   4096.0f

// Current/Voltage analog pins
static const int PIN_CURR = 4;
static const int PIN_VOLT = 12;

// Thermistors / LM35 etc.
static const int PIN_T1 = 25;
static const int PIN_T2 = 26;

// Wheel speed sensor (hall)
static const int PIN_WHEEL = 13;
static const float WHEEL_CIRCUM_M = 2.0f;

// LED
static const int PIN_LED = 2;

// ------------------- Timing / Rate Control -------------------
static const uint32_t TX_INTERVAL_MS = 100;  // 10 Hz TX (paper: low latency)
static const uint32_t SENSOR_PERIOD_MS = 50; // 20 Hz sampling (paper)

// ----------------------- FreeRTOS -----------------------------
TaskHandle_t thSensors;
TaskHandle_t thRadio;
TaskHandle_t thWheel;

// -------------------- Shared Telemetry ------------------------
typedef struct {
  uint16_t seq;      // increments per TX
  uint32_t t_ms;     // millis() at packet creation
  int16_t  v_centiV; // Voltage 0.01 V
  int16_t  i_centiA; // Current 0.01 A
  int16_t  t1_deciC; // Temp 0.1 C
  int16_t  t2_deciC; // Temp 0.1 C
  uint16_t speed_cms; // wheel speed cm/s
  uint16_t dist_dm;   // distance in decimeters
  int32_t  lat_1e5;   // latitude * 1e5
  int32_t  lon_1e5;   // longitude * 1e5
} __attribute__((packed)) TelemetryFrame;

volatile uint32_t wheel_last_us = 0;
volatile uint32_t wheel_period_us = 0;
volatile uint32_t wheel_pulses = 0;

TinyGPSPlus gps;

SemaphoreHandle_t mtx;  // protect shared snapshot
TelemetryFrame snapshot = {0};
float g_voltage = 0, g_current = 0, g_t1 = 0, g_t2 = 0;
float g_speed_ms = 0, g_distance_m = 0;
double g_lat = 0, g_lon = 0;

// ---------------------- Utils: CRC16-X25 ----------------------
static uint16_t crc16_x25(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0x8408;
      else         crc = (crc >> 1);
    }
  }
  return ~crc;
}

// ------------------- ADC Helpers & Cal -----------------------
static inline float adcToVoltage(int adc) {
  // TODO: replace with your divider ratio
  // Example: divider = R2/(R1+R2); V_in = V_adc / divider
  const float divider = 0.100f;      // <-- put your real ratio
  float v_adc = (adc * ADC_VREF_mV / ADC_RESOLUTION) / 1000.0f;
  return v_adc / divider;
}

static inline float adcToCurrent(int adc) {
  // Example ACS709-ish: sensitivity 0.185 V/A, Vref=1.65 V
  const float vref = 1.65f;          // adjust with calibration
  const float sens = 0.185f;         // V/A
  float v_adc = (adc * ADC_VREF_mV / ADC_RESOLUTION) / 1000.0f;
  return (v_adc - vref) / sens;
}

static inline float adcToTempLM35(int adc) {
  // LM35 ~ 10 mV/°C
  float mV = (adc * ADC_VREF_mV / ADC_RESOLUTION);
  return mV / 10.0f;
}

// ---------------- Wheel ISR ----------------
void IRAM_ATTR wheelISR() {
  uint32_t now = micros();
  uint32_t dt = now - wheel_last_us;
  wheel_last_us = now;
  wheel_period_us = dt;
  wheel_pulses++;
}

// --------------- Sensor Task (20 Hz) ----------------
void taskSensors(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // Read ADCs
    int aV = analogRead(PIN_VOLT);
    int aI = analogRead(PIN_CURR);
    int aT1 = analogRead(PIN_T1);
    int aT2 = analogRead(PIN_T2);

    float V = adcToVoltage(aV);
    float I = adcToCurrent(aI);
    float T1 = adcToTempLM35(aT1);
    float T2 = adcToTempLM35(aT2);

    // Wheel speed/distance
    uint32_t period_us = wheel_period_us; // non-atomic read is ok-ish
    float speed_mps = 0.0f;
    if (period_us > 0) {
      float rev_per_s = 1e6f / (float)period_us;
      speed_mps = rev_per_s * WHEEL_CIRCUM_M;
    }
    static uint32_t last_pulses = 0;
    uint32_t p = wheel_pulses;
    uint32_t dp = p - last_pulses;
    last_pulses = p;
    g_distance_m += dp * WHEEL_CIRCUM_M;

    // GPS parse (non-blocking)
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    if (gps.location.isValid()) {
      g_lat = gps.location.lat();
      g_lon = gps.location.lng();
    }

    // publish to globals (protect)
    if (xSemaphoreTake(mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_voltage = V;
      g_current = I;
      g_t1 = T1;
      g_t2 = T2;
      g_speed_ms = speed_mps;
      xSemaphoreGive(mtx);
    }

    vTaskDelayUntil(&last, period);
  }
}

// ----------- Radio Task (10 Hz, build ≤50B frame) -----------
void taskRadio(void* arg) {
  static uint16_t seq = 0;
  const TickType_t period = pdMS_TO_TICKS(TX_INTERVAL_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    TelemetryFrame fr{};
    fr.seq = ++seq;
    fr.t_ms = millis();

    // snapshot under mutex
    double lat, lon;
    float V,I,T1,T2,spd,dist;
    if (xSemaphoreTake(mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
      V = g_voltage;
      I = g_current;
      T1 = g_t1;
      T2 = g_t2;
      spd = g_speed_ms;
      dist = g_distance_m;
      lat = g_lat;
      lon = g_lon;
      xSemaphoreGive(mtx);
    }

    // scale -> integers
    fr.v_centiV = (int16_t) roundf(V * 100.0f);
    fr.i_centiA = (int16_t) roundf(I * 100.0f);
    fr.t1_deciC = (int16_t) roundf(T1 * 10.0f);
    fr.t2_deciC = (int16_t) roundf(T2 * 10.0f);
    fr.speed_cms = (uint16_t) constrain((int) roundf(spd * 100.0f), 0, 65535);
    fr.dist_dm   = (uint16_t) constrain((int) roundf(dist * 10.0f), 0, 65535);
    fr.lat_1e5   = (int32_t) lrint(lat * 1e5);
    fr.lon_1e5   = (int32_t) lrint(lon * 1e5);

    // build buffer + CRC
    uint8_t buf[sizeof(TelemetryFrame) + 2];
    memcpy(buf, &fr, sizeof(TelemetryFrame));
    uint16_t crc = crc16_x25(buf, sizeof(TelemetryFrame));
    buf[sizeof(TelemetryFrame) + 0] = (uint8_t)(crc & 0xFF);
    buf[sizeof(TelemetryFrame) + 1] = (uint8_t)(crc >> 8);

    // send (fixed addressing as you had) — adjust ADDH/ADDL/CHAN to your setup
    ResponseStatus rs = e32ttl.sendFixedMessage(0x00, 0x03, 0x04, buf, sizeof(buf));
    // Debug
    Serial.print("[TX] seq="); Serial.print(fr.seq);
    Serial.print(" V="); Serial.print(V,2);
    Serial.print(" I="); Serial.print(I,2);
    Serial.print(" spd="); Serial.print(spd,2);
    Serial.print(" lat="); Serial.print(lat,5);
    Serial.print(" lon="); Serial.print(lon,5);
    Serial.print(" bytes="); Serial.print(sizeof(buf));
    Serial.print(" resp="); Serial.println(rs.getResponseDescription());

    digitalWrite(PIN_LED, !digitalWrite(PIN_LED, HIGH)); // blink
    vTaskDelayUntil(&last, period);
  }
}

// --------------- Wheel Task (optional prints) ---------------
void taskWheel(void* arg) {
  for (;;) {
    // could add health checks or diagnostics
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_WHEEL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL), wheelISR, RISING);

  // ADC resolution defaults to 12-bit on ESP32; no need to set
  // Start GPS UART
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // LoRa E32
  Serial2.begin(9600, SERIAL_8N1); // UART2 default pins per your wiring
  e32ttl.begin();

  mtx = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreatePinnedToCore(taskSensors, "sensors", 4096, NULL, 10, &thSensors, 0);
  xTaskCreatePinnedToCore(taskRadio,   "radio",   4096, NULL,  9, &thRadio,   1);
  xTaskCreatePinnedToCore(taskWheel,   "wheel",   2048, NULL,  5, &thWheel,   1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // idle; RTOS tasks do the work
}
