#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "spartan_cockpit_frame.h"

#ifndef ESP_NOW_WIFI_CHANNEL
#define ESP_NOW_WIFI_CHANNEL 6
#endif

static volatile uint16_t g_lastSeq = 0;
static volatile uint32_t g_rxCount = 0;
static volatile float g_lambda = 0.0f;
static volatile uint16_t g_rpm = 0;
static volatile int16_t g_advance_x10 = 0;

static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != static_cast<int>(kSpartanCockpitFrameSize)) {
    return;
  }
  SpartanCockpitFrame frame;
  memcpy(&frame, data, kSpartanCockpitFrameSize);
  if (!spartanCockpitFrameValid(frame)) {
    return;
  }
  g_lastSeq = frame.seq;
  g_rxCount++;
  if (frame.flags & kSpartanFlagLambdaValid) {
    g_lambda = frame.lambda_x1000 / 1000.0f;
  }
  g_rpm = frame.rpm;
  g_advance_x10 = frame.advance_x10;
}

static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  if (esp_wifi_set_channel(ESP_NOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("[ESP-NOW] channel set failed");
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return false;
  }
  if (esp_now_register_recv_cb(onEspNowRecv) != ESP_OK) {
    Serial.println("[ESP-NOW] recv cb failed");
    return false;
  }
  Serial.printf("[ESP-NOW] listening ch%d\n", ESP_NOW_WIFI_CHANNEL);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[Waveshare] Spartan cockpit ESP-NOW client (seed)");
  initEspNow();
}

void loop() {
  static uint32_t lastLog = 0;
  const uint32_t now = millis();
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.printf("[cockpit] rx=%lu seq=%u rpm=%u adv=%.1f lambda=%.3f\n",
                  static_cast<unsigned long>(g_rxCount),
                  g_lastSeq,
                  g_rpm,
                  g_advance_x10 / 10.0f,
                  g_lambda);
  }
  delay(50);
}
