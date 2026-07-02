#pragma once
// [ESP-NOW] Cockpit-Broadcast an M5/Waveshare (inkl. #if-Fallbacks). 1:1 aus
// main.cpp, an Originalstelle included -- keine Logikaenderung.
#if ENABLE_ESP_NOW_HUB
uint8_t espNowEffectiveChannel()
{
#if ENABLE_WEB_GUI
  if (hubEspNowChannelPref >= 1 && hubEspNowChannelPref <= 14) {
    return hubEspNowChannelPref;
  }
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t channel = WiFi.channel();
    if (channel > 0 && channel <= 14) {
      return channel;
    }
  }
#endif
  return ESP_NOW_WIFI_CHANNEL;
}

void teardownEspNowHub()
{
  if (!espNowReady) {
    return;
  }
  esp_now_del_peer(kEspNowBroadcastAddr);
  esp_now_deinit();
  espNowReady = false;
  espNowActiveChannel = 0;
}

void onEspNowSend(const uint8_t *mac, esp_now_send_status_t status)
{
  (void)mac;
  if (status == ESP_NOW_SEND_SUCCESS) {
    if (espNowTxCount < UINT32_MAX) {
      espNowTxCount++;
    }
  } else if (espNowTxFailCount < UINT32_MAX) {
    espNowTxFailCount++;
  }
}

void setupEspNowHub()
{
  // Emulator-Modus: ESP-NOW BLEIBT an, damit M5 und Touch die simulierten Daten
  // per Funk bekommen (Fan-out). Der Hub ist hier BLE-Peripherie (advertised
  // 123\TUNE+) UND ESP-NOW-Sender — die Displays nutzen ESP-NOW, niemand muss
  // sich per BLE-Direkt mit dem Hub-Emu verbinden.
#if ENABLE_WEB_GUI
  if (WiFi.getMode() == WIFI_OFF) {
    Serial.println("ESP-NOW:     WiFi not ready yet");
    return;
  }
#endif

  const uint8_t channel = espNowEffectiveChannel();
  if (espNowReady && espNowActiveChannel == channel) {
    return;
  }
  if (espNowReady) {
    teardownEspNowHub();
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW:     init failed");
    logHubEvent("espnow_tx", "init|fail");
    return;
  }
  esp_now_register_send_cb(onEspNowSend);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kEspNowBroadcastAddr, sizeof(kEspNowBroadcastAddr));
  // channel=0 => IMMER auf dem aktuellen Funkkanal senden. Verhindert den
  // Fehler "Peer channel is not equal to the home channel, send fail!", der
  // ALLE Sends scheitern ließ, wenn AP/STA-Kanal vom Peer-Kanal abwich (z.B.
  // nach Profilwechsel). Die Clients empfangen auf dem Kanal des Hub-Funks
  // (im Bus = AP-Kanal 6).
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW:     broadcast peer add failed");
    esp_now_deinit();
    logHubEvent("espnow_tx", "peer|fail");
    return;
  }

  espNowReady = true;
  espNowActiveChannel = channel;
  Serial.printf("ESP-NOW:     cockpit broadcast ready on channel %u%s\n",
                channel,
                channel == ESP_NOW_WIFI_CHANNEL ? "" : " (STA home WiFi)");
  char espNowDetail[16];
  snprintf(espNowDetail, sizeof(espNowDetail), "ready|ch%u", channel);
  logHubEvent("espnow_tx", espNowDetail);
}

void updateEspNowHub()
{
  // Emulator-Modus: ESP-NOW BLEIBT aktiv (Fan-out der simulierten Daten an
  // M5 + Touch). Nur wenn der Nutzer ESP-NOW bewusst abschaltet, ist es aus.
  if (!hubFeatEspNow) {
    if (espNowReady) {
      teardownEspNowHub();
    }
    return;
  }
  const uint8_t channel = espNowEffectiveChannel();
  if (!espNowReady || espNowActiveChannel != channel) {
    setupEspNowHub();
  }
  if (!espNowReady) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastEspNowSendMs < kEspNowSendIntervalMs) {
    return;
  }
  lastEspNowSendMs = now;

#if ENABLE_BLE_HUB
  const SpartanReading snapshot = readingSnapshot();
  uint16_t rpm = 0;
  float advance = 0.0f;
  uint8_t map = 0;
  float tuneVolt = 0.0f;
  float tuneTemp = 0.0f;
  float tuneCoil = 0.0f;
  bool tuneFresh = false;
  bool tuneConn = false;
  if (hubFeatBle123) {
    const TuneSnapshot tune = tuneSnapshot();
    tuneFresh = tune.lastRxMs != 0 && (now - tune.lastRxMs) <= 3000;
    tuneConn = tuneConnected;
    rpm = static_cast<uint16_t>(tune.rpm);
    advance = tune.advance;
    map = static_cast<uint8_t>(tune.map);
    tuneVolt = tune.voltage;
    tuneTemp = tune.temperature;
    tuneCoil = tune.coilCurrent;
  }

  SpartanCockpitFrame frame;
  spartanCockpitEncode(&frame,
                       espNowSeq++,
                       snapshot.lambda,
                       snapshot.valid,
                       rpm,
                       advance,
                       map,
                       snapshot.status,
                       tuneFresh,
                       tuneConn,
                       tuneVolt,
                       tuneTemp,
                       tuneCoil);
  esp_now_send(kEspNowBroadcastAddr, reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
#endif
}
#else
void setupEspNowHub() {}
void updateEspNowHub() {}
#endif
