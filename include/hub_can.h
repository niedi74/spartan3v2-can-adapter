#pragma once
// [CAN] setupCan()/updateCan(): Spartan-RX (0x400) + Cockpit-TX (0x510) auf dem
// gemeinsamen TWAI-Controller. 1:1 aus main.cpp, an Originalstelle included.
// [CAN-DEV] Pins/Bitrate/IDs kommen aus den *Cfg-Laufzeitvariablen (NVS, Default =
// Build-Flags) -- Aenderung braucht Neustart, siehe loadHubFeatures()/canConfigChanged.
static twai_timing_config_t canTimingFromKbps(uint16_t kbps)
{
  switch (kbps) {
    case 1000: return TWAI_TIMING_CONFIG_1MBITS();
    case 250:  return TWAI_TIMING_CONFIG_250KBITS();
    case 125:  return TWAI_TIMING_CONFIG_125KBITS();
    default:   return TWAI_TIMING_CONFIG_500KBITS();
  }
}

void setupCan()
{
#if ENABLE_SPARTAN_CAN
  if (!hubFeatCan) {
    Serial.println("CAN:         disabled (Dev-Tab)");
    return;
  }
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(canTxPinCfg),
      static_cast<gpio_num_t>(canRxPinCfg),
      TWAI_MODE_NORMAL);
  twai_timing_config_t timing = canTimingFromKbps(canBitrateKbps);
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
    Serial.println("CAN start failed (install)");
    return;
  }
  if (twai_start() != ESP_OK) {
    // Treiber ist installiert, aber nicht gestartet -- ohne Uninstall wuerde
    // ein erneuter setupCan()-Versuch (z.B. nach Pin-Aenderung) auf einen
    // bereits belegten Treiber treffen statt sauber neu zu installieren.
    Serial.println("CAN start failed (start) -- Treiber wird deinstalliert");
    twai_driver_uninstall();
    return;
  }

  canReady = true;
  Serial.printf("CAN:         %u kbit/s RX=%u TX=%u Spartan ID=0x%03X Cockpit ID=0x%03X\n",
                canBitrateKbps, canRxPinCfg, canTxPinCfg, spartanCanIdCfg, cockpitCanIdCfg);
#endif
}

// [CAN-DEV] Laufzeit-Aus (Dev-Tab-Schalter) -- Treiber sauber stoppen/deinstallieren,
// damit die Pins frei werden und kein halbtoter TWAI-Zustand haengen bleibt.
void stopCan()
{
#if ENABLE_SPARTAN_CAN
  if (!canReady) return;
  twai_stop();
  twai_driver_uninstall();
  canReady = false;
  Serial.println("CAN:         disabled (Dev-Tab) -> Treiber gestoppt");
#endif
}

void updateCan()
{
#if ENABLE_SPARTAN_CAN
  if (!canReady) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastCanStatusMs >= 1000) {
    lastCanStatusMs = now;
    if (twai_get_status_info(&canStatus) == ESP_OK) {
      if (canStatus.state == TWAI_STATE_BUS_OFF) {
        // Initiate hardware recovery (waits for 128 recessive bits, non-blocking).
        // State will transition to TWAI_STATE_STOPPED when complete.
        if (twai_initiate_recovery() == ESP_OK) {
          Serial.println("CAN:         Bus-Off → recovery initiated");
        }
        if (canStatusErrors < UINT32_MAX) canStatusErrors++;
      } else if (canStatus.state == TWAI_STATE_STOPPED) {
        // Recovery completed — restart the driver.
        if (twai_start() == ESP_OK) {
          Serial.println("CAN:         recovered from Bus-Off, restarted");
        } else {
          if (canStatusErrors < UINT32_MAX) canStatusErrors++;
        }
      } else if (canStatus.state != TWAI_STATE_RUNNING ||
                 canStatus.tx_error_counter > 127 ||
                 canStatus.rx_error_counter > 127) {
        if (canStatusErrors < UINT32_MAX) canStatusErrors++;
      }
    }
  }

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.identifier != spartanCanIdCfg || message.data_length_code < 4) {
      continue;
    }

    const uint16_t rawLambda = (static_cast<uint16_t>(message.data[0]) << 8) | message.data[1];
    SpartanReading fresh;
    fresh.lambda = rawLambda / 1000.0f;
    fresh.temperatureC = static_cast<uint16_t>(message.data[2]) * 10;
    fresh.status = message.data[3];
    fresh.receivedMs = millis();
    fresh.valid = true;
    fresh.fromCan = true;
    fresh.fromDemo = false;
    storeReading(fresh);
    digitalWrite(STATUS_LED_PIN, fresh.status == 3 ? HIGH : LOW);
  }

  // --- Cockpit-Frame an Display(s) senden: 0x510, 8 Byte, ~10 Hz ---
  // Gleicher TWAI-Controller wie der Spartan-RX (NORMAL-Mode -> sendefaehig).
  if (now - lastCockpitCanTxMs >= cockpitCanTxIntervalMsCfg) {
    lastCockpitCanTxMs = now;
    const SpartanReading snap = readingSnapshot();
    uint16_t rpm = 0;
    float    advance = 0.0f;
    uint8_t  mapKpa = 0;
    bool     tuneFresh = false;
    if (hubFeatBle123) {
      const TuneSnapshot tune = tuneSnapshot();
      tuneFresh = tune.lastRxMs != 0 && (now - tune.lastRxMs) <= 3000;
      rpm       = static_cast<uint16_t>(tune.rpm);
      advance   = tune.advance;
      mapKpa    = static_cast<uint8_t>(tune.map);
    }
    const uint16_t lambdaX1000 = snap.valid ? static_cast<uint16_t>(snap.lambda * 1000.0f + 0.5f) : 0;
    const int16_t  advX10      = static_cast<int16_t>(advance * 10.0f + (advance >= 0 ? 0.5f : -0.5f));
    uint8_t flags = 0;
    if (snap.valid) flags |= 0x01;   // Lambda gueltig
    if (tuneFresh)  flags |= 0x02;   // 123-Daten frisch (rpm/adv/map gueltig)

    twai_message_t tx = {};
    tx.identifier        = cockpitCanIdCfg;
    tx.data_length_code  = 8;
    tx.data[0] = static_cast<uint8_t>(lambdaX1000 >> 8);
    tx.data[1] = static_cast<uint8_t>(lambdaX1000 & 0xFF);
    tx.data[2] = static_cast<uint8_t>(rpm >> 8);
    tx.data[3] = static_cast<uint8_t>(rpm & 0xFF);
    tx.data[4] = static_cast<uint8_t>((advX10 >> 8) & 0xFF);
    tx.data[5] = static_cast<uint8_t>(advX10 & 0xFF);
    tx.data[6] = mapKpa;
    tx.data[7] = flags;
    if (twai_transmit(&tx, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (cockpitCanTxCount < UINT32_MAX) cockpitCanTxCount++;
    } else {
      if (cockpitCanTxErrors < UINT32_MAX) cockpitCanTxErrors++;
    }
  }
#endif
}
