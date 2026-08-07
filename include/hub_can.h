#pragma once
// [CAN] setupCan()/updateCan(): Spartan-RX (0x400) + Cockpit-TX (0x510) auf dem
// gemeinsamen TWAI-Controller. 1:1 aus main.cpp, an Originalstelle included.
// [CAN-DEV] Pins/Bitrate/IDs kommen aus den *Cfg-Laufzeitvariablen (NVS, Default =
// Build-Flags) -- Aenderung braucht Neustart, siehe loadHubFeatures()/canConfigChanged.
//
// [COCKPIT-WIRE-FORMAT] Der 0x510-Cockpit-Frame ist ein eigenes, schlankes 8-Byte-
// Format (NICHT das reichhaltigere SpartanCockpitFrame aus spartan_cockpit_frame.h --
// das ist 17 Byte und passt nicht in einen einzelnen klassischen CAN-Frame mit
// DLC=8; es wird aktuell nirgends aufgerufen). Byte-Layout, big-endian:
//   [0-1] lambda_x1000 (uint16, 0 wenn keine Sonde)   [2-3] rpm (uint16)
//   [4-5] advance_x10 (int16)                          [6]   map (uint8, kPa)
//   [7]   flags: Bit0=kCockpitFlagLambdaValid, Bit1=kCockpitFlagTuneFresh,
//                Bits2-3=status_code (0=ERR/1=WAIT/2=HEAT/3=OK, siehe
//                docs/lambda-status-logik.md), Bit4=kCockpitFlagRealCan,
//                Bits5-7 reserviert/0.
constexpr uint8_t kCockpitFlagLambdaValid = 0x01;
constexpr uint8_t kCockpitFlagTuneFresh   = 0x02;
constexpr uint8_t kCockpitStatusBitShift  = 2;
constexpr uint8_t kCockpitStatusBitMask   = 0x03;  // << kCockpitStatusBitShift
// [SICHERHEIT] Bit4 = 1 NUR wenn der Lambda-Wert wirklich von einem echten
// Spartan-Frame ueber CAN kommt (SpartanReading.fromCan). 0 heisst: Demo-Modus,
// Lambda-Testmodus, oder ADC-Fallback -- der Wert ist SIMULIERT/ERSATZ, keine
// reale Messung. Ohne dieses Bit konnte ein Display, das nur den CAN-Cockpit-
// Frame liest (nicht /api/status), Demo-Daten nicht von echten unterscheiden --
// das hat am 2026-07-14 dazu gefuehrt, dass der Vergaser anhand simulierter
// Werte verstellt wurde, waehrend CAN zum Spartan zeitweise ausgefallen war.
constexpr uint8_t kCockpitFlagRealCan     = 0x10;

// [COCKPIT-EXT-FRAME] Zweites Frame (ID = cockpitCanIdCfg+1, z.B. 0x511) fuer
// Daten, die bisher NUR per HTTP /api/status ans Display gingen (123-Spannung/
// -Temperatur/-Spulenstrom, Geschwindigkeit). Grund: an manchen Einbauorten ist
// das Hub-WLAN zum Display zu schwach/instabil, CAN ist dort die robustere
// Verbindung. Byte-Layout, big-endian:
//   [0-1] tune_volt_x100 (uint16, 0 wenn 123 nicht verbunden/frisch)
//   [2]   tune_temp_c (int8)         [3] tune_coil_x10 (uint8)
//   [4-5] speed_kmh_x10 (uint16)     [6] flags: Bit0=kCockpitExtFlagTuneFresh
//   [7]   reserviert/0
constexpr uint8_t kCockpitExtFlagTuneFresh = 0x01;

// [COCKPIT-EXT2-FRAME] Drittes Frame (ID = cockpitCanIdCfg+2, z.B. 0x512) fuer
// Kilometerstand/Trip/Motorstunden -- das Display ist das Fahrt-Frontend, diese
// Werte sollen bei schwachem WLAN nicht fehlen (Uhrzeit + Live-Tuning-Schreib-
// zugriff bleiben bewusst HTTP-only, siehe docs/lambda-status-logik.md).
//   [0-3] odo_km_x10 (uint32, big-endian, Gesamtstrecke)
//   [4-5] trip_km_x10 (uint16)         [6-7] engine_hours_x10 (uint16)
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
        // [CAN-STUCK-RECOVERY-FIX] Bisher wurde twai_initiate_recovery() JEDE
        // Sekunde neu aufgerufen, solange der Bus im Bus-Off haengt. Bleibt der
        // Bus laenger gestoert (z.B. ein anderer Knoten sendet waehrend seines
        // eigenen Reboots kurz Muell), kann die ESP-IDF-Recovery haengen bleiben
        // -- bisher half dann NUR ein manueller Hub-Reboot (Beobachtung 2026-07-19:
        // Display-Reboot -> Lambda ueber CAN weg, kam ohne Hub-Reboot/Spartan-
        // Stromzyklus nicht zurueck). Jetzt: haengt der Bus laenger als
        // kCanBusOffHardResetMs im Bus-Off/Stopped-Zustand, macht der Hub SELBST
        // einen kompletten TWAI-Neustart (Uninstall+Reinstall) -- das erreicht
        // exakt das, was vorher nur der manuelle Reboot geleistet hat.
        if (canBusOffSinceMs == 0) canBusOffSinceMs = now;
        if (now - canBusOffSinceMs > kCanBusOffHardResetMs) {
          Serial.println("CAN:         Bus-Off haengt > 5s -> harter TWAI-Neustart");
          logHubEvent("can_busoff", "hard_reset");
          canBusOffSinceMs = 0;
          stopCan();
          setupCan();
        } else if (twai_initiate_recovery() == ESP_OK) {
          Serial.println("CAN:         Bus-Off → recovery initiated");
        }
        if (canStatusErrors < UINT32_MAX) canStatusErrors++;
      } else if (canStatus.state == TWAI_STATE_STOPPED) {
        // Recovery completed — restart the driver.
        if (twai_start() == ESP_OK) {
          Serial.println("CAN:         recovered from Bus-Off, restarted");
          canBusOffSinceMs = 0;
        } else {
          if (canStatusErrors < UINT32_MAX) canStatusErrors++;
        }
      } else if (canStatus.state != TWAI_STATE_RUNNING ||
                 canStatus.tx_error_counter > 127 ||
                 canStatus.rx_error_counter > 127) {
        if (canStatusErrors < UINT32_MAX) canStatusErrors++;
      } else {
        canBusOffSinceMs = 0;   // Bus laeuft wieder normal -> Watchdog scharf halten
      }
    }
  }

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd) continue;

    // [CAN-TUNE-CMD] Display -> Hub: Live-Zuendwinkel per CAN bedienen, ID =
    // Cockpit-ID+3 (Default 0x513). Nutzt exakt denselben, bereits abgesicherten
    // Pfad wie /api/tune/live (Dead-Man 60s, Einzelschritte, nur bei streamender
    // 123 wirksam) -- keine neue Sicherheitslogik, nur ein zweiter Eingang.
    // Byte 0 = Kommando: 0=ping 1=up 2=down 3=reset 4=mode-toggle.
    if (message.identifier == static_cast<uint32_t>(cockpitCanIdCfg) + 3) {
      if (message.data_length_code >= 1) {
        tuneLastLiveApiMs = millis();
        switch (message.data[0]) {
          case 1: tuneAdvStep(+1); break;
          case 2: tuneAdvStep(-1); break;
          case 3: tuneAdvReset(); break;
          case 4: tuneModeToggle(); break;
          default: break;   // 0 = reines Ping (Dead-Man fuettern, sonst nichts)
        }
      }
      continue;
    }

    if (message.identifier != spartanCanIdCfg || message.data_length_code < 4) {
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
    // [STALE-LAMBDA-FIX] valid allein reicht nicht: stirbt die Quelle (CAN weg,
    // kein Fallback aktiv), bliebe der letzte Wert sonst fuer immer "gueltig" im
    // Frame. Simulierte Quellen (Demo/Test/ADC) schreiben sub-sekuendlich und
    // bleiben dadurch immer frisch.
    const bool lambdaFreshNow = snap.valid && (now - snap.receivedMs) <= kLambdaFreshMs;
    const uint16_t lambdaX1000 = lambdaFreshNow ? static_cast<uint16_t>(snap.lambda * 1000.0f + 0.5f) : 0;
    const int16_t  advX10      = static_cast<int16_t>(advance * 10.0f + (advance >= 0 ? 0.5f : -0.5f));
    // [COCKPIT-STATUS] flags&0x01 heisst nur "irgendein Lambda-Wert kam an" -- das
    // ist WAEHREND WAIT/HEAT genauso true wie bei OK (snap.valid wird in jedem
    // Lesepfad unconditional gesetzt, siehe docs/lambda-status-logik.md). Displays
    // ueber CAN konnten die Sonden-Aufwaermphase damit nicht erkennen. status_code
    // (0..3, siehe statusTextC()) jetzt zusaetzlich in Bits 2-3 gepackt -- bestehende
    // flags&0x01/0x02-Konsumenten bleiben unveraendert kompatibel.
    uint8_t flags = 0;
    if (lambdaFreshNow) flags |= kCockpitFlagLambdaValid;   // Wert kam an UND ist frisch; sagt NICHTS ueber status_code
    if (tuneFresh)      flags |= kCockpitFlagTuneFresh;
    flags |= static_cast<uint8_t>((snap.status & kCockpitStatusBitMask) << kCockpitStatusBitShift);
    // 0 = Demo/Test/ADC ODER veralteter CAN-Wert -- in beiden Faellen NICHT als
    // echte aktuelle Messung vertrauen.
    if (snap.fromCan && lambdaFreshNow) flags |= kCockpitFlagRealCan;

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

    // --- Ext-Frame: 123-Volt/Temp/Coil + Geschwindigkeit, gleiche ID+1 ---
    float voltScaled = 0.0f, tempRounded = 0.0f, coilScaled = 0.0f;
    if (hubFeatBle123 && tuneFresh) {
      const TuneSnapshot tune = tuneSnapshot();
      voltScaled  = tune.voltage * 100.0f + 0.5f;
      tempRounded = tune.temperature + (tune.temperature >= 0 ? 0.5f : -0.5f);
      coilScaled  = tune.coilCurrent * 10.0f + 0.5f;
    }
    if (voltScaled < 0.0f) voltScaled = 0.0f;
    if (voltScaled > 65535.0f) voltScaled = 65535.0f;
    if (tempRounded < -128.0f) tempRounded = -128.0f;
    if (tempRounded > 127.0f) tempRounded = 127.0f;
    if (coilScaled < 0.0f) coilScaled = 0.0f;
    if (coilScaled > 255.0f) coilScaled = 255.0f;
    float speedScaled = speedKmh * 10.0f + 0.5f;
    if (speedScaled < 0.0f) speedScaled = 0.0f;
    if (speedScaled > 65535.0f) speedScaled = 65535.0f;

    const uint16_t voltX100  = static_cast<uint16_t>(voltScaled);
    const int8_t   tempC     = static_cast<int8_t>(tempRounded);
    const uint8_t  coilX10   = static_cast<uint8_t>(coilScaled);
    const uint16_t speedX10  = static_cast<uint16_t>(speedScaled);

    twai_message_t tx2 = {};
    tx2.identifier       = static_cast<uint32_t>(cockpitCanIdCfg) + 1;
    tx2.data_length_code = 8;
    tx2.data[0] = static_cast<uint8_t>(voltX100 >> 8);
    tx2.data[1] = static_cast<uint8_t>(voltX100 & 0xFF);
    tx2.data[2] = static_cast<uint8_t>(tempC);
    tx2.data[3] = coilX10;
    tx2.data[4] = static_cast<uint8_t>(speedX10 >> 8);
    tx2.data[5] = static_cast<uint8_t>(speedX10 & 0xFF);
    tx2.data[6] = tuneFresh ? kCockpitExtFlagTuneFresh : 0;
    tx2.data[7] = 0;
    if (twai_transmit(&tx2, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (cockpitCanTxCount < UINT32_MAX) cockpitCanTxCount++;
    } else {
      if (cockpitCanTxErrors < UINT32_MAX) cockpitCanTxErrors++;
    }

    // --- Ext2-Frame: Odo/Trip/Motorstunden, gleiche ID+2 ---
    double odoKmScaled = static_cast<double>(odoMm) / 100000.0 + 0.5;    // mm -> km*10
    double tripKmScaled = static_cast<double>(tripMm) / 100000.0 + 0.5;
    double engHoursScaled = static_cast<double>(engineSeconds) / 360.0 + 0.5;  // s -> h*10
    if (odoKmScaled > 4294967295.0) odoKmScaled = 4294967295.0;
    if (tripKmScaled > 65535.0) tripKmScaled = 65535.0;
    if (engHoursScaled > 65535.0) engHoursScaled = 65535.0;
    const uint32_t odoKmX10 = static_cast<uint32_t>(odoKmScaled);
    const uint16_t tripKmX10 = static_cast<uint16_t>(tripKmScaled);
    const uint16_t engHoursX10 = static_cast<uint16_t>(engHoursScaled);

    twai_message_t tx3 = {};
    tx3.identifier       = static_cast<uint32_t>(cockpitCanIdCfg) + 2;
    tx3.data_length_code = 8;
    tx3.data[0] = static_cast<uint8_t>((odoKmX10 >> 24) & 0xFF);
    tx3.data[1] = static_cast<uint8_t>((odoKmX10 >> 16) & 0xFF);
    tx3.data[2] = static_cast<uint8_t>((odoKmX10 >> 8) & 0xFF);
    tx3.data[3] = static_cast<uint8_t>(odoKmX10 & 0xFF);
    tx3.data[4] = static_cast<uint8_t>(tripKmX10 >> 8);
    tx3.data[5] = static_cast<uint8_t>(tripKmX10 & 0xFF);
    tx3.data[6] = static_cast<uint8_t>(engHoursX10 >> 8);
    tx3.data[7] = static_cast<uint8_t>(engHoursX10 & 0xFF);
    if (twai_transmit(&tx3, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (cockpitCanTxCount < UINT32_MAX) cockpitCanTxCount++;
    } else {
      if (cockpitCanTxErrors < UINT32_MAX) cockpitCanTxErrors++;
    }
  }
#endif
}
