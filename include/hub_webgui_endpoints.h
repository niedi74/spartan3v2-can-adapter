#pragma once
// [WEBGUI-ENDPOINTS] setupWebGui(): alle HTTP-Endpoints (server.on) + collectHeaders
// + DNS/AP-Start. 1:1 aus main.cpp ausgelagert, an Originalstelle included
// (gleiche TU, im anonymen namespace) -- keine Logikaenderung.
void setupWebGui()
{
#if ENABLE_WEB_GUI
  static const char* collectedHeaders[] = {"X-Device", "User-Agent", "X-OTA-Token"};
  server.collectHeaders(collectedHeaders, 3);
  ensurePreferences();
  otaToken = networkPreferences.getString("ota_tok", "");   // [OTA-LOCK] leer = gesperrt
  timezoneIdx = networkPreferences.getUChar("tz_idx", kTimezoneDefault);
  if (timezoneIdx >= kTimezoneCount) timezoneIdx = kTimezoneDefault;
  logColumnMask = networkPreferences.getUShort("log_cols", kLogColDefault);
  logFsReady = initializeLogFilesystem(false);
  logCurrentBytes = 0;
  logOldBytes = 0;
  refreshLogSizeCache();
  // [KURVE] Migration: alter Einzel-Slot /curve.123 -> Slot 1
  if (logFsReady && SPIFFS.exists("/curve.123") && !SPIFFS.exists(kCurveFiles[0])) {
    SPIFFS.rename("/curve.123", kCurveFiles[0]);
  }
  restoreCurvesFromW25Q();  // [KURVE-W25Q] nach Format-Recovery: Slots vom Chip zurueckholen
  refreshCurveSlotCache();  // [KURVE] Slot-Bitmaske initial fuellen (danach nur Upload/Delete)
  Serial.printf("Logs:        SPIFFS %s, current=%s, old=%s\n",
                logFsReady ? "OK" : "FAIL",
                humanBytes(logFileSize(kLogFile)).c_str(),
                humanBytes(logFileSize(kOldLogFile)).c_str());
  ensureLogHeader();
  // Boot-time only: if ensureLogHeader() failed to open the log file (e.g.
  // power-loss corrupted the SPIFFS FAT), recover once via format here where
  // blocking for 3-10 s is acceptable.
  if (!logFsReady) {
    Serial.println("Logs:        boot format-recovery (header_open_failed)");
    if (initializeLogFilesystem(true)) {
      logFsReady = true;
      refreshLogSizeCache();
      ensureLogHeader();
      logHubEvent("log_fs", "format_recovered_header");
      restoreCurvesFromW25Q();     // [KURVE-W25Q] Format hat die Slots geleert -> vom Chip
      refreshCurveSlotCache();
    }
  }
  // WiFi-Profile laden
  // Slot 0 = Bus (kein STA), Slot 1 = Zuhause, Slot 2 = Handy
  // Migration: "ssid"/"pass" → p1_ssid/p1_pass wenn noch kein "wifi_prof" vorhanden
  if (!networkPreferences.isKey("wifi_prof") && networkPreferences.isKey("ssid")) {
    const String migrSsid = networkPreferences.getString("ssid", "");
    const String migrPass = networkPreferences.getString("pass", "");
    if (migrSsid.length() > 0) {
      networkPreferences.putString("p1_ssid", migrSsid);
      networkPreferences.putString("p1_pass", migrPass);
      networkPreferences.putUChar("wifi_prof", 1);
      Serial.printf("WiFi:        Migriert '%s' -> Zuhause-Profil\n", migrSsid.c_str());
    }
  }
  {
    String p1 = networkPreferences.getString("p1_ssid", HOME_WIFI_SSID);
    String p1p = networkPreferences.getString("p1_pass", HOME_WIFI_PASSWORD);
    strlcpy(g_hubWifiProfiles[1].ssid, p1.c_str(), sizeof(g_hubWifiProfiles[1].ssid));
    strlcpy(g_hubWifiProfiles[1].pass, p1p.c_str(), sizeof(g_hubWifiProfiles[1].pass));
    String p2 = networkPreferences.getString("p2_ssid", "");
    String p2p = networkPreferences.getString("p2_pass", "");
    strlcpy(g_hubWifiProfiles[2].ssid, p2.c_str(), sizeof(g_hubWifiProfiles[2].ssid));
    strlcpy(g_hubWifiProfiles[2].pass, p2p.c_str(), sizeof(g_hubWifiProfiles[2].pass));
    // [WIFI-STATIC] je Profil: ipm (0=DHCP/1=Static) + ip/gw/mask
    for (uint8_t slot = 1; slot <= 2; slot++) {
      char kIpm[8], kIp[8], kGw[8], kMask[8];
      snprintf(kIpm, sizeof(kIpm), "p%u_ipm", slot);
      snprintf(kIp, sizeof(kIp), "p%u_ip", slot);
      snprintf(kGw, sizeof(kGw), "p%u_gw", slot);
      snprintf(kMask, sizeof(kMask), "p%u_mask", slot);
      g_hubWifiProfiles[slot].ipMode = networkPreferences.getUChar(kIpm, 0);
      strlcpy(g_hubWifiProfiles[slot].ip, networkPreferences.getString(kIp, "").c_str(), sizeof(g_hubWifiProfiles[slot].ip));
      strlcpy(g_hubWifiProfiles[slot].gw, networkPreferences.getString(kGw, "").c_str(), sizeof(g_hubWifiProfiles[slot].gw));
      strlcpy(g_hubWifiProfiles[slot].mask, networkPreferences.getString(kMask, "255.255.255.0").c_str(), sizeof(g_hubWifiProfiles[slot].mask));
    }
  }
  hubWifiProfile = networkPreferences.getUChar("wifi_prof", DEFAULT_WIFI_PROFILE);
  if (hubWifiProfile > 2) hubWifiProfile = 0;
  // [WIFI-MAC-OVR] geraeteweit (nicht pro Profil) -- betrifft die STA-Hardwareadresse
  strlcpy(g_wifiMacOverride, networkPreferences.getString("mac_ovr", "").c_str(), sizeof(g_wifiMacOverride));

  const bool busMode = (hubWifiProfile == 0);
  const char* staSsid  = busMode ? "" : g_hubWifiProfiles[hubWifiProfile].ssid;
  const char* staPass  = busMode ? "" : g_hubWifiProfiles[hubWifiProfile].pass;
  savedWifiSsid = String(staSsid);
  haveSavedWifi = !busMode && strlen(staSsid) > 0;

  WiFi.setHostname(hubHostname.c_str());
  WiFi.mode(WIFI_AP_STA);
  applyWifiMacOverrideIfNeeded();   // [WIFI-MAC-OVR] direkt nach mode(), vor jedem Connect
#if ENABLE_BLE_HUB
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif
  ensureHubSoftAp();
  startHubMdns();
  if (!hubFeatAp) {
    Serial.println("Web GUI:     SoftAP Spartan3-Setup disabled (Setup)");
  }

  if (busMode) {
    Serial.println("Home WiFi:   Bus-Modus, nur AP");
  } else if (haveSavedWifi && hubFeatWifi && !vehicleActive()) {   // [VARIANTE-A] Boot-Connect nur im Stand
    applyStaticIpIfNeeded(hubWifiProfile);   // [WIFI-STATIC] vor WiFi.begin()!
    WiFi.begin(staSsid, staPass);
    homeWifiConnectStartedMs = millis();
    Serial.printf("Home WiFi:   Profil %d '%s' verbindet\n", hubWifiProfile, staSsid);
  } else if (haveSavedWifi && !hubFeatWifi) {
    Serial.println("Home WiFi:   STA disabled (Setup)");
  } else {
    Serial.println("Home WiFi:   not configured");
  }

  server.on("/", []() {
    server.send_P(200, "text/html", kHubIndexHtml);
  });

  const auto sendStatus = []() {
    if (webOtaRejectBusy()) return;
    recordWifiHttpPoller();
    server.send(200, "application/json", statusJson());
  };
  server.on("/state", sendStatus);
  server.on("/api/status", sendStatus);
  server.on("/uart_config", HTTP_GET, []() {
    uint8_t rx = (uint8_t)server.arg("rx").toInt();
    uint8_t tx = (uint8_t)server.arg("tx").toInt();
    ensurePreferences();
    networkPreferences.putUChar("uart_rx", rx);
    networkPreferences.putUChar("uart_tx", tx);
    Serial.printf("Bridge UART: konfiguriert RX=%d TX=%d -> Reboot\n", rx, tx);
    server.send(200,"application/json","{\"ok\":true,\"rx\":"+String(rx)+",\"tx\":"+String(tx)+"}");
    delay(300); ESP.restart();
  });
  server.on("/hub_feat", HTTP_GET, []() {
    // Dev-Tab: Hub-Features zur Laufzeit schalten (kein Reflash).
    const String name = server.arg("name");
    const bool on = (server.arg("val") == "on" || server.arg("val") == "1");
    bool known = true;
    if (name == "ble123") hubFeatBle123 = on;
    else if (name == "log") hubFeatLog = on;
    else if (name == "can") hubFeatCan = on;
    else known = false;
    if (known) {
      saveHubFeatures();
      applyHubFeatures();
      logHubEvent("hub_feat", "web");
    }
    server.send(known ? 200 : 400, "application/json",
                String("{\"ok\":") + (known ? "true" : "false") + "}");
  });
  // [CAN-DEV] Pins/Bitrate/IDs -- braucht sauberen TWAI-Reinit mit neuer GPIO-
  // Zuordnung, deshalb Speichern + Neustart statt Live-Anwendung.
  server.on("/can_config", HTTP_POST, []() {
    const int rx = server.arg("rx").toInt();
    const int tx = server.arg("tx").toInt();
    const int kbps = server.arg("kbps").toInt();
    const long sid = strtol(server.arg("sid").c_str(), nullptr, 16);
    const long cid = strtol(server.arg("cid").c_str(), nullptr, 16);
    const int txms = server.hasArg("txms") ? server.arg("txms").toInt() : cockpitCanTxIntervalMsCfg;
    if (rx < 0 || rx > 48 || tx < 0 || tx > 48 || rx == tx) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"Pins 0..48, RX != TX\"}");
      return;
    }
    if (kbps != 125 && kbps != 250 && kbps != 500 && kbps != 1000) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"kbps: 125|250|500|1000\"}");
      return;
    }
    if (sid < 0 || sid > 0x7FF || cid < 0 || cid > 0x7FF) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"IDs 0x000..0x7FF (Standard-ID)\"}");
      return;
    }
    if (txms < 20 || txms > 5000) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"txms 20..5000\"}");
      return;
    }
    canRxPinCfg = static_cast<uint8_t>(rx);
    canTxPinCfg = static_cast<uint8_t>(tx);
    canBitrateKbps = static_cast<uint16_t>(kbps);
    spartanCanIdCfg = static_cast<uint16_t>(sid);
    cockpitCanIdCfg = static_cast<uint16_t>(cid);
    cockpitCanTxIntervalMsCfg = static_cast<uint16_t>(txms);
    saveCanConfig();
    Serial.printf("CAN config:  RX=%d TX=%d %dkbit Spartan=0x%03lX Cockpit=0x%03lX %dms -> Neustart\n",
                  rx, tx, kbps, sid, cid, txms);
    logHubEvent("can_config", "web");
    server.send(200, "application/json", "{\"ok\":true,\"restart\":true}");
    delay(400);
    ESP.restart();
  });
  server.on("/lambda_test", HTTP_POST, []() {
    if (!server.hasArg("mode") || !setLambdaTestMode(server.arg("mode"))) {
      server.send(400, "application/json",
                  "{\"ok\":false,\"error\":\"mode must be off, fixed or sweep\"}");
      return;
    }
    if (server.hasArg("return")) {
      server.sendHeader("Location", "/", true);
      server.send(303, "text/plain", "");
      return;
    }
    const String body = String("{\"ok\":true,\"mode\":\"") +
        lambdaTestModeText() + "\"}";
    server.send(200, "application/json", body);
  });
  server.on("/api/ota/progress", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", otaProgressJson());
  });
  // [TUNE-LIVE] Live-Zuendwinkel: mode (T an/aus), step (A/R), reset (auf 0),
  // ping (Dead-Man-Keepalive der GUI). Sicherheitskritisch -> nur bei streaming,
  // GUI-Lock ist nur Optik: [TUNE-SAFE] serverseitig gilt der OTA-Token (wenn
  // gesetzt), sonst kann jedes Geraet im Funknetz die Zuendung verstellen.
  server.on("/api/tune/live", HTTP_POST, []() {
    if (otaToken.length() > 0 &&
        (!server.hasHeader("X-OTA-Token") || server.header("X-OTA-Token") != otaToken)) {
      server.send(403, "application/json", "{\"ok\":false,\"error\":\"token\"}");
      return;
    }
    tuneLastLiveApiMs = millis();   // [TUNE-SAFE] Dead-Man fuettern
    const String act = server.arg("act");
    bool ok = false;
    if (act == "mode")        ok = tuneModeToggle();
    else if (act == "up")     ok = tuneAdvStep(+1);
    else if (act == "down")   ok = tuneAdvStep(-1);
    else if (act == "reset")  ok = tuneAdvReset();
    else if (act == "ping")   ok = true;
    else { server.send(400, "application/json", "{\"ok\":false,\"error\":\"act=mode|up|down|reset|ping\"}"); return; }
    String body = String("{\"ok\":") + (ok ? "true" : "false") +
                  ",\"tune_mode\":" + (tuneModeActive ? "true" : "false") +
                  ",\"tune_adv_steps\":" + String(tuneAdvSteps) +
                  ",\"streaming\":" + (tuneStreaming() ? "true" : "false") + "}";
    server.send(ok ? 200 : 409, "application/json", body);
  });
  server.on("/api/ota/token", HTTP_POST, []() {
    // [OTA-LOCK] Token setzen/aendern (leer = OTA wieder sperren). Hinter dem AP-Passwort;
    // ein Fremd-/Fehl-Push kennt weder diesen Endpoint noch den Token -> OTA prallt ab.
    String t = server.arg("token");
    t.trim();
    ensurePreferences();
    networkPreferences.putString("ota_tok", t);
    otaToken = t;
    Serial.printf("OTA:         Token %s\n", t.length() ? "gesetzt (OTA entsperrt)" : "geloescht (OTA gesperrt)");
    logHubEvent("ota_token", t.length() ? "set" : "clear");
    server.send(200, "application/json",
                String("{\"ok\":true,\"ota_locked\":") + (t.length() ? "false" : "true") + "}");
  });
  server.on("/uart_test", HTTP_GET, []() {
    if (bridgeUartRxPin == 0) { server.send(200,"application/json","{\"ok\":false,\"msg\":\"UART nicht konfiguriert\"}"); return; }
    Serial1.printf("T,9999,45.0,100,3.0,14.0,80\n"); Serial1.flush(); delay(50);
    String rx = ""; while (Serial1.available()) rx += (char)Serial1.read();
    server.send(200,"application/json","{\"sent\":\"T,9999\",\"recv\":\""+rx+"\",\"rx\":"+String(bridgeUartRxPin)+",\"tx\":"+String(bridgeUartTxPin)+"}");
  });
  server.on("/restart", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html",
                "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>Restart</title></head><body style='font-family:system-ui;background:#0b1210;color:#e6ede8;padding:20px'>"
                "<h1>Hub startet neu...</h1><p>Bitte 5-10 Sekunden warten und die Seite neu laden.</p>"
                "</body></html>");
    delay(700);
    ESP.restart();
  });
  server.on("/update", HTTP_POST, []() {
    if (otaAuthFailed) {   // [OTA-LOCK] Upload wurde ohne gueltigen Token verworfen
      otaAuthFailed = false;
      server.sendHeader("Connection", "close");
      server.send(403, "text/plain",
                  otaToken.length() == 0 ? "OTA gesperrt: kein Token gesetzt"
                                         : "OTA: falscher Token");
      return;
    }
    const bool ok = !Update.hasError() && Update.remaining() == 0;
    server.sendHeader("Connection", "close");
    if (!ok && !otaMarkerFound && !otaMarkerForce) {   // [OTA-GUARD]
      server.send(422, "text/plain",
                  "Abgelehnt: Image traegt keinen Hub-FW-Marker (falsche Firmware?). "
                  "Override: /update?force=1");
    } else {
      server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
    }
    if (ok) {
      delay(700);
      ESP.restart();
    } else {
      otaBusy = false;
    }
  }, []() {
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      // [OTA-LOCK] Nur mit gueltigem Token flashen. Leerer Token = OTA komplett gesperrt
      // (sicherer Default nach Flash/erase). Schuetzt vor versehentlichem Fremd-OTA.
      otaAuthFailed = false;
      const bool tokOk = otaToken.length() > 0 &&
                         server.hasHeader("X-OTA-Token") &&
                         server.header("X-OTA-Token") == otaToken;
      if (!tokOk) {
        otaAuthFailed = true;
        Serial.printf("OTA:         REJECT (%s) von %s\n",
                      otaToken.length() == 0 ? "gesperrt/kein Token" : "falscher Token",
                      server.client().remoteIP().toString().c_str());
        logHubEvent("ota_reject", otaToken.length() == 0 ? "locked" : "badtoken");
        return;   // NICHT Update.begin() -> Firmware bleibt unberuehrt
      }
      otaMarkerForce = server.hasArg("force");   // [OTA-GUARD] bewusster Override
      webOtaBeginUpload(upload.filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (otaAuthFailed) return;
      webOtaWriteChunk(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (otaAuthFailed) return;
      webOtaFinishUpload(upload.totalSize);
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      webOtaAbortUpload();
    }
  });
  server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
  server.on("/gen_204", []() { server.send(204, "text/plain", ""); });
  server.on("/hotspot-detect.html", []() { server.send(200, "text/html", "<html><body>Success</body></html>"); });
  server.on("/ncsi.txt", []() { server.send(200, "text/plain", "Microsoft NCSI"); });
  server.on("/connecttest.txt", []() { server.send(200, "text/plain", "Microsoft Connect Test"); });
  server.on("/download", HTTP_GET, []() { sendLogFile(kLogFile, "spartan_hub_drive.csv"); });
  server.on("/download_old", HTTP_GET, []() { sendLogFile(kOldLogFile, "spartan_hub_drive_old.csv"); });
  // [KURVE] Hinterlegte 123-Zuendkurve (.123-XML): lesen (Anzeige/Download) + hochladen.
  server.on("/curve", HTTP_GET, []() {
    const char *path = curveFile(server.hasArg("slot") ? server.arg("slot").toInt() : 1);
    if (!logFsReady || !SPIFFS.exists(path)) { server.send(404, "text/plain", "Slot leer"); return; }
    File f = SPIFFS.open(path, FILE_READ);
    if (!f) { server.send(500, "text/plain", "Lesefehler"); return; }
    server.streamFile(f, "application/xml");
    f.close();
  });
  server.on("/curve", HTTP_POST, []() {
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  }, []() {
    static File cf;
    static int slot = 1;
    HTTPUpload &up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
      slot = server.hasArg("slot") ? server.arg("slot").toInt() : 1;
      if (logFsReady) cf = SPIFFS.open(curveFile(slot), FILE_WRITE);
      Serial.printf("Kurve:       Upload Slot%d '%s' -> %s\n", slot, up.filename.c_str(), cf ? "open" : "FS-Fehler");
    } else if (up.status == UPLOAD_FILE_WRITE) {
      if (cf) cf.write(up.buf, up.currentSize);
    } else if (up.status == UPLOAD_FILE_END) {
      if (cf) { cf.close(); Serial.printf("Kurve:       Slot%d gespeichert (%u Bytes)\n", slot, static_cast<unsigned>(up.totalSize)); }
      logHubEvent("curve", "upload");
      refreshCurveSlotCache();
      saveCurveToW25Q(slot);   // [KURVE-W25Q] Chip-Spiegel aktualisieren
    } else if (up.status == UPLOAD_FILE_ABORTED) {
      if (cf) { cf.close(); SPIFFS.remove(curveFile(slot)); }
      refreshCurveSlotCache();
      saveCurveToW25Q(slot);   // [KURVE-W25Q] Abbruch -> Chip-Kopie ebenfalls leeren
    }
  });
  server.on("/curve_delete", HTTP_POST, []() {
    const int slot = server.hasArg("slot") ? server.arg("slot").toInt() : 1;
    if (logFsReady) SPIFFS.remove(curveFile(slot));
    logHubEvent("curve", "delete");
    refreshCurveSlotCache();
    saveCurveToW25Q(slot);   // [KURVE-W25Q] Chip-Spiegel mitloeschen
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
#if ENABLE_BLE_HUB
  // [KURVE-READ] echte EEPROM-Kurve aus der 123 lesen: POST startet, GET liefert
  // Status + erfasste Roh-Bytes als Hex (Browser dekodiert 10@..13@ -> Kurve).
  server.on("/api/curve_read", HTTP_POST, []() {
    const bool ok = startCurveRead();
    server.send(ok ? 200 : 409, "application/json",
                ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"123 nicht streaming\"}");
  });
  server.on("/api/curve_read", HTTP_GET, []() {
    String j = "{\"active\":"; j += curveReadActive ? "true" : "false";
    j += ",\"len\":"; j += String(static_cast<unsigned>(curveReadLen));
    j += ",\"raw\":\"";
    static const char *H = "0123456789abcdef";
    const uint16_t n = curveReadLen;
    for (uint16_t i = 0; i < n; i++) { uint8_t b = static_cast<uint8_t>(curveReadBuf[i]); j += H[b >> 4]; j += H[b & 0xF]; }
    j += "\"}";
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", j);
  });
#endif
  server.on("/log/events", HTTP_GET, []() {
    uint16_t limit = 100;
    if (server.hasArg("limit")) {
      const int requested = server.arg("limit").toInt();
      if (requested > 0 && requested <= kHubEventMax) {
        limit = static_cast<uint16_t>(requested);
      }
    }
    server.send(200, "application/json", hubEventsJson(limit));
  });
  server.on("/log/events.csv", HTTP_GET, []() {
    String csv = "ms,epoch,type,detail\n";
    portENTER_CRITICAL(&hubEventMux);
    const uint16_t count = hubEventCount;
    const uint16_t start = count <= kHubEventMax
        ? static_cast<uint16_t>((hubEventHead + kHubEventMax - count) % kHubEventMax)
        : 0;
    for (uint16_t i = 0; i < count; i++) {
      const uint16_t idx = static_cast<uint16_t>((start + i) % kHubEventMax);
      const HubEvent &ev = hubEvents[idx];
      csv += String(ev.ms);
      csv += ",";
      csv += String(ev.epoch);
      csv += ",";
      csv += String(ev.type);
      csv += ",\"";
      csv += jsonEscape(String(ev.detail));
      csv += "\"\n";
    }
    portEXIT_CRITICAL(&hubEventMux);
    server.sendHeader("Content-Disposition", "attachment; filename=hub_events.csv");
    server.send(200, "text/csv", csv);
  });
  server.on("/log/events_clear", HTTP_POST, []() {
    clearHubEvents();
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/log/fs_reset", HTTP_POST, []() {
    logCurrentBytes = 0;
    logOldBytes = 0;
    const bool ok = initializeLogFilesystem(true);
    if (ok) {
      ensureLogHeader();
      refreshLogSizeCache();
    }
    const String body = String("{\"ok\":") + (ok ? "true" : "false") +
        ",\"error\":\"" + jsonEscape(logError) + "\"}";
    server.send(ok ? 200 : 500, "application/json", body);
  });
  server.on("/clear", HTTP_POST, []() {
    if (logFsReady) {
      SPIFFS.remove(kLogFile);
      logCurrentBytes = 0;
      ensureLogHeader();
    }
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/timezone", HTTP_POST, []() {
    int idx = server.arg("tz_idx").toInt();
    saveTimezone(static_cast<uint8_t>(idx));
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/ntp_sync", HTTP_POST, []() {
    if (WiFi.status() == WL_CONNECTED) {
      startNtpIfNeeded();
      requestNtpResync();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  // [TIME-DISPLAY-RTC] Fallback-Zeitquelle: Cockpit-Display hat einen batteriegepufferten
  // RTC-Chip und pusht seine Zeit periodisch her. Nur uebernehmen wenn der Hub noch KEINE
  // gueltige Zeit hat (kein Heimnetz -> kein NTP) -- NTP bleibt immer die bessere Quelle
  // und wird nie ueberschrieben. epoch=0/unplausibel wird ignoriert statt die Uhr zu zerschiessen.
  server.on("/api/time_sync", HTTP_POST, []() {
    if (systemTimeValid()) {
      server.send(200, "application/json", "{\"ok\":true,\"applied\":false,\"reason\":\"already_synced\"}");
      return;
    }
    const long epoch = server.arg("epoch").toInt();
    if (epoch < 1700000000L || epoch > 4000000000L) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"epoch unplausibel\"}");
      return;
    }
    struct timeval tv = { static_cast<time_t>(epoch), 0 };
    settimeofday(&tv, nullptr);
    ntpSynced = true;        // Zeit gueltig -> /api/status liefert time_epoch
    rtcSyncFromSystem();     // in den DS3231 zurueckschreiben (bootstrappt OSF)
    logHubEvent("time_sync", "display_rtc");
    Serial.printf("Time:        von Display-RTC uebernommen (epoch=%ld)\n", epoch);
    server.send(200, "application/json", "{\"ok\":true,\"applied\":true}");
  });
  // Bootstrap/Manuell: DS3231 unbedingt auf uebergebene UTC-Epoch setzen und
  // Systemzeit uebernehmen (unabhaengig vom already_synced-Zustand).
  server.on("/api/rtc/set", HTTP_POST, []() {
    const long epoch = server.arg("epoch").toInt();
    if (epoch < 1700000000L || epoch > 4000000000L) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"epoch unplausibel\"}");
      return;
    }
    struct timeval tv = { static_cast<time_t>(epoch), 0 };
    settimeofday(&tv, nullptr);
    ntpSynced = true;
    const bool ok = rtcPresent && rtcWriteEpoch(static_cast<time_t>(epoch));
    if (ok) rtcTimeValid = true;
    logHubEvent("rtc_set", ok ? "ok" : "no_rtc");
    Serial.printf("RTC:         manuell gesetzt (epoch=%ld, rtc=%s)\n", epoch, ok ? "ok" : "fehlt");
    String resp = "{\"ok\":";
    resp += ok ? "true" : "false";
    resp += ",\"rtc_present\":";
    resp += rtcPresent ? "true" : "false";
    resp += "}";
    server.send(200, "application/json", resp);
  });
  server.on("/log_columns", HTTP_POST, []() {
    uint16_t mask = 0;
    if (server.hasArg("spartan")) mask |= kLogColSpartan;
    if (server.hasArg("tune")) mask |= kLogColTune;
    if (server.hasArg("speed")) mask |= kLogColSpeed;
    if (server.hasArg("heater")) mask |= kLogColHeater;
    if (server.hasArg("hours")) mask |= kLogColHours;
    if (mask == 0) mask = kLogColSpartan;
    logColumnMask = mask;
    ensurePreferences();
    networkPreferences.putUShort("log_cols", logColumnMask);
    if (logFsReady) {
      SPIFFS.remove(kLogFile);
      logCurrentBytes = 0;
      ensureLogHeader();
    }
    Serial.printf("Logs:        columns mask=0x%04X\n", logColumnMask);
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
#if SPEED_REED_PIN >= 0
  server.on("/speed", HTTP_POST, []() {
    long tire = server.arg("tire").toInt();
    long trim = server.arg("trim").toInt();
    if (tire < 500 || tire > 4000) {
      server.send(400, "text/plain", "Reifenumfang 500..4000 mm");
      return;
    }
    if (trim < 500 || trim > 1500) {
      server.send(400, "text/plain", "Trim 500..1500 (1000 = 1.000)");
      return;
    }
    tireCircMm = static_cast<uint16_t>(tire);
    speedTrimPermil = static_cast<uint16_t>(trim);
    ensurePreferences();
    networkPreferences.putUShort("tire_mm", tireCircMm);
    networkPreferences.putUShort("trim_pm", speedTrimPermil);
    Serial.printf("Speed:       saved tire=%u mm, trim=%u permil\n",
                  tireCircMm, speedTrimPermil);
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/hourmeter", HTTP_POST, []() {
    // [WARTUNG] Betriebsstunden setzen -- v.a. Sonde bei Sensorwechsel auf 0.
    const String m = server.arg("meter");
    const double h = server.arg("hours").toDouble();
    if (h < 0.0 || h > 200000.0) { server.send(400, "text/plain", "Stunden 0..200000"); return; }
    const uint64_t secs = static_cast<uint64_t>(h * 3600.0 + 0.5);
    if      (m == "sensor") sensorSeconds = secs;
    else if (m == "engine") engineSeconds = secs;
    else if (m == "device") deviceSeconds = secs;
    else { server.send(400, "text/plain", "meter=sensor|engine|device"); return; }
    saveHourmeters();
    hubCfgDirty = true;   // [W25Q-CFG] neuer Stand mit ins Chip-Backup (v3)
    Serial.printf("Wartung:     %s-Betriebsstunden gesetzt auf %.2f h\n", m.c_str(), h);
    logHubEvent("hourmeter", m.c_str());
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
#if SPEED_REED_PIN >= 0
  server.on("/odo", HTTP_POST, []() {
    // [ODOMETER] trip=reset ODER total_km setzen (z.B. Tacho-Stand uebernehmen).
    if (server.arg("trip") == "reset") {
      tripMm = 0;
      Serial.println("Odometer:    Teilstrecke auf 0");
    } else if (server.hasArg("total_km")) {
      const double km = server.arg("total_km").toDouble();
      if (km < 0.0 || km > 2000000.0) { server.send(400, "text/plain", "km 0..2000000"); return; }
      odoMm = static_cast<uint64_t>(km * 1000000.0 + 0.5);
      Serial.printf("Odometer:    Gesamt gesetzt auf %.1f km\n", km);
    } else { server.send(400, "text/plain", "trip=reset oder total_km=..."); return; }
    ensurePreferences();
    networkPreferences.putULong64("odo_mm", odoMm);
    networkPreferences.putULong64("trip_mm", tripMm);
    odoLastSavedMm = odoMm;
    hubCfgDirty = true;   // [W25Q-CFG] neuer Stand mit ins Chip-Backup (v3)
    logHubEvent("odometer", server.hasArg("total_km") ? "set" : "trip_reset");
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
#endif
#endif
  server.on("/wifi_scan", HTTP_GET, []() {
    // Netzwerke scannen (blockiert ~3-5s; der AP kann kurz aussetzen).
    int n = WiFi.scanNetworks();
    String j = "[";
    int added = 0;
    for (int i = 0; i < n && added < 25; i++) {
      String s = WiFi.SSID(i);
      if (s.length() == 0) continue;
      if (added) j += ",";
      j += "{\"ssid\":\"" + jsonEscape(s) + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
           ",\"lock\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1) + "}";
      added++;
    }
    j += "]";
    WiFi.scanDelete();
    server.send(200, "application/json", j);
  });
  server.on("/wifi_connect", HTTP_POST, []() {
    // Auswählen + verbinden: SSID/Passwort -> Profil 1 (Zuhause), aktiv, WLAN an, Reboot.
    String ssid = server.arg("ssid"); ssid.trim();
    String pass = server.arg("pass");
    if (ssid.length() == 0) { server.send(400, "text/plain", "SSID fehlt"); return; }
    ensurePreferences();
    strlcpy(g_hubWifiProfiles[1].ssid, ssid.c_str(), sizeof(g_hubWifiProfiles[1].ssid));
    strlcpy(g_hubWifiProfiles[1].pass, pass.c_str(), sizeof(g_hubWifiProfiles[1].pass));
    networkPreferences.putString("p1_ssid", ssid);
    networkPreferences.putString("p1_pass", pass);
    networkPreferences.putUChar("wifi_prof", 1);
    hubWifiProfile = 1;
    hubFeatWifi = true;
    saveHubFeatures();
    Serial.printf("WiFi connect: SSID='%s' -> Profil 1, Reboot\n", ssid.c_str());
    server.send(200, "text/html",
                "<meta http-equiv='refresh' content='7;url=/'>Verbinde mit '" + ssid +
                "' - Geraet startet neu...");
    delay(300);
    ESP.restart();
  });
  server.on("/wifi_prof", HTTP_POST, []() {
    int slot = server.arg("slot").toInt();
    if (slot < 0 || slot > 2) slot = 0;
    hubWifiProfile = (uint8_t)slot;
    networkPreferences.putUChar("wifi_prof", hubWifiProfile);
    WiFi.disconnect(false, false);
    homeWifiConnectStartedMs = 0;
    homeWifiDisabledForRoadAp = false;
    if (slot == 0) {
      haveSavedWifi = false;
      savedWifiSsid = "";
      Serial.println("WiFi Profil: Bus (AP only)");
    } else {
      const char* ssid = g_hubWifiProfiles[slot].ssid;
      const char* pass = g_hubWifiProfiles[slot].pass;
      savedWifiSsid = String(ssid);
      haveSavedWifi = strlen(ssid) > 0;
      if (haveSavedWifi && hubFeatWifi) {
        Serial.printf("WiFi Profil: %d '%s'\n", slot, ssid);
        hubCfgDirty = true;   // [W25Q-CFG] Backup aktualisieren
        connectHomeWifiAligned(ssid, pass);  // [WLAN-KANAL] AP folgt Router-Kanal
      } else {
        Serial.printf("WiFi Profil: %d SSID leer oder WiFi aus\n", slot);
      }
    }
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/wifi_profile_save", HTTP_POST, []() {
    int slot = server.arg("slot").toInt();
    if (slot < 1 || slot > 2) { server.send(400, "text/plain", "Ungültiger Slot"); return; }
    String ssid = server.arg("ssid"); ssid.trim();
    String pass = server.arg("pass");
    strlcpy(g_hubWifiProfiles[slot].ssid, ssid.c_str(), sizeof(g_hubWifiProfiles[slot].ssid));
    if (pass.length() > 0)
      strlcpy(g_hubWifiProfiles[slot].pass, pass.c_str(), sizeof(g_hubWifiProfiles[slot].pass));
    // [WIFI-STATIC] ipm=0/1, bei Static muessen ip+gw gueltig sein, sonst DHCP behalten
    const bool wantStatic = server.arg("ipm").toInt() == 1;
    String ip = server.arg("ip"); ip.trim();
    String gw = server.arg("gw"); gw.trim();
    String mask = server.arg("mask"); mask.trim();
    if (mask.length() == 0) mask = "255.255.255.0";
    IPAddress chkIp, chkGw, chkMask;
    const bool ipOk = wantStatic && chkIp.fromString(ip) && chkGw.fromString(gw) && chkMask.fromString(mask);
    g_hubWifiProfiles[slot].ipMode = ipOk ? 1 : 0;
    strlcpy(g_hubWifiProfiles[slot].ip, ip.c_str(), sizeof(g_hubWifiProfiles[slot].ip));
    strlcpy(g_hubWifiProfiles[slot].gw, gw.c_str(), sizeof(g_hubWifiProfiles[slot].gw));
    strlcpy(g_hubWifiProfiles[slot].mask, mask.c_str(), sizeof(g_hubWifiProfiles[slot].mask));
    if (wantStatic && !ipOk) {
      Serial.printf("WiFi Profil: %d Static-IP unvollstaendig/ungueltig - bleibt DHCP\n", slot);
    }
    char kIpm[8], kIp[8], kGw[8], kMask[8];
    snprintf(kIpm, sizeof(kIpm), "p%d_ipm", slot);
    snprintf(kIp, sizeof(kIp), "p%d_ip", slot);
    snprintf(kGw, sizeof(kGw), "p%d_gw", slot);
    snprintf(kMask, sizeof(kMask), "p%d_mask", slot);
    networkPreferences.putUChar(kIpm, g_hubWifiProfiles[slot].ipMode);
    networkPreferences.putString(kIp, ip);
    networkPreferences.putString(kGw, gw);
    networkPreferences.putString(kMask, mask);
    if (slot == 1) {
      networkPreferences.putString("p1_ssid", ssid);
      if (pass.length() > 0) networkPreferences.putString("p1_pass", pass);
    } else {
      networkPreferences.putString("p2_ssid", ssid);
      if (pass.length() > 0) networkPreferences.putString("p2_pass", pass);
    }
    Serial.printf("WiFi Profil: %d gespeichert SSID='%s' ipMode=%u\n", slot, ssid.c_str(), g_hubWifiProfiles[slot].ipMode);
    // Speichern aktiviert das Profil gleich + verbindet (kein extra Modus-Tipp noetig).
    if (strlen(g_hubWifiProfiles[slot].ssid) > 0 && hubFeatWifi) {
      hubWifiProfile = (uint8_t)slot;
      networkPreferences.putUChar("wifi_prof", hubWifiProfile);
      WiFi.disconnect(false, false);
      homeWifiDisabledForRoadAp = false;
      savedWifiSsid = String(g_hubWifiProfiles[slot].ssid);
      haveSavedWifi = true;
      Serial.printf("WiFi Profil: %d -> aktiviert + verbinde\n", slot);
      hubCfgDirty = true;   // [W25Q-CFG] Backup aktualisieren
      connectHomeWifiAligned(g_hubWifiProfiles[slot].ssid, g_hubWifiProfiles[slot].pass);
    }
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  // [WIFI-MAC-OVR] Manuelle STA-MAC setzen/loeschen. Wirkt erst nach Neustart sauber
  // (esp_wifi_set_mac() waehrend einer laufenden Verbindung ist unzuverlaessig).
  server.on("/wifi_mac", HTTP_POST, []() {
    const String mac = server.arg("mac");
    uint8_t chk[6];
    if (mac.length() > 0 && !parseMac6(mac.c_str(), chk)) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"Format AA:BB:CC:DD:EE:FF\"}");
      return;
    }
    ensurePreferences();
    networkPreferences.putString("mac_ovr", mac);
    Serial.printf("WiFi MAC:    Override %s -> Neustart\n", mac.length() ? mac.c_str() : "geloescht");
    logHubEvent("wifi_mac", mac.length() ? "set" : "clear");
    server.send(200, "application/json", "{\"ok\":true,\"restart\":true}");
    delay(400);
    ESP.restart();
  });
  server.on("/wifi", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    String password = server.arg("pass");
    ssid.trim();
    if (ssid.length() == 0) {
      server.send(400, "text/plain", "WLAN-Name fehlt.");
      return;
    }
    if (password.length() == 0 && networkPreferences.isKey("ssid") &&
        ssid == networkPreferences.getString("ssid", "")) {
      password = networkPreferences.getString("pass", "");
    }
    if (password.length() == 0) {
      server.send(400, "text/plain", "WLAN-Passwort fehlt. Nur beim gleichen gespeicherten WLAN darf es leer bleiben.");
      return;
    }

    networkPreferences.putString("ssid", ssid);
    networkPreferences.putString("pass", password);
    haveSavedWifi = true;
    savedWifiSsid = ssid;
    WiFi.disconnect();
    if (hubFeatWifi) {
      homeWifiDisabledForRoadAp = false;
      Serial.printf("Home WiFi:   saved and connecting to '%s'\n", ssid.c_str());
      hubCfgDirty = true;   // [W25Q-CFG] Backup aktualisieren
      connectHomeWifiAligned(ssid.c_str(), password.c_str());  // [WLAN-KANAL]
    } else {
      homeWifiConnectStartedMs = 0;
      Serial.printf("Home WiFi:   saved '%s' (STA off in Setup)\n", ssid.c_str());
    }
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/wifi_clear", HTTP_POST, []() {
    networkPreferences.remove("ssid");
    networkPreferences.remove("pass");
    haveSavedWifi = false;
    savedWifiSsid = "";
    WiFi.disconnect();
    hubCfgDirty = true;   // [W25Q-CFG] Backup auf geleerten Stand bringen
    Serial.println("Home WiFi:   credentials cleared");
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/ble_target", HTTP_POST, []() {
#if ENABLE_BLE_HUB
    const String mac = normalizeMacInput(server.arg("tune_mac"));
    if (!looksLikeMacAddress(mac)) {
      server.send(400, "text/plain", "123 BLE-Adresse ungueltig. Format aa:bb:cc:dd:ee:ff");
      return;
    }
    ensurePreferences();
    tuneSavedAddress = mac;
    networkPreferences.putString("tune_mac", tuneSavedAddress);
    resetTuneClient();
    tuneDoConnect = false;
    scheduleTuneScan();
    Serial.printf("123TUNE BLE: target override %s\n", tuneSavedAddress.c_str());
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
#else
    server.send(400, "text/plain", "BLE hub nicht aktiv");
#endif
  });
  server.on("/uart_cmd", HTTP_POST, []() {
    sendSpartanUartCommand(server.arg("cmd"));
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/spartan_can", HTTP_POST, []() {
    const int canId = constrain(server.arg("canid").toInt(), 0, 2047);
    const int baud = server.arg("baud").toInt();
    const int format = constrain(server.arg("format").toInt(), 0, 5);
    const int dataRate = constrain(server.arg("candrate").toInt(), 1, 9999);
    const int term = server.arg("term").toInt() == 0 ? 0 : 1;
    sendSpartanUartCommand("SETCANFORMAT" + String(format));
    sendSpartanUartCommand("SETCANID" + String(canId));
    sendSpartanUartCommand("SETCANBAUD" + String(baud == 1000000 ? 1000000 : 500000));
    sendSpartanUartCommand("SETCANDR" + String(dataRate));
    sendSpartanUartCommand("SETCANR" + String(term));
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/spartan_output", HTTP_POST, []() {
    const int perf = constrain(server.arg("perf").toInt(), 0, 2);
    const int slowheat = constrain(server.arg("slowheat").toInt(), 0, 3);
    const int nbmode = server.arg("nbmode").toInt() == 0 ? 0 : 2;
    sendSpartanUartCommand("SETPERF" + String(perf));
    sendSpartanUartCommand("SETSLOWHEAT" + String(slowheat));
    sendSpartanUartCommand("SETNBMODE" + String(nbmode));
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/hub_features", HTTP_POST, []() {
    const bool wasApOn = hubFeatAp;
    bool nextAp = server.hasArg("ap");
    bool nextWifi = server.hasArg("wifi");
    String guardMessage;
    if (!nextAp && !nextWifi) {
      nextAp = true;
      guardMessage = "AP bleibt an: WLAN und SoftAP duerfen nicht gleichzeitig aus sein.";
    } else if (!nextAp && nextWifi && !haveSavedWifi && WiFi.status() != WL_CONNECTED) {
      nextAp = true;
      guardMessage = "AP bleibt an: erst Home-WLAN speichern/verbinden, dann SoftAP ausschalten.";
    }
    hubFeatAp = nextAp;
    hubFeatWifi = nextWifi;
    hubFeatLog = server.hasArg("log");
    hubFeatBle123 = server.hasArg("ble123");
    saveHubFeatures();
    logHubEvent("hub_feat", guardMessage.length() ? "guarded" : "saved");
    if (guardMessage.length() > 0) {
      applyHubFeatures();
      server.send(200, "text/html",
                  "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>AP bleibt an</title></head><body style='font-family:system-ui;background:#0b1210;color:#e6ede8;padding:20px'>"
                  "<h1>SoftAP bleibt an</h1><p>" + guardMessage + "</p>"
                  "<p><a href='/'>Zurueck</a></p></body></html>");
      return;
    }
    if (wasApOn && !hubFeatAp) {
      server.send(200, "text/html",
                  "<!doctype html><html lang='de'><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>AP aus</title></head><body style='font-family:system-ui;background:#0b1210;color:#e6ede8;padding:20px'>"
                  "<h1>SoftAP wird ausgeschaltet</h1>"
                  "<p>Die Einstellung ist gespeichert. Wenn du gerade ueber Spartan3-Setup verbunden bist, bricht diese Verbindung jetzt ab.</p>"
                  "<p>Weiterer Zugriff geht ueber Heimnetz/VPN/LAN-IP oder seriell.</p>"
                  "</body></html>");
      delay(900);
      applyHubFeatures();
      return;
    }
    applyHubFeatures();
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/ap_config", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    String password = server.arg("pass");
    String ip = server.arg("ip");
    String mask = server.arg("mask");
    ssid.trim();
    ip.trim();
    mask.trim();
    IPAddress parsedIp, parsedMask;
    if (ssid.length() == 0) {
      server.send(400, "text/plain", "AP-Name fehlt.");
      return;
    }
    if (password.length() > 0 && password.length() < 8) {
      server.send(400, "text/plain", "AP-Passwort muss leer oder mindestens 8 Zeichen lang sein.");
      return;
    }
    if (!parseApAddress(ip, "192.168.4.1", parsedIp) ||
        !parseApAddress(mask, "255.255.255.0", parsedMask)) {
      server.send(400, "text/plain", "AP-IP oder Netzmaske ungueltig.");
      return;
    }
    saveHubApConfig(ssid, password, ip, mask);
    hubCfgDirty = true;   // [W25Q-CFG] AP-Config + mDNS ins Backup
    // mDNS-Name (optional): sanitisieren auf a-z 0-9 '-', speichern, Responder neu starten.
    String mdns = server.arg("mdns");
    mdns.trim();
    mdns.toLowerCase();
    String clean;
    for (size_t i = 0; i < mdns.length(); i++) {
      char c = mdns[i];
      if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') clean += c;
    }
    if (clean.length() > 0 && clean != hubHostname) {
      hubHostname = clean;
      networkPreferences.putString("mdns_host", hubHostname);
    }
    if (hubFeatAp) {
      WiFi.softAPdisconnect(true);
      delay(100);
      ensureHubSoftAp();
    }
    startHubMdns();
    logHubEvent("ap_config", "saved");
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://spartanhub.local/", true);
    server.send(302, "text/plain", "");
  });

  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  server.begin();
  Serial.printf(
      "Web GUI:     WiFi '%s', password '%s', http://%s/\n",
      WEB_AP_SSID,
      WEB_AP_PASSWORD,
      WiFi.softAPIP().toString().c_str());
#endif
}
