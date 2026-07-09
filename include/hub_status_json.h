#pragma once
// [STATUS-JSON] statusJson(): kompletter /api/status-Payload. 1:1 aus main.cpp,
// an Originalstelle included (gleiche TU) -- keine Logikaenderung.
String statusJson()
{
  const SpartanReading snapshot = readingSnapshot();
#if ENABLE_BLE_HUB
  const TuneSnapshot tune = tuneSnapshot();
#if ENABLE_BLE_DISPLAY
  const uint8_t clients = getBleClientCount();
#endif
#endif
  const uint32_t now = millis();
  String json;
  json.reserve(3600);  // verhindert wiederholte Heap-Reallokationen
  json += "{\"valid\":";
  json += snapshot.valid ? "true" : "false";
  json += ",\"lambda\":";
  json += String(snapshot.lambda, 3);
  json += ",\"temperature\":";
  json += String(snapshot.temperatureC);
  json += ",\"status\":\"";
  json += statusTextC(snapshot.status);
  json += "\",\"status_code\":";
  json += String(snapshot.status);
  json += ",\"source\":\"";
  json += sourceTextC(snapshot);
  json += "\",\"lambda_test_mode\":\"";
  json += lambdaTestModeText();
  json += "\",\"lambda_test_active\":";
  json += lambdaTestMode != LambdaTestMode::Off ? "true" : "false";
  json += ",\"can_ready\":";
  json += canReady ? "true" : "false";
  json += ",\"age_ms\":";
  json += snapshot.valid ? String(now - snapshot.receivedMs) : "0";
  json += ",\"can_state\":";
  json += String(static_cast<int>(canStatus.state));
  json += ",\"can_tx_errors\":";
  json += String(canStatus.tx_error_counter);
  json += ",\"can_rx_errors\":";
  json += String(canStatus.rx_error_counter);
  json += ",\"can_status_errors\":";
  json += String(canStatusErrors);
  json += ",\"cockpit_can_tx\":";
  json += String(cockpitCanTxCount);
  json += ",\"cockpit_can_tx_err\":";
  json += String(cockpitCanTxErrors);
  json += ",\"heap_free\":";
  json += String(heap_caps_get_free_size(MALLOC_CAP_8BIT));
  json += ",\"fw_build\":\"";
  json += FW_BUILD;
  json += "\",\"fw_role\":\"";
  json += DEVICE_ROLE;
#if MINIMAL_123
  json += "-min123";
#endif
  json += "\"";
  json += ",\"flash_ext_detected\":";
  json += w25qDetected ? "true" : "false";
  json += ",\"flash_ext_jedec\":\"";
  char jbuf[8]; snprintf(jbuf, sizeof(jbuf), "%06X", (unsigned)(w25qJedecId & 0xFFFFFF));
  json += jbuf;
  json += "\",\"flash_ext_mb\":";
  json += String(w25qSizeMB);
  json += ",\"flash_ext_mfg\":\"";
  json += w25qMfg;
  json += "\"";
  json += ",\"cfg_backup\":";
  json += (w25qDetected && networkPreferences.isKey("cfg_w25q")) ? "true" : "false";
#if ENABLE_WEB_GUI
  json += ",\"ota_busy\":";
  json += otaBusy ? "true" : "false";
  json += ",\"ota_locked\":";
  json += otaToken.length() == 0 ? "true" : "false";
  json += ",\"ota_rx\":";
  json += String(static_cast<unsigned long>(otaRxBytes));
  json += ",\"ota_written\":";
  json += String(static_cast<unsigned long>(Update.progress()));
#endif
  json += ",\"device_hours\":";
  json += String(static_cast<double>(deviceSeconds) / 3600.0, 2);
  json += ",\"engine_hours\":";
  json += String(static_cast<double>(engineSeconds) / 3600.0, 2);
  json += ",\"sensor_hours\":";
  json += String(static_cast<double>(sensorSeconds) / 3600.0, 2);
#if ENABLE_SPARTAN_HEATER_ANALOG
  json += ",\"heater_adc_mv\":";
  json += String(heaterAdcMilliVolts);
  json += ",\"heater_volts\":";
  json += String(heaterStatusVolts, 2);
  json += ",\"heater_status_code\":";
  json += String(heaterStatusCode);
#endif
#if ENABLE_BLE_HUB
#if ENABLE_BLE_DISPLAY
  json += ",\"ble_clients\":";
  json += String(clients);
#endif
  json += ",\"ble_name\":\"";
  json += BLE_HUB_NAME;
  json += "\",\"ble_address\":\"";
  json += bleAddress;
  json += "\",\"ble_display\":";
  json += ENABLE_BLE_DISPLAY ? "true" : "false";
#endif
  json += ",\"hub_feat_ap\":";
  json += hubFeatAp ? "true" : "false";
  json += ",\"hub_feat_wifi\":";
  json += hubFeatWifi ? "true" : "false";
  json += ",\"hub_feat_log\":";
  json += hubFeatLog ? "true" : "false";
  json += ",\"hub_feat_ble123\":";
  json += hubFeatBle123 ? "true" : "false";
  // [CAN-DEV] Dev-Tab: aktueller Ein/Aus-Zustand + die Parameter, die einen Neustart brauchen
  json += ",\"hub_feat_can\":";
  json += hubFeatCan ? "true" : "false";
  json += ",\"can_rx_pin\":";
  json += String(canRxPinCfg);
  json += ",\"can_tx_pin\":";
  json += String(canTxPinCfg);
  json += ",\"can_bitrate_kbps\":";
  json += String(canBitrateKbps);
  json += ",\"can_spartan_id\":\"";
  json += String(spartanCanIdCfg, HEX);
  json += "\",\"can_cockpit_id\":\"";
  json += String(cockpitCanIdCfg, HEX);
  json += "\",\"can_tx_interval_ms\":";
  json += String(cockpitCanTxIntervalMsCfg);
#if ENABLE_BLE_HUB
  json += ",\"ble_hub_clients\":[";
  for (uint8_t i = 0; i < bleHubClientCount; i++) {
    if (i > 0) json += ",";
    json += "{\"addr\":\"";
    json += bleHubClients[i].address;
    json += "\",\"id_addr\":\"";
    json += bleHubClients[i].idAddress;
    json += "\",\"handle\":";
    json += String(bleHubClients[i].handle);
    json += ",\"mtu\":";
    json += String(bleHubClients[i].mtu);
    json += ",\"interval_ms\":";
    json += String(static_cast<float>(bleHubClients[i].intervalMs10) / 10.0f, 1);
    json += ",\"age_ms\":";
    json += String(now - bleHubClients[i].connectedMs);
    json += "}";
  }
  json += "],\"ble_scan\":[";
  for (uint8_t i = 0; i < bleScanDeviceCount; i++) {
    if (i > 0) json += ",";
    json += "{\"addr\":\"";
    json += bleScanDevices[i].address;
    json += "\",\"name\":\"";
    json += jsonEscape(bleScanDevices[i].name);
    json += "\",\"rssi\":";
    json += String(bleScanDevices[i].rssi);
    json += ",\"age_ms\":";
    json += String(now - bleScanDevices[i].seenMs);
    json += ",\"tune\":";
    json += bleScanDevices[i].tuneLike ? "true" : "false";
    json += "}";
  }
  json += "]";
#endif
#if ENABLE_WEB_GUI
  json += ",\"wifi_saved\":";
  json += haveSavedWifi ? "true" : "false";
  json += ",\"wifi_saved_ssid\":\"";
  json += savedWifiSsid;
  json += "\"";
  json += ",\"wifi_prof\":";
  json += String(hubWifiProfile);
  json += ",\"wifi_prof_labels\":[\"Hub-AP\",\"Zuhause\",\"S24\"]";
  json += ",\"wifi_prof_ssids\":[\"\"";
  json += ",\"" + String(g_hubWifiProfiles[1].ssid) + "\"";
  json += ",\"" + String(g_hubWifiProfiles[2].ssid) + "\"]";
  json += ",\"wifi_connected\":";
  json += WiFi.status() == WL_CONNECTED ? "true" : "false";
  json += ",\"wifi_sta_reason\":";
  json += String(lastStaReason);
  json += ",\"wifi_sta_event\":";
  json += String(lastStaEvent);
  json += ",\"wifi_status\":";
  json += String((int)WiFi.status());
  json += ",\"wifi_ssid\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  json += "\",\"wifi_ip\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  json += "\",\"wifi_mac\":\"";       // [WIFI-MAC-OVR] effektive STA-MAC (Werk oder Override)
  json += WiFi.macAddress();
  json += "\",\"wifi_mac_override\":\"";
  json += g_wifiMacOverride;
  json += "\",\"ap_ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",\"wifi_sta_channel\":";
  json += String(WiFi.status() == WL_CONNECTED ? WiFi.channel() : 0);
  json += ",\"wifi_sta_rssi\":";
  json += String(WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0);
  json += ",\"wifi_ap_channel\":";
  json += String((int)hubApChannel);
  json += ",\"wifi_home_not_found\":";
  json += homeWifiNotFound ? "true" : "false";
  json += ",\"vehicle_active\":";
  json += vehicleActive() ? "true" : "false";
  json += ",\"ap_retry_count\":";
  json += String(apRetryCount);
  json += ",\"log_ready\":";
  json += logFsReady ? "true" : "false";
  json += ",\"log_error\":\"";
  json += jsonEscape(logError);
  json += "\"";
  json += ",\"log_current_bytes\":";
  json += String(static_cast<unsigned long>(logFileSize(kLogFile)));
  json += ",\"log_old_bytes\":";
  json += String(static_cast<unsigned long>(logFileSize(kOldLogFile)));
  json += ",\"log_columns\":";
  json += String(logColumnMask);
  json += ",\"time_valid\":";
  json += systemTimeValid() ? "true" : "false";
  json += ",\"time_text\":\"";
  json += csvTimeText(now);
  json += "\"";
  json += ",\"ntp_synced\":";
  json += ntpSynced ? "true" : "false";
  json += ",\"rtc_present\":";
  json += rtcPresent ? "true" : "false";
  json += ",\"rtc_valid\":";
  json += rtcTimeValid ? "true" : "false";
  if (ntpSynced && systemTimeValid()) {
    json += ",\"time_epoch\":";
    json += String(static_cast<unsigned long>(time(nullptr)));
  }
  json += ",\"ntp_time\":\"";
  json += ntpSynced && systemTimeValid() ? csvTimeText(now) : String("-");
  json += "\"";
  json += ",\"timezone\":\"";
  json += jsonEscape(timezoneLabel(timezoneIdx));
  json += "\"";
  json += ",\"timezone_idx\":";
  json += String(timezoneIdx);
  json += ",\"ntp_server\":\"";
  json += kNtpServerPrimary;
  json += "\"";
  json += ",\"ntp_last_sync_ms\":";
  json += String(lastNtpSyncMs);
  json += ",\"ntp_last_sync_age_s\":";
  json += lastNtpSyncMs == 0 ? "0" : String((now - lastNtpSyncMs) / 1000UL);
  refreshWifiApStations();
  json += ",\"mdns_host\":\"";
  json += jsonEscape(hubHostname);
  json += "\",\"wifi_ap_ssid\":\"";
  json += jsonEscape(hubApSsid);
  json += "\",\"wifi_ap_password\":\"";
  json += jsonEscape(hubApPassword);
  json += "\",\"wifi_ap_ip\":\"";
  json += jsonEscape(hubApIp);
  json += "\",\"wifi_ap_mask\":\"";
  json += jsonEscape(hubApMask);
  json += "\",\"wifi_ap_range\":\"";
  json += jsonEscape(hubApIp);
  json += "/";
  json += jsonEscape(hubApMask);
  json += "\",\"wifi_ap_station_count\":";
  json += String(wifiApStationCount);
  json += ",\"wifi_ap_stations\":[";
  for (uint8_t i = 0; i < wifiApStationCount; i++) {
    if (i > 0) json += ",";
    json += "{\"mac\":\"";
    json += wifiApStations[i].mac;
    json += "\",\"ip\":\"";
    json += wifiApStations[i].ip.length() ? wifiApStations[i].ip : "-";
    json += "\",\"rssi\":";
    json += String(wifiApStations[i].rssi);
    json += ",\"age_ms\":";
    json += String(now - wifiApStations[i].firstSeenMs);
    json += ",\"last_seen_ms\":";
    json += String(now - wifiApStations[i].lastSeenMs);
    json += "}";
  }
  json += "],\"wifi_http_pollers\":[";
  for (uint8_t i = 0; i < wifiHttpPollerCount; i++) {
    if (i > 0) json += ",";
    json += "{\"ip\":\"";
    json += wifiHttpPollers[i].ip;
    json += "\",\"mac\":\"";
    json += wifiHttpPollers[i].mac.length() ? wifiHttpPollers[i].mac : "-";
    json += "\",\"via\":\"";
    json += wifiHttpPollers[i].via;
    json += "\",\"device\":\"";
    json += jsonEscape(wifiHttpPollers[i].deviceId.length() ? wifiHttpPollers[i].deviceId : "-");
    json += "\",\"user_agent\":\"";
    json += jsonEscape(wifiHttpPollers[i].userAgent.length() ? wifiHttpPollers[i].userAgent : "-");
    json += "\",\"poll_count\":";
    json += String(wifiHttpPollers[i].pollCount);
    json += ",\"age_ms\":";
    json += String(now - wifiHttpPollers[i].lastPollMs);
    json += ",\"first_poll_age_ms\":";
    json += String(now - wifiHttpPollers[i].firstPollMs);
    json += "}";
  }
  json += "],\"hub_event_count\":";
  json += String(hubEventCount);
#endif
  json += ",\"uart_command\":\"";
  json += lastUartCommand;
  json += "\",\"uart_response\":\"";
  json += lastUartResponse;
  json += "\",\"uart_state\":\"";
  json += lastUartState;
  json += "\",\"uart_age_ms\":";
  json += String(lastUartCommandMs == 0 ? 0UL : static_cast<unsigned long>(now - lastUartCommandMs));
  json += ",\"uart_response_age_ms\":";
  json += String(lastUartResponseMs == 0 ? 0UL : static_cast<unsigned long>(now - lastUartResponseMs));
#if ENABLE_BLE_HUB
  json += ",\"tune_connected\":";
  json += tuneConnected ? "true" : "false";
  json += ",\"tune_link_state\":\"";
  json += tuneLinkStateText(tuneLinkState);
  json += "\",\"tune_mode\":";
  json += tuneModeActive ? "true" : "false";
  json += ",\"tune_adv_steps\":";
  json += String(tuneAdvSteps);
  json += ",\"tune_fail_streak\":";
  json += String(tuneFailStreak);
  json += ",\"tune_rx\":";
  json += String(tune.rxCount);
  json += ",\"tune_age_ms\":";
  json += tune.lastRxMs == 0 ? "0" : String(now - tune.lastRxMs);
  json += ",\"tune_scan_seen\":";
  json += String(tuneScanSeen);
  json += ",\"tune_scan_candidates\":";
  json += String(tuneScanCandidates);
  json += ",\"tune_bad_length\":";
  json += String(tune.badLengthCount);
  json += ",\"tune_unknown_opcode\":";
  json += String(tune.unknownOpcodeCount);
  json += ",\"tune_saved_address\":\"";
  json += tuneSavedAddress;
  json += "\"";
  json += ",\"rpm\":";
  json += String(static_cast<int>(tune.rpm));
  json += ",\"advance\":";
  json += String(tune.advance, 1);
  json += ",\"map\":";
  json += String(static_cast<int>(tune.map));
  json += ",\"tune_temp\":";
  json += String(static_cast<int>(tune.temperature));
  json += ",\"volt\":";
  json += String(tune.voltage, 1);
  json += ",\"tune_amp\":";
  json += String(tune.coilCurrent, 2);
  json += ",\"uart_rx_pin\":";
  json += String(bridgeUartRxPin);
  json += ",\"uart_tx_pin\":";
  json += String(bridgeUartTxPin);
#endif
#if SPEED_REED_PIN >= 0
  json += ",\"speed_hz\":";
  json += String(speedHz, 2);
  json += ",\"speed_kmh\":";
  json += String(speedKmh, 1);
  json += ",\"speed_pulses\":";
  json += String(static_cast<unsigned long>(speedPulseCount));
  json += ",\"speed_tire_mm\":";
  json += String(tireCircMm);
  json += ",\"speed_trim_permil\":";
  json += String(speedTrimPermil);
  json += ",\"speed_pulses_per_rev\":";
  json += String(PULSES_PER_REV);
  json += ",\"odo_km\":";
  json += String(static_cast<double>(odoMm) / 1000000.0, 1);
  json += ",\"trip_km\":";
  json += String(static_cast<double>(tripMm) / 1000000.0, 2);
#endif
  // [KURVE] gecachte Slot-Bitmaske (refreshCurveSlotCache bei Boot/Upload/Delete)
  // statt 3x SPIFFS.exists() pro Poll -- statusJson laeuft mit 5-10 Hz.
  json += ",\"curve_slots\":";
  json += String((int)curveSlotMask);
  json += "}";
  return json;
}
