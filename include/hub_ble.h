#pragma once
// [BLE] 123TUNE+-Client (Scan/Connect/Notify/Decode/Tune-Cmds T/A/R) + BM6-
// Batterie-BLE + setupBleHub/updateBleHub, inkl. #if-Fallbacks. 1:1 aus main.cpp,
// an Originalstelle im anonymen namespace included -- keine Logikaenderung.
#if ENABLE_BLE_HUB
void decodeTuneFrame(const uint8_t *data, size_t length)
{
  if (length == 1 && data[0] == 0x0D) {
    return;
  }
  if (length < 3 || length > 20) {
    portENTER_CRITICAL(&stateMux);
    if (tuneBadLengthCount < UINT32_MAX) {
      tuneBadLengthCount++;
    }
    portEXIT_CRITICAL(&stateMux);
    return;
  }
  // Gemeinsamer Decoder (include/tune123_decode.h) — eine Quelle fuer alle Repos.
  const Tune123Decoded dec = tune123Decode(data[0],
                                           tune123HexNibble(data[1]),
                                           tune123HexNibble(data[2]));
  portENTER_CRITICAL(&stateMux);
  switch (dec.field) {
    case Tune123Field::Rpm:         tuneRpm = dec.value; break;
    case Tune123Field::Advance:     tuneAdvance = dec.value; break;
    case Tune123Field::Map:         tuneMap = dec.value; break;
    case Tune123Field::Temperature: tuneTemperature = dec.value; break;
    case Tune123Field::CoilCurrent: tuneCoilCurrent = dec.value; break;
    case Tune123Field::Voltage:     tuneVoltage = dec.value; break;
    case Tune123Field::None:        break;
  }
  if (!dec.known) {
    if (tuneUnknownOpcodeCount < UINT32_MAX) {
      tuneUnknownOpcodeCount++;
    }
  } else if (tuneRxCount < UINT32_MAX) {
    tuneRxCount++;
  }
  portEXIT_CRITICAL(&stateMux);
  // millis() ausserhalb der Critical Section — ist nicht zeitkritisch genug
  // um den Jitter zu rechtfertigen, spart aber Interrupt-Latenz.
  tuneLastRxMs = millis();
}

String bleStatusPayload()
{
#if ENABLE_BLE_DISPLAY
  const SpartanReading snapshot = readingSnapshot();
  const TuneSnapshot tune = tuneSnapshot();
  // Kompakt-Format fuer M5 Gateway:
  //   L<lambda>R<rpm>A<adv>M<map>  (Basis)
  //   V<bm6_volt>                  (BM6 Batterie main)
  //   W<bm6_aux_volt>              (BM6 Zusatzbatterie, time-sliced)
  //   S<speed_kmh>                 (Reed)
  //   I<123_volt>                  (123 interne Spannung)
  //   T<123_temp>                  (123 interne Temp)
  //   C<coil_current>              (123 Zuendspulenstrom)
  char payload[72];
  int n = snprintf(payload, sizeof(payload), "L%.2fR%dA%dM%d",
                   snapshot.valid ? snapshot.lambda : 0.0f,
                   static_cast<int>(tune.rpm),
                   static_cast<int>(tune.advance),
                   static_cast<int>(tune.map));
#if ENABLE_BM6
  const uint32_t nowMs = millis();
  if (n > 0 && n < static_cast<int>(sizeof(payload)) &&
      bm6LastRxMs != 0 && (nowMs - bm6LastRxMs) < kBm6CacheMaxAgeMs) {
    n += snprintf(payload + n, sizeof(payload) - n, "V%.2f", bm6Voltage);
  }
  if (n > 0 && n < static_cast<int>(sizeof(payload)) &&
      bm6AuxLastRxMs != 0 && (nowMs - bm6AuxLastRxMs) < kBm6CacheMaxAgeMs) {
    n += snprintf(payload + n, sizeof(payload) - n, "W%.2f", bm6AuxVoltage);
  }
#endif
#if SPEED_REED_PIN >= 0
  if (n > 0 && n < static_cast<int>(sizeof(payload)))
    n += snprintf(payload + n, sizeof(payload) - n, "S%d",
                  static_cast<int>(speedKmh + 0.5f));
#endif
  // 123TUNE+ interne Werte (nur wenn verbunden)
  if (tune.rxCount > 0 && n > 0 && n < static_cast<int>(sizeof(payload))) {
    snprintf(payload + n, sizeof(payload) - n, "I%.1fT%dC%.1f",
             tune.voltage, static_cast<int>(tune.temperature),
             tune.coilCurrent);
  }
  return String(payload);
#else
  return String("");
#endif
}

#if ENABLE_BLE_DISPLAY
class BleServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer *, NimBLEConnInfo &connInfo) override
  {
    addBleHubClient(connInfo);
    portENTER_CRITICAL(&stateMux);
    if (bleClientCount < UINT8_MAX) {
      bleClientCount++;
    }
    const uint8_t clients = bleClientCount;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("BLE hub:     client connected %s handle=%u mtu=%u count=%u\n",
                  connInfo.getAddress().toString().c_str(),
                  connInfo.getConnHandle(),
                  connInfo.getMTU(),
                  clients);
    char detail[48];
    snprintf(detail, sizeof(detail), "connect|%s|h%u", connInfo.getAddress().toString().c_str(),
             connInfo.getConnHandle());
    logHubEvent("ble_client", detail);
    // NimBLE stops advertising after the first connect — restart so more
    // cockpit displays (M5 + Waveshare) can join while slots remain.
    if (bleAdvertisingStarted && clients < kBleHubClientMax) {
      NimBLEDevice::startAdvertising();
      Serial.printf("BLE hub:     re-advertising for client %u/%u\n",
                    clients, kBleHubClientMax);
    }
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &connInfo, int) override
  {
    removeBleHubClient(connInfo.getConnHandle());
    portENTER_CRITICAL(&stateMux);
    if (bleClientCount > 0) {
      bleClientCount--;
    }
    const uint8_t clients = bleClientCount;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("BLE hub:     client disconnected %s handle=%u count=%u\n",
                  connInfo.getAddress().toString().c_str(),
                  connInfo.getConnHandle(),
                  clients);
    char detail[48];
    snprintf(detail, sizeof(detail), "disconnect|%s|h%u", connInfo.getAddress().toString().c_str(),
             connInfo.getConnHandle());
    logHubEvent("ble_client", detail);
    if (bleAdvertisingStarted) {
      NimBLEDevice::startAdvertising();
    }
  }
};

class BleCommandCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override
  {
    sendSpartanUartCommand(characteristic->getValue().c_str());
  }
};
#endif

void scheduleTuneScan(bool fastRetry = false)
{
  if (!fastRetry) {
    tuneFailStreak++;
  }
  // Backoff: max 5s damit Reconnect immer schnell passiert (war: bis 30s!)
  uint32_t delayMs = fastRetry ? 500UL : kTuneReconnectDelayMs;
  if (!fastRetry && tuneFailStreak > 1) {
    delayMs = kTuneReconnectDelayMs * min((uint32_t)tuneFailStreak, 2UL); // max 2× = 10s
    if (delayMs > 5000UL) delayMs = 5000UL;  // niemals länger als 5s
  }
  tuneNextScanMs = millis() + delayMs;
  setTuneLinkState(TuneLinkState::Idle, fastRetry ? "retry_fast" : "retry_scheduled");
}

void resetTuneClient()
{
  if (tuneClient == nullptr) return;
  if (tuneClient->isConnected()) {
    tuneClient->disconnect();
  }
  NimBLEDevice::deleteClient(tuneClient);
  tuneClient = nullptr;
  tuneNusRx = nullptr;
  tuneConnected = false;
  tuneLastPingMs = 0;
  tuneModeActive = false; tuneAdvSteps = 0;  // [TUNE-LIVE] Offset verfaellt bei Trennung
  setTuneLinkState(TuneLinkState::Idle, "reset");
}

class TuneClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onDisconnect(NimBLEClient *, int reason) override
  {
    tuneConnected = false;
    tuneNusRx = nullptr;
    tuneModeActive = false; tuneAdvSteps = 0;  // [TUNE-LIVE] Offset verfaellt bei Trennung
    char detail[24];
    snprintf(detail, sizeof(detail), "disconnect|r%d", reason);
    Serial.printf("123TUNE BLE: disconnected reason=%d\n", reason);
    logHubEvent("tune_state", detail);
    tuneFailStreak = 0;
#if BLE_BRIDGE
    // Bridge: kein Scan nach Disconnect — direkt auf bekannte MAC reconnecten.
    if (tuneSavedAddress.length() == 17) {
      tuneTargetAddress = NimBLEAddress(std::string(tuneSavedAddress.c_str()), BLE_ADDR_RANDOM);
      tuneDoConnect = true;
      Serial.println("123TUNE BLE: Bridge -> direkt reconnect (kein Scan)");
    } else {
      scheduleTuneScan(true);
    }
#else
    scheduleTuneScan(true);
#endif
  }
};

void onTuneNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool)
{
  // [KURVE-READ] Im Lese-Modus die Roh-Bytes sammeln (Blockantworten 10@..13@),
  // NICHT als Live-Frame dekodieren. Der Browser trennt spaeter Header/Hex/Rauschen.
  if (curveReadActive) {
    // Live-Frames sind ~5 Byte, Block-Antwort-Fragmente (10@..13@) ~20 Byte.
    // Filter length>8 -> nur die Block-Fragmente sammeln, Live-Rauschen raus.
    if (length > 8) {
      portENTER_CRITICAL(&stateMux);
      for (size_t i = 0; i < length && curveReadLen < sizeof(curveReadBuf); i++) {
        curveReadBuf[curveReadLen++] = static_cast<char>(data[i]);
      }
      portEXIT_CRITICAL(&stateMux);
    }
    return;
  }
  Serial.printf("123TUNE BLE: notify len=%u :", static_cast<unsigned>(length));
  for (size_t i = 0; i < length && i < 20; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
  decodeTuneFrame(data, length);
  if (tuneLinkState == TuneLinkState::Subscribed) {
    setTuneLinkState(TuneLinkState::Streaming, "notify");
  }
}

class TuneScanCallbacks : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice *device) override
  {
    tuneScanSeen++;
    String address = device->getAddress().toString().c_str();
    address.toLowerCase();
    String name = device->getName().c_str();
    name.toLowerCase();
    const bool addressMatches = address == kTuneTargetAddress;
    const bool savedAddressMatches = tuneSavedAddress.length() > 0 && address == tuneSavedAddress;
    const bool advertisesNus = device->isAdvertisingService(NimBLEUUID(kTuneNusServiceUuid));
    const bool nameLooksLikeTune = name.indexOf("123") >= 0 || name.indexOf("tune") >= 0 || name.indexOf("raytac") >= 0;
    recordBleScanDevice(device, advertisesNus || nameLooksLikeTune || addressMatches || savedAddressMatches, false);
    if (!addressMatches && !savedAddressMatches && !(advertisesNus && nameLooksLikeTune)) {
      return;
    }

    tuneTargetAddress = device->getAddress();
    if (tuneSavedAddress != address) {
      tuneSavedAddress = address;
#if ENABLE_WEB_GUI
      ensurePreferences();
      networkPreferences.putString("tune_mac", tuneSavedAddress);
#endif
      Serial.printf("123TUNE BLE: saved address %s\n", tuneSavedAddress.c_str());
    }
    tuneDoConnect = true;
    tuneNextScanMs = 0;
    tuneScanCandidates++;
    NimBLEDevice::getScan()->stop();
    markBleCentralScanEnded();
    Serial.printf("123TUNE BLE: found %s name='%s' nus=%d addrMatch=%d\n",
                  address.c_str(),
                  name.c_str(),
                  advertisesNus ? 1 : 0,
                  (addressMatches || savedAddressMatches) ? 1 : 0);
  }

  void onScanEnd(const NimBLEScanResults &, int reason) override
  {
    if (!tuneConnected && !tuneDoConnect) {
      markBleCentralScanEnded();
      Serial.printf("123TUNE BLE: scan end reason=%d seen=%lu candidates=%lu\n",
                    reason,
                    static_cast<unsigned long>(tuneScanSeen),
                    static_cast<unsigned long>(tuneScanCandidates));
      scheduleTuneScan(false);
    } else {
      markBleCentralScanEnded();
    }
  }
};

TuneClientCallbacks tuneClientCallbacks;
TuneScanCallbacks tuneScanCallbacks;

void startTuneScan()
{
  if (tuneConnected || tuneDoConnect || bleCentralScanActive) return;
  pauseBleDisplayAdvertising();
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    scan->stop();
    delay(50);
  }
  scan->setScanCallbacks(&tuneScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(24);
  tuneScanSeen = 0;
  tuneScanCandidates = 0;
  bleCentralScanActive = true;
  setTuneLinkState(TuneLinkState::Scanning, "start");
  Serial.println("123TUNE BLE: scan 6s (15% duty)...");
  if (!scan->start(kTuneScanWindowMs, false)) {
    Serial.println("123TUNE BLE: scan start failed");
    markBleCentralScanEnded();
    scheduleTuneScan(false);
  }
}

void connectTune()
{
  tuneDoConnect = false;
  if (tuneConnected) return;
  markBleCentralScanEnded();
  pauseBleDisplayAdvertising();
  setTuneLinkState(TuneLinkState::Connecting, tuneTargetAddress.toString().c_str());
  if (tuneClient != nullptr && !tuneClient->isConnected()) {
    resetTuneClient();
  }
  if (tuneClient == nullptr) {
    tuneClient = NimBLEDevice::createClient();
    tuneClient->setClientCallbacks(&tuneClientCallbacks, false);
  }
  // Echte 123TUNE+ stabilisiert sich auf 30ms Interval, Supervision 4000ms.
  // min=24(30ms) max=30(37.5ms) enger als vorher (16-40ms) → weniger Jitter.
  tuneClient->setConnectionParams(24, 30, 0, 400);
  Serial.println("123TUNE BLE: connecting...");
  if (!tuneClient->connect(tuneTargetAddress, true, false, false)) {
    Serial.println("123TUNE BLE: connect failed");
    resetTuneClient();
    scheduleTuneScan(false);
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(1000));  // non-blocking: andere Tasks (WebServer) laufen weiter

  NimBLERemoteService *service = tuneClient->getService(kTuneNusServiceUuid);
  if (service == nullptr) {
    Serial.println("123TUNE BLE: NUS service missing");
    tuneClient->disconnect();
    scheduleTuneScan(false);
    return;
  }
  NimBLERemoteCharacteristic *tx = service->getCharacteristic(kTuneNusTxUuid);
  tuneNusRx = service->getCharacteristic(kTuneNusRxUuid);
  if (tx == nullptr || !tx->canNotify()) {
    Serial.println("123TUNE BLE: NUS TX notify missing");
    tuneClient->disconnect();
    scheduleTuneScan(false);
    return;
  }
  if (tuneNusRx == nullptr) {
    Serial.println("123TUNE BLE: NUS RX missing");
  } else {
    Serial.printf("123TUNE BLE: RX h=%u W=%d WNR=%d\n",
                  tuneNusRx->getHandle(),
                  tuneNusRx->canWrite(),
                  tuneNusRx->canWriteNoResponse());
  }
  Serial.printf("123TUNE BLE: TX h=%u N=%d I=%d\n",
                tx->getHandle(),
                tx->canNotify(),
                tx->canIndicate());

  NimBLERemoteDescriptor *cccd = tx->getDescriptor(NimBLEUUID(static_cast<uint16_t>(0x2902)));
  if (cccd != nullptr) {
    uint16_t off = 0x0000;
    bool offOk = cccd->writeValue(reinterpret_cast<uint8_t *>(&off), 2, true);
    Serial.printf("123TUNE BLE: CCCD off %s\n", offOk ? "OK" : "FAIL");
    vTaskDelay(pdMS_TO_TICKS(150));
  } else {
    Serial.println("123TUNE BLE: CCCD missing");
  }

  const bool ok = tx->subscribe(true, onTuneNotify, true);
  tuneConnected = ok;
  Serial.printf("123TUNE BLE: subscribe %s\n", ok ? "OK" : "FAIL");
  if (!ok) {
    tuneClient->disconnect();
    scheduleTuneScan(false);
    return;
  }
  tuneConnStartMs = millis();

  tuneFailStreak = 0;
  tuneStaleReconnectMs = 0;
  setTuneLinkState(TuneLinkState::Subscribed, "ok");
  resumeBleDisplayAdvertising();

  if (cccd != nullptr) {
    NimBLEAttValue value = cccd->readValue();
    Serial.printf("123TUNE BLE: CCCD read len=%u :", static_cast<unsigned>(value.length()));
    for (size_t i = 0; i < value.length(); i++) {
      Serial.printf(" %02X", value.data()[i]);
    }
    Serial.println();
  }

  if (tuneNusRx != nullptr) {
    const uint8_t ping = '$';
    const uint8_t enter = '\r';
    const bool pingOk = tuneNusRx->writeValue(&ping, 1, false);
    vTaskDelay(pdMS_TO_TICKS(120));
    const bool enterOk = tuneNusRx->writeValue(&enter, 1, false);
    tuneLastPingMs = millis();
    Serial.printf("123TUNE BLE: wake '$' %s, CR %s\n",
                  pingOk ? "OK" : "FAIL",
                  enterOk ? "OK" : "FAIL");
  }
}

void sendTunePing()
{
  if (!tuneConnected || tuneClient == nullptr || !tuneClient->isConnected() || tuneNusRx == nullptr) return;
  const uint32_t now = millis();
  if (now - tuneLastPingMs < kTunePingIntervalMs) return;
  tuneLastPingMs = now;

  const uint8_t ping = '$';
  // Write OHNE Response (false): unter Notify-Flut (Motor laeuft) kollidiert ein
  // Write-MIT-Response mit dem Notify-Strom -> GATT-Prozedurfehler -> lokale
  // Terminierung (reason 534). Die 123-RX unterstuetzt WRITE_NO_RESPONSE.
  const bool ok = tuneNusRx->writeValue(&ping, 1, false);
  Serial.printf("123TUNE BLE: ping -> %s\n", ok ? "OK" : "FAIL");
}

bool sendTuneCommand(const char *command)
{
  if (!tuneConnected || tuneClient == nullptr || !tuneClient->isConnected() || tuneNusRx == nullptr) {
    Serial.println("123TUNE BLE: command blocked, not connected");
    return false;
  }

  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%s\r$", command);
  const bool ok = tuneNusRx->writeValue(reinterpret_cast<uint8_t *>(buffer), strlen(buffer), true);
  Serial.printf("123TUNE BLE: cmd '%s\\r$' -> %s\n", command, ok ? "OK" : "FAIL");
  tuneLastPingMs = millis();
  return ok;
}

// [TUNE-LIVE] Ein einzelnes Roh-Kommandozeichen an die 123 senden (T/A/R), OHNE
// Terminator/Checksum und OHNE Response (write-no-response) -- exakt wie es die
// 123TUNE+ App macht und wie der $-Ping, damit es unter der Notify-Flut nicht mit
// reason 534 kollidiert. Nur erlaubt, wenn die 123 wirklich streamt (frische Daten).
bool tuneStreaming()
{
  return tuneConnected && tuneClient != nullptr && tuneClient->isConnected() &&
         tuneNusRx != nullptr && tuneLinkState == TuneLinkState::Streaming;
}

bool sendTuneRaw(char c)
{
  if (!tuneStreaming()) {
    Serial.printf("123TUNE BLE: tune-cmd '%c' blockiert (kein streaming)\n", c);
    return false;
  }
  const uint8_t b = static_cast<uint8_t>(c);
  const bool ok = tuneNusRx->writeValue(&b, 1, false);
  Serial.printf("123TUNE BLE: tune-cmd '%c' -> %s\n", c, ok ? "OK" : "FAIL");
  return ok;
}

// Tuning-Modus umschalten (T). Beim Verlassen loescht das Geraet den Offset.
bool tuneModeToggle()
{
  if (!sendTuneRaw('T')) return false;
  tuneModeActive = !tuneModeActive;
  if (!tuneModeActive) tuneAdvSteps = 0;   // verlassen -> Offset weg
  logHubEvent("tune_mode", tuneModeActive ? "on" : "off");
  return true;
}

// Zuendung vor (A) / zurueck (R) um einen Schritt -- nur im Tuning-Modus.
bool tuneAdvStep(int dir)
{
  if (!tuneModeActive) { Serial.println("123TUNE BLE: adv-step blockiert (Tuning-Modus aus)"); return false; }
  const char c = (dir >= 0) ? 'A' : 'R';
  if (!sendTuneRaw(c)) return false;
  tuneAdvSteps += (dir >= 0) ? 1 : -1;
  return true;
}

// Offset auf 0: die gezaehlten Schritte gegenlaeufig zuruecksenden (bleibt im
// Tuning-Modus). Kleiner Abstand zwischen den Writes gegen Notify-Kollision.
bool tuneAdvReset()
{
  if (!tuneModeActive) return false;
  int guard = 0;
  while (tuneAdvSteps != 0 && guard++ < 60) {
    const char c = (tuneAdvSteps > 0) ? 'R' : 'A';
    if (!sendTuneRaw(c)) return false;
    tuneAdvSteps += (tuneAdvSteps > 0) ? -1 : 1;
    delay(30);
  }
  logHubEvent("tune_reset", "0");
  return true;
}

// [KURVE-READ] EEPROM-Kurvenlesung starten (nur wenn 123 streamt). updateTuneBle()
// staffelt dann die Bloecke 10@..13@; onTuneNotify sammelt die Antworten.
bool startCurveRead()
{
  if (!tuneStreaming()) return false;
  portENTER_CRITICAL(&stateMux);
  curveReadLen = 0;
  portEXIT_CRITICAL(&stateMux);
  curveReadStartMs = millis();
  curveReadPhase = 1;
  curveReadActive = true;
  Serial.println("Kurve-Read:  Start (10@..13@)");
  logHubEvent("curve_read", "start");
  return true;
}

// Pump: aus updateTuneBle() aufrufen. ADAPTIV: die echte 123 sendet jede Block-
// Antwort in 3 Fragmenten (~59 B); ein zu frueh gesendeter Folgebefehl wuergt die
// restlichen Fragmente ab (real beobachtet: nur 4x20 B kamen an). Deshalb den
// naechsten Block erst schicken, wenn die Antwort ruhig ist (>=350 ms keine neuen
// Bytes) oder 1 s Block-Timeout. Gesamtfenster 6 s.
void pumpCurveRead()
{
  if (!curveReadActive) return;
  const uint32_t now = millis();
  static uint16_t lastLen = 0;
  static uint32_t lastGrowMs = 0, blockSentMs = 0;
  static const char *B[4] = { "10@\r", "11@\r", "12@\r", "13@\r" };
  if (curveReadPhase == 1 && blockSentMs != 0 && curveReadStartMs > blockSentMs) blockSentMs = 0;  // neuer Lauf
  if (curveReadLen != lastLen) { lastLen = curveReadLen; lastGrowMs = now; }
  const bool blockDone = (blockSentMs == 0) ||                                  // noch nichts gesendet
                         (lastGrowMs > blockSentMs && now - lastGrowMs >= 350) || // Antwort kam + ist ruhig
                         (now - blockSentMs >= 1000);                            // Block-Timeout
  if (curveReadPhase >= 1 && curveReadPhase <= 4 && blockDone) {
    if (tuneNusRx) tuneNusRx->writeValue(reinterpret_cast<const uint8_t *>(B[curveReadPhase - 1]), 4, false);
    blockSentMs = now;
    Serial.printf("Kurve-Read:  Block %d gesendet (len=%u)\n", curveReadPhase, static_cast<unsigned>(curveReadLen));
    curveReadPhase++;
  }
  const bool lastBlockDone = curveReadPhase > 4 &&
                             ((lastGrowMs > blockSentMs && now - lastGrowMs >= 400) || now - blockSentMs >= 1200);
  if (lastBlockDone || now - curveReadStartMs > 6000) {
    curveReadActive = false;
    curveReadPhase = 0;
    blockSentMs = 0;
    Serial.printf("Kurve-Read:  Fenster zu, %u Bytes erfasst\n", static_cast<unsigned>(curveReadLen));
    logHubEvent("curve_read", "done");
  }
}

void runTuneReadDump()
{
  Serial.println("123TUNE BLE: read dump v@ 10@ 11@ 12@ 13@");
  sendTuneCommand("v@");
  delay(500);
  sendTuneCommand("10@");
  delay(500);
  sendTuneCommand("11@");
  delay(500);
  sendTuneCommand("12@");
  delay(500);
  sendTuneCommand("13@");
}

#if ENABLE_BM6
bool bm6AesEncrypt(const uint8_t in[16], uint8_t out[16])
{
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (mbedtls_aes_setkey_enc(&ctx, kBm6AesKey, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  uint8_t iv[16] = {0};
  const int rc = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, 16, iv, in, out);
  mbedtls_aes_free(&ctx);
  return rc == 0;
}

bool bm6AesDecrypt(const uint8_t in[16], uint8_t out[16])
{
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (mbedtls_aes_setkey_dec(&ctx, kBm6AesKey, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  uint8_t iv[16] = {0};
  const int rc = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, 16, iv, in, out);
  mbedtls_aes_free(&ctx);
  return rc == 0;
}

void scheduleBm6Scan()
{
  bm6NextScanMs = millis() + kBm6ReconnectDelayMs;
}

void resetBm6Client();

bool bm6AuxConfigured()
{
  return looksLikeMacAddress(bm6AuxSavedAddress);
}

const String &bm6ActiveSavedMac()
{
  return (bm6ActiveSlot == 0) ? bm6SavedAddress : bm6AuxSavedAddress;
}

void rotateBm6Slot()
{
  if (!bm6AuxConfigured()) return;
  bm6ActiveSlot = bm6ActiveSlot == 0 ? 1 : 0;
  bm6SlotStartedMs = millis();
  resetBm6Client();
  bm6DoConnect = false;
  scheduleBm6Scan();
  Serial.printf("BM6 BLE:     rotate -> %s addr=%s\n",
                bm6ActiveSlot == 0 ? "main" : "aux",
                bm6ActiveSavedMac().c_str());
}

void maybeRotateBm6Slot()
{
  if (!bm6AuxConfigured()) return;
  const uint32_t slotMs = bm6ActiveSlot == 0 ? kBm6MainSlotMs : kBm6AuxSlotMs;
  if (bm6SlotStartedMs == 0) {
    bm6SlotStartedMs = millis();
    return;
  }
  if (millis() - bm6SlotStartedMs < slotMs) return;
  rotateBm6Slot();
}

void resetBm6Client()
{
  if (bm6Client == nullptr) return;
  if (bm6Client->isConnected()) {
    bm6Client->disconnect();
  }
  NimBLEDevice::deleteClient(bm6Client);
  bm6Client = nullptr;
  bm6WriteChar = nullptr;
  bm6Connected = false;
  bm6LastTriggerMs = 0;
}

class Bm6ClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onDisconnect(NimBLEClient *, int reason) override
  {
    bm6Connected = false;
    bm6WriteChar = nullptr;
    Serial.printf("BM6 BLE:     disconnected reason=%d\n", reason);
    char detail[24];
    snprintf(detail, sizeof(detail), "disconnect|r%d", reason);
    logHubEvent("bm6_state", detail);
    scheduleBm6Scan();
  }
};

Bm6ClientCallbacks bm6ClientCallbacks;

void decodeBm6Notify(const uint8_t *data, size_t length)
{
  if (length != 16) {
    bm6DecodeFailCount++;
    Serial.printf("BM6 BLE:     notify wrong len=%u\n", static_cast<unsigned>(length));
    return;
  }

  uint8_t plain[16] = {0};
  if (!bm6AesDecrypt(data, plain)) {
    bm6DecodeFailCount++;
    Serial.println("BM6 BLE:     AES decrypt fail");
    return;
  }

  if (plain[0] == 0xD1 && plain[1] == 0x55 && plain[2] == 0x07) {
    const bool tempNeg = plain[3] != 0;
    const int8_t tempVal = static_cast<int8_t>(plain[4]);
    const uint16_t voltRaw = (static_cast<uint16_t>(plain[7]) << 8) | plain[8];
    const float volt = static_cast<float>(voltRaw) / 100.0f;
    const int8_t temp = tempNeg ? -tempVal : tempVal;
    const uint32_t now = millis();
    if (bm6ActiveSlot == 0) {
      bm6Voltage = volt;
      bm6Temperature = temp;
      bm6LastRxMs = now;
      bm6RxCount++;
      Serial.printf("BM6 BLE:     main V=%.2fV T=%d C rx=%lu\n",
                    bm6Voltage,
                    static_cast<int>(bm6Temperature),
                    static_cast<unsigned long>(bm6RxCount));
    } else {
      bm6AuxVoltage = volt;
      bm6AuxTemperature = temp;
      bm6AuxLastRxMs = now;
      bm6AuxRxCount++;
      Serial.printf("BM6 BLE:     aux  V=%.2fV T=%d C rx=%lu\n",
                    bm6AuxVoltage,
                    static_cast<int>(bm6AuxTemperature),
                    static_cast<unsigned long>(bm6AuxRxCount));
    }
  } else {
    bm6DecodeFailCount++;
    Serial.printf("BM6 BLE:     unknown frame %02X %02X %02X\n", plain[0], plain[1], plain[2]);
  }
}

void onBm6Notify(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool)
{
  decodeBm6Notify(data, length);
}

bool bm6SendTrigger()
{
  if (!bm6Connected || bm6WriteChar == nullptr) return false;
  uint8_t cipher[16] = {0};
  if (!bm6AesEncrypt(kBm6TriggerPlain, cipher)) {
    Serial.println("BM6 BLE:     trigger encrypt fail");
    return false;
  }
  const bool ok = bm6WriteChar->writeValue(cipher, 16, true);
  bm6LastTriggerMs = millis();
  Serial.printf("BM6 BLE:     trigger -> %s\n", ok ? "OK" : "FAIL");
  return ok;
}

class Bm6ScanCallbacks : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice *device) override
  {
    String addr = device->getAddress().toString().c_str();
    addr.toLowerCase();
    String name = device->getName().c_str();
    name.toLowerCase();
    const bool bm6Like = addr == bm6SavedAddress || addr == bm6AuxSavedAddress ||
                         addr == kBm6TargetAddress ||
                         name.indexOf("bm6") >= 0 || name.indexOf("battery") >= 0;
    recordBleScanDevice(device, false, bm6Like);
    if (addr != bm6ActiveSavedMac()) return;
    bm6TargetAddress = device->getAddress();
    if (bm6ActiveSlot == 0 && bm6SavedAddress != addr) {
      bm6SavedAddress = addr;
#if ENABLE_WEB_GUI
      ensurePreferences();
      networkPreferences.putString("bm6_mac", bm6SavedAddress);
#endif
      Serial.printf("BM6 BLE:     saved main address %s\n", bm6SavedAddress.c_str());
    }
    bm6DoConnect = true;
    bm6NextScanMs = 0;
    NimBLEDevice::getScan()->stop();
    markBleCentralScanEnded();
    Serial.printf("BM6 BLE:     found %s\n", addr.c_str());
  }

  void onScanEnd(const NimBLEScanResults &, int reason) override
  {
    if (!bm6Connected && !bm6DoConnect) {
      markBleCentralScanEnded();
      Serial.printf("BM6 BLE:     scan end reason=%d\n", reason);
      scheduleBm6Scan();
    } else {
      markBleCentralScanEnded();
    }
  }
};

Bm6ScanCallbacks bm6ScanCallbacks;

void startBm6Scan()
{
  if (bm6Connected || bm6DoConnect) return;
  if (!bleRadioFreeForBm6Scan()) {
    scheduleBm6Scan();
    return;
  }
  pauseBleDisplayAdvertising();
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    scan->stop();
    delay(50);
  }
  scan->setScanCallbacks(&bm6ScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(24);
  bleCentralScanActive = true;
  logHubEvent("bm6_state", "scan|start");
  Serial.println("BM6 BLE:     scan 10s...");
  if (!scan->start(kBm6ScanWindowMs, false)) {
    Serial.println("BM6 BLE:     scan start failed");
    markBleCentralScanEnded();
    scheduleBm6Scan();
  }
}

void connectBm6()
{
  bm6DoConnect = false;
  if (bm6Connected) return;
  if (bm6Client != nullptr && !bm6Client->isConnected()) {
    resetBm6Client();
  }
  if (bm6Client == nullptr) {
    bm6Client = NimBLEDevice::createClient();
    bm6Client->setClientCallbacks(&bm6ClientCallbacks, false);
  }

  // BM6 bewusst entspannt takten: es liefert nur alle ~2 s (kBm6TriggerIntervalMs)
  // einen Spannungswert. Das frühere 6/12 (7,5–15 ms) belegte fast die gesamte
  // Funkzeit und zwang den NimBLE-Host, den parallelen 123-Link periodisch zu
  // kappen (disconnect reason 534 = local host). 24/48 (30–60 ms) mit slave
  // latency 2 lässt den 123-Link durchgehend stabil laufen, BM6 bleibt frisch.
  bm6Client->setConnectionParams(24, 48, 2, 400);
  const String &targetMac = bm6ActiveSavedMac();
  if (looksLikeMacAddress(targetMac)) {
    bm6TargetAddress = NimBLEAddress(std::string(targetMac.c_str()), BLE_ADDR_PUBLIC);
  }
  Serial.printf("BM6 BLE:     connecting %s (%s)...\n",
                targetMac.c_str(),
                bm6ActiveSlot == 0 ? "main" : "aux");
  if (!bm6Client->connect(bm6TargetAddress, true, false, false)) {
    Serial.println("BM6 BLE:     connect failed");
    resetBm6Client();
    scheduleBm6Scan();
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(500));

  NimBLERemoteService *svc = bm6Client->getService(kBm6ServiceUuid);
  if (svc == nullptr) {
    Serial.println("BM6 BLE:     FFF0 service missing");
    bm6Client->disconnect();
    scheduleBm6Scan();
    return;
  }
  bm6WriteChar = svc->getCharacteristic(kBm6WriteUuid);
  NimBLERemoteCharacteristic *notifyChar = svc->getCharacteristic(kBm6NotifyUuid);
  if (bm6WriteChar == nullptr || notifyChar == nullptr || !notifyChar->canNotify()) {
    Serial.println("BM6 BLE:     FFF3/FFF4 missing");
    bm6Client->disconnect();
    scheduleBm6Scan();
    return;
  }

  const bool subOk = notifyChar->subscribe(true, onBm6Notify, true);
  Serial.printf("BM6 BLE:     subscribe %s\n", subOk ? "OK" : "FAIL");
  if (!subOk) {
    bm6Client->disconnect();
    scheduleBm6Scan();
    return;
  }
  bm6Connected = true;
  bm6SlotStartedMs = millis();
  logHubEvent("bm6_state", bm6ActiveSlot == 0 ? "connect|main" : "connect|aux");
  bm6SendTrigger();
}

void updateBm6Ble()
{
  // Die 123 hat Vorrang. BM6 ist nachrangig:
  //  - bestehende BM6-Verbindung wird gehalten (nur Trigger),
  //  - Slot-Rotation (disconnect+rescan) nur wenn die 123 NICHT verbunden ist,
  //  - 123 bekommt beim Boot 30 s exklusiv; danach darf BM6 auch ohne 123
  //    kurze Scanfenster nutzen. Das haelt Batterie-Daten im Tischtest sichtbar,
  //    waehrend bleRadioFreeForBm6Scan() parallele Scans verhindert.
  const uint32_t now = millis();
  if (bm6Connected) {
    if (now - bm6LastRxMs > 5000 && now - bm6LastTriggerMs > bm6PollIntervalMs) {
      bm6SendTrigger();
    }
    if (!tuneConnected) maybeRotateBm6Slot();
    return;
  }
  const bool bm6FallbackAllowed = tuneConnected || (now - bm6SlotStartedMs >= 30000UL);
  if (!bm6FallbackAllowed) return;
  if (bm6DoConnect) {
    connectBm6();
    return;
  }
  if (bm6NextScanMs != 0 && static_cast<int32_t>(now - bm6NextScanMs) >= 0) {
    bm6NextScanMs = 0;
    startBm6Scan();
  }
}
#endif

void updateTuneBle()
{
  const uint32_t now = millis();
  if (tuneDoConnect) {
    connectTune();
    return;
  }
  pumpCurveRead();  // [KURVE-READ] Blockbefehle staffeln, Fenster schliessen
  // Waehrend der Kurvenlesung keine $-Pings/Stale-Logik dazwischenfunken.
  if (curveReadActive) return;
  sendTunePing();
  if (tuneConnected) {
    const TuneSnapshot tune = tuneSnapshot();
    // Frische relativ zur AKTUELLEN Verbindung messen: lastRxMs kann noch von der
    // vorigen Verbindung stammen (wird nicht beim Connect zurückgesetzt). Ohne diesen
    // Boden feuerte stale_rx fälschlich ~alle 2 s und kappte eine voll funktionierende
    // Verbindung (disconnect reason 534, hubInitiated). tuneConnStartMs gibt der neuen
    // Verbindung ein volles kTuneStaleRxMs-Fenster, bevor sie als stale gilt.
    const uint32_t tuneRxRef = (tune.lastRxMs > tuneConnStartMs) ? tune.lastRxMs : tuneConnStartMs;
    if (tuneConnStartMs != 0 && (now - tuneRxRef) > kTuneStaleRxMs &&
        tune.rpm >= ENGINE_RUNNING_RPM_THRESHOLD) {
      if (tuneStaleReconnectMs == 0 || (now - tuneStaleReconnectMs) > 5000) {
        tuneStaleReconnectMs = now;
        logHubEvent("tune_state", "stale|rx_timeout");
        resetTuneClient();
        scheduleTuneScan(true);
        return;
      }
    }
    return;
  }
  if (tuneNextScanMs != 0 && static_cast<int32_t>(now - tuneNextScanMs) >= 0) {
    if (!bleCentralScanActive && !tuneDoConnect) {
      tuneNextScanMs = 0;
      startTuneScan();
    } else {
      // Funk gerade vom BM6-Scan belegt (nur EIN zentraler Scanner). Timer NICHT
      // löschen, sonst bleibt die 123 dauerhaft ohne Rescan hängen. Kurz erneut
      // versuchen — 123 hat Vorrang und greift sich den Scanner, sobald frei.
      tuneNextScanMs = now + 300;
    }
  }
}

void updateTuneDebug()
{
  const uint32_t now = millis();
  if (now - lastTuneDebugMs < 2000) return;
  lastTuneDebugMs = now;
  const TuneSnapshot tune = tuneSnapshot();
#if defined(ENABLE_BLE_HUB)
  // Dynamische Coexistenz: Motor läuft = BLE Vorrang (kein WiFi-Jitter).
  // Motor aus = Balance (WebGUI erreichbar für Konfiguration).
  static bool lastEngineRunning = false;
  const bool engineRunning = tune.rpm > ENGINE_RUNNING_RPM_THRESHOLD;
  if (engineRunning != lastEngineRunning) {
    lastEngineRunning = engineRunning;
    if (engineRunning) {
      esp_coex_preference_set(ESP_COEX_PREFER_BT);
      Serial.println("Coex:        Motor läuft -> PREFER_BT (max BLE-Stabilität)");
    } else {
      esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
      Serial.println("Coex:        Motor aus -> PREFER_BALANCE (WebGUI aktiv)");
    }
  }
#endif
  Serial.printf("123TUNE BLE: conn=%d rx=%lu rpm=%d adv=%.1f map=%d age=%lu\n",
                tuneConnected ? 1 : 0,
                static_cast<unsigned long>(tune.rxCount),
                static_cast<int>(tune.rpm),
                tune.advance,
                static_cast<int>(tune.map),
                tune.lastRxMs == 0 ? 0UL : static_cast<unsigned long>(now - tune.lastRxMs));
}

void startBleHubAdvertising()
{
#if ENABLE_BLE_DISPLAY
  if (bleAdvertisingStarted || bleCentralScanActive || tuneDoConnect) return;
  if (tuneLinkState == TuneLinkState::Scanning || tuneLinkState == TuneLinkState::Connecting) return;
  NimBLEDevice::startAdvertising();
  bleAdvertisingStarted = true;
  bleAdvPausedForRadio = false;
  Serial.println("BLE hub:     advertising started");
  logHubEvent("ble_client", "advertising|start");
#endif
}


void setupBleHub()
{
  NimBLEDevice::init(BLE_HUB_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  // MTU 69 statt 23: die echte 123 sendet EEPROM-Blockantworten (10@..13@) als EINE
  // ~59-Byte-Notify -- bei MTU 23 schneidet der Stack auf 20 B ab (Kurve-Read bekam
  // real nur 4x20 B pro Block). 69 = 59 + ATT-Header + Reserve; bewusst knapp, um
  // das Verhalten des Live-Streams (5-B-Frames) nicht zu veraendern.
  NimBLEDevice::setMTU(69);
  bleAddress = NimBLEDevice::getAddress().toString().c_str();
#if ENABLE_BLE_DISPLAY
  bleHubSetupMs = millis();
#endif
#if ENABLE_WEB_GUI
  ensurePreferences();
  tuneSavedAddress = networkPreferences.isKey("tune_mac")
      ? networkPreferences.getString("tune_mac", kTuneTargetAddress)
      : String(kTuneTargetAddress);
  tuneSavedAddress.toLowerCase();
#if ENABLE_BM6
  bm6SavedAddress = networkPreferences.isKey("bm6_mac")
      ? networkPreferences.getString("bm6_mac", kBm6TargetAddress)
      : String(kBm6TargetAddress);
  bm6SavedAddress.toLowerCase();
  bm6AuxSavedAddress = networkPreferences.isKey("bm6_aux_mac")
      ? networkPreferences.getString("bm6_aux_mac", "")
      : String("");
  bm6AuxSavedAddress.toLowerCase();
  bm6SlotStartedMs = millis();
#endif
#else
  tuneSavedAddress = kTuneTargetAddress;
#if ENABLE_BM6
  bm6SavedAddress = kBm6TargetAddress;
#endif
#endif

#if ENABLE_BLE_DISPLAY
  NimBLEServer *bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  NimBLEService *service = bleServer->createService(kBleServiceUuid);
  bleStatusCharacteristic = service->createCharacteristic(
      kBleStatusUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *commandCharacteristic = service->createCharacteristic(
      kBleCommandUuid,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  commandCharacteristic->setCallbacks(new BleCommandCallbacks());

  bleServer->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(kBleServiceUuid);
  advertising->setName(BLE_HUB_NAME);
#else
  Serial.println("BLE display: disabled, cockpit uses ESP-NOW broadcast");
#endif

  Serial.printf("BLE stack:   '%s' addr=%s\n", BLE_HUB_NAME, bleAddress.c_str());
#if ENABLE_BLE_DISPLAY
  Serial.printf("BLE hub:     service=%s\n", kBleServiceUuid);
  Serial.println("BLE hub:     advertising waits for 123TUNE or 30s fallback");
#endif
  Serial.printf("123TUNE BLE: target %s\n", tuneSavedAddress.c_str());
  logHubEvent("tune_state", "boot|target_saved");
  if (hubFeatBle123) {
    startTuneScan();
  } else {
    Serial.println("123TUNE BLE: disabled (Setup — Display verbindet direkt)");
  }
#if ENABLE_BM6
  Serial.printf("BM6 BLE:     target %s\n", bm6SavedAddress.c_str());
  if (hubFeatBleBm6) {
    scheduleBm6Scan();
  } else {
    Serial.println("BM6 BLE:     disabled (Setup)");
  }
#endif
}

void updateBleHub()
{
  if (hubFeatBle123) {
    updateTuneBle();
  } else if (tuneConnected || tuneDoConnect) {
    resetTuneClient();
  }
#if ENABLE_BM6
  if (hubFeatBleBm6) {
    updateBm6Ble();
  } else if (bm6Connected || bm6DoConnect) {
    resetBm6Client();
  }
#endif
  updateTuneDebug();
#if ENABLE_BLE_DISPLAY
  const uint32_t now = millis();
  if (!bleAdvertisingStarted &&
      (tuneConnected || static_cast<int32_t>(now - bleHubSetupMs - kBleHubAdvertiseFallbackMs) >= 0)) {
    startBleHubAdvertising();
  }
  if (bleStatusCharacteristic == nullptr || now - lastBleNotifyMs < kBleNotifyIntervalMs) {
    return;
  }
  lastBleNotifyMs = now;

  const String payload = bleStatusPayload();
  bleStatusCharacteristic->setValue(payload.c_str());
  if (getBleClientCount() > 0) {
    bleStatusCharacteristic->notify();
  }
#endif
}
#else
void setupBleHub() {}
void updateBleHub() {}
#endif
