#pragma once
// [BLE] 123TUNE+-Client (Scan/Connect/Notify/Decode/Tune-Cmds T/A/R) +
// setupBleHub/updateBleHub, inkl. #if-Fallbacks. 1:1 aus main.cpp,
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

// [KURVE-READ] Laufende Kurvenlesung sofort abbrechen (Disconnect/Reset). Ohne
// das blockiert curveReadActive in updateTuneBle() den Reconnect bis zum 6-s-
// Fenster, und der halbe Puffer wuerde spaeter als fertige Kurve dekodiert.
// Teilstand wird verworfen -> Browser meldet "keine Kurvendaten" statt Falschkurve.
void abortCurveRead(const char *why)
{
  if (!curveReadActive) return;
  curveReadActive = false;
  curveReadPhase = 0;
  portENTER_CRITICAL(&stateMux);
  curveReadLen = 0;
  portEXIT_CRITICAL(&stateMux);
  Serial.printf("Kurve-Read:  Abbruch (%s), Teilpuffer verworfen\n", why);
  logHubEvent("curve_read", "abort");
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
  abortCurveRead("reset");                   // [KURVE-READ] Lesefenster nicht ueberleben lassen
  setTuneLinkState(TuneLinkState::Idle, "reset");
}

class TuneClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onDisconnect(NimBLEClient *, int reason) override
  {
    tuneConnected = false;
    tuneNusRx = nullptr;
    tuneModeActive = false; tuneAdvSteps = 0;  // [TUNE-LIVE] Offset verfaellt bei Trennung
    abortCurveRead("disconnect");              // [KURVE-READ] sonst blockiert das Lesefenster den Reconnect
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
#if ARDUINO_USB_CDC_ON_BOOT
      Serial0.printf("123 BLOCK  (%2u B): ", static_cast<unsigned>(length));
      for (size_t i = 0; i < length; i++) Serial0.print(data[i] >= 32 && data[i] < 127 ? (char)data[i] : '.');
      Serial0.println();
#endif
    }
    return;
  }
  Serial.printf("123TUNE BLE: notify len=%u :", static_cast<unsigned>(length));
  for (size_t i = 0; i < length && i < 20; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
#if ARDUINO_USB_CDC_ON_BOOT
  // [UART0-MIRROR] Live-Frame kompakt auf die CH343-Konsole: Opcode + Roh-ASCII.
  { Serial0.printf("123 %02X ", data[0]);
    for (size_t i = 0; i < length; i++) Serial0.print(data[i] >= 32 && data[i] < 127 ? (char)data[i] : '.');
    Serial0.println(); }
#endif
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
    recordBleScanDevice(device, advertisesNus || nameLooksLikeTune || addressMatches || savedAddressMatches);
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
// [TUNE-SAFE] Hartes Limit +/-10: mehr ist kein Feintuning mehr, und der
// Reset-Guard (60 Iterationen) muss den Offset immer vollstaendig abbauen koennen.
#define TUNE_ADV_MAX_STEPS 10
bool tuneAdvStep(int dir)
{
  if (!tuneModeActive) { Serial.println("123TUNE BLE: adv-step blockiert (Tuning-Modus aus)"); return false; }
  const int next = tuneAdvSteps + ((dir >= 0) ? 1 : -1);
  if (next > TUNE_ADV_MAX_STEPS || next < -TUNE_ADV_MAX_STEPS) {
    Serial.printf("123TUNE BLE: adv-step blockiert (Limit %+d erreicht)\n", tuneAdvSteps);
    return false;
  }
  const char c = (dir >= 0) ? 'A' : 'R';
  if (!sendTuneRaw(c)) return false;
  tuneAdvSteps = next;
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
#if ARDUINO_USB_CDC_ON_BOOT
    Serial0.printf(">>> Kurve-Read: sende %s\n", B[curveReadPhase - 1]);
#endif
    curveReadPhase++;
  }
  const bool lastBlockDone = curveReadPhase > 4 &&
                             ((lastGrowMs > blockSentMs && now - lastGrowMs >= 400) || now - blockSentMs >= 1200);
  if (lastBlockDone || now - curveReadStartMs > 6000) {
    curveReadActive = false;
    curveReadPhase = 0;
    blockSentMs = 0;
    Serial.printf("Kurve-Read:  Fenster zu, %u Bytes erfasst\n", static_cast<unsigned>(curveReadLen));
#if ARDUINO_USB_CDC_ON_BOOT
    Serial0.printf(">>> Kurve-Read: fertig, %u Bytes\n", static_cast<unsigned>(curveReadLen));
#endif
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


void updateTuneBle()
{
  const uint32_t now = millis();
  // [TUNE-SAFE] Dead-Man: verliert die GUI den Hub (Handy-WLAN weg), darf der
  // Zuend-Offset nicht am laufenden Motor stehen bleiben. Ohne Tune-API-Aktivitaet
  // (Steps/Ping der GUI) fuer 60 s -> Offset abbauen und Modus verlassen.
  if (tuneModeActive && tuneLastLiveApiMs != 0 &&
      (now - tuneLastLiveApiMs) > kTuneDeadManMs) {
    Serial.println("123TUNE BLE: Dead-Man (60s ohne GUI) -> Offset 0 + Tune aus");
    tuneAdvReset();
    if (tuneModeActive) tuneModeToggle();
    logHubEvent("tune_deadman", "auto_off");
  }
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
      // Funk gerade anderweitig belegt (nur EIN zentraler Scanner). Timer NICHT
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
#else
  tuneSavedAddress = kTuneTargetAddress;
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
  Serial.println("BLE display: disabled (Cockpit-Daten kommen per CAN 0x510)");
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
}

void updateBleHub()
{
  if (hubFeatBle123) {
    updateTuneBle();
  } else if (tuneConnected || tuneDoConnect) {
    resetTuneClient();
  }
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
