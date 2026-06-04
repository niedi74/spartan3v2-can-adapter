#include <Arduino.h>
#include <driver/twai.h>
#include <esp_heap_caps.h>
#include <cstring>

#ifndef ENABLE_WEB_GUI
#define ENABLE_WEB_GUI 0
#endif

#ifndef ENABLE_BLE_HUB
#define ENABLE_BLE_HUB 0
#endif

#if ENABLE_WEB_GUI
#include <DNSServer.h>
#include <FS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <WiFi.h>
#endif

#if ENABLE_BLE_HUB
#include <NimBLEDevice.h>
#include <mbedtls/aes.h>
#endif

#ifdef USE_M5_DISPLAY
#include <M5Unified.h>
#else
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#endif

#ifndef DEVICE_ROLE
#define DEVICE_ROLE "motorraum"
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2
#endif

#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR 0x27
#endif

#ifndef LCD_COLS
#define LCD_COLS 16
#endif

#ifndef LCD_ROWS
#define LCD_ROWS 2
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif

#ifndef WEB_AP_SSID
#define WEB_AP_SSID "Spartan3-Setup"
#endif

#ifndef WEB_AP_PASSWORD
#define WEB_AP_PASSWORD "lambda123"
#endif

#ifndef HOME_WIFI_SSID
#define HOME_WIFI_SSID ""
#endif

#ifndef HOME_WIFI_PASSWORD
#define HOME_WIFI_PASSWORD ""
#endif

#ifndef ENABLE_SPARTAN_CAN
#define ENABLE_SPARTAN_CAN 1
#endif

#ifndef CAN_RX_PIN
#define CAN_RX_PIN 16
#endif

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 17
#endif

#ifndef SPARTAN_CAN_ID
#define SPARTAN_CAN_ID 0x400
#endif

#ifndef ENABLE_SPARTAN_DEMO
#define ENABLE_SPARTAN_DEMO 0
#endif

#ifndef ENABLE_SPARTAN_ANALOG
#define ENABLE_SPARTAN_ANALOG 0
#endif

#ifndef SPARTAN_ANALOG_PIN
#define SPARTAN_ANALOG_PIN 34
#endif

#ifndef ENABLE_SPARTAN_HEATER_ANALOG
#define ENABLE_SPARTAN_HEATER_ANALOG 0
#endif

#ifndef SPARTAN_HEATER_PIN
#define SPARTAN_HEATER_PIN 35
#endif

// With a 10k high-side and 20k low-side divider, ADC voltage is input * 2/3.
#ifndef ANALOG_DIVIDER_NUM
#define ANALOG_DIVIDER_NUM 3
#endif

#ifndef ANALOG_DIVIDER_DEN
#define ANALOG_DIVIDER_DEN 2
#endif

// ---- Reed-Sensor Geschwindigkeit ------------------------------------------
// Reed-Kontakt gegen GND an SPEED_REED_PIN. INPUT_PULLUP, fallende Flanke =
// Magnet vorbei. Bei 10 Magneten/Umdrehung sind das 10 Pulse pro Radumdrehung.
//
//   Pulse pro Sekunde * (TIRE_CIRC_MM / PULSES_PER_REV) / 1000.0 = m/s
//   km/h = m/s * 3.6
//
// Reifen 205/80 R14: Umfang ~2147 mm
// Trim-Faktor (per Web GUI / Prefs) erlaubt GPS-Kalibrierung.
#ifndef SPEED_REED_PIN
#define SPEED_REED_PIN -1   // -1 = disabled
#endif

#ifndef PULSES_PER_REV
#define PULSES_PER_REV 10
#endif

#ifndef TIRE_CIRC_MM_DEFAULT
#define TIRE_CIRC_MM_DEFAULT 2147
#endif

#ifndef SPEED_TRIM_PERMIL_DEFAULT
#define SPEED_TRIM_PERMIL_DEFAULT 1000   // 1000 = 1.000 (kein Trim)
#endif

#ifndef SPEED_DEBOUNCE_US
// Bei 200 km/h und 4657 P/km: ~258 Hz -> Periode 3.9 ms. 500 us debounce
// blockt sicher kein Echtsignal aber filtert mech. Prellungen.
#define SPEED_DEBOUNCE_US 500
#endif

#ifndef ENGINE_RUNNING_RPM_THRESHOLD
#define ENGINE_RUNNING_RPM_THRESHOLD 650
#endif

#ifndef HOURMETER_SAVE_INTERVAL_MS
#define HOURMETER_SAVE_INTERVAL_MS 30000
#endif

#ifndef ENABLE_SPARTAN_UART
#define ENABLE_SPARTAN_UART 1
#endif

#ifndef SPARTAN_UART_RX_PIN
#define SPARTAN_UART_RX_PIN 26
#endif

#ifndef SPARTAN_UART_TX_PIN
#define SPARTAN_UART_TX_PIN 27
#endif

#ifndef BLE_HUB_NAME
#define BLE_HUB_NAME "Spartan3-Hub"
#endif

namespace {
#ifdef USE_M5_DISPLAY
constexpr uint8_t kDisplayRows = 2;
#else
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
#endif

constexpr uint32_t kDisplayIntervalMs = 200;
constexpr uint32_t kBleNotifyIntervalMs = 250;
constexpr uint32_t kCanStaleMs = 500;
constexpr uint32_t kHomeWifiConnectWindowMs = 15000;
constexpr float kLambdaAtZeroVolt = 0.68f;
constexpr float kLambdaAtFiveVolt = 1.36f;
#if ENABLE_BLE_HUB
constexpr char kBleServiceUuid[] = "7f510001-5a6b-4d2a-9f20-14a7f3e20000";
constexpr char kBleStatusUuid[] = "7f510002-5a6b-4d2a-9f20-14a7f3e20000";
constexpr char kBleCommandUuid[] = "7f510003-5a6b-4d2a-9f20-14a7f3e20000";
constexpr char kTuneTargetAddress[] = "ef:a8:b2:de:e0:9e";
constexpr char kTuneNusServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kTuneNusRxUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kTuneNusTxUuid[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
constexpr uint32_t kTuneScanWindowMs = 10000;
constexpr uint32_t kTuneReconnectDelayMs = 5000;
constexpr uint32_t kTunePingIntervalMs = 1650;
constexpr uint32_t kBleHubAdvertiseFallbackMs = 30000;

#ifndef ENABLE_BM6
#define ENABLE_BM6 1
#endif

constexpr char kBm6TargetAddress[] = "3c:ab:72:80:06:6a";
constexpr char kBm6ServiceUuid[] = "0000fff0-0000-1000-8000-00805f9b34fb";
constexpr char kBm6WriteUuid[] = "0000fff3-0000-1000-8000-00805f9b34fb";
constexpr char kBm6NotifyUuid[] = "0000fff4-0000-1000-8000-00805f9b34fb";
constexpr uint32_t kBm6ScanWindowMs = 10000;
constexpr uint32_t kBm6ReconnectDelayMs = 8000;
constexpr uint32_t kBm6TriggerIntervalMs = 2000;
const uint8_t kBm6AesKey[16] = {
    0x6C, 0x65, 0x61, 0x67, 0x65, 0x6E, 0x64, 0xFF,
    0xFE, 0x30, 0x31, 0x30, 0x30, 0x30, 0x30, 0x39};
const uint8_t kBm6TriggerPlain[16] = {
    0xD1, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

struct SpartanReading {
  float lambda = 0.0f;
  uint16_t temperatureC = 0;
  uint8_t status = 0;
  uint32_t receivedMs = 0;
  bool valid = false;
  bool fromCan = false;
  bool fromDemo = false;
};

struct TuneSnapshot {
  float rpm = 0.0f;
  float advance = 0.0f;
  float map = 0.0f;
  float temperature = 0.0f;
  float voltage = 0.0f;
  float coilCurrent = 0.0f;
  uint32_t rxCount = 0;
  uint32_t lastRxMs = 0;
  uint32_t badLengthCount = 0;
  uint32_t unknownOpcodeCount = 0;
};

SpartanReading reading;
bool canReady = false;
twai_status_info_t canStatus = {};
uint32_t lastCanStatusMs = 0;
uint32_t canStatusErrors = 0;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
float tuneRpm = 0.0f;
float tuneAdvance = 0.0f;
float tuneMap = 0.0f;
float tuneTemperature = 0.0f;
float tuneVoltage = 0.0f;
float tuneCoilCurrent = 0.0f;
uint32_t tuneRxCount = 0;
uint32_t tuneLastRxMs = 0;
uint32_t tuneBadLengthCount = 0;
uint32_t tuneUnknownOpcodeCount = 0;
uint32_t lastTuneDebugMs = 0;
float heaterStatusVolts = 0.0f;
uint32_t heaterAdcMilliVolts = 0;
uint8_t heaterStatusCode = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastBleNotifyMs = 0;
uint64_t deviceSeconds = 0;
uint64_t engineSeconds = 0;
uint64_t sensorSeconds = 0;
uint32_t lastHourmeterTickMs = 0;
uint32_t lastHourmeterSaveMs = 0;

// ---- Reed-Speed-State (ISR + Loop) ----
#if SPEED_REED_PIN >= 0
volatile uint32_t speedPulseCount = 0;       // monoton steigend, ISR-only
volatile uint32_t speedLastEdgeUs = 0;       // letzter Flanken-Zeitstempel
uint32_t speedPrevPulseCount = 0;            // Sampling-Diff loop()
uint32_t speedPrevSampleMs = 0;
float speedHz = 0.0f;
float speedKmh = 0.0f;
uint16_t tireCircMm = TIRE_CIRC_MM_DEFAULT;  // konfigurierbar
uint16_t speedTrimPermil = SPEED_TRIM_PERMIL_DEFAULT; // 1000 = 1.000
#endif

String uartLine;
String lastUartCommand = "";
String lastUartResponse = "";
String lastUartState = "bereit";
uint32_t lastUartCommandMs = 0;
uint32_t lastUartResponseMs = 0;
String savedWifiSsid = "";
#if ENABLE_BLE_HUB
NimBLECharacteristic *bleStatusCharacteristic = nullptr;
volatile uint8_t bleClientCount = 0;
String bleAddress = "";
bool bleAdvertisingStarted = false;
uint32_t bleHubSetupMs = 0;
NimBLEClient *tuneClient = nullptr;
NimBLERemoteCharacteristic *tuneNusRx = nullptr;
NimBLEAddress tuneTargetAddress;
volatile bool tuneDoConnect = false;
volatile bool tuneConnected = false;
uint32_t tuneNextScanMs = 0;
uint32_t tuneLastPingMs = 0;
uint32_t tuneScanSeen = 0;
uint32_t tuneScanCandidates = 0;
String tuneSavedAddress = "";
#if ENABLE_BM6
NimBLEClient *bm6Client = nullptr;
NimBLERemoteCharacteristic *bm6WriteChar = nullptr;
NimBLEAddress bm6TargetAddress;
String bm6SavedAddress = "";
bool bm6DoConnect = false;
bool bm6Connected = false;
uint32_t bm6NextScanMs = 0;
uint32_t bm6LastTriggerMs = 0;
uint32_t bm6LastRxMs = 0;
uint32_t bm6RxCount = 0;
uint32_t bm6DecodeFailCount = 0;
float bm6Voltage = 0.0f;
int8_t bm6Temperature = 0;
#endif
#endif
#if ENABLE_WEB_GUI
WebServer server(80);
DNSServer dns;
Preferences networkPreferences;
bool networkPreferencesReady = false;
bool haveSavedWifi = false;
uint32_t homeWifiConnectStartedMs = 0;
bool homeWifiDisabledForRoadAp = false;
uint32_t lastApRetryMs = 0;
uint32_t apRetryCount = 0;
bool logFsReady = false;
uint32_t lastCsvAppendMs = 0;
const char *kLogFile = "/drive.csv";
const char *kOldLogFile = "/drive_old.csv";
const char *kLogHeader =
    "ms;source;lambda_valid;lambda;spartan_temp_c;spartan_status;rpm;advance;map;"
    "tune_volt;bm6_volt;bm6_temp;speed_kmh;speed_hz;speed_pulses;"
    "heater_v;device_h;engine_h;sensor_h";
const size_t kMaxLogBytes = 1200000;
const uint32_t kLogIntervalMs = 500;
#endif

void ensurePreferences()
{
#if ENABLE_WEB_GUI
  if (!networkPreferencesReady) {
    networkPreferences.begin("net", false);
    networkPreferencesReady = true;
  }
#endif
}

String normalizeMacInput(const String &raw)
{
  String mac = raw;
  mac.trim();
  mac.toLowerCase();
  return mac;
}

bool looksLikeMacAddress(const String &mac)
{
  if (mac.length() != 17) return false;
  for (uint8_t i = 0; i < mac.length(); i++) {
    const char c = mac[i];
    if (i % 3 == 2) {
      if (c != ':') return false;
    } else if (!isHexadecimalDigit(c)) {
      return false;
    }
  }
  return true;
}

String fitLine(const String &text)
{
#ifdef USE_M5_DISPLAY
  return text;
#else
  String line = text.substring(0, LCD_COLS);
  while (line.length() < LCD_COLS) {
    line += ' ';
  }
  return line;
#endif
}

void displayLine(uint8_t row, const String &text)
{
#ifdef USE_M5_DISPLAY
  if (row >= kDisplayRows) {
    return;
  }

  const int y = row == 0 ? 28 : 118;
  M5.Display.fillRect(0, y, M5.Display.width(), 74, BLACK);
  M5.Display.setCursor(10, y + 8);
  M5.Display.print(text);
#else
  if (row >= LCD_ROWS) {
    return;
  }

  lcd.setCursor(0, row);
  lcd.print(fitLine(text));
#endif
}

void setupDisplay()
{
#ifdef USE_M5_DISPLAY
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextColor(GREEN, BLACK);
  M5.Display.setTextSize(3);
  M5.Display.clear(BLACK);
#else
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
#endif

  displayLine(0, "SPARTAN 3 v2");
#if ENABLE_SPARTAN_DEMO
  displayLine(1, "DEMO start");
#else
  displayLine(1, "CAN start");
#endif
}

// statusText(String) entfernt — nur statusTextC(const char*) verwenden.

const char *statusTextC(uint8_t status)
{
  switch (status) {
    case 1:
      return "WAIT";
    case 2:
      return "HEAT";
    case 3:
      return "OK";
    default:
      return "ERR";
  }
}

const char *sourceTextC(const SpartanReading &snapshot)
{
  if (snapshot.fromCan) {
    return "CAN";
  }
  if (snapshot.fromDemo) {
    return "DEMO";
  }
  if (snapshot.valid) {
    return "ADC";
  }
  return "NONE";
}

// sourceText(String) entfernt — nur sourceTextC(const char*) verwenden.

SpartanReading readingSnapshot()
{
  portENTER_CRITICAL(&stateMux);
  const SpartanReading snapshot = reading;
  portEXIT_CRITICAL(&stateMux);
  return snapshot;
}

void storeReading(const SpartanReading &fresh)
{
  portENTER_CRITICAL(&stateMux);
  reading = fresh;
  portEXIT_CRITICAL(&stateMux);
}

#if ENABLE_WEB_GUI
String humanBytes(size_t bytes);
size_t logFileSize(const char *path);
void ensureLogHeader();
void sendLogFile(const char *path, const char *downloadName);
#endif

#if ENABLE_BLE_HUB
TuneSnapshot tuneSnapshot()
{
  TuneSnapshot snapshot;
  portENTER_CRITICAL(&stateMux);
  snapshot.rpm = tuneRpm;
  snapshot.advance = tuneAdvance;
  snapshot.map = tuneMap;
  snapshot.temperature = tuneTemperature;
  snapshot.voltage = tuneVoltage;
  snapshot.coilCurrent = tuneCoilCurrent;
  snapshot.rxCount = tuneRxCount;
  snapshot.lastRxMs = tuneLastRxMs;
  snapshot.badLengthCount = tuneBadLengthCount;
  snapshot.unknownOpcodeCount = tuneUnknownOpcodeCount;
  portEXIT_CRITICAL(&stateMux);
  return snapshot;
}

uint8_t getBleClientCount()
{
  portENTER_CRITICAL(&stateMux);
  const uint8_t count = bleClientCount;
  portEXIT_CRITICAL(&stateMux);
  return count;
}
#endif

void sendSpartanUartCommand(const String &command)
{
#if ENABLE_SPARTAN_UART
  String sanitized;
  for (uint16_t i = 0; i < command.length(); i++) {
    const char c = command[i];
    if (isAlphaNumeric(c) || c == '.' || c == '-' || c == '_') {
      sanitized += c;
    }
  }
  sanitized.trim();
  sanitized.toUpperCase();
  if (sanitized.length() == 0 || sanitized.length() > 32) {
    Serial.println("UART command rejected");
    return;
  }

  lastUartCommand = sanitized;
  lastUartResponse = "";
  lastUartState = "gesendet - warte auf Antwort";
  lastUartCommandMs = millis();
  lastUartResponseMs = 0;
  Serial2.println(sanitized);
  Serial.printf("UART command sent: %s\n", sanitized.c_str());
#else
  (void)command;
#endif
}

String statusJson()
{
  const SpartanReading snapshot = readingSnapshot();
#if ENABLE_BLE_HUB
  const TuneSnapshot tune = tuneSnapshot();
  const uint8_t clients = getBleClientCount();
#endif
  const uint32_t now = millis();
  String json;
  json.reserve(768);  // verhindert wiederholte Heap-Reallokationen
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
  json += "\",\"can_ready\":";
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
  json += ",\"heap_free\":";
  json += String(heap_caps_get_free_size(MALLOC_CAP_8BIT));
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
  json += ",\"ble_clients\":";
  json += String(clients);
  json += ",\"ble_name\":\"";
  json += BLE_HUB_NAME;
  json += "\",\"ble_address\":\"";
  json += bleAddress;
  json += "\"";
#endif
#if ENABLE_WEB_GUI
  json += ",\"wifi_saved\":";
  json += haveSavedWifi ? "true" : "false";
  json += ",\"wifi_saved_ssid\":\"";
  json += savedWifiSsid;
  json += "\"";
  json += ",\"wifi_connected\":";
  json += WiFi.status() == WL_CONNECTED ? "true" : "false";
  json += ",\"wifi_ssid\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  json += "\",\"wifi_ip\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  json += "\",\"ap_ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",\"ap_retry_count\":";
  json += String(apRetryCount);
  json += ",\"log_ready\":";
  json += logFsReady ? "true" : "false";
  json += ",\"log_current_bytes\":";
  json += String(static_cast<unsigned long>(logFileSize(kLogFile)));
  json += ",\"log_old_bytes\":";
  json += String(static_cast<unsigned long>(logFileSize(kOldLogFile)));
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
#if ENABLE_BM6
  json += ",\"bm6_connected\":";
  json += bm6Connected ? "true" : "false";
  json += ",\"bm6_voltage\":";
  json += String(bm6Voltage, 2);
  json += ",\"bm6_temperature\":";
  json += String(static_cast<int>(bm6Temperature));
  json += ",\"bm6_rx_count\":";
  json += String(static_cast<unsigned long>(bm6RxCount));
  json += ",\"bm6_decode_fail\":";
  json += String(static_cast<unsigned long>(bm6DecodeFailCount));
  json += ",\"bm6_age_ms\":";
  json += String(bm6LastRxMs == 0 ? 0UL : static_cast<unsigned long>(now - bm6LastRxMs));
  json += ",\"bm6_saved_address\":\"";
  json += bm6SavedAddress;
  json += "\"";
#endif
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
#endif
  json += "}";
  return json;
}

#if ENABLE_WEB_GUI
String humanBytes(size_t bytes)
{
  if (bytes < 1024) return String(bytes) + " B";
  if (bytes < 1024UL * 1024UL) return String(bytes / 1024.0f, 1) + " KB";
  return String(bytes / (1024.0f * 1024.0f), 2) + " MB";
}

size_t logFileSize(const char *path)
{
  if (!logFsReady || !SPIFFS.exists(path)) return 0;
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return 0;
  const size_t size = f.size();
  f.close();
  return size;
}

void ensureLogHeader()
{
  if (!logFsReady) return;
  bool needsHeader = !SPIFFS.exists(kLogFile);
  if (!needsHeader) {
    File existing = SPIFFS.open(kLogFile, FILE_READ);
    needsHeader = !existing || existing.size() == 0;
    if (existing) existing.close();
  }
  if (!needsHeader) return;
  File f = SPIFFS.open(kLogFile, FILE_WRITE);
  if (!f) return;
  f.println(kLogHeader);
  f.close();
}

void rotateLogIfNeeded()
{
  if (!logFsReady || !SPIFFS.exists(kLogFile)) return;
  File f = SPIFFS.open(kLogFile, FILE_READ);
  const size_t size = f ? f.size() : 0;
  if (f) f.close();
  if (size < kMaxLogBytes) return;
  SPIFFS.remove(kOldLogFile);
  SPIFFS.rename(kLogFile, kOldLogFile);
  ensureLogHeader();
}

bool shouldLogCsv(const SpartanReading &spartan, const TuneSnapshot &tune)
{
  if (tune.rpm > ENGINE_RUNNING_RPM_THRESHOLD) return true;
#if SPEED_REED_PIN >= 0
  if (speedKmh > 0.5f) return true;
#endif
  return spartan.valid;
}

void appendLiveCsv()
{
  if (!logFsReady) return;
  const uint32_t now = millis();
  if (now - lastCsvAppendMs < kLogIntervalMs) return;
  lastCsvAppendMs = now;

  const SpartanReading spartan = readingSnapshot();
#if ENABLE_BLE_HUB
  const TuneSnapshot tune = tuneSnapshot();
#else
  const TuneSnapshot tune;
#endif
  if (!shouldLogCsv(spartan, tune)) return;

  rotateLogIfNeeded();
  ensureLogHeader();
  File f = SPIFFS.open(kLogFile, FILE_APPEND);
  if (!f) return;

  f.printf("%lu;%s;%u;%.3f;%u;%u;%.0f;%.1f;%.0f;%.1f;%.2f;%d;%.1f;%.2f;%lu;%.2f;%.2f;%.2f;%.2f\n",
           static_cast<unsigned long>(now),
           sourceTextC(spartan),
           spartan.valid ? 1 : 0,
           spartan.lambda,
           spartan.temperatureC,
           spartan.status,
           tune.rpm,
           tune.advance,
           tune.map,
           tune.voltage,
#if ENABLE_BM6
           bm6Voltage,
           static_cast<int>(bm6Temperature),
#else
           0.0f,
           0,
#endif
#if SPEED_REED_PIN >= 0
           speedKmh,
           speedHz,
           static_cast<unsigned long>(speedPulseCount),
#else
           0.0f,
           0.0f,
           0UL,
#endif
           heaterStatusVolts,
           static_cast<double>(deviceSeconds) / 3600.0,
           static_cast<double>(engineSeconds) / 3600.0,
           static_cast<double>(sensorSeconds) / 3600.0);
  f.close();
}

void sendLogFile(const char *path, const char *downloadName)
{
  if (!logFsReady || !SPIFFS.exists(path)) {
    server.send(404, "text/plain", "Logdatei nicht vorhanden.");
    return;
  }
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Logdatei kann nicht geoeffnet werden.");
    return;
  }
  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + downloadName + "\"");
  server.streamFile(f, "text/csv");
  f.close();
}
#else
void appendLiveCsv() {}
#endif

#if ENABLE_BLE_HUB
int hexNibble(uint8_t c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

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
  const int hi = hexNibble(data[1]);
  const int lo = hexNibble(data[2]);
  const int raw = (hi << 4) | lo;

  bool knownOpcode = true;
  portENTER_CRITICAL(&stateMux);
  switch (data[0]) {
    case 0x30: tuneRpm = hi * 800.0f + lo * 50.0f; break;
    case 0x31: tuneAdvance = hi * 3.2f + lo * 0.2f; break;
    case 0x32: tuneMap = static_cast<float>(raw); break;
    case 0x33: tuneTemperature = static_cast<float>(raw - 30); break;
    case 0x35: tuneCoilCurrent = raw / 8.65f; break;
    case 0x41: tuneVoltage = raw / 4.54f; break;
    case 0x42: break;
    default:
      knownOpcode = false;
      if (tuneUnknownOpcodeCount < UINT32_MAX) {
        tuneUnknownOpcodeCount++;
      }
      break;
  }
  if (knownOpcode && tuneRxCount < UINT32_MAX) {
    tuneRxCount++;
  }
  portEXIT_CRITICAL(&stateMux);
  // millis() ausserhalb der Critical Section — ist nicht zeitkritisch genug
  // um den Jitter zu rechtfertigen, spart aber Interrupt-Latenz.
  tuneLastRxMs = millis();
}

String bleStatusPayload()
{
  const SpartanReading snapshot = readingSnapshot();
  const TuneSnapshot tune = tuneSnapshot();
  // Kompakt-Format fuer M5 Gateway:
  //   L<lambda>R<rpm>A<adv>M<map>  (Basis)
  //   V<bm6_volt>                  (BM6 Batterie)
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
  if (n > 0 && n < static_cast<int>(sizeof(payload)))
    n += snprintf(payload + n, sizeof(payload) - n, "V%.2f", bm6Voltage);
#endif
#if SPEED_REED_PIN >= 0
  if (n > 0 && n < static_cast<int>(sizeof(payload)))
    n += snprintf(payload + n, sizeof(payload) - n, "S%d",
                  static_cast<int>(speedKmh + 0.5f));
#endif
  // 123TUNE+ interne Werte (nur wenn verbunden)
  if (tune.rxCount > 0 && n > 0 && n < static_cast<int>(sizeof(payload))) {
    n += snprintf(payload + n, sizeof(payload) - n, "I%.1fT%dC%.1f",
                  tune.voltage, static_cast<int>(tune.temperature),
                  tune.coilCurrent);
  }
  return String(payload);
}

class BleServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override
  {
    portENTER_CRITICAL(&stateMux);
    if (bleClientCount < UINT8_MAX) {
      bleClientCount++;
    }
    const uint8_t clients = bleClientCount;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("BLE hub:     client connected, count=%u\n", clients);
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override
  {
    portENTER_CRITICAL(&stateMux);
    if (bleClientCount > 0) {
      bleClientCount--;
    }
    const uint8_t clients = bleClientCount;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("BLE hub:     client disconnected, count=%u\n", clients);
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

void scheduleTuneScan()
{
  tuneNextScanMs = millis() + kTuneReconnectDelayMs;
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
}

class TuneClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onDisconnect(NimBLEClient *, int reason) override
  {
    tuneConnected = false;
    tuneNusRx = nullptr;
    Serial.printf("123TUNE BLE: disconnected reason=%d\n", reason);
    scheduleTuneScan();
  }
};

void onTuneNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool)
{
  Serial.printf("123TUNE BLE: notify len=%u :", static_cast<unsigned>(length));
  for (size_t i = 0; i < length && i < 20; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
  decodeTuneFrame(data, length);
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
    Serial.printf("123TUNE BLE: found %s name='%s' nus=%d addrMatch=%d\n",
                  address.c_str(),
                  name.c_str(),
                  advertisesNus ? 1 : 0,
                  (addressMatches || savedAddressMatches) ? 1 : 0);
  }

  void onScanEnd(const NimBLEScanResults &, int reason) override
  {
    if (!tuneConnected && !tuneDoConnect) {
      Serial.printf("123TUNE BLE: scan end reason=%d seen=%lu candidates=%lu\n",
                    reason,
                    static_cast<unsigned long>(tuneScanSeen),
                    static_cast<unsigned long>(tuneScanCandidates));
      scheduleTuneScan();
    }
  }
};

TuneClientCallbacks tuneClientCallbacks;
TuneScanCallbacks tuneScanCallbacks;

void startTuneScan()
{
  if (tuneConnected || tuneDoConnect) return;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&tuneScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  tuneScanSeen = 0;
  tuneScanCandidates = 0;
  Serial.println("123TUNE BLE: scan 10s...");
  if (!scan->start(kTuneScanWindowMs, false)) {
    Serial.println("123TUNE BLE: scan start failed");
    scheduleTuneScan();
  }
}

void connectTune()
{
  tuneDoConnect = false;
  if (tuneConnected) return;
  if (tuneClient != nullptr && !tuneClient->isConnected()) {
    resetTuneClient();
  }
  if (tuneClient == nullptr) {
    tuneClient = NimBLEDevice::createClient();
    tuneClient->setClientCallbacks(&tuneClientCallbacks, false);
  }
  tuneClient->setConnectionParams(16, 32, 0, 400);
  Serial.println("123TUNE BLE: connecting...");
  if (!tuneClient->connect(tuneTargetAddress, true, false, false)) {
    Serial.println("123TUNE BLE: connect failed");
    resetTuneClient();
    scheduleTuneScan();
    return;
  }
  delay(750);

  NimBLERemoteService *service = tuneClient->getService(kTuneNusServiceUuid);
  if (service == nullptr) {
    Serial.println("123TUNE BLE: NUS service missing");
    tuneClient->disconnect();
    scheduleTuneScan();
    return;
  }
  NimBLERemoteCharacteristic *tx = service->getCharacteristic(kTuneNusTxUuid);
  tuneNusRx = service->getCharacteristic(kTuneNusRxUuid);
  if (tx == nullptr || !tx->canNotify()) {
    Serial.println("123TUNE BLE: NUS TX notify missing");
    tuneClient->disconnect();
    scheduleTuneScan();
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
    delay(150);
  } else {
    Serial.println("123TUNE BLE: CCCD missing");
  }

  const bool ok = tx->subscribe(true, onTuneNotify, true);
  tuneConnected = ok;
  Serial.printf("123TUNE BLE: subscribe %s\n", ok ? "OK" : "FAIL");
  if (!ok) {
    tuneClient->disconnect();
    scheduleTuneScan();
    return;
  }

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
    const bool pingOk = tuneNusRx->writeValue(&ping, 1, true);
    delay(120);
    const bool enterOk = tuneNusRx->writeValue(&enter, 1, true);
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
  const bool ok = tuneNusRx->writeValue(&ping, 1, true);
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
    bm6Voltage = static_cast<float>(voltRaw) / 100.0f;
    bm6Temperature = tempNeg ? -tempVal : tempVal;
    bm6LastRxMs = millis();
    bm6RxCount++;
    Serial.printf("BM6 BLE:     V=%.2fV T=%d C rx=%lu\n",
                  bm6Voltage,
                  static_cast<int>(bm6Temperature),
                  static_cast<unsigned long>(bm6RxCount));
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
    if (addr != bm6SavedAddress && addr != kBm6TargetAddress) return;
    bm6TargetAddress = device->getAddress();
    if (bm6SavedAddress != addr) {
      bm6SavedAddress = addr;
#if ENABLE_WEB_GUI
      ensurePreferences();
      networkPreferences.putString("bm6_mac", bm6SavedAddress);
#endif
      Serial.printf("BM6 BLE:     saved address %s\n", bm6SavedAddress.c_str());
    }
    bm6DoConnect = true;
    bm6NextScanMs = 0;
    NimBLEDevice::getScan()->stop();
    Serial.printf("BM6 BLE:     found %s\n", addr.c_str());
  }

  void onScanEnd(const NimBLEScanResults &, int reason) override
  {
    if (!bm6Connected && !bm6DoConnect) {
      Serial.printf("BM6 BLE:     scan end reason=%d\n", reason);
      scheduleBm6Scan();
    }
  }
};

Bm6ScanCallbacks bm6ScanCallbacks;

void startBm6Scan()
{
  if (bm6Connected || bm6DoConnect) return;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&bm6ScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  Serial.println("BM6 BLE:     scan 10s...");
  if (!scan->start(kBm6ScanWindowMs, false)) {
    Serial.println("BM6 BLE:     scan start failed");
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

  bm6Client->setConnectionParams(6, 12, 0, 150);
  Serial.println("BM6 BLE:     connecting...");
  if (!bm6Client->connect(bm6TargetAddress, true, false, false)) {
    Serial.println("BM6 BLE:     connect failed");
    resetBm6Client();
    scheduleBm6Scan();
    return;
  }
  delay(500);

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
  bm6SendTrigger();
}

void updateBm6Ble()
{
  if (bm6DoConnect) {
    connectBm6();
    return;
  }
  if (bm6Connected) {
    const uint32_t now = millis();
    if (now - bm6LastRxMs > 5000 && now - bm6LastTriggerMs > kBm6TriggerIntervalMs) {
      bm6SendTrigger();
    }
    return;
  }
  if (bm6NextScanMs != 0 && static_cast<int32_t>(millis() - bm6NextScanMs) >= 0) {
    bm6NextScanMs = 0;
    startBm6Scan();
  }
}
#endif

void updateTuneBle()
{
  if (tuneDoConnect) {
    connectTune();
    return;
  }
  sendTunePing();
  if (!tuneConnected && tuneNextScanMs != 0 && static_cast<int32_t>(millis() - tuneNextScanMs) >= 0) {
    tuneNextScanMs = 0;
    startTuneScan();
  }
}

void updateTuneDebug()
{
  const uint32_t now = millis();
  if (now - lastTuneDebugMs < 2000) return;
  lastTuneDebugMs = now;
  const TuneSnapshot tune = tuneSnapshot();
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
  if (bleAdvertisingStarted) return;
  NimBLEDevice::startAdvertising();
  bleAdvertisingStarted = true;
  Serial.println("BLE hub:     advertising started");
}

void setupBleHub()
{
  NimBLEDevice::init(BLE_HUB_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(23);
  bleAddress = NimBLEDevice::getAddress().toString().c_str();
  bleHubSetupMs = millis();
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
#endif
#else
  tuneSavedAddress = kTuneTargetAddress;
#if ENABLE_BM6
  bm6SavedAddress = kBm6TargetAddress;
#endif
#endif

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

  Serial.printf("BLE hub:     '%s' addr=%s service=%s\n",
                BLE_HUB_NAME,
                bleAddress.c_str(),
                kBleServiceUuid);
  Serial.printf("123TUNE BLE: target %s\n", tuneSavedAddress.c_str());
  Serial.println("BLE hub:     advertising waits for 123TUNE or 30s fallback");
  startTuneScan();
#if ENABLE_BM6
  Serial.printf("BM6 BLE:     target %s\n", bm6SavedAddress.c_str());
  scheduleBm6Scan();
#endif
}

void updateBleHub()
{
  updateTuneBle();
#if ENABLE_BM6
  updateBm6Ble();
#endif
  updateTuneDebug();
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
}
#else
void setupBleHub() {}
void updateBleHub() {}
#endif

void setupHourmeters()
{
  ensurePreferences();
  deviceSeconds = networkPreferences.getULong64("dev_sec", 0);
  engineSeconds = networkPreferences.getULong64("eng_sec", 0);
  sensorSeconds = networkPreferences.getULong64("sns_sec", 0);
  lastHourmeterTickMs = millis();
  lastHourmeterSaveMs = millis();
  Serial.printf("Hours:       device=%.2f h engine=%.2f h sensor=%.2f h\n",
                static_cast<double>(deviceSeconds) / 3600.0,
                static_cast<double>(engineSeconds) / 3600.0,
                static_cast<double>(sensorSeconds) / 3600.0);
}

void saveHourmeters()
{
  ensurePreferences();
  networkPreferences.putULong64("dev_sec", deviceSeconds);
  networkPreferences.putULong64("eng_sec", engineSeconds);
  networkPreferences.putULong64("sns_sec", sensorSeconds);
  lastHourmeterSaveMs = millis();
}

void updateHourmeters()
{
  const uint32_t now = millis();
  if (lastHourmeterTickMs == 0) {
    lastHourmeterTickMs = now;
    return;
  }
  const uint32_t elapsedMs = now - lastHourmeterTickMs;
  if (elapsedMs < 1000) return;

  const uint32_t elapsedSec = elapsedMs / 1000;
  lastHourmeterTickMs += elapsedSec * 1000;
  deviceSeconds += elapsedSec;

  const SpartanReading snapshot = readingSnapshot();
#if ENABLE_BLE_HUB
  const TuneSnapshot tune = tuneSnapshot();
  const bool rpmRunning = tune.rpm > ENGINE_RUNNING_RPM_THRESHOLD;
#else
  const bool rpmRunning = false;
#endif
#if SPEED_REED_PIN >= 0
  const bool speedRunning = speedKmh > 0.5f;
#else
  const bool speedRunning = false;
#endif
  const bool engineRunning = rpmRunning || speedRunning;
  const bool sensorActive = snapshot.status == 3 || heaterStatusCode >= 2 || snapshot.temperatureC >= 700;

  if (engineRunning) {
    engineSeconds += elapsedSec;
  }
  if (sensorActive) {
    sensorSeconds += elapsedSec;
  }

  if (now - lastHourmeterSaveMs >= HOURMETER_SAVE_INTERVAL_MS) {
    saveHourmeters();
  }
}

void setupWebGui()
{
#if ENABLE_WEB_GUI
  ensurePreferences();
  logFsReady = SPIFFS.begin(true);
  Serial.printf("Logs:        SPIFFS %s, current=%s, old=%s\n",
                logFsReady ? "OK" : "FAIL",
                humanBytes(logFileSize(kLogFile)).c_str(),
                humanBytes(logFileSize(kOldLogFile)).c_str());
  ensureLogHeader();
  const String savedSsid = networkPreferences.isKey("ssid") ? networkPreferences.getString("ssid", "") : "";
  const String savedPassword = networkPreferences.isKey("pass") ? networkPreferences.getString("pass", "") : "";
  savedWifiSsid = savedSsid;
  String stationSsid = savedSsid;
  String stationPassword = savedPassword;
  const bool usingBuiltInWifi = stationSsid.length() == 0 && strlen(HOME_WIFI_SSID) > 0;
  if (usingBuiltInWifi) {
    stationSsid = HOME_WIFI_SSID;
    stationPassword = HOME_WIFI_PASSWORD;
    savedWifiSsid = stationSsid;
  }
  haveSavedWifi = stationSsid.length() > 0;

  WiFi.mode(WIFI_AP_STA);
#if ENABLE_BLE_HUB
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif
  WiFi.softAPConfig(
      IPAddress(192, 168, 4, 1),
      IPAddress(192, 168, 4, 1),
      IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD, 6, 0, 4)) {
    Serial.println("Web GUI:     access point start failed");
    return;
  }

  if (haveSavedWifi) {
    WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
    homeWifiConnectStartedMs = millis();
    Serial.printf("Home WiFi:   connecting to '%s'%s\n",
                  stationSsid.c_str(),
                  usingBuiltInWifi ? " (built-in)" : "");
  } else {
    Serial.println("Home WiFi:   not configured");
  }

  server.on("/", []() {
    server.send(200, "text/html", R"HTML(
<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spartan 3 v2 Setup</title>
<style>
:root { color-scheme: dark; font-family: Arial, sans-serif; }
body { margin: 0; background: #0b1210; color: #e6ede8; font-size: 16px; }
main { max-width: 760px; margin: auto; padding: 18px 14px 30px; }
h1 { font-size: 1.22rem; color: #9ed85b; margin: 4px 0 14px; }
.card { padding: 16px; border: 1px solid #26372e; border-radius: 10px; background: #101a15; }
.topline { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 12px; }
.lambda { font-size: 3.2rem; font-weight: 700; color: #9ed85b; margin: 2px 0 10px; line-height: 1; }
.tag { display: inline-block; padding: 5px 10px; border-radius: 20px; background: #26372e; color: #bde87a; }
.row { display: flex; justify-content: space-between; gap: 14px; border-top: 1px solid #26372e; padding: 12px 0; }
.row strong { text-align: right; }
.setup { margin-top: 14px; padding: 16px; border: 1px solid #26372e; border-radius: 10px; background: #101a15; }
input, select { display: block; box-sizing: border-box; width: 100%; min-height: 44px; margin: 8px 0 13px; padding: 12px; border: 1px solid #35453c; border-radius: 8px; background: #0b1210; color: #e6ede8; font-size: 1rem; }
button { min-height: 44px; padding: 11px 14px; margin-right: 7px; border: 0; border-radius: 8px; background: #78ad43; color: #081005; font-weight: 700; font-size: .98rem; }
button.secondary { background: #26372e; color: #e6ede8; }
button.danger { background: #8b3c2e; color: #ffe8dc; }
.buttonlink { display: block; box-sizing: border-box; min-height: 44px; padding: 12px 14px; border-radius: 8px; background: #78ad43; color: #081005; font-weight: 700; text-align: center; text-decoration: none; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.grid button { width: 100%; margin: 0; }
.hint { color: #9ca99f; font-size: .92rem; margin-top: 12px; line-height: 1.45; }
.mono { font-family: Consolas, monospace; color: #cbeaa7; overflow-wrap: anywhere; }
.ok { color: #9ed85b; }
.warn { color: #ffd166; }
.bad { color: #ff6b6b; }
.metrics { display: grid; grid-template-columns: repeat(2,minmax(0,1fr)); gap: 10px; margin: 12px 0 4px; }
.metric { min-height: 64px; padding: 12px; border: 1px solid #26372e; border-radius: 8px; background: #0d1712; }
.metric span { display: block; color: #9ca99f; font-size: .78rem; }
.metric strong { display: block; margin-top: 5px; font-size: 1.35rem; color: #e6ede8; overflow-wrap: anywhere; }
.metric.wide { grid-column: 1 / -1; }
pre { white-space: pre-wrap; max-height: 220px; overflow: auto; padding: 12px; background: #08100c; border-radius: 10px; }
.tabs { display: flex; gap: 8px; margin: 4px 0 14px; position: sticky; top: 0; padding-top: 6px; padding-bottom: 6px; background: #0b1210; z-index: 10; }
.tab { flex: 1; padding: 14px 18px; border-radius: 10px; background: #1a2922; color: #cbeaa7; font-weight: 700; cursor: pointer; }
.tab.on { background: #9ed85b; color: #081005; }
.tab-section[hidden] { display: none !important; }
details.setup { padding: 0; overflow: hidden; }
details.setup > summary { list-style: none; cursor: pointer; padding: 16px; font-weight: 700; color: #e6ede8; }
details.setup > summary::-webkit-details-marker { display: none; }
details.setup > summary::after { content: "+"; float: right; color: #9ed85b; }
details.setup[open] > summary::after { content: "-"; }
details.setup > .inside { padding: 0 16px 16px; }
@media (max-width: 520px) {
  main { padding: 14px 10px 26px; }
  .lambda { font-size: 2.85rem; }
  .row { align-items: flex-start; }
  .metrics { grid-template-columns: 1fr 1fr; gap: 8px; }
  .metric strong { font-size: 1.12rem; }
  .grid { grid-template-columns: 1fr; }
  button { width: 100%; margin: 0 0 8px; }
}
</style>
</head>
<body><main>
<h1>SPARTAN 3 v2 Motorraum Hub</h1>
<div class="tabs">
<button type="button" id="tabLive" class="tab on" onclick="showTab('live')">Live</button>
<button type="button" id="tabLog" class="tab" onclick="showTab('log')">Log</button>
<button type="button" id="tabSetup" class="tab" onclick="showTab('setup')">Setup</button>
</div>
<div class="tab-section" data-tab="live">
<div class="card">
<div class="topline"><span id="source" class="tag">START</span><span id="wifiTop" class="mono">offline</span></div>
<div class="lambda" id="lambda">-.---</div>
<div class="metrics">
<div class="metric"><span>Status</span><strong id="status">warte</strong></div>
<div class="metric"><span>CAN</span><strong id="can">-</strong></div>
<div class="metric"><span>Temp</span><strong id="temp">- C</strong></div>
<div class="metric"><span>Speed</span><strong id="liveSpeed">0.0 km/h</strong></div>
<div class="metric wide"><span>123 RPM / ADV / MAP</span><strong id="main123">0 / 0.0 / 0</strong></div>
<div class="metric"><span>BM6</span><strong id="liveBm6">- V</strong></div>
<div class="metric"><span>Sonde h</span><strong id="liveHours">0.00</strong></div>
</div>
<p class="hint">Messstelle hinten im Auspuff: Lambda kann bei Falschluft magerer wirken als der Motor wirklich laeuft.</p>
</div>
<details class="setup">
<summary>123Tune BLE Diagnose</summary>
<div class="inside">
<div class="row"><span>Verbindung</span><strong id="tconn">-</strong></div>
<div class="row"><span>RX Frames</span><strong id="trx">0</strong></div>
<div class="row"><span>RX Alter</span><strong id="tage">0 ms</strong></div>
<div class="row"><span>Scan gesehen / Kandidaten</span><strong id="tscan">0 / 0</strong></div>
<div class="row"><span>Frame Fehler</span><strong id="terr">0 / 0</strong></div>
<div class="row"><span>RPM / ADV / MAP</span><strong id="tvals">0 / 0.0 / 0</strong></div>
<div class="row"><span>123 Adresse</span><strong id="taddr" class="mono">-</strong></div>
</div>
</details>
<details class="setup" open>
<summary>BM6 Batteriemonitor</summary>
<div class="inside">
<p class="hint">Leagend BM6 V2.0. Liefert Bordnetz-Spannung und Umgebungstemperatur per BLE.</p>
<div class="row"><span>Verbindung</span><strong id="bm6conn">-</strong></div>
<div class="row"><span>Spannung</span><strong id="bm6volt">- V</strong></div>
<div class="row"><span>Temperatur</span><strong id="bm6temp">- C</strong></div>
<div class="row"><span>RX Frames</span><strong id="bm6rx">0</strong></div>
<div class="row"><span>RX Alter</span><strong id="bm6age">0 ms</strong></div>
<div class="row"><span>Decode Fehler</span><strong id="bm6err">0</strong></div>
<div class="row"><span>BM6 Adresse</span><strong class="mono">3c:ab:72:80:06:6a</strong></div>
</div>
</details>
<details class="setup">
<summary>Live Meta</summary>
<div class="inside">
<div class="row"><span>WLAN / IP</span><strong id="liveWifiMeta" class="mono">-</strong></div>
<div class="row"><span>Speed Hz / Pulse</span><strong id="liveSpeedMeta">0.00 / 0</strong></div>
<div class="row"><span>123 Alter / RX</span><strong id="liveTuneMeta">0 ms / 0</strong></div>
<div class="row"><span>BM6 Alter / RX</span><strong id="liveBm6Meta">0 ms / 0</strong></div>
<div class="row"><span>Geraet / Motor / Sonde</span><strong id="liveHoursMeta">0 / 0 / 0 h</strong></div>
</div>
</details>
</div><!-- /tab live -->
<div class="tab-section" data-tab="log" hidden>
<details class="setup" open>
<summary>Logbuch</summary>
<div class="inside">
<p class="hint">Schreibt CSV ins ESP32-Dateisystem, wenn Motor/Daten aktiv sind. Current ist die laufende Datei, Last ist die vorige rotierte Datei.</p>
<div class="row"><span>Status</span><strong id="logstatus">-</strong></div>
<div class="row"><span>Current</span><strong id="logcurrent">0 B</strong></div>
<div class="row"><span>Last rotated</span><strong id="logold">0 B</strong></div>
<div class="grid">
<a href="/download" class="buttonlink">Current CSV</a>
<a href="/download_old" class="buttonlink">Last CSV</a>
</div>
<form action="/clear" method="post" style="margin-top:12px"><button class="secondary" type="submit">Current Log loeschen</button></form>
</div>
</details>
<details class="setup">
<summary>CSV Inhalt</summary>
<div class="inside">
<p class="hint">Spalten: ms, Quelle, Lambda, Spartan Temp/Status, 123 RPM/ADV/MAP, BM6, Speed, Heater und Betriebsstunden.</p>
</div>
</details>
</div><!-- /tab log -->
<div class="tab-section" data-tab="setup" hidden>
<details class="setup" open>
<summary>System Diagnose</summary>
<div class="inside">
<div class="row"><span>CAN State / TX / RX</span><strong id="candiag">-</strong></div>
<div class="row"><span>CAN Statusfehler</span><strong id="canerr">0</strong></div>
<div class="row"><span>Heap frei</span><strong id="heap">-</strong></div>
<div class="row"><span>Geraet / Motor / Sonde</span><strong id="hours">0 / 0 / 0 h</strong></div>
<div class="row"><span>AP IP / Retry</span><strong id="apdiag">-</strong></div>
<button class="secondary" type="button" onclick="copyJson()">JSON kopieren</button>
<details class="setup">
<summary>JSON Rohdaten</summary>
<div class="inside">
<pre id="jsondump">{}</pre>
</div>
</details>
</div>
</details>
<details class="setup" open>
<summary>Geschwindigkeit Setup (Reed-Sensor)</summary>
<div class="inside">
<p class="hint">Reed-Kontakt gegen GND auf GPIO 27. Default: 10 Pulse pro Radumdrehung, Reifen 205/80 R14 = 2147 mm. Trim per GPS abgleichen: Trim = GPS / Reed.</p>
<div class="row"><span>Live Frequenz</span><strong id="spdhz">- Hz</strong></div>
<div class="row"><span>Live Geschwindigkeit</span><strong id="spdkmh">- km/h</strong></div>
<div class="row"><span>Pulse gesamt</span><strong id="spdpc">0</strong></div>
<div class="row"><span>Pulse pro Umdrehung</span><strong id="spdppr">10</strong></div>
<form action="/speed" method="post" style="margin-top:12px">
<label>Reifenumfang (mm) <input type="number" name="tire" min="500" max="4000" id="ctlTire" value="2147"></label>
<label>Trim (x1000) <input type="number" name="trim" min="500" max="1500" id="ctlTrim" value="1000"></label>
<p class="hint">Beispiel: Tacho zeigt 60.0 km/h, GPS sagt 58.4 km/h -> Trim = 1000 * 58.4 / 60.0 = 973.</p>
<button type="submit">Speichern</button>
</form>
</div>
</details>
<details class="setup">
<summary>BLE Hub / Gateway</summary>
<div class="inside">
<p class="hint">Fuer M5/Waveshare Gateway-Modus. Der M5 scannt nach Name + Service UUID; die Adresse ist nur Debug/Fast-Reconnect.</p>
<p class="hint">Road-AP: Handy mit <span class="mono">Spartan3-Setup</span> verbinden. Spartan ist <a class="mono" href="http://192.168.4.1/">192.168.4.1</a>, M5 Dial ist <a class="mono" href="http://192.168.4.2/">192.168.4.2</a>.</p>
<div class="row"><span>Status</span><strong id="bleenabled">-</strong></div>
<div class="row"><span>Name</span><strong id="blename" class="mono">-</strong></div>
<div class="row"><span>Adresse</span><strong id="bleaddr" class="mono">-</strong></div>
<div class="row"><span>Clients</span><strong id="bleclients">0</strong></div>
<div class="row"><span>Service</span><strong class="mono">7f510001-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
<div class="row"><span>Status Notify</span><strong class="mono">7f510002-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
<div class="row"><span>Command Write</span><strong class="mono">7f510003-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
</div>
</details>
<details class="setup" open>
<summary>WLAN / Hotspot</summary>
<div class="inside">
<div class="row"><span>Verbindung</span><strong id="wifi">nicht eingerichtet</strong></div>
<div class="row"><span>ESP32 IP</span><strong id="lanip">-</strong></div>
<div class="row"><span>Gespeichert</span><strong id="wifisaved">-</strong></div>
<form action="/wifi" method="post">
<label for="wifiPreset">Profil</label><select id="wifiPreset">
<option value="">Manuell</option>
<option value="Android-AP1" data-pass="Frankfurt1">S24 Hotspot Android-AP1</option>
<option value="Z00-Station" data-pass="">Z00-Station</option>
</select>
<label for="ssid">WLAN-Name</label><input id="ssid" name="ssid" required>
<label for="pass">Passwort</label><input id="pass" name="pass" type="password" placeholder="leer lassen = vorhandenes Passwort behalten">
<button type="submit">Speichern &amp; verbinden</button>
</form>
<p class="hint">Im Auto am einfachsten Handy-Hotspot starten und Laptop, M5 und ESP32 in dasselbe Netz lassen. Die ESP32-IP steht oben und im USB-Log.</p>
<form action="/wifi_clear" method="post" style="margin-top:12px"><button class="secondary" type="submit">Gespeichertes WLAN loeschen</button></form>
</div>
</details>
<details class="setup" open>
<summary>BLE Zielgeraete</summary>
<div class="inside">
<p class="hint">Feste Favoriten fuer 123 und BM6. Manuell bleibt moeglich, damit wir spaeter weitere Geraete schnell aufnehmen koennen.</p>
<div class="row"><span>123 aktuell</span><strong id="taddrsetup" class="mono">-</strong></div>
<form action="/ble_target" method="post">
<label for="tunePreset">123 Profil</label><select id="tunePreset">
<option value="">Manuell</option>
<option value="ef:a8:b2:de:e0:9e">123 #1 ef:a8:b2:de:e0:9e</option>
</select>
<label for="tune_mac">123 BLE-Adresse</label><input id="tune_mac" name="tune_mac" placeholder="aa:bb:cc:dd:ee:ff">
<button type="submit">123 Ziel speichern</button>
</form>
<div class="row"><span>BM6 aktuell</span><strong id="bm6addrsetup" class="mono">-</strong></div>
<form action="/bm6_target" method="post">
<label for="bm6Preset">BM6 Profil</label><select id="bm6Preset">
<option value="">Manuell</option>
<option value="3c:ab:72:80:06:6a">BM6 #1 3c:ab:72:80:06:6a</option>
</select>
<label for="bm6_mac">BM6 BLE-Adresse</label><input id="bm6_mac" name="bm6_mac" placeholder="aa:bb:cc:dd:ee:ff">
<button type="submit">BM6 Ziel speichern</button>
</form>
</div>
</details>
<details class="setup">
<summary>Spartan UART-Konfiguration</summary>
<div class="inside">
<p class="hint">Nur benutzen, wenn Orange/Gelb/Grau ueber Pegelwandler mit dem ESP32 verbunden sind. Die Befehle gehen direkt an Spartan UART.</p>
<div class="row"><span>Status</span><strong id="ustate">bereit</strong></div>
<div class="row"><span>Letzter Befehl</span><strong id="ucmd" class="mono">-</strong></div>
<div class="row"><span>Antwort / Timeout</span><strong id="uresp" class="mono">-</strong></div>
<div class="row"><span>Alter Befehl / Antwort</span><strong id="uage">-</strong></div>
<form action="/uart_cmd" method="post" class="grid" data-async="uart">
<button name="cmd" value="GETFW">GETFW</button>
<button name="cmd" value="GETHW">GETHW</button>
<button name="cmd" value="GETCANID">GETCANID</button>
<button name="cmd" value="GETCANBAUD">GETCANBAUD</button>
<button name="cmd" value="GETCANFORMAT">GETCANFORMAT</button>
<button name="cmd" value="GETCANDR">GETCANDR</button>
<button name="cmd" value="GETCANR">GETCANR</button>
<button name="cmd" value="GETTYPE">GETTYPE</button>
</form>
<form action="/uart_cmd" method="post" data-async="uart">
<label for="cmd">Expertenbefehl</label><input id="cmd" name="cmd" placeholder="z.B. SETCANID1024">
<button type="submit">Senden</button>
</form>
<form action="/spartan_can" method="post">
<label for="canid">CAN ID dezimal</label><input id="canid" name="canid" value="1024" inputmode="numeric">
<label for="baud">CAN Baud</label><select id="baud" name="baud"><option value="500000">500000 factory / ESP32 aktuell</option><option value="1000000">1000000</option></select>
<label for="format">CAN Format</label><select id="format" name="format"><option value="0">0 Default Lambda / MS3</option><option value="1">1 Link ECU</option><option value="2">2 Adaptronic</option><option value="3">3 Haltech WBC1</option><option value="4">4 %O2 x100</option><option value="5">5 Extended</option></select>
<label for="candrate">CAN Datenrate Hz</label><input id="candrate" name="candrate" value="50" inputmode="numeric">
<label for="term">Terminierung</label><select id="term" name="term"><option value="1">Ein</option><option value="0">Aus</option></select>
<button type="submit">CAN-Setup senden</button>
</form>
<form action="/spartan_output" method="post">
<label for="perf">Performance</label><select id="perf" name="perf"><option value="1">1 High 10 ms</option><option value="0">0 Standard 20 ms</option><option value="2">2 Lean</option></select>
<label for="slowheat">Slowheat</label><select id="slowheat" name="slowheat"><option value="0">0 normal</option><option value="1">1 langsam</option><option value="2">2 wartet auf MS3 CAN RPM</option><option value="3">3 wartet auf Abgas 350 C</option></select>
<label for="nbmode">Brown Output</label><select id="nbmode" name="nbmode"><option value="2">2 Heater Status</option><option value="0">0 Simulated Narrowband</option></select>
<button type="submit">Betriebsart senden</button>
</form>
</div>
</details>
<p class="hint">Aktuell ist der Bring-up-Modus aktiv. DEMO-Werte pruefen Display und Web-GUI, bis der CAN-Transceiver angeschlossen ist.</p>
</div><!-- /tab setup -->
<script>
let lastJson = {};
function showTab(name) {
  document.querySelectorAll('.tab-section').forEach(s => {
    s.hidden = s.dataset.tab !== name;
  });
  document.getElementById('tabLive').classList.toggle('on', name === 'live');
  document.getElementById('tabLog').classList.toggle('on', name === 'log');
  document.getElementById('tabSetup').classList.toggle('on', name === 'setup');
  try { localStorage.setItem('spartanTab', name); } catch (e) {}
}
try {
  const saved = localStorage.getItem('spartanTab');
  if (saved === 'setup' || saved === 'log') showTab(saved);
} catch (e) {}
document.getElementById('tunePreset').addEventListener('change', (e) => {
  document.getElementById('tune_mac').value = e.target.value || '';
});
document.getElementById('bm6Preset').addEventListener('change', (e) => {
  document.getElementById('bm6_mac').value = e.target.value || '';
});
document.getElementById('wifiPreset').addEventListener('change', (e) => {
  const option = e.target.selectedOptions[0];
  const ssid = option.value || '';
  document.getElementById('ssid').value = ssid;
  document.getElementById('pass').value = option.dataset.pass || '';
});
document.querySelectorAll('form[data-async=\"uart\"]').forEach((form) => {
  form.addEventListener('submit', async (e) => {
    e.preventDefault();
    const submitter = e.submitter;
    const fd = new FormData(form);
    if (submitter && submitter.name) {
      fd.set(submitter.name, submitter.value);
    }
    try {
      await fetch(form.action, { method: 'POST', body: fd, cache: 'no-store' });
      lastJson.uart_state = 'gesendet - warte auf Antwort';
      lastJson.uart_response = '';
      refresh();
    } catch (err) {}
  });
});
function cls(el, state) { el.className = state; }
function fmtBytes(n) {
  n = Number(n || 0);
  if (n < 1024) return n + ' B';
  if (n < 1048576) return (n / 1024).toFixed(1) + ' KB';
  return (n / 1048576).toFixed(2) + ' MB';
}
async function copyJson() {
  try { await navigator.clipboard.writeText(JSON.stringify(lastJson, null, 2)); } catch(e) {}
}
async function refresh() {
  try {
    const r = await fetch('/state', {cache:'no-store'});
    const d = await r.json();
    lastJson = d;
    document.getElementById('source').textContent = d.source;
    document.getElementById('lambda').textContent = d.valid ? d.lambda.toFixed(3) : '-.---';
    document.getElementById('status').textContent = d.status;
    document.getElementById('temp').textContent = d.valid ? d.temperature + ' C' : '- C';
    document.getElementById('main123').textContent = (d.rpm ?? 0) + ' / ' + Number(d.advance ?? 0).toFixed(1) + ' / ' + (d.map ?? 0);
    document.getElementById('can').textContent = d.can_ready ? 'aktiv' : 'Fehler';
    document.getElementById('wifiTop').textContent = d.wifi_connected ? d.wifi_ip : (d.ap_ip || 'offline');
    cls(document.getElementById('source'), d.source === 'CAN' ? 'tag ok' : (d.source === 'DEMO' ? 'tag warn' : 'tag bad'));
    cls(document.getElementById('status'), d.status === 'OK' ? 'ok' : (d.status === 'HEAT' || d.source === 'DEMO' ? 'warn' : 'bad'));
    cls(document.getElementById('can'), d.can_ready && d.can_state === 1 ? 'ok' : 'bad');
    document.getElementById('bleenabled').textContent = d.ble_name ? 'aktiv' : 'nicht im Build';
    document.getElementById('blename').textContent = d.ble_name || '-';
    document.getElementById('bleaddr').textContent = d.ble_address || '-';
    document.getElementById('bleclients').textContent = d.ble_clients ?? '0';
    document.getElementById('wifi').textContent = d.wifi_connected ? d.wifi_ssid : (d.wifi_saved ? 'verbindet...' : 'nicht eingerichtet');
    document.getElementById('lanip').textContent = d.wifi_connected ? d.wifi_ip : '-';
    document.getElementById('wifisaved').textContent = d.wifi_saved_ssid || '-';
    document.getElementById('liveWifiMeta').textContent = d.wifi_connected ? ((d.wifi_ssid || '-') + ' / ' + (d.wifi_ip || '-')) : ('AP ' + (d.ap_ip || '-'));
    document.getElementById('logstatus').textContent = d.log_ready ? 'bereit' : 'Dateisystem fehlt';
    document.getElementById('logcurrent').textContent = fmtBytes(d.log_current_bytes);
    document.getElementById('logold').textContent = fmtBytes(d.log_old_bytes);
    document.getElementById('ucmd').textContent = d.uart_command || '-';
    const ucmdAge = d.uart_age_ms ?? 0;
    const urspAge = d.uart_response_age_ms ?? 0;
    let uartState = d.uart_state || 'bereit';
    let uartResp = d.uart_response || '';
    if ((d.uart_command || '').length && !uartResp && ucmdAge > 1800) {
      uartState = 'Timeout - keine Antwort';
      uartResp = 'keine Antwort vom Spartan auf UART';
    }
    document.getElementById('ustate').textContent = uartState;
    cls(document.getElementById('ustate'), uartState.indexOf('Timeout') >= 0 ? 'bad' : (uartResp ? 'ok' : 'warn'));
    document.getElementById('uresp').textContent = uartResp || '-';
    document.getElementById('uage').textContent = Math.round(ucmdAge) + ' ms / ' + (uartResp ? Math.round(urspAge) + ' ms' : '-');
    document.getElementById('tconn').textContent = d.tune_connected ? 'verbunden' : 'scan/retry';
    cls(document.getElementById('tconn'), d.tune_connected ? 'ok' : 'warn');
    document.getElementById('trx').textContent = d.tune_rx ?? 0;
    document.getElementById('tage').textContent = (d.tune_age_ms ?? 0) + ' ms';
    document.getElementById('tscan').textContent = (d.tune_scan_seen ?? 0) + ' / ' + (d.tune_scan_candidates ?? 0);
    document.getElementById('terr').textContent = (d.tune_bad_length ?? 0) + ' / ' + (d.tune_unknown_opcode ?? 0);
    document.getElementById('tvals').textContent = (d.rpm ?? 0) + ' / ' + Number(d.advance ?? 0).toFixed(1) + ' / ' + (d.map ?? 0);
    document.getElementById('taddr').textContent = d.tune_saved_address || '-';
    document.getElementById('taddrsetup').textContent = d.tune_saved_address || '-';
    var tuneMac = document.getElementById('tune_mac');
    if (tuneMac && document.activeElement !== tuneMac) tuneMac.value = d.tune_saved_address || '';
    document.getElementById('candiag').textContent = (d.can_state ?? '-') + ' / ' + (d.can_tx_errors ?? 0) + ' / ' + (d.can_rx_errors ?? 0);
    document.getElementById('canerr').textContent = d.can_status_errors ?? 0;
    document.getElementById('heap').textContent = d.heap_free ? Math.round(d.heap_free / 1024) + ' KB' : '-';
    document.getElementById('hours').textContent = Number(d.device_hours ?? 0).toFixed(2) + ' / ' + Number(d.engine_hours ?? 0).toFixed(2) + ' / ' + Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    document.getElementById('liveHours').textContent = Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    document.getElementById('liveHoursMeta').textContent = Number(d.device_hours ?? 0).toFixed(2) + ' / ' + Number(d.engine_hours ?? 0).toFixed(2) + ' / ' + Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    document.getElementById('apdiag').textContent = (d.ap_ip || '-') + ' / ' + (d.ap_retry_count ?? 0);
    document.getElementById('bm6conn').textContent = d.bm6_connected ? 'verbunden' : 'scan/retry';
    cls(document.getElementById('bm6conn'), d.bm6_connected ? 'ok' : 'warn');
    document.getElementById('bm6volt').textContent = Number(d.bm6_voltage ?? 0).toFixed(2) + ' V';
    document.getElementById('liveBm6').textContent = Number(d.bm6_voltage ?? 0).toFixed(2) + ' V';
    document.getElementById('bm6temp').textContent = (d.bm6_temperature ?? 0) + ' C';
    document.getElementById('bm6rx').textContent = d.bm6_rx_count ?? 0;
    document.getElementById('bm6age').textContent = (d.bm6_age_ms ?? 0) + ' ms';
    document.getElementById('bm6err').textContent = d.bm6_decode_fail ?? 0;
    document.getElementById('liveTuneMeta').textContent = (d.tune_age_ms ?? 0) + ' ms / ' + (d.tune_rx ?? 0);
    document.getElementById('liveBm6Meta').textContent = (d.bm6_age_ms ?? 0) + ' ms / ' + (d.bm6_rx_count ?? 0);
    var bm6AddrSetup = document.getElementById('bm6addrsetup');
    if (bm6AddrSetup) bm6AddrSetup.textContent = d.bm6_saved_address || '-';
    var bm6Mac = document.getElementById('bm6_mac');
    if (bm6Mac && document.activeElement !== bm6Mac) bm6Mac.value = d.bm6_saved_address || '';
    var spdHz = document.getElementById('spdhz');
    if (spdHz) {
      spdHz.textContent = Number(d.speed_hz ?? 0).toFixed(2) + ' Hz';
      document.getElementById('spdkmh').textContent = Number(d.speed_kmh ?? 0).toFixed(1) + ' km/h';
      document.getElementById('liveSpeed').textContent = Number(d.speed_kmh ?? 0).toFixed(1) + ' km/h';
      document.getElementById('liveSpeedMeta').textContent = Number(d.speed_hz ?? 0).toFixed(2) + ' Hz / ' + (d.speed_pulses ?? 0);
      document.getElementById('spdpc').textContent = d.speed_pulses ?? 0;
      document.getElementById('spdppr').textContent = d.speed_pulses_per_rev ?? 10;
      var ctlTire = document.getElementById('ctlTire');
      var ctlTrim = document.getElementById('ctlTrim');
      if (ctlTire && document.activeElement !== ctlTire) ctlTire.value = d.speed_tire_mm ?? 2147;
      if (ctlTrim && document.activeElement !== ctlTrim) ctlTrim.value = d.speed_trim_permil ?? 1000;
    }
    document.getElementById('jsondump').textContent = JSON.stringify(d, null, 2);
  } catch (e) {}
}
refresh();
setInterval(() => { if (!document.hidden) refresh(); }, 350);
</script></main></body></html>)HTML");
  });

  const auto sendStatus = []() {
    server.send(200, "application/json", statusJson());
  };
  server.on("/state", sendStatus);
  server.on("/api/status", sendStatus);
  server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
  server.on("/gen_204", []() { server.send(204, "text/plain", ""); });
  server.on("/hotspot-detect.html", []() { server.send(200, "text/html", "<html><body>Success</body></html>"); });
  server.on("/ncsi.txt", []() { server.send(200, "text/plain", "Microsoft NCSI"); });
  server.on("/connecttest.txt", []() { server.send(200, "text/plain", "Microsoft Connect Test"); });
  server.on("/download", HTTP_GET, []() { sendLogFile(kLogFile, "spartan_hub_drive.csv"); });
  server.on("/download_old", HTTP_GET, []() { sendLogFile(kOldLogFile, "spartan_hub_drive_old.csv"); });
  server.on("/clear", HTTP_POST, []() {
    if (logFsReady) {
      SPIFFS.remove(kLogFile);
      ensureLogHeader();
    }
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
#endif
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
    WiFi.disconnect();
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.printf("Home WiFi:   saved and connecting to '%s'\n", ssid.c_str());
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
  });
  server.on("/wifi_clear", HTTP_POST, []() {
    networkPreferences.remove("ssid");
    networkPreferences.remove("pass");
    haveSavedWifi = false;
    savedWifiSsid = "";
    WiFi.disconnect();
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
  server.on("/bm6_target", HTTP_POST, []() {
#if ENABLE_BLE_HUB && ENABLE_BM6
    const String mac = normalizeMacInput(server.arg("bm6_mac"));
    if (!looksLikeMacAddress(mac)) {
      server.send(400, "text/plain", "BM6 BLE-Adresse ungueltig. Format aa:bb:cc:dd:ee:ff");
      return;
    }
    ensurePreferences();
    bm6SavedAddress = mac;
    networkPreferences.putString("bm6_mac", bm6SavedAddress);
    resetBm6Client();
    bm6DoConnect = false;
    scheduleBm6Scan();
    Serial.printf("BM6 BLE:     target override %s\n", bm6SavedAddress.c_str());
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
#else
    server.send(400, "text/plain", "BM6 nicht aktiv");
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
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
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

void updateWebGui()
{
#if ENABLE_WEB_GUI
  const uint32_t now = millis();
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  const wl_status_t wifiStatus = WiFi.status();
  if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0) && now - lastApRetryMs >= 10000) {
    lastApRetryMs = now;
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),
        IPAddress(192, 168, 4, 1),
        IPAddress(255, 255, 255, 0));
    if (WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD, 6, 0, 4)) {
      Serial.printf("Web GUI:     access point retry OK, http://%s/\n", WiFi.softAPIP().toString().c_str());
    } else {
      apRetryCount++;
      Serial.printf("Web GUI:     access point retry failed (%lu)\n", static_cast<unsigned long>(apRetryCount));
    }
  }
  if (wifiStatus != lastWifiStatus) {
    lastWifiStatus = wifiStatus;
    if (wifiStatus == WL_CONNECTED) {
      homeWifiDisabledForRoadAp = false;
      homeWifiConnectStartedMs = 0;
      Serial.printf("Home WiFi:   connected to '%s', http://%s/\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
  }
  if (haveSavedWifi && !homeWifiDisabledForRoadAp && wifiStatus != WL_CONNECTED &&
      homeWifiConnectStartedMs != 0 && now - homeWifiConnectStartedMs >= kHomeWifiConnectWindowMs) {
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(true);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),
        IPAddress(192, 168, 4, 1),
        IPAddress(255, 255, 255, 0));
    WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD, 6, 0, 4);
    homeWifiDisabledForRoadAp = true;
    Serial.printf("Home WiFi:   unavailable, road AP only '%s' http://%s/\n",
                  WEB_AP_SSID,
                  WiFi.softAPIP().toString().c_str());
  }
  dns.processNextRequest();
  server.handleClient();
#endif
}

void setupCan()
{
#if ENABLE_SPARTAN_CAN
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(CAN_TX_PIN),
      static_cast<gpio_num_t>(CAN_RX_PIN),
      TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&general, &timing, &filter) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("CAN start failed");
    return;
  }

  canReady = true;
  Serial.printf("CAN:         500 kbit/s RX=%u TX=%u Spartan ID=0x%03X\n", CAN_RX_PIN, CAN_TX_PIN, SPARTAN_CAN_ID);
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
      if (canStatus.state != TWAI_STATE_RUNNING ||
          canStatus.tx_error_counter > 127 ||
          canStatus.rx_error_counter > 127) {
        if (canStatusErrors < UINT32_MAX) {
          canStatusErrors++;
        }
      }
    }
  }

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.identifier != SPARTAN_CAN_ID || message.data_length_code < 4) {
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
#endif
}

void updateDemo()
{
#if ENABLE_SPARTAN_DEMO
  const uint32_t now = millis();
  SpartanReading snapshot = readingSnapshot();
  if (snapshot.fromCan && now - snapshot.receivedMs <= kCanStaleMs) {
    return;
  }

  SpartanReading fresh;
  if (now < 8000) {
    fresh.lambda = 1.000f;
    fresh.temperatureC = static_cast<uint16_t>(now / 12);
    fresh.status = 2;
  } else {
    const int32_t sweep = static_cast<int32_t>((now / 80) % 200) - 100;
    fresh.lambda = 1.000f + sweep / 1000.0f;
    fresh.temperatureC = 780;
    fresh.status = 3;
  }

  fresh.receivedMs = now;
  fresh.valid = true;
  fresh.fromCan = false;
  fresh.fromDemo = true;
  storeReading(fresh);
  digitalWrite(STATUS_LED_PIN, fresh.status == 3 ? HIGH : LOW);
#endif
}

void updateAnalog()
{
#if ENABLE_SPARTAN_ANALOG
  const SpartanReading snapshot = readingSnapshot();
  if ((snapshot.fromCan || snapshot.fromDemo) && millis() - snapshot.receivedMs <= kCanStaleMs) {
    return;
  }

  const uint32_t adcMilliVolts = analogReadMilliVolts(SPARTAN_ANALOG_PIN);
  const float inputVolts = (adcMilliVolts / 1000.0f) * ANALOG_DIVIDER_NUM / ANALOG_DIVIDER_DEN;
  const float constrainedVolts = constrain(inputVolts, 0.0f, 5.0f);
  SpartanReading fresh;
  fresh.lambda = kLambdaAtZeroVolt
      + (kLambdaAtFiveVolt - kLambdaAtZeroVolt) * constrainedVolts / 5.0f;
  fresh.temperatureC = 0;
  fresh.status = 3;
  fresh.receivedMs = millis();
  fresh.valid = true;
  fresh.fromCan = false;
  fresh.fromDemo = false;
  storeReading(fresh);
#endif
}

void updateHeaterAnalog()
{
#if ENABLE_SPARTAN_HEATER_ANALOG
  heaterAdcMilliVolts = analogReadMilliVolts(SPARTAN_HEATER_PIN);
  heaterStatusVolts = (heaterAdcMilliVolts / 1000.0f) * ANALOG_DIVIDER_NUM / ANALOG_DIVIDER_DEN;

  if (heaterStatusVolts < 0.5f) {
    heaterStatusCode = 0;
  } else if (heaterStatusVolts < 1.5f) {
    heaterStatusCode = 1;
  } else if (heaterStatusVolts < 2.5f) {
    heaterStatusCode = 2;
  } else {
    heaterStatusCode = 3;
  }
#endif
}

#if SPEED_REED_PIN >= 0
void IRAM_ATTR speedReedIsr()
{
  const uint32_t now = micros();
  if (now - speedLastEdgeUs < SPEED_DEBOUNCE_US) return;
  speedLastEdgeUs = now;
  speedPulseCount++;
}

void setupSpeedReed()
{
  // Konfig aus Prefs (falls vorhanden) lesen.
#if ENABLE_WEB_GUI
  ensurePreferences();
  if (networkPreferences.isKey("tire_mm")) {
    tireCircMm = networkPreferences.getUShort("tire_mm", TIRE_CIRC_MM_DEFAULT);
  }
  if (networkPreferences.isKey("trim_pm")) {
    speedTrimPermil = networkPreferences.getUShort("trim_pm", SPEED_TRIM_PERMIL_DEFAULT);
  }
#endif
  pinMode(SPEED_REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_REED_PIN), speedReedIsr, FALLING);
  speedPrevSampleMs = millis();
  Serial.printf("Speed:       Reed GPIO%d, %u pulses/rev, tire=%u mm, trim=%u permil\n",
                SPEED_REED_PIN, PULSES_PER_REV, tireCircMm, speedTrimPermil);
}

void updateSpeedReed()
{
  const uint32_t now = millis();
  const uint32_t dtMs = now - speedPrevSampleMs;
  if (dtMs < 250) return;  // 4 Hz Sampling

  noInterrupts();
  const uint32_t curCount = speedPulseCount;
  interrupts();
  const uint32_t pulses = curCount - speedPrevPulseCount;
  speedPrevPulseCount = curCount;
  speedPrevSampleMs = now;

  const float hz = (1000.0f * static_cast<float>(pulses)) / static_cast<float>(dtMs);
  speedHz = hz;
  // km/h = Hz * (TIRE_CIRC_MM / PULSES_PER_REV) / 1000 * 3.6 * trim
  //      = Hz * tireCircMm * 0.0036 / PULSES_PER_REV * (trim/1000)
  speedKmh = hz * static_cast<float>(tireCircMm) * 0.0036f
             / static_cast<float>(PULSES_PER_REV)
             * (static_cast<float>(speedTrimPermil) / 1000.0f);
}
#else
void setupSpeedReed() {}
void updateSpeedReed() {}
#endif

void setupUart()
{
#if ENABLE_SPARTAN_UART
  Serial2.begin(9600, SERIAL_8N1, SPARTAN_UART_RX_PIN, SPARTAN_UART_TX_PIN);
  Serial.printf(
      "UART:        9600 8N1 RX=%u TX=%u (configuration only; level shift Spartan TX)\n",
      SPARTAN_UART_RX_PIN,
      SPARTAN_UART_TX_PIN);
  Serial.println("UART bridge: type >GETFW or >GETCANID in USB serial monitor");
  Serial.println("123TUNE BLE: type !read for v@/10@/11@/12@/13@ dump or !v@ for one command");
#endif
}

void updateUart()
{
#if ENABLE_SPARTAN_UART
  if (lastUartCommandMs != 0 && lastUartResponseMs == 0 && millis() - lastUartCommandMs > 1800) {
    lastUartState = "Timeout - keine Antwort";
  }
  while (Serial2.available()) {
    const char c = static_cast<char>(Serial2.read());
    Serial.write(c);
    if (c == '\r' || c == '\n') {
      lastUartResponse.trim();
      if (lastUartResponse.length() > 0) {
        lastUartState = "Antwort empfangen";
        lastUartResponseMs = millis();
      }
    } else if (lastUartResponse.length() < 96 && isPrintable(c)) {
      lastUartResponse += c;
      lastUartState = "Antwort empfangen";
      lastUartResponseMs = millis();
    }
  }

  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (uartLine.startsWith(">") && uartLine.length() > 1) {
        sendSpartanUartCommand(uartLine.substring(1));
#if ENABLE_BLE_HUB
      } else if (uartLine == "!read") {
        runTuneReadDump();
      } else if (uartLine.startsWith("!") && uartLine.length() > 1) {
        String command = uartLine.substring(1);
        command.trim();
        if (command.length() > 0 && command.length() < 16) {
          sendTuneCommand(command.c_str());
        }
#endif
      }
      uartLine = "";
    } else {
      uartLine += c;
    }
  }
#endif
}

void updateDisplay()
{
  const uint32_t now = millis();
  if (now - lastDisplayMs < kDisplayIntervalMs) {
    return;
  }
  lastDisplayMs = now;

  const SpartanReading snapshot = readingSnapshot();
  const bool stale = !snapshot.valid || (snapshot.fromCan && now - snapshot.receivedMs > kCanStaleMs);
  if (stale) {
#if ENABLE_SPARTAN_UART && !ENABLE_SPARTAN_CAN
    displayLine(0, lastUartCommand.length() ? "UART " + lastUartCommand : "UART bereit");
    displayLine(1, lastUartResponse.length() ? lastUartResponse : ">GETFW testen");
#else
    displayLine(0, "SPARTAN CAN");
    displayLine(1, canReady ? "warte Daten..." : "CAN FEHLER");
#endif
    return;
  }

  // Feste 2-Zeilen-Anzeige (kein Springen mehr):
  //   Zeile 1: Lambda + RPM + Zuendwinkel (die 3 Hauptwerte)
  //   Zeile 2: Sensor-Temp + Status + 123-Verbindungsstatus
#if ENABLE_BLE_HUB
  const TuneSnapshot tune = tuneSnapshot();
  const bool tuneFresh = tune.lastRxMs != 0 && (now - tune.lastRxMs) <= 3000;
  const char *tuneState = "123:--";
  if (tuneConnected && tuneFresh) {
    tuneState = "123:OK";    // verbunden + frische Daten
  } else if (tuneConnected) {
    tuneState = "123:CON";   // verbunden, keine Daten (Zuendung aus)
  } else if (tuneDoConnect || tuneNextScanMs == 0) {
    tuneState = "123:SCN";   // sucht
  }
  const int rpmI = static_cast<int>(tune.rpm);
  const int advI = static_cast<int>(tune.advance + (tune.advance >= 0 ? 0.5f : -0.5f));

  char line1[24];
  // z.B. "L1.01 900 14"  -> Lambda, RPM, Zuendwinkel
  snprintf(line1, sizeof(line1), "L%.2f %d %d", snapshot.lambda, rpmI, advI);
  displayLine(0, line1);

  char line2[24];
  // z.B. "770C OK 123:OK"
  snprintf(line2, sizeof(line2), "%uC %s %s",
           snapshot.temperatureC, statusTextC(snapshot.status), tuneState);
  displayLine(1, line2);
#else
  char line1[24];
  snprintf(line1, sizeof(line1), "LAM %.3f %s", snapshot.lambda, sourceTextC(snapshot));
  displayLine(0, line1);
  char line2[24];
  snprintf(line2, sizeof(line2), "%uC %s", snapshot.temperatureC, statusTextC(snapshot.status));
  displayLine(1, line2);
#endif
}

void printBootDetails()
{
  Serial.println();
  Serial.println("Motorraum Spartan 3 v2 display");
  Serial.println("-------------------------------");
  Serial.printf("Device role: %s\n", DEVICE_ROLE);
#ifdef USE_M5_DISPLAY
  Serial.println("Display:     M5Unified");
#else
  Serial.printf("LCD:         %ux%u I2C 0x%02X SDA=%u SCL=%u\n", LCD_COLS, LCD_ROWS, LCD_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
#endif
#if ENABLE_SPARTAN_ANALOG
  Serial.printf("Analog:      pin=%u fallback enabled, divider=%u/%u\n", SPARTAN_ANALOG_PIN, ANALOG_DIVIDER_NUM, ANALOG_DIVIDER_DEN);
#else
  Serial.println("Analog:      fallback disabled at build time");
#endif
#if ENABLE_SPARTAN_HEATER_ANALOG
  Serial.printf("Heater ADC:  pin=%u enabled, divider=%u/%u\n", SPARTAN_HEATER_PIN, ANALOG_DIVIDER_NUM, ANALOG_DIVIDER_DEN);
#else
  Serial.println("Heater ADC:  disabled");
#endif
#if ENABLE_SPARTAN_DEMO
  Serial.println("Demo:        enabled until real CAN data is received");
#else
  Serial.println("Demo:        disabled");
#endif
#if ENABLE_WEB_GUI
  Serial.println("Web GUI:     enabled as WiFi access point");
#else
  Serial.println("Web GUI:     disabled");
#endif
#if ENABLE_BLE_HUB
  Serial.println("BLE hub:     enabled as GATT peripheral");
#else
  Serial.println("BLE hub:     disabled");
#endif
}

}  // namespace

void setup()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  setupDisplay();
  printBootDetails();
  setupBleHub();
  setupWebGui();
  setupHourmeters();
  setupCan();
  setupUart();
  setupSpeedReed();
#if ENABLE_SPARTAN_ANALOG
  analogSetPinAttenuation(SPARTAN_ANALOG_PIN, ADC_11db);
#endif
#if ENABLE_SPARTAN_HEATER_ANALOG
  analogSetPinAttenuation(SPARTAN_HEATER_PIN, ADC_11db);
#endif
}

void loop()
{
#ifdef USE_M5_DISPLAY
  M5.update();
#endif
  updateCan();
  updateDemo();
  updateAnalog();
  updateHeaterAnalog();
  updateUart();
  updateSpeedReed();
  updateHourmeters();
  appendLiveCsv();
  updateWebGui();
  updateBleHub();
  updateDisplay();
  delay(10);
}
