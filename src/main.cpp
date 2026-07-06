#include <Arduino.h>
#include <driver/twai.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <time.h>

#ifndef ENABLE_WEB_GUI
#define ENABLE_WEB_GUI 0
#endif

#ifndef ENABLE_BLE_HUB
#define ENABLE_BLE_HUB 0
#endif

#ifndef ENABLE_BLE_DISPLAY
#define ENABLE_BLE_DISPLAY ENABLE_BLE_HUB
#endif

#include "tune123_decode.h"  // gemeinsamer 123-Decoder (Hub/M5/Waveshare)

#if ENABLE_WEB_GUI
#include <DNSServer.h>
#include <FS.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <SPI.h>   // externer W25Q128 (FSPI) - Erkennungs-Check
#include <esp_netif.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#endif
#if defined(ENABLE_BLE_HUB)
#include "esp_coexist.h"   // BLE/WiFi-Funkpriorität (123-BLE-Empfang stabil halten)
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

// Default-WLAN-Profil nach Erase: 0=Bus(AP-only), 1=Zuhause, 2=Handy.
// Der Test-Hub (COM14) setzt das per Build-Flag auf 1, damit er sich direkt
// ins Heim-WLAN anmeldet. Der Live-Bus-Hub bleibt bei 0 (nur AP).
#ifndef DEFAULT_WIFI_PROFILE
#define DEFAULT_WIFI_PROFILE 0
#endif

// 123-Emulator: AUSGEGLIEDERT. Der Emu ist ein eigenes Geraet (separater ESP,
// COM17/COM22) und lebt im Branch `emu123`. Die Hub-Firmware ist reiner Client.

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

// Cockpit-Frame, das der Hub an die Displays (2.8" Touch / M5) sendet.
// 8 Byte, Big-Endian: [lambda_x1000][rpm][adv_x10 int16][map][flags]
#ifndef COCKPIT_CAN_TX_ID
#define COCKPIT_CAN_TX_ID 0x510
#endif
#ifndef COCKPIT_CAN_TX_INTERVAL_MS
#define COCKPIT_CAN_TX_INTERVAL_MS 100   // 10 Hz
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

// [FW-VERSION] Build-Stempel (Compile-Datum/-Zeit) -> WebGUI + /api/status, damit
// man auf jedem Geraet sieht, WELCHER Firmware-Stand laeuft (wichtig nach dem
// versehentlichen OTA, das Display-FW auf den Hub geschoben hatte).
#ifndef FW_BUILD
#define FW_BUILD (__DATE__ " " __TIME__)
#endif

// [W25Q-CFG] Config-Backup auf den externen W25Q128 -> WLAN-Daten + Setup ueberleben
// jeden Firmware-Flash (sogar erase_flash), weil der Chip dabei nie angefasst wird.
// Global deklariert, damit die Funktionen aus dem anonymen Namespace (loadHubFeatures,
// updateWebGui, Endpoints) erreichbar sind; Definition global neben detectW25Q().
void saveConfigToW25Q();
void restoreConfigFromW25Q();
// [KURVE-W25Q] Zuendkurven-Slots auf den Chip spiegeln (Format-Recovery-Schutz)
void saveCurveToW25Q(int slot);
void restoreCurvesFromW25Q();
bool hubCfgDirty = false;   // Config geaendert -> Backup beim naechsten Halt schreiben

namespace {
#ifdef USE_M5_DISPLAY
constexpr uint8_t kDisplayRows = 2;
#else
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
bool lcdReady = false;
#endif

constexpr uint32_t kDisplayIntervalMs = 200;
constexpr uint32_t kBleNotifyIntervalMs = 250;
constexpr uint32_t kCanStaleMs = 500;
// BLE-Bridge UART: optionaler zweiter ESP32 liefert 123-Daten per UART.
// Pin-Belegung wird zur Laufzeit per Dev-Tab gesetzt und in NVS gespeichert.
// RX-Pin 0 = UART-Bridge deaktiviert (Default nach Erase).
// Protokoll: "T,rpm,adv,map,coil,volt,temp\n" oder "BLE: LIVE rpm=X adv=Y map=Z"
static uint8_t bridgeUartRxPin = 0;   // 0 = aus; wird aus NVS geladen
static uint8_t bridgeUartTxPin = 0;
constexpr uint32_t kHomeWifiConnectWindowMs = 45000;  // 45s: BLE-Koexistenz verlangsamt den STA-Connect; 15s wuergte ihn ab
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
constexpr uint32_t kTuneScanWindowMs = 6000;
constexpr uint32_t kTuneReconnectDelayMs = 5000;
constexpr uint32_t kTunePingIntervalMs = 1000;  // 1s: häufigere Keepalives (war 1650ms)
constexpr uint32_t kBleHubAdvertiseFallbackMs = 30000;
// Stale-Timeout je nach Modus:
// Echte 123TUNE+ (nRF52810): kontinuierlich 8-12 Hz, 6s = 75× Sicherheitsmarge
constexpr uint32_t kTuneStaleRxMs = 6000;
constexpr uint32_t kTuneReconnectBackoffMaxMs = 30000;

enum class TuneLinkState : uint8_t {
  Idle = 0,
  Scanning,
  Connecting,
  Subscribed,
  Streaming,
};

#endif

struct SpartanReading {
  float lambda = 0.0f;
  uint16_t temperatureC = 0;
  uint8_t status = 0;
  uint32_t receivedMs = 0;
  bool valid = false;
  bool fromCan = false;
  bool fromDemo = false;
  bool fromTest = false;
};

enum class LambdaTestMode : uint8_t {
  Off = 0,
  Fixed = 1,
  Sweep = 2,
};

LambdaTestMode lambdaTestMode = LambdaTestMode::Off;
uint32_t lastLambdaTestMs = 0;

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
uint32_t lastCockpitCanTxMs = 0;
uint32_t cockpitCanTxCount  = 0;
uint32_t cockpitCanTxErrors = 0;
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

// [VARIANTE-A] Vorwaerts-Deklaration: Motor laeuft/faehrt? -> dann KEINE neuen
// Heim-/S24-STA-Versuche, damit der Hub-AP (Touch/Displays) bockstabil bleibt.
bool vehicleActive();
uint16_t tireCircMm = TIRE_CIRC_MM_DEFAULT;  // konfigurierbar
uint16_t speedTrimPermil = SPEED_TRIM_PERMIL_DEFAULT; // 1000 = 1.000
// [ODOMETER] Gesamt-km + Teilstrecke aus den Reed-Pulsen (exakter als Speed-
// Integration): mm = Pulse * Radumfang * Trim / (1000 * PULSES_PER_REV).
// Persistiert in NVS (odo_mm/trip_mm), gespart geschrieben (60s, nur bei Aenderung).
uint64_t odoMm = 0;                          // Gesamtstrecke in mm (Lebensdauer)
uint64_t tripMm = 0;                         // Teilstrecke in mm (Reset per GUI)
uint64_t odoLastSavedMm = 0;
uint32_t lastOdoSaveMs = 0;
#endif

String uartLine;
String lastUartCommand = "";
String lastUartResponse = "";
String lastUartState = "bereit";
uint32_t lastUartCommandMs = 0;
uint32_t lastUartResponseMs = 0;
String savedWifiSsid = "";
#if ENABLE_BLE_HUB
#if ENABLE_BLE_DISPLAY
NimBLECharacteristic *bleStatusCharacteristic = nullptr;
volatile uint8_t bleClientCount = 0;
bool bleAdvertisingStarted = false;
bool bleAdvPausedForRadio = false;
uint32_t bleHubSetupMs = 0;
#endif
String bleAddress = "";
TuneLinkState tuneLinkState = TuneLinkState::Idle;
uint8_t tuneFailStreak = 0;
volatile bool bleCentralScanActive = false;
uint32_t tuneStaleReconnectMs = 0;
struct BleHubClient {
  String address;
  String idAddress;
  uint16_t handle = 0;
  uint16_t mtu = 0;
  uint16_t intervalMs10 = 0;
  uint32_t connectedMs = 0;
};
constexpr uint8_t kBleHubClientMax = 4;
BleHubClient bleHubClients[kBleHubClientMax];
uint8_t bleHubClientCount = 0;
NimBLEClient *tuneClient = nullptr;
NimBLERemoteCharacteristic *tuneNusRx = nullptr;
NimBLEAddress tuneTargetAddress;
volatile bool tuneDoConnect = false;
volatile bool tuneConnected = false;
uint32_t tuneNextScanMs = 0;
uint32_t tuneLastPingMs = 0;
// [TUNE-LIVE] Live-Zuendwinkel-Verstellung (123 Tuning-Modus): T=an/aus, A=vor, R=zurueck.
// Reiner Runtime-Offset auf den aktuellen Advance -> NICHT in die Kurve geschrieben,
// verschwindet beim Verlassen (T) und bei Verbindungsabriss. Sicherheitskritisch:
// nur bei streaming, hinter dem Lock in der GUI. tuneAdvSteps = unser kommandierter
// Netto-Offset in Schritten (der echte Winkel steht live auf dem 0x31-Advance-Gauge).
bool tuneModeActive = false;
int  tuneAdvSteps = 0;
// [TUNE-SAFE] Dead-Man: Zeitstempel der letzten /api/tune/live-Aktion (inkl. GUI-Ping).
// Bleibt die GUI 60 s stumm (Handy-WLAN abgerissen), raeumt updateTuneBle() den
// Offset ab und verlaesst den Tune-Modus.
uint32_t tuneLastLiveApiMs = 0;
constexpr uint32_t kTuneDeadManMs = 60000;
// [TUNE-SAFE] Reset-Drain: tuneAdvReset() setzt nur das Flag, updateTuneBle() sendet
// die Gegenschritte einzeln (30ms-Abstand) -- der alte delay(30)-Loop blockierte den
// gesamten loop() (WebServer, CAN-0x510, Log) bis zu 1,8 s im HTTP-Handler.
volatile bool tuneResetDraining = false;
uint32_t tuneResetLastStepMs = 0;
// [KURVE-READ] EEPROM-Kurve aus der 123 lesen (Befehle 10@..13@). Im Lese-Modus
// sammelt onTuneNotify die Roh-Antworten in curveReadBuf; updateTuneBle staffelt die
// Blockbefehle; der Browser dekodiert. Reiner Capture -> minimale Chip-Last.
volatile bool curveReadActive = false;
char     curveReadBuf[640];
volatile uint16_t curveReadLen = 0;
uint32_t curveReadStartMs = 0;
uint8_t  curveReadPhase = 0;   // 1..4 = naechster zu sendender Block, 0 = idle
uint32_t tuneScanSeen = 0;
uint32_t tuneScanCandidates = 0;
String tuneSavedAddress = "";
uint32_t tuneConnStartMs = 0;  // Start der aktuellen 123-Verbindung (für Stale-Bezug)
struct BleScanDevice {
  String address;
  String name;
  int rssi = 0;
  bool tuneLike = false;
  uint32_t seenMs = 0;
};
constexpr uint8_t kBleScanDeviceMax = 12;
BleScanDevice bleScanDevices[kBleScanDeviceMax];
uint8_t bleScanDeviceCount = 0;
#endif
#if ENABLE_WEB_GUI
WebServer server(80);
DNSServer dns;
Preferences networkPreferences;
bool networkPreferencesReady = false;
bool otaBusy = false;
bool otaAuthFailed = false;   // [OTA-LOCK] Token fehlte/falsch -> Upload verworfen (Firmware bleibt)
size_t otaRxBytes = 0;
uint32_t otaStartedMs = 0;
String otaToken;              // [OTA-LOCK] leer = OTA gesperrt (Schutz vor Fremd-/Fehl-OTA)
bool hubFeatAp = true;
bool hubFeatWifi = true;
bool hubFeatLog = true;
bool hubFeatBle123 = false;
// WiFi-Profile: 0=Bus(AP-only), 1=Zuhause, 2=Handy
struct HubWifiProfile {
  char ssid[33];
  char pass[65];
  const char* label;
  // [WIFI-STATIC] ipMode 0=DHCP (Default), 1=Static. Nur fuer STA-Profile 1/2 relevant.
  // Motivation: Geraete ohne FritzBox-Reservierung (z.B. Tischtest-Emu) wandern sonst
  // bei jedem Boot auf eine neue Lease-Adresse.
  uint8_t ipMode;
  char ip[16];
  char gw[16];
  char mask[16];
};
static HubWifiProfile g_hubWifiProfiles[3] = {
  { "", "", "Bus",     0, "", "", "" },   // Slot 0: AP-only, kein STA
  { "", "", "Zuhause", 0, "", "", "" },   // Slot 1: NVS p1_ssid/p1_pass
  { "", "", "Handy",   0, "", "", "" },   // Slot 2: NVS p2_ssid/p2_pass
};
// [WIFI-STATIC] WiFi.config() muss VOR WiFi.begin() laufen. true = statisch angewendet.
bool applyStaticIpIfNeeded(uint8_t profileIdx)
{
  if (profileIdx < 1 || profileIdx > 2) return false;
  const HubWifiProfile &p = g_hubWifiProfiles[profileIdx];
  if (p.ipMode != 1) return false;
  IPAddress ip, gw, mask;
  if (!ip.fromString(p.ip) || !gw.fromString(p.gw) || !mask.fromString(p.mask)) {
    Serial.printf("WiFi Static: Profil %d ungueltige IP/GW/Mask - falle auf DHCP zurueck\n", profileIdx);
    return false;
  }
  WiFi.config(ip, gw, mask);
  Serial.printf("WiFi Static: Profil %d -> %s (GW %s, Mask %s)\n", profileIdx, p.ip, p.gw, p.mask);
  return true;
}

// [WIFI-MAC-OVR] Manuelle STA-MAC statt Werks-eFuse. Hintergrund: billige ESP32-Klone
// haben die MAC teils nicht sauber ins eFuse programmiert und liefern dann bei mehreren
// Geraeten die GLEICHE Default-MAC -> Verwechslungen/Konflikte im LAN (genau das Muster,
// das den Emu zeitweise mit einem Fremdgeraet verwechselbar machte). Leer = Werks-MAC.
char g_wifiMacOverride[18] = "";   // "AA:BB:CC:DD:EE:FF" oder leer

bool parseMac6(const char *s, uint8_t out[6])
{
  if (!s || strlen(s) < 17) return false;
  int v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) {
    if (v[i] < 0 || v[i] > 255) return false;
    out[i] = static_cast<uint8_t>(v[i]);
  }
  return true;
}

// Vor WiFi.mode()/erstem Connect aufrufen. Multicast-Bit (LSB von Byte 0) wird hart
// geloescht -- eine Multicast-MAC als Unicast-Stationsadresse haengt sich sonst im
// WLAN auf, ohne dass es offensichtlich waere.
void applyWifiMacOverrideIfNeeded()
{
  if (strlen(g_wifiMacOverride) == 0) return;
  uint8_t mac[6];
  if (!parseMac6(g_wifiMacOverride, mac)) {
    Serial.printf("WiFi MAC:    Override '%s' ungueltig - ignoriert\n", g_wifiMacOverride);
    return;
  }
  mac[0] &= 0xFE;   // Multicast-Bit raus
  if (esp_wifi_set_mac(WIFI_IF_STA, mac) == ESP_OK) {
    Serial.printf("WiFi MAC:    Override aktiv -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("WiFi MAC:    esp_wifi_set_mac fehlgeschlagen (WiFi-Treiber noch nicht bereit?)");
  }
}
uint8_t hubWifiProfile = 0;  // aktiv: 0=Bus, 1=Zuhause, 2=Handy
bool haveSavedWifi = false;
uint32_t homeWifiConnectStartedMs = 0;
bool homeWifiDisabledForRoadAp = false;
uint32_t lastApRetryMs = 0;
uint32_t apRetryCount = 0;
bool logFsReady = false;
String logError = "";
bool logErrorReported = false;
size_t logCurrentBytes = 0;
size_t logOldBytes = 0;
uint32_t lastCsvAppendMs = 0;
struct TimezoneEntry {
  const char *label;
  const char *posix;
};
const TimezoneEntry kTimezones[] = {
    {"Europe/Berlin (CET/CEST)", "CET-1CEST,M3.5.0,M10.5.0/3"},
    {"UTC", "UTC0"},
    {"Europe/London (GMT/BST)", "GMT0BST,M3.5.0/1,M10.5.0"},
    {"America/New_York (EST/EDT)", "EST5EDT,M3.2.0,M11.1.0"},
    {"America/Los_Angeles (PST/PDT)", "PST8PDT,M3.2.0,M11.1.0"},
    {"Asia/Tokyo (JST)", "JST-9"},
};
const uint8_t kTimezoneCount = static_cast<uint8_t>(sizeof(kTimezones) / sizeof(kTimezones[0]));
const uint8_t kTimezoneDefault = 0;
const char *kNtpServerPrimary = "pool.ntp.org";
const char *kNtpServerSecondary = "time.nist.gov";
const char *kNtpServerTertiary = "de.pool.ntp.org";
constexpr uint32_t kNtpResyncIntervalMs = 15UL * 60UL * 1000UL;
uint8_t timezoneIdx = kTimezoneDefault;
bool ntpStarted = false;
bool ntpSynced = false;
bool ntpResyncRequested = false;
uint32_t lastNtpSyncMs = 0;
const char *kLogFile = "/drive.csv";
const char *kOldLogFile = "/drive_old.csv";
// [KURVE] bis zu 3 hinterlegte 123-Zuendkurven (.123-XML) als Slots.
const char *kCurveFiles[3] = { "/curve1.123", "/curve2.123", "/curve3.123" };
inline const char *curveFile(int slot) { if (slot < 1 || slot > 3) slot = 1; return kCurveFiles[slot - 1]; }
// [KURVE] Slot-Belegung als Cache (Bit0=Slot1..Bit2=Slot3): statusJson wird mit
// 5-10 Hz gepollt, 3x SPIFFS.exists() pro Poll frisst FS-Zeit im loop() (gleiches
// Muster wie refreshLogSizeCache). Refresh nur bei Boot/Upload/Delete.
uint8_t curveSlotMask = 0;
void refreshCurveSlotCache()
{
  uint8_t m = 0;
  if (logFsReady) { for (int s = 1; s <= 3; s++) if (SPIFFS.exists(curveFile(s))) m |= (1 << (s - 1)); }
  curveSlotMask = m;
}
const size_t kMaxLogBytes = 200000;  // 200 KB: kleine Dateien halten SPIFFS schnell (grosse Logs blockieren loop() -> Webserver/Display-Timeouts)
const uint32_t kLogIntervalMs = 500;
const uint16_t kLogColSpartan = 0x0001;
const uint16_t kLogColTune = 0x0002;
// 0x0004 war die BM6-Spalte (entfernt); Bitwerte bleiben stabil, damit in NVS
// gespeicherte Masken (log_cols) weiterhin richtig interpretiert werden.
const uint16_t kLogColSpeed = 0x0008;
const uint16_t kLogColHeater = 0x0010;
const uint16_t kLogColHours = 0x0020;
const uint16_t kLogColDefault = kLogColSpartan | kLogColTune |
                                kLogColSpeed | kLogColHeater | kLogColHours;
uint16_t logColumnMask = kLogColDefault;
struct WifiApStation {
  String mac;
  String ip;
  int8_t rssi = 0;
  uint32_t firstSeenMs = 0;
  uint32_t lastSeenMs = 0;
};
constexpr uint8_t kWifiApStationMax = 4;
WifiApStation wifiApStations[kWifiApStationMax];
uint8_t wifiApStationCount = 0;
String hubApSsid = WEB_AP_SSID;
String hubApPassword = WEB_AP_PASSWORD;
String hubApIp = "192.168.4.1";  // in WebGUI (Dev -> Access Point) frei einstellbar
// [WLAN-KANAL] Single-Radio-Fix: der ESP32-S3 hat nur EINE 2,4-GHz-Funkeinheit,
// AP und STA muessen sich denselben Kanal teilen. Frueher war der SoftAP fest auf
// Kanal 6 -> verband sich der Hub mit einem Home-Router auf einem ANDEREN Kanal,
// assoziierte er zwar (Auth OK), bekam aber keine IP (DHCP scheitert). hubApChannel
// folgt jetzt dem Router-Kanal (per Scan ermittelt) und wird in NVS gemerkt, damit
// der AP nach einem Reboot gleich richtig hochkommt (kein Scan noetig).
uint8_t hubApChannel = 6;                 // aktueller SoftAP-Kanal (folgt dem Home-Router)
volatile bool homeWifiNotFound = false;   // letzter Connect: AP nicht gefunden (reason 201)
volatile bool apSuspendedForConnect = false;  // AP haengt waehrend STA-Connect (Single-Radio)
String hubHostname = "spartanhub";  // mDNS-Name -> http://<name>.local, in WebGUI editierbar
String hubApMask = "255.255.255.0";
struct WifiHttpPoller {
  String ip;
  String mac;
  String deviceId;
  String userAgent;
  String via;
  uint32_t firstPollMs = 0;
  uint32_t lastPollMs = 0;
  uint32_t pollCount = 0;
};
constexpr uint8_t kWifiHttpPollerMax = 8;
WifiHttpPoller wifiHttpPollers[kWifiHttpPollerMax];
uint8_t wifiHttpPollerCount = 0;
struct HubEvent {
  uint32_t ms = 0;
  uint32_t epoch = 0;
  char type[12] = {};
  char detail[72] = {};
};
constexpr uint16_t kHubEventMax = 200;
HubEvent hubEvents[kHubEventMax];
uint16_t hubEventHead = 0;
uint16_t hubEventCount = 0;
portMUX_TYPE hubEventMux = portMUX_INITIALIZER_UNLOCKED;
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

void saveHubFeatures()
{
  ensurePreferences();
  networkPreferences.putUChar("hf_ver", 1);
  networkPreferences.putBool("hf_ap", hubFeatAp);
  networkPreferences.putBool("hf_wifi", hubFeatWifi);
  networkPreferences.putBool("hf_log", hubFeatLog);
  networkPreferences.putBool("hf_ble123", hubFeatBle123);
  networkPreferences.putUChar("lambda_test", static_cast<uint8_t>(lambdaTestMode));
}

void startHubMdns();  // fwd: in onHubWifiEvent (STA GOT IP) benoetigt
volatile int lastStaReason = 0;        // [WLAN-DIAG] letzter STA-Disconnect-Reason
volatile uint8_t lastStaEvent = 0;     // 1=assoziiert 2=GOT_IP 3=disconnected
bool        w25qDetected = false;       // externer W25Q128-Flash erkannt?
uint32_t    w25qJedecId  = 0;
uint32_t    w25qSizeMB   = 0;
const char *w25qMfg      = "-";

// [WLAN-DIAG] Loggt STA-Events inkl. Reason-Code, damit ein fehlschlagender
// Heimnetz-Connect eindeutig wird (201=NO_AP_FOUND/Signal, 15/204=HANDSHAKE/Passwort,
// 202=AUTH_FAIL, 203=ASSOC_FAIL, 8=ASSOC_LEAVE).
void onHubWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      lastStaEvent = 1;
      Serial.println("WiFi-EVT:    STA assoziiert (AP gefunden+Auth OK), warte auf IP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      lastStaEvent = 2;
      Serial.printf("WiFi-EVT:    GOT IP %s\n", WiFi.localIP().toString().c_str());
      startHubMdns();  // mDNS auf neuem STA-Netz (z.B. S24) neu registrieren
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      lastStaEvent = 3;
      lastStaReason = info.wifi_sta_disconnected.reason;
      homeWifiNotFound = (lastStaReason == 201);  // 201 = NO_AP_FOUND (Reichweite/Band)
      Serial.printf("WiFi-EVT:    STA disconnected reason=%d\n",
                    info.wifi_sta_disconnected.reason);
      break;
    default:
      break;
  }
}

void loadHubFeatures()
{
  ensurePreferences();
  restoreConfigFromW25Q();   // [W25Q-CFG] frisches NVS (nach Flash/erase) aus Backup fuellen
  // Erst-Backup UND Versions-Upgrade (3 = HUB_CFG_VERSION in hub_w25q.h): altes v2-Blob
  // wird zeitnah als v3 (mit Betriebsstunden/Odometer) neu geschrieben.
  if (w25qDetected && networkPreferences.getUChar("cfg_w25q", 0) != 3) hubCfgDirty = true;
  hubApSsid = networkPreferences.getString("ap_ssid", WEB_AP_SSID);
  hubApPassword = networkPreferences.getString("ap_pass", WEB_AP_PASSWORD);
  hubApIp = networkPreferences.getString("ap_ip", "192.168.4.1");
  hubApChannel = networkPreferences.getUChar("ap_chan", 6);  // [WLAN-KANAL] letzter Home-Kanal
  if (hubApChannel < 1 || hubApChannel > 13) hubApChannel = 6;
  hubApMask = networkPreferences.getString("ap_mask", "255.255.255.0");
  hubHostname = networkPreferences.getString("mdns_host", "spartanhub");
  hubHostname.trim();
  if (hubHostname.length() == 0) hubHostname = "spartanhub";
  if (hubApSsid.length() == 0) hubApSsid = WEB_AP_SSID;
  if (hubApMask.length() == 0) hubApMask = "255.255.255.0";
  if (!networkPreferences.isKey("hf_ver")) {
#if BLE_BRIDGE
    // BLE-Coprozessor: KEIN WLAN/AP (Funk frei, kein Brownout), nur 123-BLE AN.
    hubFeatAp = false; hubFeatWifi = false; hubFeatLog = false;
    hubFeatBle123 = true;
#else
#ifdef MINIMAL_123
    // Minimal-Modus: NUR 123-BLE + Lambda(CAN) + Log + Web. Eine einzige BLE-
    // Verbindung entlastet den NimBLE-ACL-Pool -> stabile 123.
    hubFeatAp = true;       // AP fuer Web/API-Zugriff waehrend Fahrt
    hubFeatWifi = true;     // STA an: Hub im Heimnetz erreichbar (Verifikation + Zugriff)
    hubFeatLog = true;
    hubFeatBle123 = true;
#else
    hubFeatAp = true;
    hubFeatWifi = true;
    hubFeatLog = true;
#ifdef DEFAULT_BLE123_ON
    hubFeatBle123 = true;
#else
    hubFeatBle123 = false;
#endif
#endif
#endif
    saveHubFeatures();
    Serial.println("Hub feats:   defaults gesetzt");
    return;
  }
  bool defaultWifi = strlen(HOME_WIFI_SSID) > 0;
  if (networkPreferences.isKey("ssid")) {
    defaultWifi = networkPreferences.getString("ssid", "").length() > 0;
  }
  hubFeatAp = networkPreferences.getBool("hf_ap", true);
  hubFeatWifi = networkPreferences.getBool("hf_wifi", defaultWifi);
  hubFeatLog = networkPreferences.getBool("hf_log", true);
  hubFeatBle123 = networkPreferences.getBool("hf_ble123", false);
  const uint8_t savedLambdaTest = networkPreferences.getUChar("lambda_test", 0);
  lambdaTestMode = savedLambdaTest <= static_cast<uint8_t>(LambdaTestMode::Sweep)
      ? static_cast<LambdaTestMode>(savedLambdaTest)
      : LambdaTestMode::Off;
}

bool parseApAddress(const String &text, const char *fallback, IPAddress &out)
{
  String value = text;
  value.trim();
  if (!out.fromString(value)) {
    out.fromString(fallback);
    return false;
  }
  return true;
}

void saveHubApConfig(const String &ssid, const String &password, const String &ip, const String &mask)
{
  ensurePreferences();
  hubApSsid = ssid;
  hubApSsid.trim();
  hubApPassword = password;
  hubApIp = ip;
  hubApIp.trim();
  hubApMask = mask;
  hubApMask.trim();
  if (hubApSsid.length() == 0) hubApSsid = WEB_AP_SSID;
  if (hubApMask.length() == 0) hubApMask = "255.255.255.0";
  networkPreferences.putString("ap_ssid", hubApSsid);
  networkPreferences.putString("ap_pass", hubApPassword);
  networkPreferences.putString("ap_ip", hubApIp);
  networkPreferences.putString("ap_mask", hubApMask);
}

void ensureHubSoftAp()
{
#if ENABLE_WEB_GUI
  if (!hubFeatAp) {
    WiFi.softAPdisconnect(true);
    if (hubFeatWifi || WiFi.status() == WL_CONNECTED) {
      WiFi.mode(WIFI_STA);
    } else {
      WiFi.mode(WIFI_OFF);
    }
    return;
  }
  // AP-Name immer einheitlich (Spartan3-Setup), auch im Emulator-Modus —
  // sonst sucht der M5 "Spartan3-Setup", findet im Emu nur "123-emulator" und
  // bindet sich nicht auf Kanal 6. Gleicher Name in Bus- UND Emu-Modus = M5
  // assoziiert immer korrekt, Touch (Funk-Bus) ist ohnehin AP-unabhängig.
  const char *apSsid = (hubApSsid.length() > 0) ? hubApSsid.c_str() : WEB_AP_SSID;
  // Nur skippen, wenn der AP schon mit dem RICHTIGEN Namen laeuft. (WIFI_AP_STA
  // setzt sonst eine AP-IP ohne SSID -> AP unsichtbar.)
  if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0) && WiFi.softAPSSID() == String(apSsid)) {
    return;
  }
  WiFi.onEvent(onHubWifiEvent);  // [WLAN-DIAG] STA-Reason-Codes loggen
  if (hubFeatWifi || WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  IPAddress apIp, apMask;
  parseApAddress(hubApIp, "192.168.4.1", apIp);
  parseApAddress(hubApMask, "255.255.255.0", apMask);
  WiFi.softAPConfig(apIp, apIp, apMask);
  if (WiFi.softAP(apSsid, hubApPassword.c_str(), hubApChannel, 0, 4)) {
    Serial.printf("Web GUI:     access point '%s' OK (Kanal %d), http://%s/\n",
                  apSsid, hubApChannel, WiFi.softAPIP().toString().c_str());
  }
#endif
}

bool mdnsStarted = false;
// mDNS-Responder (neu)starten: Hub bleibt als http://<hubHostname>.local erreichbar,
// egal ob ueber Hub-AP oder S24-Hotspot (wechselnde IP).
void startHubMdns()
{
  MDNS.end();
  if (MDNS.begin(hubHostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.printf("mDNS:        http://%s.local/\n", hubHostname.c_str());
  } else {
    mdnsStarted = false;
    Serial.println("mDNS:        start failed");
  }
}

// [WLAN-KANAL] Home/S24-STA verbinden. Single-Radio-Trick OHNE Scan: der SoftAP
// wird fuer den Connect kurz ausgesetzt (reine STA), damit die STA auf dem Kanal
// des Routers sauber DHCP bekommt. Nach GOT_IP bringt updateWebGui() den AP auf
// genau diesem Kanal zurueck (hubApChannel, in NVS gemerkt). Scan-frei = immun
// gegen die BLE+AP-Scan-Stoerung ("Scan fehlgeschlagen"). Der AP-Blip passiert nur
// beim manuellen Verbinden (im Stand), nie periodisch/fahrend.
void connectHomeWifiAligned(const char *ssid, const char *pass)
{
  if (vehicleActive()) {   // [VARIANTE-A] fahrend kein Connect -> AP/Touch bleibt stabil
    Serial.println("Variante A:  Motor laeuft -> Heim-Connect verschoben bis zum Stand");
    return;   // im Stand holt es applyHubFeatures() / das Motor-Gate nach
  }
  homeWifiNotFound = false;
  if (hubFeatAp) {
    apSuspendedForConnect = true;
    WiFi.mode(WIFI_STA);   // SoftAP aus -> Funkeinheit frei fuer den Router-Kanal
    Serial.println("WLAN-Kanal:  AP fuer Connect kurz ausgesetzt (reine STA, ~5-20s)");
  }
  applyStaticIpIfNeeded(hubWifiProfile);   // [WIFI-STATIC] vor WiFi.begin()!
  WiFi.begin(ssid, pass);
  homeWifiConnectStartedMs = millis();
}

String normalizeMacInput(const String &raw)
{
  String mac = raw;
  mac.trim();
  mac.toLowerCase();
  return mac;
}

String jsonEscape(const String &raw)
{
  String out;
  out.reserve(raw.length() + 4);
  for (size_t i = 0; i < raw.length(); i++) {
    const char c = raw[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c >= 0x20) {
      out += c;
    }
  }
  return out;
}

#if ENABLE_BLE_HUB
void recordBleScanDevice(const NimBLEAdvertisedDevice *device, bool tuneLike)
{
  String address = device->getAddress().toString().c_str();
  address.toLowerCase();
  String name = device->getName().c_str();
  const int rssi = device->getRSSI();
  const uint32_t now = millis();

  // nRF-artiger Scanner-Log: Name + Hersteller(Company+Daten) + Service-Flags.
  String mfg = "-";
  if (device->haveManufacturerData()) {
    std::string m = device->getManufacturerData();
    if (m.size() >= 2) {
      uint16_t comp = (uint8_t)m[0] | ((uint16_t)(uint8_t)m[1] << 8);
      char cb[12]; snprintf(cb, sizeof(cb), "C%u:", comp); mfg = cb;
      for (size_t i = 2; i < m.size() && i < 16; i++) {
        char h[3]; snprintf(h, sizeof(h), "%02X", (uint8_t)m[i]); mfg += h;
      }
    }
  }
  Serial.printf("[BLE-SCAN] %s rssi=%d name=%s mfg=%s tune=%d\n",
                address.c_str(), rssi, name.length() ? name.c_str() : "---",
                mfg.c_str(), tuneLike ? 1 : 0);

  uint8_t slot = bleScanDeviceCount;
  for (uint8_t i = 0; i < bleScanDeviceCount; i++) {
    if (bleScanDevices[i].address == address) {
      slot = i;
      break;
    }
  }
  if (slot >= kBleScanDeviceMax) {
    slot = 0;
    for (uint8_t i = 1; i < bleScanDeviceCount; i++) {
      if (bleScanDevices[i].seenMs < bleScanDevices[slot].seenMs) slot = i;
    }
  } else if (slot == bleScanDeviceCount) {
    bleScanDeviceCount++;
  }

  bleScanDevices[slot].address = address;
  bleScanDevices[slot].name = name;
  bleScanDevices[slot].rssi = rssi;
  bleScanDevices[slot].tuneLike = bleScanDevices[slot].tuneLike || tuneLike;
  bleScanDevices[slot].seenMs = now;
}
#endif

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
  if (!lcdReady) {
    return;
  }
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
  Wire.beginTransmission(LCD_I2C_ADDR);
  const uint8_t lcdProbe = Wire.endTransmission();
  if (lcdProbe != 0) {
    Serial.printf("LCD:         not found at 0x%02X (I2C err=%u), display disabled\n",
                  LCD_I2C_ADDR,
                  lcdProbe);
    lcdReady = false;
    return;
  }
  lcdReady = true;
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

SpartanReading readingSnapshot();
void storeReading(const SpartanReading &fresh);
void logHubEvent(const char *type, const char *detail);

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
  if (snapshot.fromTest) {
    return "TEST";
  }
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

const char *lambdaTestModeText()
{
  switch (lambdaTestMode) {
    case LambdaTestMode::Fixed: return "fixed";
    case LambdaTestMode::Sweep: return "sweep";
    default: return "off";
  }
}

bool setLambdaTestMode(const String &requested)
{
  String mode = requested;
  mode.trim();
  mode.toLowerCase();
  LambdaTestMode next;
  if (mode == "off" || mode == "0") next = LambdaTestMode::Off;
  else if (mode == "fixed" || mode == "1") next = LambdaTestMode::Fixed;
  else if (mode == "sweep" || mode == "2") next = LambdaTestMode::Sweep;
  else return false;

  lambdaTestMode = next;
  lastLambdaTestMs = 0;
#if ENABLE_WEB_GUI
  ensurePreferences();
  networkPreferences.putUChar("lambda_test", static_cast<uint8_t>(lambdaTestMode));
  logHubEvent("lambda_test", lambdaTestModeText());
#endif
  if (lambdaTestMode == LambdaTestMode::Off) {
    SpartanReading snapshot = readingSnapshot();
    if (snapshot.fromTest) {
      SpartanReading cleared;
      storeReading(cleared);
    }
  }
  Serial.printf("Lambda test: %s\n", lambdaTestModeText());
  return true;
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
bool systemTimeValid();
String csvTimeText(uint32_t ms);
const char *timezoneLabel(uint8_t idx);
const char *timezonePosix(uint8_t idx);
void applyTimezone();
void startNtpIfNeeded();
void updateNtp(uint32_t now);
void requestNtpResync();
void saveTimezone(uint8_t idx);
void ensureLogHeader();
void sendLogFile(const char *path, const char *downloadName);
void refreshWifiApStations();
void recordWifiHttpPoller();
String formatMacLower(const uint8_t mac[6]);
String apStationIpFromMac(const uint8_t mac[6]);
void logHubEvent(const char *type, const char *detail);
String hubEventsJson(uint16_t limit = 100);
void clearHubEvents();
#endif

#if !ENABLE_WEB_GUI
void logHubEvent(const char *, const char *) {}
#endif

#if ENABLE_BLE_HUB
const char *tuneLinkStateText(TuneLinkState state)
{
  switch (state) {
    case TuneLinkState::Scanning: return "scanning";
    case TuneLinkState::Connecting: return "connecting";
    case TuneLinkState::Subscribed: return "subscribed";
    case TuneLinkState::Streaming: return "streaming";
    default: return "idle";
  }
}

void setTuneLinkState(TuneLinkState state, const char *detail)
{
  if (tuneLinkState == state) return;
  tuneLinkState = state;
  char buf[96];
  if (detail != nullptr && detail[0] != '\0') {
    snprintf(buf, sizeof(buf), "%s|%s", tuneLinkStateText(state), detail);
  } else {
    snprintf(buf, sizeof(buf), "%s", tuneLinkStateText(state));
  }
  logHubEvent("tune_state", buf);
}

#if ENABLE_BLE_DISPLAY
void pauseBleDisplayAdvertising()
{
  if (!bleAdvertisingStarted || bleAdvPausedForRadio) return;
  NimBLEDevice::getAdvertising()->stop();
  bleAdvPausedForRadio = true;
}

void resumeBleDisplayAdvertising()
{
  if (!bleAdvPausedForRadio) return;
  bleAdvPausedForRadio = false;
  if (bleAdvertisingStarted) {
    NimBLEDevice::startAdvertising();
  }
}
#else
void pauseBleDisplayAdvertising() {}
void resumeBleDisplayAdvertising() {}
#endif

void markBleCentralScanEnded()
{
  bleCentralScanActive = false;
  resumeBleDisplayAdvertising();
}
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
#if ENABLE_BLE_DISPLAY
  portENTER_CRITICAL(&stateMux);
  const uint8_t count = bleClientCount;
  portEXIT_CRITICAL(&stateMux);
  return count;
#else
  return 0;
#endif
}

void addBleHubClient(const NimBLEConnInfo &connInfo)
{
  const uint16_t handle = connInfo.getConnHandle();
  uint8_t slot = bleHubClientCount;
  for (uint8_t i = 0; i < bleHubClientCount; i++) {
    if (bleHubClients[i].handle == handle) {
      slot = i;
      break;
    }
  }
  if (slot >= kBleHubClientMax) {
    slot = 0;
  } else if (slot == bleHubClientCount) {
    bleHubClientCount++;
  }
  bleHubClients[slot].address = connInfo.getAddress().toString().c_str();
  bleHubClients[slot].address.toLowerCase();
  bleHubClients[slot].idAddress = connInfo.getIdAddress().toString().c_str();
  bleHubClients[slot].idAddress.toLowerCase();
  bleHubClients[slot].handle = handle;
  bleHubClients[slot].mtu = connInfo.getMTU();
  bleHubClients[slot].intervalMs10 = static_cast<uint16_t>(connInfo.getConnInterval() * 125 / 10);
  bleHubClients[slot].connectedMs = millis();
}

void removeBleHubClient(uint16_t handle)
{
  for (uint8_t i = 0; i < bleHubClientCount; i++) {
    if (bleHubClients[i].handle != handle) continue;
    for (uint8_t j = i + 1; j < bleHubClientCount; j++) {
      bleHubClients[j - 1] = bleHubClients[j];
    }
    bleHubClientCount--;
    return;
  }
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

#if ENABLE_WEB_GUI
String formatMacLower(const uint8_t mac[6])
{
  char buf[18];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

String apStationIpFromMac(const uint8_t mac[6])
{
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (!netif) return "";
  esp_netif_pair_mac_ip_t macIp = {};
  memcpy(macIp.mac, mac, 6);
  if (esp_netif_dhcps_get_clients_by_mac(netif, 1, &macIp) != ESP_OK) return "";
  return IPAddress(macIp.ip.addr).toString();
}

void refreshWifiApStations()
{
  wifi_sta_list_t list = {};
  if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) return;
  const uint32_t now = millis();
  bool seen[kWifiApStationMax] = {};

  for (int i = 0; i < list.num && i < kWifiApStationMax; i++) {
    const String mac = formatMacLower(list.sta[i].mac);
    uint8_t slot = wifiApStationCount;
    for (uint8_t j = 0; j < wifiApStationCount; j++) {
      if (wifiApStations[j].mac == mac) {
        slot = j;
        break;
      }
    }
    if (slot == wifiApStationCount && slot < kWifiApStationMax) {
      wifiApStations[slot].mac = mac;
      wifiApStations[slot].firstSeenMs = now;
      wifiApStationCount++;
    }
    if (slot >= kWifiApStationMax) continue;
    seen[slot] = true;
    wifiApStations[slot].rssi = list.sta[i].rssi;
    wifiApStations[slot].lastSeenMs = now;
    const String ip = apStationIpFromMac(list.sta[i].mac);
    if (ip.length() > 0) wifiApStations[slot].ip = ip;
  }

  for (int8_t i = static_cast<int8_t>(wifiApStationCount) - 1; i >= 0; i--) {
    if (seen[i]) continue;
    if (now - wifiApStations[i].lastSeenMs < 15000) continue;
    for (uint8_t j = i + 1; j < wifiApStationCount; j++) {
      wifiApStations[j - 1] = wifiApStations[j];
    }
    wifiApStationCount--;
  }
}

void recordWifiHttpPoller()
{
  refreshWifiApStations();
  WiFiClient client = server.client();
  if (!client || !client.connected()) return;
  const IPAddress ip = client.remoteIP();
  if (ip == IPAddress(0, 0, 0, 0)) return;

  String deviceId;
  if (server.hasHeader("X-Device")) {
    deviceId = server.header("X-Device");
  } else if (server.hasArg("client")) {
    deviceId = server.arg("client");
  }
  deviceId.trim();

  String userAgent;
  if (server.hasHeader("User-Agent")) userAgent = server.header("User-Agent");
  userAgent.trim();
  if (userAgent.length() > 96) userAgent = userAgent.substring(0, 96);

  String via = "lan";
  if (ip[0] == 192 && ip[1] == 168 && ip[2] == 4) via = "ap";

  const String ipText = ip.toString();
  const uint32_t now = millis();
  uint8_t slot = wifiHttpPollerCount;
  for (uint8_t i = 0; i < wifiHttpPollerCount; i++) {
    if (wifiHttpPollers[i].ip == ipText) {
      slot = i;
      break;
    }
  }
  if (slot == wifiHttpPollerCount) {
    if (slot >= kWifiHttpPollerMax) {
      uint8_t oldest = 0;
      uint32_t oldestMs = wifiHttpPollers[0].lastPollMs;
      for (uint8_t i = 1; i < wifiHttpPollerCount; i++) {
        if (wifiHttpPollers[i].lastPollMs < oldestMs) {
          oldestMs = wifiHttpPollers[i].lastPollMs;
          oldest = i;
        }
      }
      slot = oldest;
    } else {
      wifiHttpPollers[slot].ip = ipText;
      wifiHttpPollers[slot].firstPollMs = now;
      wifiHttpPollerCount++;
    }
  }

  wifiHttpPollers[slot].via = via;
  wifiHttpPollers[slot].lastPollMs = now;
  wifiHttpPollers[slot].pollCount++;
  if (deviceId.length() > 0) wifiHttpPollers[slot].deviceId = deviceId;
  if (userAgent.length() > 0) wifiHttpPollers[slot].userAgent = userAgent;

  if (wifiHttpPollers[slot].pollCount == 1) {
    char detail[80];
    snprintf(detail, sizeof(detail), "poll|%s|%s", ipText.c_str(),
             deviceId.length() ? deviceId.c_str() : "-");
    logHubEvent("wifi_http", detail);
  }

  if (via == "ap") {
    for (uint8_t i = 0; i < wifiApStationCount; i++) {
      if (wifiApStations[i].ip == ipText) {
        wifiHttpPollers[slot].mac = wifiApStations[i].mac;
        break;
      }
    }
  }
}
#endif

#include "hub_status_json.h"

bool hubSoftApModeActive()
{
#if ENABLE_WEB_GUI
  const wifi_mode_t mode = WiFi.getMode();
  return mode == WIFI_AP || mode == WIFI_AP_STA;
#else
  return false;
#endif
}

#if ENABLE_WEB_GUI
bool webOtaRejectBusy()
{
  if (!otaBusy) return false;
  server.sendHeader("Connection", "close");
  server.send(503, "text/plain", "OTA busy");
  return true;
}

// [OTA-GUARD] Jede Hub-Firmware traegt diesen Marker im Flash-Image (die Konstante
// selbst). Der Upload-Stream wird danach durchsucht: fehlt er, ist es keine Hub-FW
// (Display/Emu/Fremd-Image) und Update.end() wird verweigert -- Schutz gegen den
// "falsche Firmware auf falsches Geraet"-Fall. Override: /update?force=1.
// Achtung: aeltere Hub-Staende (vor diesem Commit) tragen den Marker nicht --
// Downgrade per OTA braucht dann force=1 (oder Kabel).
const char kHubFwMarker[] = "SPARTAN3-HUBFW-MARK1";
bool   otaMarkerFound = false;
bool   otaMarkerForce = false;
uint8_t otaMarkerTail[24];
size_t otaMarkerTailLen = 0;

void otaScanChunkForMarker(const uint8_t *data, size_t len)
{
  if (otaMarkerFound || len == 0) return;
  const size_t ml = sizeof(kHubFwMarker) - 1;
  if (otaMarkerTailLen > 0) {   // Chunk-Grenze: Tail des vorigen + Anfang des neuen
    uint8_t joint[48];
    const size_t take = (len < ml - 1) ? len : ml - 1;
    memcpy(joint, otaMarkerTail, otaMarkerTailLen);
    memcpy(joint + otaMarkerTailLen, data, take);
    const size_t jl = otaMarkerTailLen + take;
    for (size_t i = 0; i + ml <= jl; i++) {
      if (memcmp(joint + i, kHubFwMarker, ml) == 0) { otaMarkerFound = true; return; }
    }
  }
  for (size_t i = 0; i + ml <= len; i++) {
    if (memcmp(data + i, kHubFwMarker, ml) == 0) { otaMarkerFound = true; return; }
  }
  const size_t keep = (len < ml - 1) ? len : ml - 1;
  memcpy(otaMarkerTail, data + len - keep, keep);
  otaMarkerTailLen = keep;
}

void webOtaBeginUpload(const char *filename)
{
  otaBusy = true;
  otaRxBytes = 0;
  otaMarkerFound = false;
  otaMarkerTailLen = 0;
  otaStartedMs = millis();
  WiFi.setSleep(false);
  server.client().setNoDelay(true);
  server.client().setTimeout(500);
  Serial.printf("OTA:         start %s heap=%u freeSketch=%u\n",
                filename ? filename : "?",
                ESP.getFreeHeap(),
                ESP.getFreeSketchSpace());
  if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
    Update.printError(Serial);
    otaBusy = false;
  }
}

void webOtaWriteChunk(uint8_t *data, size_t len)
{
  if (!otaBusy || !data || len == 0) return;
  otaRxBytes += len;
  otaScanChunkForMarker(data, len);   // [OTA-GUARD]
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
    Update.abort();
    otaBusy = false;
    return;
  }
  static uint32_t lastOtaLogMs = 0;
  if (millis() - lastOtaLogMs >= 2000) {
    lastOtaLogMs = millis();
    Serial.printf("OTA:         %u bytes\n", static_cast<unsigned>(Update.progress()));
  }
  yield();
  delay(1);
}

void webOtaFinishUpload(size_t totalSize)
{
  if (!otaBusy) return;
  if (!otaMarkerFound && !otaMarkerForce) {   // [OTA-GUARD]
    Update.abort();
    otaBusy = false;
    Serial.printf("OTA:         REJECT no_marker (%u bytes) - keine Hub-FW? force=1 als Override\n",
                  static_cast<unsigned>(totalSize));
    logHubEvent("ota_reject", "no_marker");
    return;
  }
  if (Update.end(true)) {
    Serial.printf("OTA:         success %u bytes\n", static_cast<unsigned>(totalSize));
  } else {
    Update.printError(Serial);
    otaBusy = false;
  }
}

void webOtaAbortUpload()
{
  Update.abort();
  otaBusy = false;
  Serial.println("OTA:         aborted");
}

String otaProgressJson()
{
  String json;
  json.reserve(128);
  json += "{\"active\":";
  json += otaBusy ? "true" : "false";
  json += ",\"rx\":";
  json += String(static_cast<unsigned long>(otaRxBytes));
  json += ",\"written\":";
  json += String(static_cast<unsigned long>(Update.progress()));
  json += ",\"total\":";
  json += String(static_cast<unsigned long>(Update.size()));
  json += ",\"age_ms\":";
  json += String(otaStartedMs == 0 ? 0UL : static_cast<unsigned long>(millis() - otaStartedMs));
  json += "}";
  return json;
}

String humanBytes(size_t bytes)
{
  if (bytes < 1024) return String(bytes) + " B";
  if (bytes < 1024UL * 1024UL) return String(bytes / 1024.0f, 1) + " KB";
  return String(bytes / (1024.0f * 1024.0f), 2) + " MB";
}

size_t logFileSize(const char *path)
{
  if (strcmp(path, kLogFile) == 0) return logCurrentBytes;
  if (strcmp(path, kOldLogFile) == 0) return logOldBytes;
  return 0;
}

size_t readLogFileSizeOnce(const char *path)
{
  if (!logFsReady) return 0;
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return 0;
  const size_t size = f.size();
  f.close();
  return size;
}

void refreshLogSizeCache()
{
  logCurrentBytes = readLogFileSizeOnce(kLogFile);
  logOldBytes = readLogFileSizeOnce(kOldLogFile);
}

bool logCol(uint16_t flag)
{
  return (logColumnMask & flag) != 0;
}

String logHeader()
{
  String header = "ms;epoch;time";
  if (logCol(kLogColSpartan)) header += ";source;lambda_valid;lambda;spartan_temp_c;spartan_status";
  if (logCol(kLogColTune)) header += ";rpm;advance;map;tune_volt;tune_temp;tune_amp;tune_mode;tune_offset";
  if (logCol(kLogColSpeed)) header += ";speed_kmh;speed_hz;speed_pulses;odo_km;trip_km";
  if (logCol(kLogColHeater)) header += ";heater_v";
  if (logCol(kLogColHours)) header += ";device_h;engine_h;sensor_h";
  return header;
}

void onNtpSyncNotification(struct timeval *tv)
{
  (void)tv;
  lastNtpSyncMs = millis();
  ntpSynced = true;
  ntpResyncRequested = false;
  Serial.println("Time:        NTP synchronized");
}

const char *timezoneLabel(uint8_t idx)
{
  if (idx >= kTimezoneCount) idx = kTimezoneDefault;
  return kTimezones[idx].label;
}

const char *timezonePosix(uint8_t idx)
{
  if (idx >= kTimezoneCount) idx = kTimezoneDefault;
  return kTimezones[idx].posix;
}

void applyTimezone()
{
  const char *tz = timezonePosix(timezoneIdx);
  setenv("TZ", tz, 1);
  tzset();
  configTzTime(tz, kNtpServerPrimary, kNtpServerSecondary, kNtpServerTertiary);
  esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  esp_sntp_set_sync_interval(kNtpResyncIntervalMs);
}

void startNtpIfNeeded()
{
  if (ntpStarted || WiFi.status() != WL_CONNECTED) return;
  applyTimezone();
  sntp_set_time_sync_notification_cb(onNtpSyncNotification);
  ntpStarted = true;
  Serial.printf("Time:        NTP started (TZ %s)\n", timezoneLabel(timezoneIdx));
}

void requestNtpResync()
{
  ntpResyncRequested = true;
  if (ntpStarted && esp_sntp_enabled()) {
    sntp_restart();
    Serial.println("Time:        NTP resync requested");
  }
}

void saveTimezone(uint8_t idx)
{
  if (idx >= kTimezoneCount) idx = kTimezoneDefault;
  if (idx == timezoneIdx) return;
  timezoneIdx = idx;
  ensurePreferences();
  networkPreferences.putUChar("tz_idx", timezoneIdx);
  applyTimezone();
  requestNtpResync();
  Serial.printf("Time:        timezone %s (%s)\n", timezoneLabel(timezoneIdx), timezonePosix(timezoneIdx));
}

void updateNtp(uint32_t now)
{
  if (WiFi.status() != WL_CONNECTED) return;
  startNtpIfNeeded();
  if (!ntpStarted) return;

  if (systemTimeValid()) {
    if (!ntpSynced) ntpSynced = true;
    if (lastNtpSyncMs == 0) lastNtpSyncMs = now;
    const bool resyncDue = lastNtpSyncMs != 0 && (now - lastNtpSyncMs) >= kNtpResyncIntervalMs;
    if ((ntpResyncRequested || resyncDue) && esp_sntp_enabled()) {
      sntp_restart();
      if (ntpResyncRequested) Serial.println("Time:        NTP resync in progress");
    }
  }
}

bool systemTimeValid()
{
  time_t now = time(nullptr);
  return now > 1700000000;
}

void logHubEvent(const char *type, const char *detail)
{
  if (type == nullptr) return;
  HubEvent ev = {};
  ev.ms = millis();
  if (systemTimeValid()) {
    ev.epoch = static_cast<uint32_t>(time(nullptr));
  }
  strncpy(ev.type, type, sizeof(ev.type) - 1);
  strncpy(ev.detail, detail ? detail : "", sizeof(ev.detail) - 1);
  portENTER_CRITICAL(&hubEventMux);
  hubEvents[hubEventHead] = ev;
  hubEventHead = static_cast<uint16_t>((hubEventHead + 1) % kHubEventMax);
  if (hubEventCount < kHubEventMax) {
    hubEventCount++;
  }
  portEXIT_CRITICAL(&hubEventMux);
}

String hubEventsJson(uint16_t limit)
{
  if (limit == 0 || limit > kHubEventMax) {
    limit = 100;
  }
  String json = "[";
  portENTER_CRITICAL(&hubEventMux);
  const uint16_t count = hubEventCount;
  const uint16_t start = count <= kHubEventMax ? static_cast<uint16_t>((hubEventHead + kHubEventMax - count) % kHubEventMax) : 0;
  uint16_t emitted = 0;
  for (uint16_t i = 0; i < count && emitted < limit; i++) {
    const uint16_t idx = static_cast<uint16_t>((start + count - 1 - i) % kHubEventMax);
    const HubEvent &ev = hubEvents[idx];
    if (emitted > 0) json += ",";
    json += "{\"ms\":";
    json += String(ev.ms);
    json += ",\"epoch\":";
    json += String(ev.epoch);
    json += ",\"type\":\"";
    json += jsonEscape(String(ev.type));
    json += "\",\"detail\":\"";
    json += jsonEscape(String(ev.detail));
    json += "\"}";
    emitted++;
  }
  portEXIT_CRITICAL(&hubEventMux);
  json += "]";
  return json;
}

void clearHubEvents()
{
  portENTER_CRITICAL(&hubEventMux);
  hubEventHead = 0;
  hubEventCount = 0;
  portEXIT_CRITICAL(&hubEventMux);
  logHubEvent("system", "events_cleared");
}

String csvTimeText(uint32_t ms)
{
  if (!systemTimeValid()) {
    return String("boot+") + String(static_cast<unsigned long>(ms)) + "ms";
  }
  time_t now = time(nullptr);
  struct tm info;
  localtime_r(&now, &info);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &info);
  return String(buf);
}

void setLogError(const char *message)
{
  logError = message ? message : "unknown";
  if (!logErrorReported) {
    logErrorReported = true;
    Serial.printf("Logs:        disabled: %s\n", logError.c_str());
    logHubEvent("log_error", logError.c_str());
  }
}

void clearLogError()
{
  logError = "";
  logErrorReported = false;
}

bool probeLogFilesystem()
{
  const char *probePath = "/.rwtest";
  File probe = SPIFFS.open(probePath, FILE_WRITE);
  if (!probe) return false;
  const size_t written = probe.print("ok");
  probe.close();
  SPIFFS.remove(probePath);
  return written == 2;
}

bool initializeLogFilesystem(bool forceFormat = false)
{
  clearLogError();
  SPIFFS.end();
  if (forceFormat && !SPIFFS.format()) {
    logFsReady = false;
    setLogError("format_failed");
    return false;
  }

  logFsReady = SPIFFS.begin(true);
  if (!logFsReady) {
    setLogError("mount_failed");
    return false;
  }

  const bool hadLogs = SPIFFS.exists(kLogFile) || SPIFFS.exists(kOldLogFile);
  if (!probeLogFilesystem()) {
    if (!forceFormat && !hadLogs) {
      Serial.println("Logs:        write probe failed, one-time format recovery");
      SPIFFS.end();
      if (SPIFFS.format() && SPIFFS.begin(false) && probeLogFilesystem()) {
        logFsReady = true;
        clearLogError();
        logHubEvent("log_fs", "format_recovered");
        return true;
      }
    }
    logFsReady = false;
    setLogError("write_probe_failed");
    return false;
  }

  logHubEvent("log_fs", forceFormat ? "formatted_ok" : "ready");
  return true;
}

void ensureLogHeader()
{
  if (!logFsReady) return;
  const String expectedHeader = logHeader();
  bool needsHeader = logCurrentBytes == 0;
  if (!needsHeader) {
    File existing = SPIFFS.open(kLogFile, FILE_READ);
    needsHeader = !existing || existing.size() == 0;
    if (existing && existing.size() > 0) {
      String firstLine = existing.readStringUntil('\n');
      firstLine.trim();
      if (firstLine != expectedHeader) {
        existing.close();
        SPIFFS.remove(kOldLogFile);
        SPIFFS.rename(kLogFile, kOldLogFile);
        needsHeader = true;
      }
    }
    if (existing) existing.close();
  }
  if (!needsHeader) return;
  File f = SPIFFS.open(kLogFile, FILE_WRITE);
  if (!f) {
    // Do NOT call SPIFFS.format() here — this is called from appendLiveCsv()
    // every 500 ms; a blocking format (3-10 s) mid-drive would trigger BLE
    // supervision timeout and TWAI FIFO overflow. Boot-time recovery is
    // handled in initializeLogFilesystem().
    logFsReady = false;
    logCurrentBytes = 0;
    logOldBytes = 0;
    setLogError("header_open_failed");
    return;
  }
  f.println(expectedHeader);
  logCurrentBytes = f.size();
  f.close();
}

void rotateLogIfNeeded()
{
  if (!logFsReady || logCurrentBytes == 0) return;
  File f = SPIFFS.open(kLogFile, FILE_READ);
  const size_t size = f ? f.size() : 0;
  if (f) f.close();
  if (size < kMaxLogBytes) return;
  SPIFFS.remove(kOldLogFile);
  SPIFFS.rename(kLogFile, kOldLogFile);
  logOldBytes = size;
  logCurrentBytes = 0;
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

// [VARIANTE-A] Fahrzeug aktiv = Motor laeuft (RPM ueber Schwelle) ODER faehrt (Reed).
// Waehrenddessen blockiert der Hub jede Heim-/S24-STA-Aktivitaet (kein Connect, kein
// Auto-Reconnect, kein Scan/Kanalwechsel) -> der Hub-AP bleibt fuer die Displays stabil.
bool vehicleActive()
{
#if ENABLE_BLE_HUB
  if (tuneSnapshot().rpm > ENGINE_RUNNING_RPM_THRESHOLD) return true;
#endif
#if SPEED_REED_PIN >= 0
  if (speedKmh > 0.5f) return true;
#endif
  return false;
}

void appendLiveCsv()
{
  if (!hubFeatLog || !logFsReady) return;
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
  if (!logFsReady) return;
  File f = SPIFFS.open(kLogFile, FILE_APPEND);
  if (!f) {
    logFsReady = false;
    logCurrentBytes = 0;
    logOldBytes = 0;
    setLogError("append_failed");
    return;
  }

  const time_t epoch = systemTimeValid() ? time(nullptr) : 0;
  f.printf("%lu;%lu;%s",
           static_cast<unsigned long>(now),
           static_cast<unsigned long>(epoch),
           csvTimeText(now).c_str());
  if (logCol(kLogColSpartan)) {
    f.printf(";%s;%u;%.3f;%u;%u",
             sourceTextC(spartan),
             spartan.valid ? 1 : 0,
             spartan.lambda,
             spartan.temperatureC,
             spartan.status);
  }
  if (logCol(kLogColTune)) {
    // [TUNE-SAFE] tune_mode/tune_offset mitschreiben: ohne sie ist ein Live-Eingriff
    // in die Zuendung im Log unsichtbar und Kurven-Vergleiche sind wertlos.
#if ENABLE_BLE_HUB
    const unsigned tMode = tuneModeActive ? 1u : 0u;
    const int tOffset = tuneAdvSteps;
#else
    const unsigned tMode = 0u;
    const int tOffset = 0;
#endif
    f.printf(";%.0f;%.1f;%.0f;%.1f;%.0f;%.2f;%u;%d",
             tune.rpm,
             tune.advance,
             tune.map,
             tune.voltage,
             tune.temperature,
             tune.coilCurrent,
             tMode,
             tOffset);
  }
  if (logCol(kLogColSpeed)) {
#if SPEED_REED_PIN >= 0
    f.printf(";%.1f;%.2f;%lu",
             speedKmh,
             speedHz,
             static_cast<unsigned long>(speedPulseCount));
#else
    f.print(";0.0;0.00;0");
#endif
    // [ODOMETER] Gesamt-/Teilstrecke mitloggen -- bisher nur im Status-JSON sichtbar,
    // Kurven-/Fahrtvergleiche ueber die CSV brauchen auch die km-Referenz.
    f.printf(";%.3f;%.3f",
             static_cast<double>(odoMm) / 1000000.0,
             static_cast<double>(tripMm) / 1000000.0);
  }
  if (logCol(kLogColHeater)) {
    f.printf(";%.2f", heaterStatusVolts);
  }
  if (logCol(kLogColHours)) {
    f.printf(";%.2f;%.2f;%.2f",
             static_cast<double>(deviceSeconds) / 3600.0,
             static_cast<double>(engineSeconds) / 3600.0,
             static_cast<double>(sensorSeconds) / 3600.0);
  }
  f.println();
  logCurrentBytes = f.size();
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

#include "hub_ble.h"

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

#if ENABLE_WEB_GUI
void printHubFeatStatus()
{
  Serial.printf("Hub feats:   AP=%s WLAN=%s LOG=%s 123=%s\n",
                hubFeatAp ? "on" : "off",
                hubFeatWifi ? "on" : "off",
                hubFeatLog ? "on" : "off",
                hubFeatBle123 ? "on" : "off");
}

void applyHubFeatures()
{
  if (!hubFeatAp) {
    WiFi.softAPdisconnect(true);
    if (hubFeatWifi || WiFi.status() == WL_CONNECTED) {
      WiFi.mode(WIFI_STA);
    } else {
      WiFi.mode(WIFI_OFF);
    }
  } else {
    ensureHubSoftAp();
  }
  if (!hubFeatWifi) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect(false, false);
    }
    homeWifiConnectStartedMs = 0;
    homeWifiDisabledForRoadAp = false;
  } else if (haveSavedWifi && WiFi.status() != WL_CONNECTED &&
             homeWifiConnectStartedMs == 0 && !homeWifiDisabledForRoadAp && hubWifiProfile > 0 &&
             !vehicleActive()) {   // [VARIANTE-A] fahrend keinen Reconnect anstossen
    const char* ssid = g_hubWifiProfiles[hubWifiProfile].ssid;
    const char* pass = g_hubWifiProfiles[hubWifiProfile].pass;
    if (strlen(ssid) > 0) {
      WiFi.begin(ssid, pass);
      homeWifiConnectStartedMs = millis();
      Serial.printf("Home WiFi:   reconnecting to '%s' (Profil %d)\n", ssid, hubWifiProfile);
    }
  }
#if ENABLE_BLE_HUB
  if (!hubFeatBle123 && tuneConnected) {
    resetTuneClient();
  } else if (hubFeatBle123 && !tuneConnected && !tuneDoConnect && tuneNextScanMs == 0) {
    scheduleTuneScan(true);
  }
#endif
}

bool handleHubFeatSerialLine(const String &line)
{
  if (line.startsWith("lambda test ")) {
    const String mode = line.substring(12);
    if (!setLambdaTestMode(mode)) {
      Serial.println("Lambda test: use off, fixed or sweep");
    }
    return true;
  }
  // [WIFI-MAC-OVR] Recovery ohne WLAN: falls ein Override die STA-Verbindung
  // zerschiesst, ist der Hub nur noch per USB/Serial erreichbar.
  if (line.equalsIgnoreCase("hub wifi mac clear")) {
    ensurePreferences();
    networkPreferences.putString("mac_ovr", "");
    Serial.println("WiFi MAC:    Override geloescht -> Neustart");
    delay(300);
    ESP.restart();
    return true;
  }
  if (line.equalsIgnoreCase("hub logfs reset")) {
    logCurrentBytes = 0;
    logOldBytes = 0;
    if (initializeLogFilesystem(true)) {
      ensureLogHeader();
      refreshLogSizeCache();
      Serial.println("Logs:        filesystem reset OK");
    }
    return true;
  }
  if (!line.startsWith("hub feat ")) {
    return false;
  }
  String rest = line.substring(9);
  rest.trim();
  if (rest.length() == 0) {
    return true;
  }
  if (rest.equalsIgnoreCase("status")) {
    printHubFeatStatus();
    return true;
  }

  const int sp = rest.indexOf(' ');
  if (sp < 0) {
    Serial.println("Hub feats:   usage: hub feat <name> on|off | status");
    return true;
  }
  String feat = rest.substring(0, sp);
  String arg = rest.substring(sp + 1);
  arg.trim();
  feat.toLowerCase();

  bool changed = false;
  if (feat == "ap") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatAp = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatAp = false;
      changed = true;
    }
  } else if (feat == "wifi") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatWifi = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatWifi = false;
      changed = true;
    }
  } else if (feat == "log") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatLog = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatLog = false;
      changed = true;
    }
  } else if (feat == "ble123") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatBle123 = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatBle123 = false;
      changed = true;
    }
  } else {
    Serial.println("Hub feats:   unknown feature (ap/wifi/log/ble123)");
    return true;
  }

  if (changed) {
    saveHubFeatures();
    applyHubFeatures();
    logHubEvent("hub_feat", "serial");
    Serial.println("Hub feats:   saved");
    printHubFeatStatus();
  } else {
    Serial.println("Hub feats:   use on or off");
  }
  return true;
}
#endif

#include "hub_webgui_page.h"

#include "hub_webgui_endpoints.h"

void updateWebGui()
{
#if ENABLE_WEB_GUI
  const uint32_t now = millis();
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  const wl_status_t wifiStatus = WiFi.status();
  // [VARIANTE-A] Motor-Gate: faehrt der Wagen, schaltet der Hub den IDF-Auto-Reconnect
  // AUS -> faellt die Heim-STA fahrend weg, sucht der Funk NICHT (kein AP/Touch-Blip).
  // Eine bestehende Verbindung bleibt. Im Stand wieder an + ggf. einmal verbinden.
  // Drossel: ~1x/s (vehicleActive() liest tuneSnapshot, nicht jeden Loop noetig).
  static uint32_t lastVarACheckMs = 0;
  static bool lastDriving = false;
  static bool varAInit = false;
  if (now - lastVarACheckMs >= 1000) {
    lastVarACheckMs = now;
    const bool driving = vehicleActive();
    if (driving != lastDriving || !varAInit) {
      varAInit = true;
      lastDriving = driving;
      WiFi.setAutoReconnect(!driving);
      if (driving) {
        Serial.println("Variante A:  Motor laeuft/faehrt -> Heim-WLAN pausiert (AP/Touch stabil)");
      } else {
        Serial.println("Variante A:  Stand -> Heim-WLAN wieder erlaubt");
        if (hubFeatWifi && haveSavedWifi && hubWifiProfile > 0 &&
            wifiStatus != WL_CONNECTED && homeWifiConnectStartedMs == 0) {
          homeWifiDisabledForRoadAp = false;   // im Stand frischen Versuch zulassen
          connectHomeWifiAligned(g_hubWifiProfiles[hubWifiProfile].ssid,
                                 g_hubWifiProfiles[hubWifiProfile].pass);
        }
      }
    }
  }
  // [W25Q-CFG] geaenderte Config zeitnah auf den Backup-Chip schreiben (rar, user-getriggert,
  // ~50ms Erase) -> WLAN-Daten + Setup ueberleben jeden Firmware-Flash.
  if (hubCfgDirty && w25qDetected) {
    hubCfgDirty = false;
    saveConfigToW25Q();
  }
  if (!hubFeatAp && hubSoftApModeActive()) {
    WiFi.softAPdisconnect(true);
    if (hubFeatWifi || wifiStatus == WL_CONNECTED) {
      WiFi.mode(WIFI_STA);
    } else {
      WiFi.mode(WIFI_OFF);
    }
    Serial.println("Web GUI:     SoftAP forced off by Setup");
  }
  if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0) && hubFeatAp && !apSuspendedForConnect &&
      now - lastApRetryMs >= 10000) {
    lastApRetryMs = now;
    ensureHubSoftAp();
    if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0)) {
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
      // [WLAN-KANAL] Single-Radio: der SoftAP folgt physisch dem STA-Kanal. hubApChannel
      // (Report + NVS + naechster softAP-Start) hier IMMER auf den echten Kanal ziehen --
      // auch beim Boot-Auto-Reconnect, wo apSuspendedForConnect nie gesetzt war (sonst
      // meldet der Dev-Tab einen veralteten AP-Kanal). Nur Variable/NVS, KEIN softAP-
      // Neustart ausserhalb des Suspend-Pfads -> kein AP-Blip waehrend der Fahrt.
      uint8_t ch = (uint8_t)WiFi.channel();
      if (ch >= 1 && ch <= 13 && ch != hubApChannel) {
        hubApChannel = ch;
        networkPreferences.putUChar("ap_chan", hubApChannel);
      }
      // AP war fuer einen manuellen Connect ausgesetzt -> jetzt auf ch zurueckbringen.
      // Aus dem loop-Kontext, damit NVS-Write/softAP nicht im WiFi-Event-Task laufen.
      if (apSuspendedForConnect) {
        apSuspendedForConnect = false;
        ensureHubSoftAp();   // AP zurueck (WIFI_AP_STA, softAP auf hubApChannel)
        startHubMdns();
        Serial.printf("WLAN-Kanal:  verbunden auf Kanal %d, AP wieder aktiv\n", hubApChannel);
      }
      Serial.printf("Home WiFi:   connected to '%s', http://%s/\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      startNtpIfNeeded();
    }
  }
  updateNtp(now);
  // [WLAN-KANAL] Bei ausgesetztem AP kuerzeres Fenster (20s), damit der AP nicht
  // 45s wegbleibt, wenn der Connect scheitert (z.B. falsches PW / nicht erreichbar).
  const uint32_t connWindow = apSuspendedForConnect ? 20000UL : kHomeWifiConnectWindowMs;
  if (hubFeatWifi && haveSavedWifi && !homeWifiDisabledForRoadAp && wifiStatus != WL_CONNECTED &&
      homeWifiConnectStartedMs != 0 && now - homeWifiConnectStartedMs >= connWindow) {
    WiFi.disconnect(false, false);
    homeWifiDisabledForRoadAp = true;
    apSuspendedForConnect = false;   // AP wird gleich wieder hochgefahren
    if (hubFeatAp) {
      WiFi.mode(WIFI_AP);
      WiFi.setSleep(true);
      IPAddress apIp, apMask;
      parseApAddress(hubApIp, "192.168.4.1", apIp);
      parseApAddress(hubApMask, "255.255.255.0", apMask);
      WiFi.softAPConfig(apIp, apIp, apMask);
      const char *roadSsid = hubApSsid.c_str();
      WiFi.softAP(roadSsid, hubApPassword.c_str(), hubApChannel, 0, 4);
      Serial.printf("Home WiFi:   unavailable, road AP only '%s' http://%s/\n",
                    roadSsid,
                    WiFi.softAPIP().toString().c_str());
    } else {
      WiFi.mode(WIFI_STA);
      Serial.println("Home WiFi:   unavailable, SoftAP remains disabled by Setup");
    }
  }
  dns.processNextRequest();
  server.handleClient();
#endif
}

#include "hub_can.h"

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

void updateLambdaTest()
{
  if (lambdaTestMode == LambdaTestMode::Off) return;
  const uint32_t now = millis();
  if (now - lastLambdaTestMs < 100) return;
  lastLambdaTestMs = now;

  SpartanReading fresh;
  fresh.lambda = 1.000f;
  if (lambdaTestMode == LambdaTestMode::Sweep) {
    const uint32_t halfCycleMs = 10000;
    const uint32_t phaseMs = now % (halfCycleMs * 2UL);
    const float phase = static_cast<float>(phaseMs <= halfCycleMs
        ? phaseMs
        : (halfCycleMs * 2UL - phaseMs)) / static_cast<float>(halfCycleMs);
    fresh.lambda = 0.85f + phase * 0.30f;
  }
  fresh.temperatureC = 780;
  fresh.status = 3;
  fresh.receivedMs = now;
  fresh.valid = true;
  fresh.fromCan = false;
  fresh.fromDemo = false;
  fresh.fromTest = true;
  storeReading(fresh);
}

void updateAnalog()
{
#if ENABLE_SPARTAN_ANALOG
  const SpartanReading snapshot = readingSnapshot();
  if ((snapshot.fromCan || snapshot.fromDemo || snapshot.fromTest) &&
      millis() - snapshot.receivedMs <= kCanStaleMs) {
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
  odoMm = networkPreferences.getULong64("odo_mm", 0);    // [ODOMETER]
  tripMm = networkPreferences.getULong64("trip_mm", 0);
  odoLastSavedMm = odoMm;
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

  // [ODOMETER] Strecke direkt aus den Pulsen (exakt, kein Integrationsfehler).
  if (pulses > 0) {
    const uint64_t addMm = static_cast<uint64_t>(pulses) * tireCircMm * speedTrimPermil
                           / (1000ULL * PULSES_PER_REV);
    odoMm += addMm;
    tripMm += addMm;
  }
  // NVS-schonend sichern: alle 60 s, und nur wenn wirklich gefahren wurde.
#if ENABLE_WEB_GUI
  if (now - lastOdoSaveMs >= 60000 && odoMm != odoLastSavedMm) {
    lastOdoSaveMs = now;
    odoLastSavedMm = odoMm;
    ensurePreferences();
    networkPreferences.putULong64("odo_mm", odoMm);
    networkPreferences.putULong64("trip_mm", tripMm);
  }
#endif
}
#else
void setupSpeedReed() {}
void updateSpeedReed() {}
#endif

void setupUart()
{
#if ENABLE_SPARTAN_UART
  pinMode(SPARTAN_UART_RX_PIN, INPUT_PULLUP);
  Serial2.begin(9600, SERIAL_8N1, SPARTAN_UART_RX_PIN, SPARTAN_UART_TX_PIN);
  Serial.printf(
      "UART:        9600 8N1 RX=%u TX=%u (configuration only; level shift Spartan TX)\n",
      SPARTAN_UART_RX_PIN,
      SPARTAN_UART_TX_PIN);
  Serial.println("UART bridge: type >GETFW or >GETCANID in USB serial monitor");
  Serial.println("123TUNE BLE: type !read for v@/10@/11@/12@/13@ dump or !v@ for one command");
#endif
}

void setupBridgeUart()
{
  ensurePreferences();
  bridgeUartRxPin = networkPreferences.getUChar("uart_rx", 0);
  bridgeUartTxPin = networkPreferences.getUChar("uart_tx", 0);
  if (bridgeUartRxPin > 0) {
    Serial1.begin(115200, SERIAL_8N1, bridgeUartRxPin, bridgeUartTxPin > 0 ? bridgeUartTxPin : -1);
    Serial.printf("Bridge UART: RX=GPIO%d TX=GPIO%d\n", bridgeUartRxPin, bridgeUartTxPin);
  } else {
    Serial.println("Bridge UART: aus (kein RX-Pin konfiguriert)");
  }
}

// Parst eine Bridge-Zeile "T,rpm,adv,map,coil,volt,temp" und schreibt direkt
// in die tune-Globals — exakt wie onTuneNotify, aber per UART statt BLE.
static void processBridgeLine(const String &line)
{
  if (line.length() < 3) return;
  // Format 2: Touch-S3 / Waveshare "BLE: LIVE rpm=X adv=Y map=Z rx=N"
  if (line.startsWith("BLE: LIVE ")) {
    float rpm = 0, adv = 0, map = 0;
    int r = sscanf(line.c_str(), "BLE: LIVE rpm=%f adv=%f map=%f", &rpm, &adv, &map);
    if (r >= 2) {
      portENTER_CRITICAL(&stateMux);
      tuneRpm = rpm; tuneAdvance = adv; tuneMap = map;
      if (tuneRxCount < UINT32_MAX) tuneRxCount++;
      portEXIT_CRITICAL(&stateMux);
      tuneLastRxMs = millis();
      tuneConnected = true;
    }
    return;
  }
  if (line[0] == 'T' && line[1] == ',') {
    // T,rpm,adv,map[,coil,volt,temp]
    int idx = 2, field = 0;
    float vals[6] = {0};
    while (idx < (int)line.length() && field < 6) {
      int comma = line.indexOf(',', idx);
      String tok = (comma < 0) ? line.substring(idx) : line.substring(idx, comma);
      vals[field++] = tok.toFloat();
      if (comma < 0) break;
      idx = comma + 1;
    }
    portENTER_CRITICAL(&stateMux);
    tuneRpm         = vals[0];
    tuneAdvance     = vals[1];
    tuneMap         = vals[2];
    tuneCoilCurrent = (field > 3) ? vals[3] : tuneCoilCurrent;
    tuneVoltage     = (field > 4) ? vals[4] : tuneVoltage;
    tuneTemperature = (field > 5) ? vals[5] : tuneTemperature;
    if (tuneRxCount < UINT32_MAX) tuneRxCount++;
    portEXIT_CRITICAL(&stateMux);
    tuneLastRxMs = millis();
    tuneConnected = true;  // Bridge liefert — gilt als "verbunden"
  }
}

static String bridgeUartBuf;

void updateBridgeUart()
{
  if (bridgeUartRxPin == 0) return;  // nicht konfiguriert
  while (Serial1.available()) {
    const char c = static_cast<char>(Serial1.read());
    if (c == '\n') {
      bridgeUartBuf.trim();
      if (bridgeUartBuf.length() > 0) processBridgeLine(bridgeUartBuf);
      bridgeUartBuf = "";
    } else if (c != '\r' && bridgeUartBuf.length() < 64) {
      bridgeUartBuf += c;
    }
  }
}

void updateUart()
{
#if ENABLE_SPARTAN_UART
  if (lastUartCommandMs != 0 && lastUartResponseMs == 0 && millis() - lastUartCommandMs > 1800) {
    lastUartState = "Timeout - keine Antwort";
  }
  while (Serial2.available()) {
    const char c = static_cast<char>(Serial2.read());
    if (lastUartCommandMs == 0) {
      continue;
    }
    if (c == '\r' || c == '\n') {
      lastUartResponse.trim();
      if (lastUartResponse.length() > 0) {
        lastUartState = "Antwort empfangen";
        lastUartResponseMs = millis();
        Serial.printf("UART RX:     %s\n", lastUartResponse.c_str());
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
#if ENABLE_WEB_GUI
      if (handleHubFeatSerialLine(uartLine)) {
      } else
#endif
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
#if ENABLE_BLE_DISPLAY
  Serial.println("BLE display: enabled as GATT peripheral");
#else
  Serial.println("BLE display: disabled (123TUNE central only)");
#endif
#else
  Serial.println("BLE stack:   disabled");
#endif
}

}  // namespace

#include "hub_w25q.h"

void setup()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
#ifdef RGB_BUILTIN
  // Onboard-WS2812 (ESP32-S3-DevKit, GPIO48) explizit AUS — sonst leuchtet sie
  // uninitialisiert zufaellig weiss. Nur auf Boards mit RGB_BUILTIN (classic ESP32
  // hat das nicht -> #ifdef schuetzt den Bus-Build).
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
#endif

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // [UART0-MIRROR] CDC_ON_BOOT=1 lenkt Serial auf den nativen USB-CDC, der auf den
  // Boards nicht rausgefuehrt ist -> ueber den CH343 (COM14/COM24) sieht man nichts.
  // UART0 haengt am CH343 -> 123-BLE-Logs zusaetzlich dorthin spiegeln = Live-Konsole.
  Serial0.begin(115200);
  Serial0.println("\nUART0-Mirror: 123-BLE-Logs aktiv (CH343-Konsole)");
#endif
  delay(500);

  detectW25Q();   // externen W25Q128-Flash erkennen (Check, noch ohne Dateisystem)

#if BLE_BRIDGE
  // BLE-Bridge: NUR 123\TUNE+, dediziert, kein WLAN, kein Hintergrund-Scan.
  // Architektur: feste MAC aus NVS (oder Compile-Default) -> direkt verbinden -> bei
  // Trennung direkt reconnect ohne erneuten Scan. Einmaliger Scan nur wenn keine MAC
  // gespeichert (erstes Setup). Danach: Verbindung halten = einzige Aufgabe.
  Serial.println("\n=== BLE-Bridge: NUR 123\\TUNE+, dediziert ===");
  hubFeatBle123 = true;
  hubFeatAp     = false;
  hubFeatWifi   = false;
  NimBLEDevice::init("BLE-Bridge-123");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(23);
  // MAC aus NVS laden (vom letzten Setup-Scan), sonst Compile-Default
  ensurePreferences();
  if (networkPreferences.isKey("tune_mac")) {
    tuneSavedAddress = networkPreferences.getString("tune_mac", kTuneTargetAddress);
  } else {
    tuneSavedAddress = kTuneTargetAddress;  // ef:a8:b2:de:e0:9e (Default-Emu/echte 123)
    Serial.printf("Bridge:      kein MAC gespeichert, nutze Default %s\n", kTuneTargetAddress);
  }
  tuneSavedAddress.toLowerCase();
  Serial.printf("Bridge:      Ziel-MAC = %s\n", tuneSavedAddress.c_str());
  // UART zum S3: TX=GPIO4, RX=GPIO5 (C6: freie Pins, kein Strapping/DAC)
  Serial1.begin(115200, SERIAL_8N1, 5, 4);  // RX=GPIO5, TX=GPIO4
  // MAC als Zieladresse setzen — direkt verbinden ohne Scan.
  // scheduleTuneScan(true) plant einen schnellen Connect-Versuch (500ms), der über
  // connectTune() direkt auf tuneTargetAddress geht (kein Scan da addrMatch=1).
  tuneTargetAddress = NimBLEAddress(std::string(tuneSavedAddress.c_str()), BLE_ADDR_RANDOM);
  scheduleTuneScan(true);  // startet schnellen Connect (500ms Delay)
  Serial.printf("Bridge:      Verbinde direkt auf %s (kein Hintergrund-Scan)\n",
                tuneSavedAddress.c_str());
  return;
#endif

  setupDisplay();
  printBootDetails();
  loadHubFeatures();
  setupBleHub();
  setupWebGui();
#if defined(ENABLE_BLE_HUB)
  // Hub/Client: BALANCE — BLE-Empfang stabil, WiFi-AP erreichbar.
  // PREFER_BT hungerte hier das AP aus (Ping-Timeouts, keine WebGUI).
  if (esp_coex_preference_set(ESP_COEX_PREFER_BALANCE) == ESP_OK) {
    Serial.println("Coex:        BLE/WiFi Balance gesetzt (PREFER_BALANCE)");
  }
#endif
  setupHourmeters();
  setupCan();
  setupUart();
  setupSpeedReed();
  setupBridgeUart();  // BLE-Bridge UART: AZ-ESP32 GPIO17->S3-GPIO4
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
#if BLE_BRIDGE
  // Bridge-Loop: NUR 123-State-Machine, kein Hintergrund-Scan.
  // Nach Trennung: direkt reconnect auf gespeicherte MAC (kein neuer Scan).
  updateTuneBle();
  {
    static uint32_t lastBridgeMs = 0;
    const uint32_t nowB = millis();
    if (nowB - lastBridgeMs >= 500) {
      lastBridgeMs = nowB;
      const TuneSnapshot t = tuneSnapshot();
      const uint32_t tuneAge = t.lastRxMs ? (nowB - t.lastRxMs) : 999999;
      // Konsole (Mensch):
      Serial.printf("BRIDGE | 123:%-3s rpm=%4d adv=%4.1f map=%3d age=%lums\n",
                    tuneConnected ? "ON " : "off", (int)t.rpm, t.advance, (int)t.map,
                    (unsigned long)tuneAge);
      // UART2 (GPIO25 TX) -> S3 GPIO4 RX: immer senden fuer Loopback-Test
      Serial1.printf("T,%d,%.1f,%d,%.1f,%.1f,%d\n",
                     (int)t.rpm, t.advance, (int)t.map,
                     t.coilCurrent, t.voltage, (int)t.temperature);
      Serial1.flush();
      Serial.printf("UART-TX GPIO4: T,%d,%.1f,%d\n", (int)t.rpm, t.advance, (int)t.map);
    }
  }
  return;
#endif
  updateCan();
  updateDemo();
  updateLambdaTest();
  updateAnalog();
  updateHeaterAnalog();
  updateUart();
  updateBridgeUart();  // 123-Werte von AZ-ESP32 BLE-Bridge per UART
  updateSpeedReed();
  updateHourmeters();
  appendLiveCsv();
  updateWebGui();
  updateBleHub();
  updateDisplay();
  delay(10);
}
