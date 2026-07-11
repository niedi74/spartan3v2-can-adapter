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

#ifndef ENABLE_ESP_NOW_HUB
#define ENABLE_ESP_NOW_HUB 0
#endif

#if ENABLE_ESP_NOW_HUB
#include <esp_now.h>
#include "spartan_cockpit_frame.h"
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
#if defined(ENABLE_BLE_HUB) && defined(ENABLE_ESP_NOW_HUB)
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

// 123-Emulator: per Build entscheiden. Emu-Build (emu_com17) = 1 (Gerät spielt die
// 123 als BLE-Peripheral nach). Test-Hub/Client (COM14) und Live-Hub = 0 (reiner
// Client, kein Emu). So bleibt eine Codebasis für beide Rollen.
#ifndef ENABLE_EMU123
#define ENABLE_EMU123 0
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
// ESP-NOW-Cockpit-Broadcast: deutlich schneller als der BLE-Status-Notify,
// damit RPM/Advance auf M5/Touch flüssig wirken (nicht "slow motion").
// 40 ms = 25 Hz. Frames sind winzig (17 B), das ist für ESP-NOW unkritisch.
constexpr uint32_t kEspNowSendIntervalMs = 40;
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
// Emu (WLAN+BLE-Koexistenz): 12s für Schub-Lücken-Pufferung
// Echte 123TUNE+ (nRF52810): kontinuierlich 8-12 Hz, 6s = 75× Sicherheitsmarge
#if ENABLE_EMU123
constexpr uint32_t kTuneStaleRxMs = 12000;
#else
constexpr uint32_t kTuneStaleRxMs = 6000;
#endif
constexpr uint32_t kTuneReconnectBackoffMaxMs = 30000;

enum class TuneLinkState : uint8_t {
  Idle = 0,
  Scanning,
  Connecting,
  Subscribed,
  Streaming,
};

#ifndef ENABLE_BM6
#define ENABLE_BM6 1
#endif

constexpr char kBm6TargetAddress[] = "3c:ab:72:7f:d0:bc";
constexpr char kBm6ServiceUuid[] = "0000fff0-0000-1000-8000-00805f9b34fb";
constexpr char kBm6WriteUuid[] = "0000fff3-0000-1000-8000-00805f9b34fb";
constexpr char kBm6NotifyUuid[] = "0000fff4-0000-1000-8000-00805f9b34fb";
constexpr uint32_t kBm6ScanWindowMs = 1500;   // 1.5s: BM6 findet sich, 123-Link ueberlebt den Scan
constexpr uint32_t kBm6ReconnectDelayMs = 8000;
constexpr uint32_t kBm6TriggerIntervalMs = 2000;  // interner Polling-Takt (verbunden)
uint32_t bm6PollIntervalMs = 60000;               // Abfrageintervall: Standard 60s, per /bm6_interval
constexpr uint32_t kBm6MainSlotMs = 45000;   // Batterie (main)
constexpr uint32_t kBm6AuxSlotMs = 15000;    // Zusatzbatterie poll window
constexpr uint32_t kBm6CacheMaxAgeMs = 120000;
const uint8_t kBm6AesKey[16] = {
    0x6C, 0x65, 0x61, 0x67, 0x65, 0x6E, 0x64, 0xFF,
    0xFE, 0x30, 0x31, 0x30, 0x30, 0x30, 0x30, 0x39};
const uint8_t kBm6TriggerPlain[16] = {
    0xD1, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
#if ENABLE_ESP_NOW_HUB
#ifndef ESP_NOW_WIFI_CHANNEL
#define ESP_NOW_WIFI_CHANNEL 6
#endif
constexpr uint8_t kEspNowBroadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
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
uint32_t tuneScanSeen = 0;
uint32_t tuneScanCandidates = 0;
String tuneSavedAddress = "";
uint32_t tuneConnStartMs = 0;  // Start der aktuellen 123-Verbindung (für Stale-Bezug)
struct BleScanDevice {
  String address;
  String name;
  int rssi = 0;
  bool tuneLike = false;
  bool bm6Like = false;
  uint32_t seenMs = 0;
};
constexpr uint8_t kBleScanDeviceMax = 12;
BleScanDevice bleScanDevices[kBleScanDeviceMax];
uint8_t bleScanDeviceCount = 0;
#if ENABLE_BM6
NimBLEClient *bm6Client = nullptr;
NimBLERemoteCharacteristic *bm6WriteChar = nullptr;
NimBLEAddress bm6TargetAddress;
String bm6SavedAddress = "";
String bm6AuxSavedAddress = "";
uint8_t bm6ActiveSlot = 0;  // 0=main (Batterie), 1=aux (Zusatzbatterie)
uint32_t bm6SlotStartedMs = 0;
bool bm6DoConnect = false;
bool bm6Connected = false;
uint32_t bm6NextScanMs = 0;
uint32_t bm6LastTriggerMs = 0;
uint32_t bm6LastRxMs = 0;
uint32_t bm6RxCount = 0;
uint32_t bm6DecodeFailCount = 0;
float bm6Voltage = 0.0f;
int8_t bm6Temperature = 0;
uint32_t bm6AuxLastRxMs = 0;
uint32_t bm6AuxRxCount = 0;
float bm6AuxVoltage = 0.0f;
int8_t bm6AuxTemperature = 0;
#endif
#endif
#if ENABLE_ESP_NOW_HUB
volatile uint32_t espNowTxCount = 0;
volatile uint32_t espNowTxFailCount = 0;
uint16_t espNowSeq = 0;
bool espNowReady = false;
uint8_t espNowActiveChannel = 0;
uint32_t lastEspNowSendMs = 0;
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
bool hubFeatEspNow = true;
bool hubFeatAp = true;
bool hubFeatWifi = true;
bool hubFeatLog = true;
bool hubFeatBle123 = false;
bool hubFeatBleBm6 = false;
bool hubFeatEmu123 = false;  // Test: Hub spielt die 123 nach (BLE-Peripheral)
int   emu123ManualRpm = -1;  // -1 = Sweep, sonst feste RPM (per WebGUI)
int   emu123CurRpm = 0;      // zuletzt gesendete Emu-Werte (fuer WebGUI/JSON)
float emu123CurAdv = 0.0f;
int   emu123CurMap = 0;
String emu123Addr = "";      // gewuenschte advertised BLE-Adresse (leer=Chip-Default)
uint8_t hubEspNowChannelPref = 0;  // 0=auto/follow STA, otherwise fixed 1..14
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
// Setzt bei DHCP/ungueltiger Static-Config explizit auf DHCP zurueck -- sonst haengt eine
// vorherige statische IP (von einem frueheren Profilwechsel in derselben Boot-Session) am
// WiFi-Treiber weiter, auch wenn das neue Zielprofil eigentlich DHCP will.
bool applyStaticIpIfNeeded(uint8_t profileIdx)
{
  if (profileIdx >= 1 && profileIdx <= 2) {
    const HubWifiProfile &p = g_hubWifiProfiles[profileIdx];
    if (p.ipMode == 1) {
      IPAddress ip, gw, mask;
      if (ip.fromString(p.ip) && gw.fromString(p.gw) && mask.fromString(p.mask)) {
        WiFi.config(ip, gw, mask);
        Serial.printf("WiFi Static: Profil %d -> %s (GW %s, Mask %s)\n", profileIdx, p.ip, p.gw, p.mask);
        return true;
      }
      Serial.printf("WiFi Static: Profil %d ungueltige IP/GW/Mask - falle auf DHCP zurueck\n", profileIdx);
    }
  }
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);   // DHCP-Reset
  return false;
}

// [WIFI-MAC-OVR] Manuelle STA-MAC statt Werks-eFuse (Verdacht auf nicht-eindeutige
// Klon-MAC). Leer = Werks-MAC. Vorsicht: eine Test-MAC kann die Router-Assoziation
// komplett zerschiessen (beobachtet auf dem Test-Hub) -- Recovery dann nur per
// Serial: "hub wifi mac clear".
char g_wifiMacOverride[18] = "";

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
    Serial.println("WiFi MAC:    esp_wifi_set_mac fehlgeschlagen");
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
const size_t kMaxLogBytes = 200000;  // 200 KB: kleine Dateien halten SPIFFS schnell (grosse Logs blockieren loop() -> Webserver/Display-Timeouts)
const uint32_t kLogIntervalMs = 500;
const uint16_t kLogColSpartan = 0x0001;
const uint16_t kLogColTune = 0x0002;
const uint16_t kLogColBm6 = 0x0004;
const uint16_t kLogColSpeed = 0x0008;
const uint16_t kLogColHeater = 0x0010;
const uint16_t kLogColHours = 0x0020;
const uint16_t kLogColDefault = kLogColSpartan | kLogColTune | kLogColBm6 |
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
  networkPreferences.putBool("hf_espnow", hubFeatEspNow);
  networkPreferences.putBool("hf_ap", hubFeatAp);
  networkPreferences.putBool("hf_wifi", hubFeatWifi);
  networkPreferences.putBool("hf_log", hubFeatLog);
  networkPreferences.putBool("hf_ble123", hubFeatBle123);
  networkPreferences.putBool("hf_blebm6", hubFeatBleBm6);
  networkPreferences.putBool("hf_emu123", hubFeatEmu123);
  networkPreferences.putString("emu_addr", emu123Addr);
  networkPreferences.putUChar("espnow_ch", hubEspNowChannelPref);
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
  if (w25qDetected && !networkPreferences.isKey("cfg_w25q")) hubCfgDirty = true;  // Erst-Backup
  hubApSsid = networkPreferences.getString("ap_ssid", WEB_AP_SSID);
  hubApPassword = networkPreferences.getString("ap_pass", WEB_AP_PASSWORD);
  hubApIp = networkPreferences.getString("ap_ip", "192.168.4.1");
  hubApChannel = networkPreferences.getUChar("ap_chan", 6);  // [WLAN-KANAL] letzter Home-Kanal
  if (hubApChannel < 1 || hubApChannel > 13) hubApChannel = 6;
  hubApMask = networkPreferences.getString("ap_mask", "255.255.255.0");
#if ENABLE_EMU123
  // [MDNS] Emu meldet sich als spartanemu.local, NIE als spartanhub.local.
  // Hub+Emu unter demselben Namen war die Ursache des versehentlichen Fremd-OTA
  // und trifft auch Status-Abfragen am falschen Geraet. Migrations-Guard: ein
  // altes "spartanhub" im Emu-NVS wird einmalig auf "spartanemu" umgezogen.
  hubHostname = networkPreferences.getString("mdns_host", "spartanemu");
  hubHostname.trim();
  if (hubHostname.length() == 0 || hubHostname == "spartanhub") {
    hubHostname = "spartanemu";
    networkPreferences.putString("mdns_host", hubHostname);
  }
#else
  hubHostname = networkPreferences.getString("mdns_host", "spartanhub");
  hubHostname.trim();
  if (hubHostname.length() == 0) hubHostname = "spartanhub";
#endif
  if (hubApSsid.length() == 0) hubApSsid = WEB_AP_SSID;
  if (hubApMask.length() == 0) hubApMask = "255.255.255.0";
  if (!networkPreferences.isKey("hf_ver")) {
#if BLE_BRIDGE
    // BLE-Coprozessor: KEIN WLAN/AP/ESP-NOW (Funk frei, kein Brownout), 123+BM6 AN.
    hubFeatEspNow = false; hubFeatAp = false; hubFeatWifi = false; hubFeatLog = false;
    hubFeatBle123 = true;  hubFeatBleBm6 = true;  hubFeatEmu123 = false;
#else
#ifdef MINIMAL_123
    // Minimal-Modus: NUR 123-BLE + Lambda(CAN) + Log + Web. KEIN BM6 (2. BLE-
    // Verbindung), KEIN ESP-NOW. Das entlastet den NimBLE-ACL-Pool -> stabile 123.
    hubFeatEspNow = false;  // kein ESP-NOW (entlastet Funk + NimBLE-Host)
    hubFeatAp = true;       // AP fuer Web/API-Zugriff waehrend Fahrt
    hubFeatWifi = true;     // STA an: Hub im Heimnetz erreichbar (Verifikation + Zugriff)
    hubFeatLog = true;
    hubFeatBle123 = true;
    hubFeatBleBm6 = false;  // BM6 AUS -> nur EINE BLE-Verbindung = stabiler ACL-Pool
#else
    hubFeatEspNow = true;
    hubFeatAp = true;
    hubFeatWifi = true;
    hubFeatLog = true;
#ifdef DEFAULT_BLE123_ON
    hubFeatBle123 = true;
    hubFeatBleBm6 = true;
#else
    hubFeatBle123 = false;
    hubFeatBleBm6 = false;
#endif
#endif
#if ENABLE_EMU123
    hubFeatEmu123 = true;
#else
    hubFeatEmu123 = false;
#endif
#endif
    hubEspNowChannelPref = 0;
    saveHubFeatures();
    Serial.println("Hub feats:   defaults gesetzt");
    return;
  }
  bool defaultWifi = strlen(HOME_WIFI_SSID) > 0;
  if (networkPreferences.isKey("ssid")) {
    defaultWifi = networkPreferences.getString("ssid", "").length() > 0;
  }
  hubFeatEspNow = networkPreferences.getBool("hf_espnow", true);
  hubFeatAp = networkPreferences.getBool("hf_ap", true);
  hubFeatWifi = networkPreferences.getBool("hf_wifi", defaultWifi);
  hubFeatLog = networkPreferences.getBool("hf_log", true);
  hubFeatBle123 = networkPreferences.getBool("hf_ble123", false);
  hubFeatBleBm6 = networkPreferences.getBool("hf_blebm6", false);
#if ENABLE_EMU123
  // Emu-Build (COM17): immer die 123 emulieren, und KEIN ESP-NOW (Funk frei für
  // flüssige BLE-Notifies; zusammen mit Coex PREFER_BT).
  hubFeatEmu123 = true;
  hubFeatEspNow = false;
#else
  // Test-Hub/Client (COM14) + Live-Hub: nie Emu. Der 123-Emulator läuft auf einem
  // separaten ESP (COM17). Dieser Hub ist reiner Client: 123 empfangen + BM6 +
  // Lambda(Demo) + loggen.
  hubFeatEmu123 = false;
#endif
  const uint8_t savedLambdaTest = networkPreferences.getUChar("lambda_test", 0);
  lambdaTestMode = savedLambdaTest <= static_cast<uint8_t>(LambdaTestMode::Sweep)
      ? static_cast<LambdaTestMode>(savedLambdaTest)
      : LambdaTestMode::Off;
  emu123Addr = networkPreferences.getString("emu_addr", "");
  hubEspNowChannelPref = networkPreferences.getUChar("espnow_ch", 0);
  if (hubEspNowChannelPref > 14) {
    hubEspNowChannelPref = 0;
  }
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
void recordBleScanDevice(const NimBLEAdvertisedDevice *device, bool tuneLike, bool bm6Like)
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
  Serial.printf("[BLE-SCAN] %s rssi=%d name=%s mfg=%s tune=%d bm6=%d\n",
                address.c_str(), rssi, name.length() ? name.c_str() : "---",
                mfg.c_str(), tuneLike ? 1 : 0, bm6Like ? 1 : 0);

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
  bleScanDevices[slot].bm6Like = bleScanDevices[slot].bm6Like || bm6Like;
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

bool bleRadioFreeForBm6Scan()
{
  if (bleCentralScanActive || tuneDoConnect) return false;
  if (tuneLinkState == TuneLinkState::Scanning || tuneLinkState == TuneLinkState::Connecting) {
    return false;
  }
  return true;
}

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
#if ENABLE_EMU123
  json += "-emu";
#endif
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
#if ENABLE_ESP_NOW_HUB
  json += ",\"esp_now_ready\":";
  json += espNowReady ? "true" : "false";
  json += ",\"esp_now_channel\":";
  json += String(espNowActiveChannel > 0 ? espNowActiveChannel : ESP_NOW_WIFI_CHANNEL);
  json += ",\"esp_now_channel_pref\":";
  json += String(hubEspNowChannelPref);
  json += ",\"esp_now_tx\":";
  json += String(espNowTxCount);
  json += ",\"esp_now_tx_fail\":";
  json += String(espNowTxFailCount);
  json += ",\"esp_now_seq\":";
  json += String(espNowSeq);
#endif
  json += ",\"hub_feat_espnow\":";
  json += hubFeatEspNow ? "true" : "false";
  json += ",\"hub_feat_ap\":";
  json += hubFeatAp ? "true" : "false";
  json += ",\"hub_feat_wifi\":";
  json += hubFeatWifi ? "true" : "false";
  json += ",\"hub_feat_log\":";
  json += hubFeatLog ? "true" : "false";
  json += ",\"hub_feat_ble123\":";
  json += hubFeatBle123 ? "true" : "false";
  json += ",\"hub_feat_blebm6\":";
  json += hubFeatBleBm6 ? "true" : "false";
  json += ",\"emu123_on\":";
  json += hubFeatEmu123 ? "true" : "false";
  json += ",\"emu123_manual\":";
  json += String(emu123ManualRpm);
  json += ",\"emu123_rpm\":";
  json += String(emu123CurRpm);
  json += ",\"emu123_adv\":";
  json += String(emu123CurAdv, 1);
  json += ",\"emu123_map\":";
  json += String(emu123CurMap);
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
    json += ",\"bm6\":";
    json += bleScanDevices[i].bm6Like ? "true" : "false";
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
  json += "\",\"tune_fail_streak\":";
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
#if ENABLE_BM6
  json += ",\"bm6_connected\":";
  json += bm6Connected ? "true" : "false";
  json += ",\"bm6_voltage\":";
  json += String(bm6Voltage, 2);
  json += ",\"bm6_temperature\":";
  json += String(static_cast<int>(bm6Temperature));
  json += ",\"bm6_rx_count\":";
  json += String(static_cast<unsigned long>(bm6RxCount));
  json += ",\"bm6_poll_sec\":";
  json += String(bm6PollIntervalMs / 1000);
  json += ",\"uart_rx_pin\":";
  json += String(bridgeUartRxPin);
  json += ",\"uart_tx_pin\":";
  json += String(bridgeUartTxPin);
  json += ",\"bm6_decode_fail\":";
  json += String(static_cast<unsigned long>(bm6DecodeFailCount));
  json += ",\"bm6_age_ms\":";
  json += String(bm6LastRxMs == 0 ? 0UL : static_cast<unsigned long>(now - bm6LastRxMs));
  json += ",\"bm6_saved_address\":\"";
  json += bm6SavedAddress;
  json += "\",\"bm6_aux_saved_address\":\"";
  json += bm6AuxSavedAddress;
  json += "\",\"bm6_active_slot\":\"";
  json += bm6ActiveSlot == 0 ? "main" : "aux";
  json += "\",\"bm6_aux_voltage\":";
  json += String(bm6AuxVoltage, 2);
  json += ",\"bm6_aux_temperature\":";
  json += String(static_cast<int>(bm6AuxTemperature));
  json += ",\"bm6_aux_rx_count\":";
  json += String(static_cast<unsigned long>(bm6AuxRxCount));
  json += ",\"bm6_aux_age_ms\":";
  json += String(bm6AuxLastRxMs == 0 ? 0UL : static_cast<unsigned long>(now - bm6AuxLastRxMs));
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

void webOtaBeginUpload(const char *filename)
{
  otaBusy = true;
  otaRxBytes = 0;
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
  if (logCol(kLogColTune)) header += ";rpm;advance;map;tune_volt;tune_temp;tune_amp";
  if (logCol(kLogColBm6)) header += ";bm6_volt;bm6_temp";
  if (logCol(kLogColSpeed)) header += ";speed_kmh;speed_hz;speed_pulses";
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
  // Emu-Modus = Pruefstand: tuneRpm ist die SELBST erzeugte Emulator-Drehzahl, kein
  // echter Motor -> Variante-A-Gate aus, sonst blockiert der Emu sein eigenes Heim-WLAN.
  if (hubFeatEmu123) return false;
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
    f.printf(";%.0f;%.1f;%.0f;%.1f;%.0f;%.2f",
             tune.rpm,
             tune.advance,
             tune.map,
             tune.voltage,
             tune.temperature,
             tune.coilCurrent);
  }
  if (logCol(kLogColBm6)) {
#if ENABLE_BLE_HUB && ENABLE_BM6
    f.printf(";%.2f;%d", bm6Voltage, static_cast<int>(bm6Temperature));
#else
    f.print(";0.00;0");
#endif
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
  setTuneLinkState(TuneLinkState::Idle, "reset");
}

class TuneClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onDisconnect(NimBLEClient *, int reason) override
  {
    tuneConnected = false;
    tuneNusRx = nullptr;
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
#if !ENABLE_EMU123 && defined(ENABLE_BLE_HUB)
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

// ===== 123\TUNE+ Emulator (Test) ==========================================
// Hub spielt die 123 als BLE-Peripheral nach: advertised NUS-Service + Name
// "123\TUNE+" + Hersteller Albertronic(2330), nimmt die Verbindung an und
// streamt sweependes RPM/ADV/MAP. So testet man Touch/M5/Hub am Schreibtisch.
static NimBLECharacteristic *emu123Tx = nullptr;
static bool     emu123Started = false;
static uint32_t emu123LastMs = 0;
static float    emu123Rpm = 800.0f;
static bool     emu123Rising = true;
// [EMU-TUNE] Live-Zuendwinkel-Emulation, damit die Tune-Kette (T/A/R) am Tisch
// vollstaendig testbar ist. T=Modus an/aus, A/R verschieben den Offset, der in
// updateEmu123() auf den gestreamten Zuendwinkel (0x31) addiert wird.
static bool     emu123TuneMode = false;
static float    emu123TuneOffset = 0.0f;
static SemaphoreHandle_t emu123PingSem = nullptr;  // signalisiert sofortigen Send nach $

static inline char emu123Nib(int n) { return (n < 10) ? ('0' + n) : ('A' + n - 10); }

static void emu123SendField(uint8_t field, int hi, int lo, bool last = false) {
  if (!emu123Tx) return;
  if (hi < 0) hi = 0; else if (hi > 15) hi = 15;
  if (lo < 0) lo = 0; else if (lo > 15) lo = 15;
  uint8_t h = (uint8_t)emu123Nib(hi);
  uint8_t l = (uint8_t)emu123Nib(lo);
  // Checksum: field + 0x10 + (h - 0x30) + (l - 0x30)  (aus 123tune_ble_protocol.md)
  uint8_t chk = (uint8_t)(field + 0x10 + (h - 0x30) + (l - 0x30));
  // Terminator: 0x0D (letztes Feld im Zyklus) oder 0x20 (Space = weitere folgen)
  uint8_t term = last ? 0x0D : 0x20;
  uint8_t buf[5] = { field, h, l, chk, term };
  emu123Tx->notify(buf, 5);
}

// Nach Disconnect muss das Advertising NEU gestartet werden, sonst kann sich
// kein Central wieder verbinden (NimBLE stoppt Advertising bei Connect).
static bool emu123Subscribed = false;

class Emu123TxCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &, uint16_t val) override {
    emu123Subscribed = (val == 1 || val == 2);
    Serial.printf("EMU123:      Subscribe CCCD=%u -> %s\n", val, emu123Subscribed?"AN":"AUS");
    if (emu123Subscribed) {
      emu123LastMs = 0;
      // Auch Semaphor geben → sofortiger voller Zyklus aus dediziertem Task
      if (emu123PingSem) xSemaphoreGiveFromISR(emu123PingSem, nullptr);
    }
  }
};
static Emu123TxCB emu123TxCB;

class Emu123ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
    Serial.println("EMU123:      Central verbunden");
    emu123Subscribed = false;
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    Serial.printf("EMU123:      Central getrennt (r=%d) -> re-advertise\n", reason);
    emu123Subscribed = false;
    emu123TuneMode = false; emu123TuneOffset = 0.0f;  // [EMU-TUNE] Offset verfaellt bei Trennung
    NimBLEDevice::getAdvertising()->start();
  }
};
static Emu123ServerCB emu123ServerCB;

void startEmu123() {
  if (emu123Started) return;
  NimBLEDevice::getScan()->stop();
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(&emu123ServerCB);

  // 1. Nordic UART Service (NUS) — Hauptdatenkanal
  NimBLEService *nus = srv->createService(NimBLEUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e"));

  // RX-Command-Handler: App schreibt AT-Commands, Emu antwortet über TX
  class EmuRxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
      std::string val = c->getValue();
      // ASCII-Command extrahieren (z.B. "$", "I@\r", "v@\r", "PW1234!")
      std::string cmd(val.begin(), val.end());
      Serial.printf("EMU123 RX:   '%s' (len=%u)\n", cmd.c_str(), val.length());
      if (!emu123Tx) return;
      // [EMU-TUNE] Live-Zuendwinkel: T=Tuning an/aus, A=vor, R=zurueck (Einzelbyte,
      // write-no-response wie vom Hub gesendet). Offset wirkt auf 0x31; beim
      // Verlassen (T) -> 0. So ist die ganze Tune-Kette am Tisch verifizierbar.
      if (val.length() == 1 && (cmd[0] == 'T' || cmd[0] == 'A' || cmd[0] == 'R')) {
        if (cmd[0] == 'T') {
          emu123TuneMode = !emu123TuneMode;
          if (!emu123TuneMode) emu123TuneOffset = 0.0f;
          Serial.printf("EMU123 TUNE: Modus %s\n", emu123TuneMode ? "AN" : "AUS");
        } else if (emu123TuneMode) {
          emu123TuneOffset += (cmd[0] == 'A') ? 1.0f : -1.0f;
          Serial.printf("EMU123 TUNE: Offset %+.1f Grad\n", emu123TuneOffset);
        } else {
          Serial.println("EMU123 TUNE: A/R ignoriert (Tuning-Modus aus)");
        }
        return;
      }
      // AT-Command-Antworten (aus APK-Analyse: AuthorizeCommand, ReadInfoCommand)
      if (cmd.find("I@") != std::string::npos) {
        // Device Info: "I@<UID>,<License>,<Flags>\r"
        std::string resp = "I@12345678,STANDARD,00\r";
        emu123Tx->notify((uint8_t*)resp.c_str(), resp.length());
        Serial.println("EMU123 TX:   I@ response");
      } else if (cmd.find("v@") != std::string::npos) {
        // Firmware/Settings: "v@<Version> <Cyl> <Curve> <Data>\r"
        std::string resp = "v@1.4c 4 1 0\r";
        emu123Tx->notify((uint8_t*)resp.c_str(), resp.length());
        Serial.println("EMU123 TX:   v@ response");
      } else if (cmd.find("PW") != std::string::npos) {
        // Auth: immer OK (kein echtes Passwort)
        std::string resp = "OK\r";
        emu123Tx->notify((uint8_t*)resp.c_str(), resp.length());
        Serial.println("EMU123 TX:   PW -> OK");
      } else if (cmd[0] == '$' || cmd[0] == 0x24) {
        // Ping — Semaphor geben damit dedizierter Task sofort sendet
        emu123LastMs = 0;
        if (emu123PingSem) xSemaphoreGiveFromISR(emu123PingSem, nullptr);
      } else if (cmd[0] == '\r' || cmd[0] == 0x0D) {
        // CR — ignorieren, wird zusammen mit $ als Ping verarbeitet
      } else if (cmd.find("v@") != std::string::npos) {
        // v@\r → Firmware/Version/Live-Werte (aus Referenz-Impl)
        if (emu123Tx && emu123Subscribed) {
          uint8_t r[18] = { 0x76,0x40,0x0D,
            0x34,0x33,  // Volt: 13.8V
            0x34,0x39,  // Temp: 75°C
            0x37,0x34,  // Pressure: 100kPa
            0x34,0x31,0x2D,0x31,0x30,0x2D,0x34,0x35,0x20 }; // "41-10-45 "
          emu123Tx->notify(r, sizeof(r));
          Serial.println("EMU123 TX:   v@ response");
          if (emu123PingSem) xSemaphoreGiveFromISR(emu123PingSem, nullptr);
        }
      } else if (cmd.find("I@") != std::string::npos) {
        // I@\r → Device Info
        if (emu123Tx && emu123Subscribed) {
          std::string resp = "I@12345678,STANDARD,00\r";
          emu123Tx->notify((uint8_t*)resp.c_str(), resp.length());
          Serial.println("EMU123 TX:   I@ response");
        }
      } else if (cmd.find("10@") != std::string::npos || cmd.find("11@") != std::string::npos ||
                 cmd.find("12@") != std::string::npos || cmd.find("13@") != std::string::npos) {
        // EEPROM-Block-Antworten (Zündkurven) — 59 Bytes in 3x20 Fragmenten
        if (emu123Tx && emu123Subscribed) {
          // Aus Referenz-Impl ble.ino: Standard VW T2b 4-Zyl Kurve
          // Block 10: RPM+Advance Punkte 1-7
          uint8_t b10[59] = {
            0x31,0x30,0x40,0x0D,  // header "10@\r"
            0x46,0x46,0x20, 0x46,0x46,0x20,  // FF FF (leer)
            0x30,0x41,0x20, 0x30,0x30,0x20,  // RPM500, Adv0
            0x30,0x45,0x20, 0x30,0x35,0x20,  // RPM750, Adv5
            0x31,0x30,0x20, 0x30,0x41,0x20,  // RPM1000, Adv10
            0x31,0x34,0x20, 0x30,0x46,0x20,  // RPM1250, Adv15
            0x31,0x45,0x20, 0x31,0x34,0x20,  // RPM1500, Adv20
            0x32,0x34,0x20, 0x31,0x38,0x20,  // RPM2000, Adv24
            0x33,0x43,0x20, 0x31,0x43,0x20,  // RPM3000, Adv28
            0x31,0x30, 0x35,0x38,0x35,0x0D   // "10" + csum + CR
          };
          const char *hdr = cmd.c_str();
          b10[0]=(uint8_t)hdr[0]; b10[1]=(uint8_t)hdr[1];  // richtigen Block-Header
          emu123Tx->notify(&b10[0], 20);
          vTaskDelay(pdMS_TO_TICKS(20));
          emu123Tx->notify(&b10[20], 20);
          vTaskDelay(pdMS_TO_TICKS(20));
          emu123Tx->notify(&b10[40], 19);
          Serial.printf("EMU123 TX:   %s response\n", hdr);
        }
      }
    }
  };
  static EmuRxCB emuRxCB;
  auto *rxChar = nus->createCharacteristic(NimBLEUUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e"),
                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(&emuRxCB);
  emu123Tx = nus->createCharacteristic(NimBLEUUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e"),
                                       NIMBLE_PROPERTY::NOTIFY);
  emu123Tx->setCallbacks(&emu123TxCB);
  // NimBLE erstellt CCCD automatisch bei NOTIFY-Property — kein expliziter Descriptor nötig

  // 2. Device Information Service (0x180A) — minimal, schnelle Discovery
  NimBLEService *dis = srv->createService(NimBLEUUID((uint16_t)0x180A));
  NimBLECharacteristic *fwRev = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A26), NIMBLE_PROPERTY::READ);
  fwRev->setValue(std::string("version: 1.4c(Albertronic BV)"));

  // Services starten
  srv->start();

  // Generic Access Device Name auf "123\TUNE+" setzen (App liest 0x2A00!)
  NimBLEDevice::setDeviceName("123\\TUNE+ ");

  // Advertising — exakt wie echte 123TUNE+ (aus nRF Connect Log)
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->stop();
  NimBLEAdvertisementData ad;
  ad.setFlags(0x06);  // LE General Discoverable, BR/EDR Not Supported
  ad.addServiceUUID(NimBLEUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e"));
  NimBLEAdvertisementData sr;
  sr.setName("123\\TUNE+");
  std::string mfg;  // Albertronic BV (0x091A) + 0x00 + 0x05506C
  mfg += (char)0x1A; mfg += (char)0x09;  // Company ID 0x091A (LE)
  mfg += (char)0x00; mfg += (char)0x05; mfg += (char)0x50; mfg += (char)0x6C;
  sr.setManufacturerData(mfg);
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->start();
  emu123Started = true;

  // Semaphor für sofortigen Send nach $ Ping
  if (!emu123PingSem) emu123PingSem = xSemaphoreCreateBinary();

  // Dedizierter Sender-Task: wartet auf $ Semaphor, sendet sofort RPM-Frame
  xTaskCreate([](void*) {
    for (;;) {
      // Auf $ oder Subscribe warten (max 50ms), dann sofort senden
      xSemaphoreTake(emu123PingSem, pdMS_TO_TICKS(50));
      {
        if (emu123Tx && emu123Subscribed) {
          // Kompletten Zyklus senden: RPM, ADV, MAP, Temp, Volt, 0x42(end)
          // Genau wie echte 123TUNE+ nach $\r — App erwartet vollen Zyklus
          const int rpm = (int)emu123CurRpm;
          const float adv = 10.0f + (rpm - 700) * 0.004f;
          const int mapv = 40 + (rpm - 700) / 50;
          const int tempRaw = (int)(75 + 30); // 75°C
          const int voltRaw = (int)(13.8f * 4.54f);

          auto sendF = [](uint8_t f, int hi, int lo, bool last) {
            uint8_t h=(uint8_t)emu123Nib(hi); uint8_t l=(uint8_t)emu123Nib(lo);
            uint8_t chk=(uint8_t)(f+0x10+(h-0x30)+(l-0x30));
            uint8_t buf[5]={f,h,l,chk,(uint8_t)(last?0x0D:0x20)};
            emu123Tx->notify(buf,5);
          };
          { int rhi=rpm/800; int rlo=(rpm-rhi*800)/50; sendF(0x30,rhi,rlo,false); }
          vTaskDelay(1);
          { int ahi=(int)(adv/3.2f); int alo=(int)((adv-ahi*3.2f)/0.2f); sendF(0x31,ahi,alo,false); }
          vTaskDelay(1);
          sendF(0x32,(mapv>>4)&0xF,mapv&0xF,false);
          vTaskDelay(1);
          sendF(0x33,(tempRaw>>4)&0xF,tempRaw&0xF,false);
          vTaskDelay(1);
          sendF(0x41,(voltRaw>>4)&0xF,voltRaw&0xF,false);
          vTaskDelay(1);
          sendF(0x42,0x4,0x6,true);  // Zyklusende mit 0x0D
          Serial.printf("EMU123 TX:   voller Zyklus rpm=%d\n", rpm);
        }
      }
    }
  }, "emu123ping", 4096, nullptr, 2, nullptr);

  Serial.println("EMU123:      AN — vollstaendige 123TUNE+ Emulation (NUS+DIS+BAT+TxPwr)");
}

// [DEAD-CODE] stopEmu123() entfernt: der emu123-Toggle schaltet bewusst per Reboot
// um (siehe "Emulator <-> Central sauber per Reboot umschalten" im /hub_feat-Handler),
// ein Laufzeit-Stop-Pfad existierte nie und haette den Sender-Task nicht sauber beendet.

void updateEmu123() {
  if (!emu123Started) { startEmu123(); return; }
  const uint32_t now = millis();
  if (now - emu123LastMs < 40) return;   // ~25 Hz, EIN Frame pro Tick
  emu123LastMs = now;
  // Sweep/feste RPM fortschreiben
  int rpm;
  if (emu123ManualRpm >= 0) {
    rpm = emu123ManualRpm;
  } else {
    emu123Rpm += emu123Rising ? 25.0f : -25.0f;
    if (emu123Rpm >= 3200.0f) emu123Rising = false;
    if (emu123Rpm <= 700.0f)  emu123Rising = true;
    rpm = (int)emu123Rpm;
  }
  if (rpm < 0) rpm = 0; else if (rpm > 12000) rpm = 12000;
  // [EMU-TUNE] Basis-Zuendwinkel + Live-Offset (A/R). Geklemmt auf gueltigen Bereich.
  float adv = 10.0f + (rpm - 700) * 0.011f + emu123TuneOffset;
  if (adv < 0.0f) adv = 0.0f; else if (adv > 48.0f) adv = 48.0f;
  int mapv = 40 + (rpm - 700) / 45; if (mapv > 95) mapv = 95; if (mapv < 0) mapv = 0;
  emu123CurRpm = rpm; emu123CurAdv = adv; emu123CurMap = mapv;

  // Plausible Hilfswerte: Temperatur steigt leicht mit Drehzahl, Spulenstrom
  // mit Last, Bordspannung leicht fallend. Encodings spiegeln die Decoder:
  //   0x33 temp: raw = temp + 30     0x35 coil: raw = coil * 8.65
  //   0x41 volt: raw = volt * 4.54
  const float tempC = 75.0f + (rpm - 700) * 0.004f;          // ~75..85 C
  const float coilA = 2.0f + (rpm - 700) * 0.0006f;          // ~2.0..3.5 A
  const float voltV = 13.8f - (rpm - 700) * 0.0001f;         // ~13.8..13.5 V
  int tempRaw = (int)(tempC + 30.0f + 0.5f); if (tempRaw > 255) tempRaw = 255; if (tempRaw < 0) tempRaw = 0;
  int coilRaw = (int)(coilA * 8.65f + 0.5f); if (coilRaw > 255) coilRaw = 255; if (coilRaw < 0) coilRaw = 0;
  int voltRaw = (int)(voltV * 4.54f + 0.5f); if (voltRaw > 255) voltRaw = 255; if (voltRaw < 0) voltRaw = 0;

  // Emu-Werte ZUSÄTZLICH in den Tune-Status spiegeln. So bekommt der ESP-NOW-
  // Fan-out (M5 + Touch gleichzeitig) im Emulator-Modus dieselben, vollständigen
  // Daten mit 25 Hz — statt nur EIN BLE-Direkt-Display über die langsame
  // 6-Felder-Round-Robin-Notify. Das ist das stabile, flüssige Test-Setup.
  portENTER_CRITICAL(&stateMux);
  tuneRpm = (float)rpm;
  tuneAdvance = adv;
  tuneMap = (float)mapv;
  tuneTemperature = tempC;
  tuneCoilCurrent = coilA;
  tuneVoltage = voltV;
  tuneLastRxMs = now;
  if (tuneRxCount < UINT32_MAX) tuneRxCount++;
  portEXIT_CRITICAL(&stateMux);

  // RPM JEDEN Tick senden (25 Hz) — das ist der Wert, der flüssig wirken muss,
  // genau wie ein echtes 123. Die Nebenwerte rotieren je ~5 Hz. Dank
  // notify(payload,len) sind zwei Notifies pro Tick zuverlässig.
  { const int rhi = rpm / 800; const int rlo = (rpm - rhi * 800) / 50;
    emu123SendField(0x30, rhi, rlo); }                                     // RPM @25Hz
  static uint8_t sec = 0;
  switch (sec) {
    case 0: { const int ahi = (int)(adv / 3.2f); const int alo = (int)((adv - ahi * 3.2f) / 0.2f);
              emu123SendField(0x31, ahi, alo); } break;                    // Zuendung
    case 1: emu123SendField(0x32, (mapv >> 4) & 0xF, mapv & 0xF); break;   // MAP
    case 2: emu123SendField(0x33, (tempRaw >> 4) & 0xF, tempRaw & 0xF); break;  // Temp
    case 3: emu123SendField(0x35, (coilRaw >> 4) & 0xF, coilRaw & 0xF); break;  // Coil
    default:
      emu123SendField(0x41, (voltRaw >> 4) & 0xF, voltRaw & 0xF); // Volt
      // 0x42-Frame = Zyklusende (konstant 70 = 0x46), terminator 0x0D
      // Echte 123 sendet diesen immer als letzten Frame pro Zyklus.
      emu123SendField(0x42, 0x4, 0x6, true);  // last=true → 0x0D terminator
      break;
  }
  sec = (sec + 1) % 5;
}

void setupBleHub()
{
  NimBLEDevice::init(BLE_HUB_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(23);
  // Emulator: gewuenschte 123-Adresse advertisen (static-random, Top-Bits 11),
  // damit auch Geraete mit fester Ziel-MAC (M5) sich verbinden.
  if (hubFeatEmu123 && emu123Addr.length() == 17) {
    NimBLEAddress a(std::string(emu123Addr.c_str()), BLE_ADDR_RANDOM);
    if (NimBLEDevice::setOwnAddr(a)) {
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
      Serial.printf("EMU123:      advertise als BLE-Adresse %s\n", emu123Addr.c_str());
    } else {
      Serial.printf("EMU123:      Adresse %s ungueltig (Top-Bits muessen 11 sein)\n", emu123Addr.c_str());
    }
  }
  bleAddress = NimBLEDevice::getAddress().toString().c_str();
  if (hubFeatEmu123 && emu123Addr.length() == 17) bleAddress = emu123Addr;
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
  if (hubFeatEmu123) {
    // Emulator-Modus: Hub IST die 123 (Peripheral). Kein Central-Scan.
    startEmu123();
    return;
  }
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
  if (hubFeatEmu123) {   // Emulator-Modus: nur die 123 nachspielen
    updateEmu123();
    return;
  }
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
  // Im Emulator-Modus speist updateEmu123() den Tune-Status, also auch dann lesen.
  if (hubFeatBle123 || hubFeatEmu123) {
    const TuneSnapshot tune = tuneSnapshot();
    tuneFresh = tune.lastRxMs != 0 && (now - tune.lastRxMs) <= 3000;
    tuneConn = tuneConnected || hubFeatEmu123;
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
  Serial.printf("Hub feats:   ESP-NOW=%s AP=%s WLAN=%s LOG=%s 123=%s BM6=%s EMU123=%s ch=%u\n",
                hubFeatEspNow ? "on" : "off",
                hubFeatAp ? "on" : "off",
                hubFeatWifi ? "on" : "off",
                hubFeatLog ? "on" : "off",
                hubFeatBle123 ? "on" : "off",
                hubFeatBleBm6 ? "on" : "off",
                hubFeatEmu123 ? "on" : "off",
                hubEspNowChannelPref);
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
#if ENABLE_ESP_NOW_HUB
  if (!hubFeatEspNow && espNowReady) {
    teardownEspNowHub();
  } else if (hubFeatEspNow && espNowReady &&
             espNowActiveChannel != espNowEffectiveChannel()) {
    teardownEspNowHub();
  }
#endif
#if ENABLE_BLE_HUB
  if (!hubFeatBle123 && tuneConnected) {
    resetTuneClient();
  } else if (hubFeatBle123 && !tuneConnected && !tuneDoConnect && tuneNextScanMs == 0) {
    scheduleTuneScan(true);
  }
#if ENABLE_BM6
  if (!hubFeatBleBm6 && bm6Connected) {
    resetBm6Client();
  } else if (hubFeatBleBm6 && !bm6Connected && !bm6DoConnect && bm6NextScanMs == 0) {
    scheduleBm6Scan();
  }
#endif
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
  // [WIFI-MAC-OVR] Recovery ohne WLAN, falls ein Override die STA-Verbindung zerschiesst.
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
    Serial.println("Hub feats:   usage: hub feat <name> on|off | espnow ch 0|6|11 | status");
    return true;
  }
  String feat = rest.substring(0, sp);
  String arg = rest.substring(sp + 1);
  arg.trim();
  feat.toLowerCase();

  bool changed = false;
  if (feat == "espnow" && arg.startsWith("ch ")) {
    int channel = arg.substring(3).toInt();
    if (channel < 0 || channel > 14) {
      channel = 0;
    }
    hubEspNowChannelPref = static_cast<uint8_t>(channel);
    changed = true;
  } else if (feat == "espnow") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatEspNow = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatEspNow = false;
      changed = true;
    }
  } else if (feat == "ap") {
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
  } else if (feat == "blebm6") {
    if (arg.equalsIgnoreCase("on") || arg == "1") {
      hubFeatBleBm6 = true;
      changed = true;
    } else if (arg.equalsIgnoreCase("off") || arg == "0") {
      hubFeatBleBm6 = false;
      changed = true;
    }
  } else if (feat == "emu123") {
    // Test-Hub ist reiner Client — emu123 läuft auf separatem ESP (COM17).
    Serial.println("Hub feats:   emu123 auf diesem Test-Hub deaktiviert (Emu = separater ESP / COM17)");
    return true;
  } else {
    Serial.println("Hub feats:   unknown feature (espnow/ap/wifi/log/ble123/blebm6)");
    return true;
  }

  if (changed) {
    saveHubFeatures();
    if (feat == "emu123") {
      // Emulator <-> Central sauber per Reboot umschalten.
      Serial.printf("Hub feats:   emu123=%s gespeichert — Reboot...\n", hubFeatEmu123 ? "on" : "off");
      delay(300);
      ESP.restart();
    }
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
  // [WIFI-MAC-OVR] geraeteweit (nicht pro Profil)
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
    if (hubFeatEmu123) {   // Schlanke Seite: nur AP + Heim-WLAN + /emu
      const bool sta = (WiFi.status() == WL_CONNECTED);
      const String apName = WiFi.softAPSSID();
      String h; h.reserve(2800);
      h += F("<!doctype html><html lang=de><head><meta charset=utf-8>"
             "<meta name=viewport content='width=device-width,initial-scale=1'><title>123 Emulator</title>"
             "<style>body{font-family:system-ui,Arial;background:#0b1210;color:#e6ede8;margin:0}"
             "main{max-width:560px;margin:auto;padding:16px}h1{color:#9ed85b;font-size:1.25rem}"
             "h2{color:#9ed85b;font-size:1rem;margin:22px 0 0}.card{background:#101a15;border:1px solid #26372e;"
             "border-radius:10px;padding:14px;margin:10px 0}input{box-sizing:border-box;width:100%;min-height:42px;"
             "margin:6px 0 12px;padding:10px;border:1px solid #35453c;border-radius:8px;background:#0b1210;color:#e6ede8;font-size:1rem}"
             "label{font-size:.9rem;color:#bde87a}button{background:#2e7d32;color:#fff;border:0;border-radius:8px;padding:11px 16px;font-size:1rem}"
             "a.btn{display:inline-block;background:#26372e;color:#bde87a;border-radius:8px;padding:10px 14px;text-decoration:none}.mut{color:#9ab}</style>"
             "</head><body><main><h1>123&#92;TUNE+ Emulator</h1>"
             "<div class=card>BLE: advertised als <b>123&#92;TUNE+</b> (Company 2330)."
             "<br><a class=btn href='/emu'>&#9654; Emulator-Steuerung (/emu)</a></div>");
      h += "<h2>Heim-WLAN</h2><div class=card>Status: <b>";
      h += sta ? ("verbunden, IP " + WiFi.localIP().toString()) : String("nicht verbunden");
      // [WIFI-STATIC] Ueber /wifi_profile_save (Slot 1) statt dem alten /wifi -- so
      // greift die vorhandene DHCP/Static-Logik ohne zweiten Code-Pfad zu pflegen.
      h += "</b><form method=POST action=/wifi_profile_save><input type=hidden name=slot value=1>"
           "<label>WLAN-Name (SSID)</label><input name=ssid value='" + savedWifiSsid + "'>"
           "<label>Passwort</label><input name=pass type=password placeholder='leer = unveraendert'>"
           "<label>IP-Vergabe</label>"
           "<select name=ipm id=ipm onchange=\"document.getElementById('ipf').hidden=(this.value!='1')\">"
           "<option value=0" + String(g_hubWifiProfiles[1].ipMode == 0 ? " selected" : "") + ">DHCP (automatisch)</option>"
           "<option value=1" + String(g_hubWifiProfiles[1].ipMode == 1 ? " selected" : "") + ">Statisch</option></select>"
           "<div id=ipf" + String(g_hubWifiProfiles[1].ipMode == 1 ? "" : " hidden") + ">"
           "<label>IP-Adresse</label><input name=ip value='" + String(g_hubWifiProfiles[1].ip) + "' placeholder='192.168.0.80'>"
           "<label>Gateway</label><input name=gw value='" + String(g_hubWifiProfiles[1].gw) + "' placeholder='192.168.0.1'>"
           "<label>Subnetzmaske</label><input name=mask value='" + String(g_hubWifiProfiles[1].mask) + "'></div>"
           "<button>Verbinden &amp; speichern</button></form></div>";
      // [WIFI-MAC-OVR] eigenes Kaertchen mit Warnung -- eine falsche/nicht-assoziierbare
      // MAC kann die Heimnetz-Verbindung komplett kappen (Recovery dann nur per Serial:
      // "hub wifi mac clear").
      h += "<h2>WLAN-MAC</h2><div class=card>Aktiv: <b>" + WiFi.macAddress() + "</b>"
           "<p class=mut>Nur bei Verdacht auf doppelte Werks-MAC (billige Klone) aendern. "
           "Leer + Speichern = zurueck zur Werks-MAC. Loest Neustart aus.</p>"
           "<form method=POST action=/wifi_mac><label>MAC-Override</label>"
           "<input name=mac value='" + String(g_wifiMacOverride) + "' placeholder='AA:BB:CC:DD:EE:FF'>"
           "<button>Speichern &amp; neustarten</button></form></div>";
      h += "<h2>Access Point</h2><div class=card>Aktiv: <b>" + apName + "</b> &middot; " + WiFi.softAPIP().toString();
      h += "<form method=POST action=/ap_config><label>AP-Name</label><input name=ssid value='" + apName + "'>"
           "<label>AP-Passwort (leer = offen, sonst min. 8)</label><input name=pass value='" + hubApPassword + "'>"
           "<label>AP-IP</label><input name=ip value='" + hubApIp + "'>"
           "<label>Netzmaske</label><input name=mask value='" + hubApMask + "'>"
           "<button>AP speichern</button></form></div>";
      h += F("<p class=mut>Reiner 123-BLE-Emulator &middot; kein CAN/ESP-NOW.</p></main></body></html>");
      server.send(200, "text/html", h);
      return;
    }
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
  server.on("/bm6_interval", HTTP_GET, []() {
    uint32_t sec = server.arg("sec").toInt();
    if (sec >= 10 && sec <= 3600) {
      bm6PollIntervalMs = sec * 1000;
      Serial.printf("BM6:         Abfrageintervall %lus\n", (unsigned long)sec);
    }
    server.send(200, "application/json",
                "{\"ok\":true,\"bm6_poll_sec\":" + String(bm6PollIntervalMs/1000) + "}");
  });
  server.on("/hub_feat", HTTP_GET, []() {
    // Dev-Tab: Hub-Features zur Laufzeit schalten (kein Reflash).
    const String name = server.arg("name");
    const bool on = (server.arg("val") == "on" || server.arg("val") == "1");
    bool known = true;
    if (name == "ble123") hubFeatBle123 = on;
    else if (name == "blebm6") hubFeatBleBm6 = on;
    else if (name == "espnow") hubFeatEspNow = on;
    else if (name == "log") hubFeatLog = on;
    else known = false;
    if (known) {
      saveHubFeatures();
      applyHubFeatures();
      logHubEvent("hub_feat", "web");
    }
    server.send(known ? 200 : 400, "application/json",
                String("{\"ok\":") + (known ? "true" : "false") + "}");
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
    server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
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
  server.on("/log_columns", HTTP_POST, []() {
    uint16_t mask = 0;
    if (server.hasArg("spartan")) mask |= kLogColSpartan;
    if (server.hasArg("tune")) mask |= kLogColTune;
    if (server.hasArg("bm6")) mask |= kLogColBm6;
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
  // [WIFI-MAC-OVR] Manuelle STA-MAC setzen/loeschen, wirkt erst nach Neustart.
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
  server.on("/emu", HTTP_GET, []() {
#if ENABLE_EMU123
    // Emu-Build: volle Emulator-Steuerung (RPM/Sweep + advertised BLE-Adresse).
    if (server.hasArg("rpm"))   emu123ManualRpm = server.arg("rpm").toInt();
    if (server.hasArg("sweep")) emu123ManualRpm = -1;
    if (server.hasArg("addr")) {   // advertised BLE-Adresse aendern (Reboot)
      String na = server.arg("addr"); na.trim(); na.toLowerCase();
      if (na == "chip") emu123Addr = "";
      else if (na.length() == 17) emu123Addr = na;
      saveHubFeatures();
      server.send(200, "text/html",
                  "<meta http-equiv='refresh' content='4;url=/emu'>BLE-Adresse gesetzt - Reboot...");
      delay(300);
      ESP.restart();
      return;
    }
    String h;
    h.reserve(2000);
    h += F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
           "<title>123 Emulator</title><style>body{font-family:system-ui;background:#0e0e0e;color:#eee;margin:16px}"
           "a.btn{display:inline-block;padding:11px 15px;margin:5px 6px 5px 0;background:#222;color:#eee;"
           "border:1px solid #444;border-radius:9px;text-decoration:none}a.on{background:#2e7d32;border-color:#2e7d32}"
           ".big{font-size:2.1rem;font-weight:700;margin:14px 0}.muted{color:#9aa}</style>"
           "<h2>123&#92;TUNE+ Emulator</h2>");
    h += "<div>Status: <b style='color:#54d273'>AKTIV</b> &middot; Modus: <b>";
    h += (emu123ManualRpm >= 0) ? ("fest " + String(emu123ManualRpm)) : String("Sweep 700-3200");
    h += "</b></div>";
    h += "<div class=muted>BLE-Adresse: <b style='color:#9ed85b'>" + bleAddress +
         "</b> &middot; Name <b>123&#92;TUNE+</b></div>";
    h += "<form method=GET action=/emu style='margin:8px 0 4px'>"
         "<input name=addr value='" + (emu123Addr.length() ? emu123Addr : bleAddress) +
         "' style='padding:9px;border-radius:8px;border:1px solid #444;background:#1a1a1a;color:#eee;width:14em'>"
         "<button style='padding:9px 13px;margin-left:6px;border-radius:8px;border:0;background:#2e7d32;color:#fff'>setzen</button></form>"
         "<div><a class='btn' href='/emu?addr=ef:a8:b2:de:e0:9e'>echte 123 (ef:a8:b2)</a>"
         "<a class='btn' href='/emu?addr=chip'>Chip-Default</a></div>"
         "<p class=muted style='margin:4px 0'>Adresse muss static-random sein (erstes Byte Top-Bits 11, "
         "z.B. e..&#47;f..&#47;c..&#47;d..).</p>";
    h += F("<div class=big id=live>--</div>"
           "<div class=muted>Drehzahl:</div>"
           "<div><a class='btn' href='/emu?sweep=1'>Sweep</a>"
           "<a class='btn' href='/emu?rpm=300'>300</a>"
           "<a class='btn' href='/emu?rpm=800'>800</a><a class='btn' href='/emu?rpm=1500'>1500</a>"
           "<a class='btn' href='/emu?rpm=2500'>2500</a><a class='btn' href='/emu?rpm=4000'>4000</a></div>"
           "<p class=muted>Advertised als &#39;123&#92;TUNE+&#39;. Hub/M5 verbinden sich damit wie mit der echten 123.</p>"
           "<script>setInterval(async()=>{try{let d=await(await fetch('/api/status')).json();"
           "document.getElementById('live').textContent=d.emu123_on?('RPM '+d.emu123_rpm+'  ADV '+d.emu123_adv+'\\u00b0  MAP '+d.emu123_map):'--';}catch(e){}},400);</script>");
    server.send(200, "text/html", h);
#else
    // Test-Hub ist reiner Client — der 123-Emulator läuft auf einem separaten ESP.
    server.send(200, "text/html",
                "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
                "<title>123 Emulator</title><body style='font-family:system-ui;background:#0e0e0e;color:#eee;margin:16px'>"
                "<h2>123 Emulator</h2><p>Dieser Test-Hub ist reiner <b>Client</b>: "
                "empf&auml;ngt 123, liest BM6, Lambda-Demo, loggt.</p>"
                "<p style='color:#9aa'>Der 123-Emulator l&auml;uft auf einem separaten ESP "
                "(COM17, <a style='color:#bde87a' href='http://192.168.0.80/emu'>192.168.0.80/emu</a>).</p>"
                "<p><a style='color:#bde87a' href='/'>&larr; zur&uuml;ck</a></p></body>");
#endif
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
    bm6ActiveSlot = 0;
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
  server.on("/bm6_aux_target", HTTP_POST, []() {
#if ENABLE_BLE_HUB && ENABLE_BM6
    const String mac = normalizeMacInput(server.arg("bm6_aux_mac"));
    if (mac.length() > 0 && !looksLikeMacAddress(mac)) {
      server.send(400, "text/plain", "BM6 Aux-Adresse ungueltig. Format aa:bb:cc:dd:ee:ff");
      return;
    }
    ensurePreferences();
    bm6AuxSavedAddress = mac;
    networkPreferences.putString("bm6_aux_mac", bm6AuxSavedAddress);
    bm6ActiveSlot = 0;
    resetBm6Client();
    bm6DoConnect = false;
    scheduleBm6Scan();
    Serial.printf("BM6 BLE:     aux target %s\n",
                  bm6AuxSavedAddress.length() ? bm6AuxSavedAddress.c_str() : "(disabled)");
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
  server.on("/hub_features", HTTP_POST, []() {
    const bool wasApOn = hubFeatAp;
    const bool nextEspNow = server.hasArg("espnow");
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
    hubFeatEspNow = nextEspNow;
    hubFeatAp = nextAp;
    hubFeatWifi = nextWifi;
    hubFeatLog = server.hasArg("log");
    hubFeatBle123 = server.hasArg("ble123");
    hubFeatBleBm6 = server.hasArg("blebm6");
    if (server.hasArg("espnow_ch")) {
      int channel = server.arg("espnow_ch").toInt();
      if (channel < 0 || channel > 14) {
        channel = 0;
      }
      hubEspNowChannelPref = static_cast<uint8_t>(channel);
    }
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
      hubFeatEmu123 ? "123-emulator" : WEB_AP_SSID,
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
      const char *roadSsid = hubFeatEmu123 ? "123-emulator" : hubApSsid.c_str();
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

  // --- Cockpit-Frame an Display(s) senden: 0x510, 8 Byte, ~10 Hz ---
  // Gleicher TWAI-Controller wie der Spartan-RX (NORMAL-Mode -> sendefaehig).
  if (now - lastCockpitCanTxMs >= COCKPIT_CAN_TX_INTERVAL_MS) {
    lastCockpitCanTxMs = now;
    const SpartanReading snap = readingSnapshot();
    uint16_t rpm = 0;
    float    advance = 0.0f;
    uint8_t  mapKpa = 0;
    bool     tuneFresh = false;
    if (hubFeatBle123 || hubFeatEmu123) {
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
    tx.identifier        = COCKPIT_CAN_TX_ID;
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
#if ENABLE_ESP_NOW_HUB
  Serial.printf("ESP-NOW:     cockpit broadcast (default ch %d, follows STA)\n", ESP_NOW_WIFI_CHANNEL);
#else
  Serial.println("ESP-NOW:     disabled");
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
  delay(500);

  detectW25Q();   // externen W25Q128-Flash erkennen (Check, noch ohne Dateisystem)

#if BLE_BRIDGE
  // BLE-Bridge: NUR 123\TUNE+, dediziert, kein BM6, kein WLAN, kein Hintergrund-Scan.
  // Architektur: feste MAC aus NVS (oder Compile-Default) -> direkt verbinden -> bei
  // Trennung direkt reconnect ohne erneuten Scan. Einmaliger Scan nur wenn keine MAC
  // gespeichert (erstes Setup). Danach: Verbindung halten = einzige Aufgabe.
  Serial.println("\n=== BLE-Bridge: NUR 123\\TUNE+, dediziert ===");
  hubFeatBle123 = true;
  hubFeatBleBm6 = false;   // BM6 = zweitrangig, separater Chip spaeter
  hubFeatEmu123 = false;
  hubFeatEspNow = false;
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
  setupEspNowHub();
#if defined(ENABLE_BLE_HUB)
#if ENABLE_EMU123
  // Emu (COM17): muss die 123 flüssig über BLE SENDEN. BLE bekommt Funkvorrang,
  // sonst blockiert WiFi die Notifies (Burst-dann-Stillstand -> Gegenstelle
  // trennt per Stale). WLAN-AP wird dabei schwächer — beim reinen Emu egal.
  if (esp_coex_preference_set(ESP_COEX_PREFER_BT) == ESP_OK) {
    Serial.println("Coex:        BLE-Vorrang gesetzt (PREFER_BT, Emu)");
  }
#else
  // Hub/Client: BALANCE — BLE-Empfang stabil, WiFi-AP/ESP-NOW erreichbar.
  // PREFER_BT hungerte hier das AP aus (Ping-Timeouts, keine WebGUI).
  if (esp_coex_preference_set(ESP_COEX_PREFER_BALANCE) == ESP_OK) {
    Serial.println("Coex:        BLE/WiFi Balance gesetzt (PREFER_BALANCE)");
  }
#endif
#endif
  if (!hubFeatEmu123) {   // Emulator = nur BLE(123) + WebGUI, kein CAN/UART/Speed
    setupHourmeters();
    setupCan();
    setupUart();
    setupSpeedReed();
    setupBridgeUart();  // BLE-Bridge UART: AZ-ESP32 GPIO17->S3-GPIO4
  } else {
    Serial.println("EMU123:      123-Emulator (CAN/UART/Speed aus, ESP-NOW Fan-out an)");
  }
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
  // Bridge-Loop: NUR 123-State-Machine, kein BM6, kein Hintergrund-Scan.
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
  if (!hubFeatEmu123) {   // im Emulator-Modus nur BLE + WebGUI
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
  }
  updateWebGui();
  updateBleHub();
  updateEspNowHub();
  updateDisplay();
  delay(10);
}
