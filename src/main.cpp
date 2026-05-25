#include <Arduino.h>
#include <driver/twai.h>

#ifndef ENABLE_WEB_GUI
#define ENABLE_WEB_GUI 0
#endif

#ifndef ENABLE_BLE_HUB
#define ENABLE_BLE_HUB 0
#endif

#if ENABLE_WEB_GUI
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#endif

#if ENABLE_BLE_HUB
#include <NimBLEDevice.h>
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

// With a 10k high-side and 20k low-side divider, ADC voltage is input * 2/3.
#ifndef ANALOG_DIVIDER_NUM
#define ANALOG_DIVIDER_NUM 3
#endif

#ifndef ANALOG_DIVIDER_DEN
#define ANALOG_DIVIDER_DEN 2
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
constexpr float kLambdaAtZeroVolt = 0.68f;
constexpr float kLambdaAtFiveVolt = 1.36f;
#if ENABLE_BLE_HUB
constexpr char kBleServiceUuid[] = "7f510001-5a6b-4d2a-9f20-14a7f3e20000";
constexpr char kBleStatusUuid[] = "7f510002-5a6b-4d2a-9f20-14a7f3e20000";
constexpr char kBleCommandUuid[] = "7f510003-5a6b-4d2a-9f20-14a7f3e20000";
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

SpartanReading reading;
bool canReady = false;
uint32_t lastDisplayMs = 0;
uint32_t lastBleNotifyMs = 0;
String uartLine;
String lastUartCommand = "";
String lastUartResponse = "";
#if ENABLE_BLE_HUB
NimBLECharacteristic *bleStatusCharacteristic = nullptr;
uint8_t bleClientCount = 0;
String bleAddress = "";
#endif
#if ENABLE_WEB_GUI
WebServer server(80);
DNSServer dns;
Preferences networkPreferences;
bool haveSavedWifi = false;
#endif

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

String statusText(uint8_t status)
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

String sourceText()
{
  if (reading.fromCan) {
    return "CAN";
  }
  if (reading.fromDemo) {
    return "DEMO";
  }
  if (reading.valid) {
    return "ADC";
  }
  return "NONE";
}

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
  Serial2.println(sanitized);
  Serial.printf("UART command sent: %s\n", sanitized.c_str());
#else
  (void)command;
#endif
}

String statusJson()
{
  String json = "{\"valid\":";
  json += reading.valid ? "true" : "false";
  json += ",\"lambda\":";
  json += String(reading.lambda, 3);
  json += ",\"temperature\":";
  json += String(reading.temperatureC);
  json += ",\"status\":\"";
  json += statusText(reading.status);
  json += "\",\"status_code\":";
  json += String(reading.status);
  json += ",\"source\":\"";
  json += sourceText();
  json += "\",\"can_ready\":";
  json += canReady ? "true" : "false";
  json += ",\"age_ms\":";
  json += reading.valid ? String(millis() - reading.receivedMs) : "0";
#if ENABLE_BLE_HUB
  json += ",\"ble_clients\":";
  json += String(bleClientCount);
  json += ",\"ble_name\":\"";
  json += BLE_HUB_NAME;
  json += "\",\"ble_address\":\"";
  json += bleAddress;
  json += "\"";
#endif
#if ENABLE_WEB_GUI
  json += ",\"wifi_saved\":";
  json += haveSavedWifi ? "true" : "false";
  json += ",\"wifi_connected\":";
  json += WiFi.status() == WL_CONNECTED ? "true" : "false";
  json += ",\"wifi_ssid\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  json += "\",\"wifi_ip\":\"";
  json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  json += "\"";
#endif
  json += ",\"uart_command\":\"";
  json += lastUartCommand;
  json += "\",\"uart_response\":\"";
  json += lastUartResponse;
  json += "\"";
  json += "}";
  return json;
}

#if ENABLE_BLE_HUB
String bleStatusPayload()
{
  String payload = "L";
  payload += reading.valid ? String(reading.lambda, 3) : "0.000";
  payload += ",T";
  payload += String(reading.temperatureC);
  payload += ",S";
  payload += String(reading.status);
  return payload;
}

class BleServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer *, NimBLEConnInfo &) override
  {
    bleClientCount++;
    Serial.printf("BLE hub:     client connected, count=%u\n", bleClientCount);
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override
  {
    if (bleClientCount > 0) {
      bleClientCount--;
    }
    Serial.printf("BLE hub:     client disconnected, count=%u\n", bleClientCount);
    NimBLEDevice::startAdvertising();
  }
};

class BleCommandCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override
  {
    sendSpartanUartCommand(characteristic->getValue().c_str());
  }
};

void setupBleHub()
{
  NimBLEDevice::init(BLE_HUB_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  bleAddress = NimBLEDevice::getAddress().toString().c_str();

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
  advertising->start();

  Serial.printf("BLE hub:     '%s' addr=%s service=%s\n",
                BLE_HUB_NAME,
                bleAddress.c_str(),
                kBleServiceUuid);
}

void updateBleHub()
{
  const uint32_t now = millis();
  if (bleStatusCharacteristic == nullptr || now - lastBleNotifyMs < kBleNotifyIntervalMs) {
    return;
  }
  lastBleNotifyMs = now;

  const String payload = bleStatusPayload();
  bleStatusCharacteristic->setValue(payload.c_str());
  if (bleClientCount > 0) {
    bleStatusCharacteristic->notify();
  }
}
#else
void setupBleHub() {}
void updateBleHub() {}
#endif

void setupWebGui()
{
#if ENABLE_WEB_GUI
  networkPreferences.begin("net", false);
  const String savedSsid = networkPreferences.isKey("ssid") ? networkPreferences.getString("ssid", "") : "";
  const String savedPassword = networkPreferences.isKey("pass") ? networkPreferences.getString("pass", "") : "";
  haveSavedWifi = savedSsid.length() > 0;

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
  if (!WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD)) {
    Serial.println("Web GUI:     access point start failed");
    return;
  }

  if (haveSavedWifi) {
    WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
    Serial.printf("Home WiFi:   connecting to '%s'\n", savedSsid.c_str());
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
body { margin: 0; background: #0b1210; color: #e6ede8; }
main { max-width: 460px; margin: auto; padding: 28px 18px; }
h1 { font-size: 1.35rem; color: #9ed85b; margin: 0 0 22px; }
.card { padding: 20px; border: 1px solid #26372e; border-radius: 16px; background: #101a15; }
.lambda { font-size: 3.4rem; font-weight: 700; color: #9ed85b; margin: 4px 0 18px; }
.tag { display: inline-block; padding: 5px 10px; border-radius: 20px; background: #26372e; color: #bde87a; }
.row { display: flex; justify-content: space-between; border-top: 1px solid #26372e; padding: 13px 0; }
.setup { margin-top: 18px; padding: 20px; border: 1px solid #26372e; border-radius: 16px; background: #101a15; }
input { display: block; box-sizing: border-box; width: 100%; margin: 8px 0 13px; padding: 12px; border: 1px solid #35453c; border-radius: 8px; background: #0b1210; color: #e6ede8; }
select { display: block; box-sizing: border-box; width: 100%; margin: 8px 0 13px; padding: 12px; border: 1px solid #35453c; border-radius: 8px; background: #0b1210; color: #e6ede8; }
button { padding: 11px 14px; margin-right: 7px; border: 0; border-radius: 8px; background: #78ad43; color: #081005; font-weight: 700; }
button.secondary { background: #26372e; color: #e6ede8; }
button.danger { background: #8b3c2e; color: #ffe8dc; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.grid button { width: 100%; margin: 0; }
.hint { color: #9ca99f; font-size: .92rem; margin-top: 18px; line-height: 1.45; }
.mono { font-family: Consolas, monospace; color: #cbeaa7; overflow-wrap: anywhere; }
</style>
</head>
<body><main>
<h1>SPARTAN 3 v2 / ESP32 Adapter</h1>
<div class="card">
<span id="source" class="tag">START</span>
<div class="lambda" id="lambda">-.---</div>
<div class="row"><span>Status</span><strong id="status">warte</strong></div>
<div class="row"><span>Sensortemperatur</span><strong id="temp">- C</strong></div>
<div class="row"><span>CAN</span><strong id="can">-</strong></div>
</div>
<div class="setup">
<strong>BLE Hub / Gateway</strong>
<p class="hint">Fuer M5/Waveshare Gateway-Modus. Der M5 soll per Name plus Service UUID scannen; die Adresse ist nur Debug/Fast-Reconnect.</p>
<div class="row"><span>Status</span><strong id="bleenabled">-</strong></div>
<div class="row"><span>Name</span><strong id="blename" class="mono">-</strong></div>
<div class="row"><span>Adresse</span><strong id="bleaddr" class="mono">-</strong></div>
<div class="row"><span>Clients</span><strong id="bleclients">0</strong></div>
<div class="row"><span>Service</span><strong class="mono">7f510001-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
<div class="row"><span>Status Notify</span><strong class="mono">7f510002-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
<div class="row"><span>Command Write</span><strong class="mono">7f510003-5a6b-4d2a-9f20-14a7f3e20000</strong></div>
</div>
<div class="setup">
<strong>Heim-WLAN</strong>
<div class="row"><span>Verbindung</span><strong id="wifi">nicht eingerichtet</strong></div>
<div class="row"><span>IP im Heimnetz</span><strong id="lanip">-</strong></div>
<form action="/wifi" method="post">
<label for="ssid">WLAN-Name</label><input id="ssid" name="ssid" required>
<label for="pass">Passwort</label><input id="pass" name="pass" type="password">
<button type="submit">Speichern &amp; verbinden</button>
</form>
<form action="/wifi_clear" method="post" style="margin-top:12px"><button class="secondary" type="submit">Heim-WLAN loeschen</button></form>
</div>
<div class="setup">
<strong>Spartan UART-Konfiguration</strong>
<p class="hint">Nur benutzen, wenn Orange/Gelb/Grau ueber Pegelwandler mit dem ESP32 verbunden sind. Die Befehle gehen direkt an Spartan UART.</p>
<div class="row"><span>Letzter Befehl</span><strong id="ucmd" class="mono">-</strong></div>
<div class="row"><span>Antwort</span><strong id="uresp" class="mono">-</strong></div>
<form action="/uart_cmd" method="post" class="grid">
<button name="cmd" value="GETFW">GETFW</button>
<button name="cmd" value="GETHW">GETHW</button>
<button name="cmd" value="GETCANID">GETCANID</button>
<button name="cmd" value="GETCANBAUD">GETCANBAUD</button>
<button name="cmd" value="GETCANFORMAT">GETCANFORMAT</button>
<button name="cmd" value="GETCANDR">GETCANDR</button>
<button name="cmd" value="GETCANR">GETCANR</button>
<button name="cmd" value="GETTYPE">GETTYPE</button>
</form>
<form action="/uart_cmd" method="post">
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
<p class="hint">Aktuell ist der Bring-up-Modus aktiv. DEMO-Werte pruefen Display und Web-GUI, bis der CAN-Transceiver angeschlossen ist.</p>
<script>
async function refresh() {
  try {
    const r = await fetch('/state', {cache:'no-store'});
    const d = await r.json();
    document.getElementById('source').textContent = d.source;
    document.getElementById('lambda').textContent = d.valid ? d.lambda.toFixed(3) : '-.---';
    document.getElementById('status').textContent = d.status;
    document.getElementById('temp').textContent = d.valid ? d.temperature + ' C' : '- C';
    document.getElementById('can').textContent = d.can_ready ? 'aktiv' : 'Fehler';
    document.getElementById('bleenabled').textContent = d.ble_name ? 'aktiv' : 'nicht im Build';
    document.getElementById('blename').textContent = d.ble_name || '-';
    document.getElementById('bleaddr').textContent = d.ble_address || '-';
    document.getElementById('bleclients').textContent = d.ble_clients ?? '0';
    document.getElementById('wifi').textContent = d.wifi_connected ? d.wifi_ssid : (d.wifi_saved ? 'verbindet...' : 'nicht eingerichtet');
    document.getElementById('lanip').textContent = d.wifi_connected ? d.wifi_ip : '-';
    document.getElementById('ucmd').textContent = d.uart_command || '-';
    document.getElementById('uresp').textContent = d.uart_response || '-';
  } catch (e) {}
}
refresh();
setInterval(refresh, 350);
</script></main></body></html>)HTML");
  });

  const auto sendStatus = []() {
    server.send(200, "application/json", statusJson());
  };
  server.on("/state", sendStatus);
  server.on("/api/status", sendStatus);
  server.on("/wifi", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    const String password = server.arg("pass");
    ssid.trim();
    if (ssid.length() == 0) {
      server.send(400, "text/plain", "WLAN-Name fehlt.");
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
    WiFi.disconnect();
    Serial.println("Home WiFi:   credentials cleared");
    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "");
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
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  const wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    lastWifiStatus = wifiStatus;
    if (wifiStatus == WL_CONNECTED) {
      Serial.printf("Home WiFi:   connected to '%s', http://%s/\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
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

  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.identifier != SPARTAN_CAN_ID || message.data_length_code < 4) {
      continue;
    }

    const uint16_t rawLambda = (static_cast<uint16_t>(message.data[0]) << 8) | message.data[1];
    reading.lambda = rawLambda / 1000.0f;
    reading.temperatureC = static_cast<uint16_t>(message.data[2]) * 10;
    reading.status = message.data[3];
    reading.receivedMs = millis();
    reading.valid = true;
    reading.fromCan = true;
    reading.fromDemo = false;
    digitalWrite(STATUS_LED_PIN, reading.status == 3 ? HIGH : LOW);
  }
#endif
}

void updateDemo()
{
#if ENABLE_SPARTAN_DEMO
  const uint32_t now = millis();
  if (reading.fromCan && now - reading.receivedMs <= kCanStaleMs) {
    return;
  }

  if (now < 8000) {
    reading.lambda = 1.000f;
    reading.temperatureC = static_cast<uint16_t>(now / 12);
    reading.status = 2;
  } else {
    const int32_t sweep = static_cast<int32_t>((now / 80) % 200) - 100;
    reading.lambda = 1.000f + sweep / 1000.0f;
    reading.temperatureC = 780;
    reading.status = 3;
  }

  reading.receivedMs = now;
  reading.valid = true;
  reading.fromCan = false;
  reading.fromDemo = true;
  digitalWrite(STATUS_LED_PIN, reading.status == 3 ? HIGH : LOW);
#endif
}

void updateAnalog()
{
#if ENABLE_SPARTAN_ANALOG
  if ((reading.fromCan || reading.fromDemo) && millis() - reading.receivedMs <= kCanStaleMs) {
    return;
  }

  const uint32_t adcMilliVolts = analogReadMilliVolts(SPARTAN_ANALOG_PIN);
  const float inputVolts = (adcMilliVolts / 1000.0f) * ANALOG_DIVIDER_NUM / ANALOG_DIVIDER_DEN;
  const float constrainedVolts = constrain(inputVolts, 0.0f, 5.0f);
  reading.lambda = kLambdaAtZeroVolt
      + (kLambdaAtFiveVolt - kLambdaAtZeroVolt) * constrainedVolts / 5.0f;
  reading.temperatureC = 0;
  reading.status = 3;
  reading.receivedMs = millis();
  reading.valid = true;
  reading.fromCan = false;
  reading.fromDemo = false;
#endif
}

void setupUart()
{
#if ENABLE_SPARTAN_UART
  Serial2.begin(9600, SERIAL_8N1, SPARTAN_UART_RX_PIN, SPARTAN_UART_TX_PIN);
  Serial.printf(
      "UART:        9600 8N1 RX=%u TX=%u (configuration only; level shift Spartan TX)\n",
      SPARTAN_UART_RX_PIN,
      SPARTAN_UART_TX_PIN);
  Serial.println("UART bridge: type >GETFW or >GETCANID in USB serial monitor");
#endif
}

void updateUart()
{
#if ENABLE_SPARTAN_UART
  while (Serial2.available()) {
    const char c = static_cast<char>(Serial2.read());
    Serial.write(c);
    if (c == '\r' || c == '\n') {
      lastUartResponse.trim();
    } else if (lastUartResponse.length() < 96 && isPrintable(c)) {
      lastUartResponse += c;
    }
  }

  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (uartLine.startsWith(">") && uartLine.length() > 1) {
        sendSpartanUartCommand(uartLine.substring(1));
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

  const bool stale = !reading.valid || (reading.fromCan && now - reading.receivedMs > kCanStaleMs);
  if (stale) {
    displayLine(0, "SPARTAN CAN");
    displayLine(1, canReady ? "warte Daten..." : "CAN FEHLER");
    return;
  }

  char lambdaLine[24];
  snprintf(lambdaLine, sizeof(lambdaLine), "LAM %.3f %s", reading.lambda, sourceText().c_str());
  displayLine(0, lambdaLine);

  if (reading.fromCan || reading.fromDemo) {
    char stateLine[24];
    snprintf(stateLine, sizeof(stateLine), "%uC %s", reading.temperatureC, statusText(reading.status).c_str());
    displayLine(1, stateLine);
  } else {
    displayLine(1, "Analog fallback");
  }
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
}

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
  setupCan();
  setupUart();
#if ENABLE_SPARTAN_ANALOG
  analogSetPinAttenuation(SPARTAN_ANALOG_PIN, ADC_11db);
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
  updateUart();
  updateWebGui();
  updateBleHub();
  updateDisplay();
  delay(5);
}
