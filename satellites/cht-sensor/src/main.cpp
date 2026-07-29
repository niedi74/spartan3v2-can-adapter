// [CHT-SENSOR] ESP32-C6: 4x MAX6675 Zylinderkopftemperatur (M10x1-Original-
// bohrungen nahe Zuendkerze, VW T3 Boxer). WLAN AP+STA (Muster aus dem Hub-
// Projekt: DHCP-Reset-Fix, Static-IP optional). CAN-TX ist vorbereitet aber
// per Compile-Flag ENABLE_CAN=0 deaktiviert, bis der SN65HVD230-Transceiver
// verbaut ist -- dann nur ENABLE_CAN=1 setzen und CAN_TX/RX_PIN verkabeln.
//
// Pin-Plan (siehe Chat-Uebergabe):
//   MAX6675 SCK=6 (geteilt), SO=7 (geteilt)
//   CS Zylinder 1=10, 2=11, 3=18, 4=19
//   CAN (vorbereitet, noch inaktiv): TX=2, RX=3

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_wifi.h>

#ifndef ENABLE_CAN
#define ENABLE_CAN 0
#endif
#if ENABLE_CAN
#include "driver/twai.h"
#endif

// ---------- Pins ----------
constexpr int kMaxSckPin = 6;
constexpr int kMaxSoPin  = 7;
constexpr int kCsPins[4] = {10, 11, 18, 19};
constexpr int kCanTxPin  = 2;
constexpr int kCanRxPin  = 3;

// ---------- WLAN (Muster aus spartan3v2-can-adapter) ----------
// [AP-CFG] Default nur als Fallback -- SSID/Passwort/IP-Range sind ab jetzt in
// der WebGUI aenderbar (NVS-persistiert), analog Hub-Projekt Dev->AccessPoint.
constexpr const char *kApSsidDefault = "CHT-Sensor";
constexpr const char *kApPasswordDefault = "chtboxer1";
IPAddress kApIpDefault(192, 168, 7, 1);   // eigenes, kollisionsfreies Subnetz
                                          // (Live-Hub=4.x, Test-Hub=5.x, Reed-Sim=6.x)
String apSsid, apPassword;
IPAddress apIp;

WebServer server(80);
Preferences prefs;

String staSsid, staPass;
uint8_t staIpMode = 0;   // 0=DHCP, 1=statisch
IPAddress staIp, staGw, staMask;
uint32_t staConnectStartMs = 0;
constexpr uint32_t kStaConnectTimeoutMs = 20000;
bool staWasConnected = false;
String macOverride;   // leer = Werks-MAC, sonst "AA:BB:CC:DD:EE:FF"

bool parseMac(const String &s, uint8_t out[6])
{
  if (s.length() != 17) return false;
  int vals[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = static_cast<uint8_t>(vals[i]);
  return true;
}

void applyMacOverrideIfSet()
{
  if (macOverride.length() == 0) return;
  uint8_t mac[6];
  if (parseMac(macOverride, mac)) {
    esp_wifi_set_mac(WIFI_IF_STA, mac);
    Serial.printf("WiFi:        MAC-Override aktiv: %s\n", macOverride.c_str());
  } else {
    Serial.printf("WiFi:        MAC-Override ungueltig, ignoriert: '%s'\n", macOverride.c_str());
  }
}

// ---------- CHT-Werte ----------
float chtC[4] = {NAN, NAN, NAN, NAN};
bool  chtFault[4] = {true, true, true, true};
uint32_t lastReadMs = 0;
constexpr uint32_t kReadIntervalMs = 500;

// [MAX6675] Bit-Bang-Read: 16 Bit, MSB zuerst. Bit15=Dummy(0), Bit14-3=Temp
// (0.25 Grad/LSB), Bit2=Open-Thermocouple-Fehler, Bit1=Device-ID, Bit0=Zustand.
float readMax6675(int csPin, bool *fault)
{
  digitalWrite(csPin, LOW);
  delayMicroseconds(10);
  uint16_t raw = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(kMaxSckPin, HIGH);
    delayMicroseconds(2);
    raw = (raw << 1) | digitalRead(kMaxSoPin);
    digitalWrite(kMaxSckPin, LOW);
    delayMicroseconds(2);
  }
  digitalWrite(csPin, HIGH);

  *fault = (raw & 0x0004) != 0;   // Bit2 = Fühler offen/nicht angeschlossen
  if (*fault) return NAN;
  const uint16_t tempBits = (raw >> 3) & 0x0FFF;
  return tempBits * 0.25f;
}

void readAllCht()
{
  for (int i = 0; i < 4; i++) {
    chtC[i] = readMax6675(kCsPins[i], &chtFault[i]);
  }
}

// ---------- WLAN-Verbindung (Muster: DHCP-Reset-Fix aus dem Hub-Projekt) ----------
void applyStaticIpIfNeeded()
{
  if (staIpMode == 1 && staIp != IPAddress(0, 0, 0, 0)) {
    WiFi.config(staIp, staGw, staMask);
  } else {
    // [WIFI-STATIC-FIX] Explizit zuruecksetzen -- sonst haengt eine vorherige
    // statische IP am Treiber, selbst wenn das neue Profil DHCP will.
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }
}

void connectSta()
{
  if (staSsid.length() == 0) return;
  applyStaticIpIfNeeded();
  WiFi.begin(staSsid.c_str(), staPass.c_str());
  staConnectStartMs = millis();
  Serial.printf("WiFi STA:    verbinde mit '%s'...\n", staSsid.c_str());
}

void loadWifiConfig()
{
  prefs.begin("cht", true);
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("pass", "");
  staIpMode = prefs.getUChar("ip_mode", 0);
  staIp.fromString(prefs.getString("ip", ""));
  staGw.fromString(prefs.getString("gw", ""));
  staMask.fromString(prefs.getString("mask", "255.255.255.0"));
  macOverride = prefs.getString("mac_ovr", "");
  apSsid = prefs.getString("ap_ssid", kApSsidDefault);
  apPassword = prefs.getString("ap_pass", kApPasswordDefault);
  if (!apIp.fromString(prefs.getString("ap_ip", ""))) apIp = kApIpDefault;
  prefs.end();
}

void saveWifiConfig()
{
  prefs.begin("cht", false);
  prefs.putString("ssid", staSsid);
  prefs.putString("pass", staPass);
  prefs.putUChar("ip_mode", staIpMode);
  prefs.putString("ip", staIp.toString());
  prefs.putString("gw", staGw.toString());
  prefs.putString("mask", staMask.toString());
  prefs.putString("mac_ovr", macOverride);
  prefs.end();
}

// ---------- CAN (vorbereitet, inaktiv bis ENABLE_CAN=1) ----------
#if ENABLE_CAN
bool canReady = false;
uint32_t lastCanTxMs = 0;
constexpr uint32_t kCanTxIntervalMs = 200;   // 5 Hz reicht fuer Temperaturen
constexpr uint32_t kCanId = 0x520;           // Frame: 4x int16 CHT x10, big-endian

void setupCan()
{
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(kCanTxPin), static_cast<gpio_num_t>(kCanRxPin), TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) == ESP_OK && twai_start() == ESP_OK) {
    canReady = true;
    Serial.println("CAN:         gestartet, 500 kbit/s, ID 0x520");
  } else {
    Serial.println("CAN:         Start fehlgeschlagen");
  }
}

void updateCanTx()
{
  if (!canReady) return;
  const uint32_t now = millis();
  if (now - lastCanTxMs < kCanTxIntervalMs) return;
  lastCanTxMs = now;

  twai_message_t tx = {};
  tx.identifier = kCanId;
  tx.data_length_code = 8;
  for (int i = 0; i < 4; i++) {
    const int16_t val = chtFault[i] ? INT16_MIN : static_cast<int16_t>(chtC[i] * 10.0f + 0.5f);
    tx.data[i * 2]     = static_cast<uint8_t>((val >> 8) & 0xFF);
    tx.data[i * 2 + 1] = static_cast<uint8_t>(val & 0xFF);
  }
  twai_transmit(&tx, pdMS_TO_TICKS(5));
}
#endif

// ---------- WebGUI ----------
String statusJson()
{
  String j = "{";
  j += "\"cht\":[";
  for (int i = 0; i < 4; i++) {
    if (i) j += ",";
    j += "{\"c\":";
    j += isnan(chtC[i]) ? "null" : String(chtC[i], 1);
    j += ",\"fault\":";
    j += chtFault[i] ? "true" : "false";
    j += "}";
  }
  j += "],";
  j += "\"ap_ssid\":\"" + apSsid + "\",";
  j += "\"ap_ip\":\"" + apIp.toString() + "\",";
  j += "\"sta_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  j += "\"sta_ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("-")) + "\",";
  j += "\"sta_ssid\":\"" + staSsid + "\",";
  j += "\"wifi_mac\":\"" + WiFi.macAddress() + "\",";
  j += "\"mac_override\":\"" + macOverride + "\",";
  j += "\"can_enabled\":" + String(ENABLE_CAN ? "true" : "false");
  j += "}";
  return j;
}

const char kPage[] PROGMEM = R"HTML(
<!doctype html><html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CHT-Sensor</title>
<style>
body{font-family:Arial;background:#0b1210;color:#e6ede8;margin:0;padding:16px}
.card{border:1px solid #26372e;border-radius:10px;padding:14px;margin-bottom:12px;background:#101a15}
.temp{font-size:2rem;font-weight:700;color:#9ed85b}
.temp.fault{color:#e08a4a;font-size:1.1rem}
.row{display:flex;justify-content:space-between;border-top:1px solid #26372e;padding:8px 0}
input,select{width:100%;box-sizing:border-box;padding:8px;margin:6px 0;background:#0b1210;border:1px solid #35453c;color:#e6ede8;border-radius:6px}
button{padding:10px 14px;background:#78ad43;border:0;border-radius:6px;color:#081005;font-weight:700}
h2{color:#9ed85b}
.tabs{display:flex;gap:6px;margin-bottom:12px}
.tab{flex:1;padding:10px;text-align:center;background:#101a15;border:1px solid #26372e;border-radius:8px;color:#e6ede8;cursor:pointer}
.tab.on{background:#26372e;color:#9ed85b;font-weight:700}
.tabsec[hidden]{display:none}
table{width:100%;border-collapse:collapse;font-size:.92rem}
th{text-align:left;color:#9ed85b;padding:6px 4px}
td{padding:6px 4px;border-top:1px solid #26372e}
.hint{color:#8a998f;font-size:.85rem}
</style></head><body>
<h2>Zylinderkopftemperatur</h2>
<div class="tabs">
<button type="button" class="tab on" id="tabLive" onclick="showTab('live')">Live</button>
<button type="button" class="tab" id="tabPlan" onclick="showTab('plan')">Anschlussplan</button>
</div>
<div class="tabsec" data-tab="live">
<div id="cyls"></div>
<div class="card">
<div class="row"><span>WLAN</span><strong id="wifi">-</strong></div>
<div class="row"><span>CAN</span><strong id="can">-</strong></div>
</div>
<div class="card">
<form id="wf">
<label>Heimnetz-SSID</label><input name="ssid" id="ssid">
<label>Passwort</label><input name="pass" id="pass" type="password">
<label>IP-Modus</label><select name="ip_mode" id="ip_mode"><option value="0">DHCP</option><option value="1">Statisch</option></select>
<label>Statische IP</label><input name="ip" id="ip" placeholder="192.168.0.xx">
<label>Gateway</label><input name="gw" id="gw" placeholder="192.168.0.1">
<label>MAC-Override (leer = Werks-MAC)</label><input name="mac_ovr" id="mac_ovr" placeholder="AA:BB:CC:DD:EE:FF">
<button type="submit">Speichern &amp; verbinden</button>
</form>
<div class="row"><span>Aktuelle MAC</span><strong id="wifimac">-</strong></div>
</div>
</div>
<div class="tabsec" data-tab="plan" hidden>
<div class="card">
<p class="hint">ESP32-C6, 4x MAX6675 (SPI-Read-only, kein MOSI noetig). SCK+SO geteilt,
je Fuehler eine eigene CS-Leitung. Aktuell 2 von 4 Kanaelen bestueckt.</p>
<table>
<tr><th>Funktion</th><th>GPIO</th><th>Hinweis</th></tr>
<tr><td>MAX6675 SCK</td><td><strong>6</strong></td><td>geteilt, alle 4 Fuehler</td></tr>
<tr><td>MAX6675 SO</td><td><strong>7</strong></td><td>geteilt, alle 4 Fuehler</td></tr>
<tr><td>CS Zylinder 1</td><td><strong>10</strong></td><td>aktuell bestueckt</td></tr>
<tr><td>CS Zylinder 2</td><td><strong>11</strong></td><td>aktuell bestueckt</td></tr>
<tr><td>CS Zylinder 3</td><td><strong>18</strong></td><td>frei / spaeter</td></tr>
<tr><td>CS Zylinder 4</td><td><strong>19</strong></td><td>frei / spaeter</td></tr>
<tr><td>CAN TX</td><td><strong>2</strong></td><td>vorbereitet, Transceiver fehlt noch</td></tr>
<tr><td>CAN RX</td><td><strong>3</strong></td><td>vorbereitet, Transceiver fehlt noch</td></tr>
</table>
</div>
<div class="card">
<div class="row"><span>Versorgung</span><strong>3,3V vom C6-Board (MAX6675-Module sind 3,3V)</strong></div>
<div class="row"><span>CAN (spaeter)</span><strong>SN65HVD230-Transceiver noetig, 500 kbit/s, ID 0x520</strong></div>
<div class="row"><span>AP aktuell</span><strong id="apInfo">-</strong></div>
</div>
<div class="card">
<form id="apf">
<label>AP-SSID</label><input name="ssid" id="apSsid">
<label>AP-Passwort (leer = offen, sonst &ge;8 Zeichen)</label><input name="pass" id="apPass">
<label>AP-IP / Gateway (Range)</label><input name="ip" id="apIp" placeholder="192.168.7.1">
<button type="submit">AP-Einstellungen speichern &amp; neustarten</button>
</form>
</div>
</div>
<script>
function showTab(name){
  document.querySelectorAll('.tabsec').forEach(s=>{s.hidden=s.dataset.tab!==name;});
  document.getElementById('tabLive').classList.toggle('on',name==='live');
  document.getElementById('tabPlan').classList.toggle('on',name==='plan');
}
let loaded=false;
async function refresh(){
  try{
    const d=await(await fetch('/api/status',{cache:'no-store'})).json();
    let html='';
    for(let i=0;i<4;i++){
      const c=d.cht[i];
      html+='<div class="card"><div class="row"><span>Zylinder '+(i+1)+'</span>'+
        (c.fault?'<span class="temp fault">Fühler offen/kein Kontakt</span>':'<span class="temp">'+c.c.toFixed(1)+' °C</span>')+
        '</div></div>';
    }
    document.getElementById('cyls').innerHTML=html;
    document.getElementById('wifi').textContent=d.sta_connected?('verbunden, '+d.sta_ip):'nicht verbunden (AP: '+d.ap_ip+')';
    document.getElementById('can').textContent=d.can_enabled?'aktiv (0x520)':'deaktiviert (Transceiver fehlt)';
    document.getElementById('wifimac').textContent=d.wifi_mac||'-';
    document.getElementById('apInfo').textContent=(d.ap_ssid||'-')+' @ '+(d.ap_ip||'-');
    if(!loaded){
      loaded=true;
      document.getElementById('ssid').value=d.sta_ssid||'';
      document.getElementById('mac_ovr').value=d.mac_override||'';
      document.getElementById('apSsid').value=d.ap_ssid||'';
      document.getElementById('apIp').value=d.ap_ip||'';
    }
  }catch(e){}
}
document.getElementById('wf').addEventListener('submit', async (e)=>{
  e.preventDefault();
  const fd=new FormData(e.target);
  await fetch('/wifi_save',{method:'POST',body:new URLSearchParams(fd)});
  alert('Gespeichert, Gerät verbindet neu.');
});
document.getElementById('apf').addEventListener('submit', async (e)=>{
  e.preventDefault();
  const fd=new FormData(e.target);
  await fetch('/ap_save',{method:'POST',body:new URLSearchParams(fd)});
  alert('AP gespeichert, Gerät startet neu -- neue SSID/IP danach verbinden.');
});
refresh(); setInterval(refresh, 1000);
</script></body></html>
)HTML";

void setupWebGui()
{
  // [ROUTING] /head zusaetzlich zu / -- fuer den Fall, dass mehrere Sensor-
  // Geraete hinter einem gemeinsamen Zugang mit Pfad pro Geraet organisiert
  // werden (analog anderer Projekte), bleibt / aber ebenfalls funktionsfaehig.
  server.on("/", []() { server.send_P(200, "text/html", kPage); });
  server.on("/head", []() { server.send_P(200, "text/html", kPage); });
  server.on("/api/status", []() { server.send(200, "application/json", statusJson()); });
  server.on("/wifi_save", HTTP_POST, []() {
    staSsid = server.arg("ssid");
    staPass = server.arg("pass");
    staIpMode = server.arg("ip_mode").toInt();
    if (staIpMode == 1) {
      staIp.fromString(server.arg("ip"));
      staGw.fromString(server.arg("gw"));
      staMask = IPAddress(255, 255, 255, 0);
    }
    macOverride = server.arg("mac_ovr");
    saveWifiConfig();
    server.send(200, "text/plain", "OK, Geraet startet neu");
    delay(300);
    ESP.restart();   // MAC-Override greift nur ab einem sauberen WiFi-Neustart
  });
  server.on("/ap_save", HTTP_POST, []() {
    const String newSsid = server.arg("ssid");
    const String newPass = server.arg("pass");
    IPAddress newIp;
    if (newSsid.length() == 0 || (newPass.length() > 0 && newPass.length() < 8) ||
        !newIp.fromString(server.arg("ip"))) {
      server.send(400, "text/plain", "Ungueltig: SSID Pflicht, Passwort leer oder >=8 Zeichen, IP muss gueltig sein");
      return;
    }
    apSsid = newSsid;
    apPassword = newPass;
    apIp = newIp;
    prefs.begin("cht", false);
    prefs.putString("ap_ssid", apSsid);
    prefs.putString("ap_pass", apPassword);
    prefs.putString("ap_ip", apIp.toString());
    prefs.end();
    server.send(200, "text/plain", "OK, Geraet startet neu");
    delay(300);
    ESP.restart();
  });
  server.begin();
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CHT-Sensor (ESP32-C6, 4x MAX6675) ===");

  pinMode(kMaxSckPin, OUTPUT);
  pinMode(kMaxSoPin, INPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(kCsPins[i], OUTPUT);
    digitalWrite(kCsPins[i], HIGH);
  }
  digitalWrite(kMaxSckPin, LOW);

  loadWifiConfig();

  WiFi.mode(WIFI_AP_STA);
  applyMacOverrideIfSet();
  WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSsid.c_str(), apPassword.length() > 0 ? apPassword.c_str() : nullptr);
  Serial.printf("WiFi AP:     '%s' -> http://%s/\n", apSsid.c_str(), apIp.toString().c_str());

  if (staSsid.length() > 0) connectSta();

  setupWebGui();

#if ENABLE_CAN
  setupCan();
#else
  Serial.println("CAN:         deaktiviert (ENABLE_CAN=0, Transceiver noch nicht verbaut)");
#endif
}

void loop()
{
  server.handleClient();

  const uint32_t now = millis();
  if (now - lastReadMs >= kReadIntervalMs) {
    lastReadMs = now;
    readAllCht();
  }

  // STA-Reconnect-Timeout: kein Dauerfeuer bei falschem Passwort/Netz nicht da
  if (WiFi.status() != WL_CONNECTED && staSsid.length() > 0 && staConnectStartMs != 0) {
    if (WiFi.status() == WL_CONNECTED) {
      staConnectStartMs = 0;
    } else if (now - staConnectStartMs > kStaConnectTimeoutMs) {
      staConnectStartMs = 0;   // Aufgeben, AP bleibt in jedem Fall erreichbar
    }
  }

#if ENABLE_CAN
  updateCanTx();
#endif
}
