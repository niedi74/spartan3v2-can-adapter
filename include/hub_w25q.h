#pragma once
// [W25Q] Externer W25Q128 SPI-Flash: Erkennung (JEDEC) + Roh-Zugriff Sektor 0
// fuer das Config-Backup. 1:1 aus main.cpp ausgelagert (gleiche Translation-Unit,
// Include an der Originalstelle) -- keine Logikaenderung.

// --- Externer W25Q128 SPI-Flash (Stufe 1, Schritt 1: nur Erkennung) ---
#define W25Q_CS_PIN   13
#define W25Q_CLK_PIN  14
#define W25Q_DI_PIN   15   // MOSI
#define W25Q_DO_PIN   18   // MISO
SPIClass w25qSpi(FSPI);

// JEDEC-ID (0x9F) lesen: [Hersteller][Typ][Kapazitaet]. W25Q128 = EF 40 18 (16 MB).
void detectW25Q()
{
  pinMode(W25Q_CS_PIN, OUTPUT);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.begin(W25Q_CLK_PIN, W25Q_DO_PIN, W25Q_DI_PIN, W25Q_CS_PIN);
  delay(2);
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(0x9F);
  const uint8_t mfg = w25qSpi.transfer(0x00);
  const uint8_t typ = w25qSpi.transfer(0x00);
  const uint8_t cap = w25qSpi.transfer(0x00);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
  w25qJedecId = ((uint32_t)mfg << 16) | ((uint32_t)typ << 8) | cap;
  w25qDetected = (mfg != 0x00 && mfg != 0xFF && cap >= 0x14 && cap <= 0x1A);
  if (w25qDetected) {
    w25qSizeMB = (1UL << cap) / (1024UL * 1024UL);   // 0x18 -> 16 MB
    w25qMfg = (mfg == 0xEF) ? "Winbond" : (mfg == 0xC8) ? "GigaDevice"
            : (mfg == 0x68) ? "Boya" : "andere";
  } else {
    w25qSizeMB = 0;
    w25qMfg = "-";
  }
  Serial.printf("W25Q:        JEDEC=0x%06X detected=%d %s %u MB\n",
                w25qJedecId, w25qDetected ? 1 : 0, w25qMfg, w25qSizeMB);
}

// --- W25Q Roh-Zugriff fuer Config-Backup (fester Sektor 0, kein Dateisystem) ---
static void w25qBusyWait()
{
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(0x05);                       // Read Status Register-1
  const uint32_t t0 = millis();
  while ((w25qSpi.transfer(0x00) & 0x01) && (millis() - t0 < 600)) { /* WIP */ }
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
}
static void w25qSimpleCmd(uint8_t cmd)          // z.B. WREN 0x06
{
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(cmd);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
}
static void w25qReadData(uint32_t addr, uint8_t *buf, size_t len)
{
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(0x03);                        // Read Data
  w25qSpi.transfer((addr >> 16) & 0xFF);
  w25qSpi.transfer((addr >> 8) & 0xFF);
  w25qSpi.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; i++) buf[i] = w25qSpi.transfer(0x00);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
}
static void w25qSectorErase(uint32_t addr)       // 4 KB
{
  w25qSimpleCmd(0x06);                           // WREN
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(0x20);
  w25qSpi.transfer((addr >> 16) & 0xFF);
  w25qSpi.transfer((addr >> 8) & 0xFF);
  w25qSpi.transfer(addr & 0xFF);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
  w25qBusyWait();
}
static void w25qPageProgram(uint32_t addr, const uint8_t *buf, size_t len)
{
  w25qSimpleCmd(0x06);                           // WREN
  w25qSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W25Q_CS_PIN, LOW);
  w25qSpi.transfer(0x02);
  w25qSpi.transfer((addr >> 16) & 0xFF);
  w25qSpi.transfer((addr >> 8) & 0xFF);
  w25qSpi.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; i++) w25qSpi.transfer(buf[i]);
  digitalWrite(W25Q_CS_PIN, HIGH);
  w25qSpi.endTransaction();
  w25qBusyWait();
}
static void w25qWriteData(uint32_t addr, const uint8_t *buf, size_t len)
{
  size_t off = 0;                                // an 256-Byte-Page-Grenzen splitten
  while (off < len) {
    const uint32_t a = addr + off;
    const size_t pageRem = 256 - (a & 0xFF);
    const size_t chunk = (len - off < pageRem) ? (len - off) : pageRem;
    w25qPageProgram(a, buf + off, chunk);
    off += chunk;
  }
}
static uint32_t hubCfgCrc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320UL & (~(crc & 1) + 1));
  }
  return ~crc;
}

#define HUB_CFG_MAGIC   0x48424346UL   // 'HBCF'
#define HUB_CFG_VERSION 3              // v3: + Betriebsstunden/Odometer (v2: effektive Globals)
#define HUB_CFG_ADDR    0x000000UL     // Sektor 0 des W25Q

#pragma pack(push, 1)
struct HubConfigBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char     p1_ssid[33]; char p1_pass[65];
  char     p2_ssid[33]; char p2_pass[65];
  uint8_t  wifi_prof;
  char     legacy_ssid[33]; char legacy_pass[65];
  char     ap_ssid[33]; char ap_pass[65]; char ap_ip[16];
  uint8_t  ap_chan;
  char     mdns_host[33];
  // [v3] Betriebsstunden + Odometer: ueberleben so auch ein erase_flash (NVS weg).
  // Stand vom letzten Backup-Trigger (manuelles Setzen/Config-Aenderung), kein Zyklus-Write.
  uint64_t dev_sec; uint64_t eng_sec; uint64_t sns_sec;
  uint64_t odo_mm;  uint64_t trip_mm;
  uint32_t crc32;
};
#pragma pack(pop)

// Aktuelle Config (aus NVS) als Blob auf den W25Q schreiben. NVS ist die Quelle der
// Wahrheit; die Endpoints haben vorher schon ins NVS geschrieben.
void saveConfigToW25Q()
{
  if (!w25qDetected) return;
  HubConfigBlob b;
  memset(&b, 0, sizeof(b));
  b.magic = HUB_CFG_MAGIC; b.version = HUB_CFG_VERSION; b.size = sizeof(b);
  // Quelle: die EFFEKTIVEN Laufzeit-Globals (inkl. HOME_WIFI_*-Build-Defaults), NICHT die
  // rohen NVS-Keys - sonst landet ein leeres Passwort im Backup, wenn das echte Passwort
  // aus dem Default kommt statt aus einem gespeicherten p1_pass.
  strlcpy(b.p1_ssid, g_hubWifiProfiles[1].ssid, sizeof(b.p1_ssid));
  strlcpy(b.p1_pass, g_hubWifiProfiles[1].pass, sizeof(b.p1_pass));
  strlcpy(b.p2_ssid, g_hubWifiProfiles[2].ssid, sizeof(b.p2_ssid));
  strlcpy(b.p2_pass, g_hubWifiProfiles[2].pass, sizeof(b.p2_pass));
  b.wifi_prof = hubWifiProfile;
  strlcpy(b.legacy_ssid, networkPreferences.getString("ssid", "").c_str(), sizeof(b.legacy_ssid));
  strlcpy(b.legacy_pass, networkPreferences.getString("pass", "").c_str(), sizeof(b.legacy_pass));
  strlcpy(b.ap_ssid, hubApSsid.c_str(), sizeof(b.ap_ssid));
  strlcpy(b.ap_pass, hubApPassword.c_str(), sizeof(b.ap_pass));
  strlcpy(b.ap_ip, hubApIp.c_str(), sizeof(b.ap_ip));
  b.ap_chan = hubApChannel;
  strlcpy(b.mdns_host, hubHostname.c_str(), sizeof(b.mdns_host));
  b.dev_sec = deviceSeconds; b.eng_sec = engineSeconds; b.sns_sec = sensorSeconds;  // [v3]
  b.odo_mm = odoMm; b.trip_mm = tripMm;
  b.crc32 = hubCfgCrc32(reinterpret_cast<const uint8_t *>(&b), sizeof(b) - sizeof(b.crc32));
  w25qSectorErase(HUB_CFG_ADDR);
  w25qWriteData(HUB_CFG_ADDR, reinterpret_cast<const uint8_t *>(&b), sizeof(b));
  networkPreferences.putUChar("cfg_w25q", HUB_CFG_VERSION);   // NVS gilt jetzt als initialisiert
  Serial.printf("Config:      Backup auf W25Q geschrieben (Profil %d, SSID1 '%s')\n",
                b.wifi_prof, b.p1_ssid);
}

// Bei frischem NVS (kein cfg_w25q-Marker, z.B. nach erase_flash) die Config aus dem
// W25Q-Backup ins NVS zurueckschreiben. Laeuft VOR dem Einlesen der NVS-Werte.
void restoreConfigFromW25Q()
{
  if (!w25qDetected) return;
  if (networkPreferences.isKey("cfg_w25q")) return;   // NVS schon initialisiert -> kein Restore
  HubConfigBlob b;
  w25qReadData(HUB_CFG_ADDR, reinterpret_cast<uint8_t *>(&b), sizeof(b));
  if (b.magic != HUB_CFG_MAGIC || b.version != HUB_CFG_VERSION || b.size != sizeof(b)) return;
  const uint32_t crc = hubCfgCrc32(reinterpret_cast<const uint8_t *>(&b), sizeof(b) - sizeof(b.crc32));
  if (crc != b.crc32) { Serial.println("Config:      W25Q-Backup CRC ungueltig - kein Restore"); return; }
  b.p1_ssid[sizeof(b.p1_ssid) - 1] = 0; b.p1_pass[sizeof(b.p1_pass) - 1] = 0;
  b.p2_ssid[sizeof(b.p2_ssid) - 1] = 0; b.p2_pass[sizeof(b.p2_pass) - 1] = 0;
  b.legacy_ssid[sizeof(b.legacy_ssid) - 1] = 0; b.legacy_pass[sizeof(b.legacy_pass) - 1] = 0;
  b.ap_ssid[sizeof(b.ap_ssid) - 1] = 0; b.ap_pass[sizeof(b.ap_pass) - 1] = 0;
  b.ap_ip[sizeof(b.ap_ip) - 1] = 0; b.mdns_host[sizeof(b.mdns_host) - 1] = 0;
  networkPreferences.putString("p1_ssid", b.p1_ssid);
  if (b.p1_pass[0]) networkPreferences.putString("p1_pass", b.p1_pass);  // leeres PW nie ueberschreiben -> Default greift
  networkPreferences.putString("p2_ssid", b.p2_ssid);
  if (b.p2_pass[0]) networkPreferences.putString("p2_pass", b.p2_pass);
  networkPreferences.putUChar("wifi_prof", b.wifi_prof);
  if (b.legacy_ssid[0]) networkPreferences.putString("ssid", b.legacy_ssid);
  if (b.legacy_pass[0]) networkPreferences.putString("pass", b.legacy_pass);
  networkPreferences.putString("ap_ssid", b.ap_ssid);
  networkPreferences.putString("ap_pass", b.ap_pass);
  networkPreferences.putString("ap_ip", b.ap_ip);
  networkPreferences.putUChar("ap_chan", b.ap_chan);
  networkPreferences.putString("mdns_host", b.mdns_host);
  // [v3] Betriebsstunden + Odometer zurueck ins NVS (loadHourmeters/setupSpeedReed lesen spaeter)
  networkPreferences.putULong64("dev_sec", b.dev_sec);
  networkPreferences.putULong64("eng_sec", b.eng_sec);
  networkPreferences.putULong64("sns_sec", b.sns_sec);
  networkPreferences.putULong64("odo_mm", b.odo_mm);
  networkPreferences.putULong64("trip_mm", b.trip_mm);
  networkPreferences.putUChar("cfg_w25q", HUB_CFG_VERSION);
  Serial.printf("Config:      aus W25Q-Backup wiederhergestellt (Profil %d, SSID1 '%s', %.1f Geraete-h, %.1f km)\n",
                b.wifi_prof, b.p1_ssid,
                static_cast<double>(b.dev_sec) / 3600.0,
                static_cast<double>(b.odo_mm) / 1000000.0);
}
