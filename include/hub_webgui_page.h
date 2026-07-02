#pragma once
// [WEBGUI-PAGE] Komplette Index-Seite (HTML/CSS/JS) der Hub-WebGUI als ein
// PROGMEM-Raw-Literal. 1:1 aus setupWebGui() ausgelagert -- keine Logikaenderung.
#if ENABLE_WEB_GUI
static const char kHubIndexHtml[] PROGMEM = R"HTML(
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
input[type="checkbox"] { display: inline-block; width: auto; min-height: 0; margin: 0 10px 0 0; transform: scale(1.25); }
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
.metrics { display: grid; grid-template-columns: repeat(auto-fit,minmax(150px,1fr)); gap: 10px; margin: 12px 0 4px; }
.metric { min-height: 64px; padding: 12px; border: 1px solid #26372e; border-radius: 8px; background: #0d1712; }
.metric span { display: block; color: #9ca99f; font-size: .78rem; }
.metric strong { display: block; margin-top: 5px; font-size: 1.35rem; color: #e6ede8; overflow-wrap: anywhere; }
.metric.wide { grid-column: span 2; }
.file { width: 100%; padding: 12px; background: #0b1210; border: 1px solid #35453c; border-radius: 8px; color: #e6ede8; }
.ota-track { height: 10px; margin-top: 10px; background: #1b2922; border: 1px solid #35453c; border-radius: 999px; overflow: hidden; }
.ota-bar { height: 100%; width: 0; background: #9ed85b; transition: width .2s; }
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
/* Querformat (kurze Hoehe, z.B. Handy in Auto-Halterung): kompakt + volle Breite,
   Kacheln fliessen in viele Spalten -> alles ohne Scrollen sichtbar. */
@media (orientation: landscape) and (max-height: 600px) {
  main { max-width: 100%; padding: 8px 16px 16px; }
  h1 { font-size: 1rem; margin: 2px 0 8px; }
  .tabs { gap: 6px; margin: 2px 0 8px; padding-top: 3px; padding-bottom: 3px; }
  .tab { padding: 8px 12px; }
  .card { padding: 10px; }
  #featBadges { display: none; }
  .lambda { font-size: 1.5rem; margin: 0; display: inline-block; }
  .topline { margin-bottom: 4px; }
  .metrics { grid-template-columns: repeat(auto-fit,minmax(135px,1fr)); gap: 6px; margin: 6px 0 4px; }
  .metric { min-height: 0; padding: 6px 9px; }
  .metric span { font-size: .68rem; }
  .metric strong { font-size: 1rem; margin-top: 2px; }
  .metric.wide { grid-column: auto; }
}
/* 123-Cockpit-Tab: VDO-Look wie die 123TUNE+ App (schwarz, Chrom-Gauges) */
.g123-card { background: #000; border-color: #1c1c1c; }
.g123-clock { margin: 0 auto 14px; width: 120px; text-align: center; background: #181818; border: 2px solid #5a5a5a; border-radius: 8px; padding: 3px 0; color: #d6d6d6; font-family: 'Courier New', monospace; font-size: 1.5rem; letter-spacing: 3px; }
.g123-gauges { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; justify-items: center; align-items: center; }
.g123-gauges .big { grid-column: 1 / -1; margin: 4px 0; }
.g123-gv { width: 100%; max-width: 165px; height: auto; display: block; }
.g123-gv.big { max-width: 300px; }
.g123-tunebar { display: flex; align-items: center; justify-content: center; gap: 14px; margin: 4px 0 2px; }
.g123-lock { width: 40px; height: 40px; border-radius: 50%; background: #1a1a1a; border: 1px solid #333; display: flex; align-items: center; justify-content: center; }
.g123-lockico { width: 11px; height: 9px; border: 2px solid #cfcfcf; border-radius: 0 0 2px 2px; position: relative; margin-top: 5px; }
.g123-lockico::before { content: ""; position: absolute; left: 1px; top: -7px; width: 5px; height: 7px; border: 2px solid #cfcfcf; border-bottom: 0; border-radius: 5px 5px 0 0; }
.g123-tunebtn { padding: 7px 20px; background: #1c1c1c; border: 1px solid #333; border-radius: 6px; color: #cfcfcf; font-weight: 700; font-size: .85rem; }
.g123-conn { padding: 5px 16px; border-radius: 20px; background: #3a2a24; color: #ffb39e; font-weight: 700; font-size: .85rem; text-align: center; }
.g123-conn.on { background: #16320f; color: #7be07b; }
/* [TUNE-LIVE] Lock (armiert) + Tune-Button-Zustand + Verstell-Panel */
.g123-lock { cursor: pointer; }
.g123-lock.armed { background: #3a2f12; border-color: #b8860b; }
.g123-lock.armed .g123-lockico::before { top: -5px; left: 5px; border-radius: 5px 5px 0 0; transform: rotate(28deg); transform-origin: left bottom; }
.g123-tunebtn { cursor: pointer; }
.g123-tunebtn:disabled { opacity: .4; }
.g123-tunebtn.on { background: #16320f; border-color: #2e7d32; color: #7be07b; }
.g123-tunepanel { display: flex; align-items: center; justify-content: center; gap: 12px; margin: 8px 0 2px; }
/* [TUNE-LIVE] hidden-Attribut durchsetzen (author display:flex wuerde es sonst
   ueberschreiben) -> Verstell-Leiste erst im Tuning-Modus sichtbar, wie im Original. */
.g123-tunepanel[hidden] { display: none; }
.g123-adv { width: 58px; height: 52px; border-radius: 10px; background: #241a12; border: 1px solid #b8860b; color: #ffcf7a; font-size: 1.6rem; font-weight: 700; cursor: pointer; }
.g123-adv:active { background: #3a2a14; }
.g123-off { min-width: 92px; text-align: center; color: #ffcf7a; }
.g123-off span { display: block; font-size: 2rem; font-weight: 700; line-height: 1; }
.g123-off small { color: #9a8; font-size: .7rem; }
.g123-reset { padding: 0 14px; height: 52px; border-radius: 10px; background: #1c1c1c; border: 1px solid #555; color: #cfcfcf; font-weight: 700; cursor: pointer; }
@media (orientation: landscape) and (max-height: 600px) {
  /* Echtes Querformat (Handy quer in der APK): KEIN Rotations-Trick mehr (der
     sprengte auf dem quer gehaltenen Handy die Skalierung -> Gauges riesig und
     abgeschnitten). Stattdessen Cockpit-Anordnung 2-1-2: links Zuendung+Volt,
     Mitte grosser DZM, rechts MAP+Temp. Groessen in vh -> passt IMMER in die
     Bildschirmhoehe, nichts wird abgeschnitten. */
  .tab-section[data-tab="g123"] {
    position: fixed; top: 0; left: 0; right: 0; bottom: 0;
    overflow: hidden; z-index: 60; background: #000;
  }
  .tab-section[data-tab="g123"] .g123-card {
    border: 0; border-radius: 0; height: 100%; box-sizing: border-box;
    padding: 1vh 2vw; display: flex; flex-direction: column;
  }
  /* Ganzer Cockpit-Stack vertikal zentriert; Gauges sizen zu ihrem Inhalt
     (flex:0 0 auto) -> der grosse DZM quillt NICHT mehr in die Bedien-Leiste. */
  .tab-section[data-tab="g123"] .g123-card { justify-content: center; padding: 1vh 2vw; }
  .g123-gauges {
    flex: 0 0 auto;
    grid-template-columns: 1fr auto 1fr; grid-template-rows: auto auto;
    gap: 0 1vw; align-items: center; justify-items: center;
  }
  .g123-gv { width: auto; max-width: none; height: 24vh; }
  .g123-gv.big { max-width: none; height: 50vh; grid-column: 2; grid-row: 1 / -1; margin: 0; }
  #g123gAdv  { grid-column: 1; grid-row: 1; }
  #g123gVA   { grid-column: 1; grid-row: 2; }
  #g123gMap  { grid-column: 3; grid-row: 1; }
  #g123gTemp { grid-column: 3; grid-row: 2; }
  /* Uhr LINKS, Lock RECHTS -- beide vertikal zentriert NEBEN dem DZM (wie Original).
     Anker: Viewport-Mitte + horizontaler Versatz in vh (DZM ist 50vh = 25vh halbe Breite). */
  .tab-section[data-tab="g123"] .g123-clock {
    position: absolute; top: 44%; left: 50%; transform: translate(calc(-50% - 31vh), -50%);
    margin: 0; width: auto; padding: .2vh 2vh; font-size: 2.4vh; letter-spacing: 2px; z-index: 5;
  }
  .tab-section[data-tab="g123"] .g123-lock {
    position: absolute; top: 44%; left: 50%; transform: translate(calc(-50% + 31vh), -50%);
    width: 6vh; height: 6vh; z-index: 5;
  }
  /* Tune-Bedienung kompakt + zentriert direkt UNTER dem DZM */
  .g123-tunebar { margin: 1vh 0 0; gap: 4vw; justify-content: center; }
  .g123-tunebtn { padding: .5vh 2.6vh; font-size: 2vh; }
  .g123-tunepanel { margin: .6vh 0 0; gap: 1.6vw; }
  .g123-adv { width: 8vh; height: 5.6vh; font-size: 3vh; }
  .g123-off { min-width: 9vh; }
  .g123-off span { font-size: 3.4vh; }
  .g123-off small { font-size: 1.4vh; }
  .g123-reset { height: 5.6vh; font-size: 2vh; padding: 0 2vh; }
  .g123-conn { position: fixed; right: 2vw; bottom: 1.2vh; padding: .5vh 2vh; font-size: 1.8vh; }
}
</style>
</head>
<body><main>
<h1>SPARTAN 3 v2 Motorraum Hub</h1>
<div class="tabs">
<button type="button" id="tabLive" class="tab on" onclick="showTab('live')">Live</button>
<button type="button" id="tab123" class="tab" onclick="showTab('g123')">123</button>
<button type="button" id="tabDiag" class="tab" onclick="showTab('diag')">Diagnose</button>
<button type="button" id="tabLog" class="tab" onclick="showTab('log')">Log</button>
<button type="button" id="tabSetup" class="tab" onclick="showTab('setup')">Setup</button>
<button type="button" id="tabDev" class="tab" onclick="showTab('dev')">Dev</button>
</div>
<div class="tab-section" data-tab="live">
<div class="card">
<div class="topline"><span id="source" class="tag">START</span><span id="wifiTop" class="mono">offline</span></div>
<p class="hint" id="featBadges" style="margin:0 0 10px;line-height:1.9">
<span class="tag" id="featAp">AP -</span>
<span class="tag" id="featWifi">WLAN -</span>
<span class="tag" id="featBle123">123 -</span>
<span class="tag" id="featLog">LOG -</span>
</p>
<div class="lambda" id="lambda">-.---</div>
<div class="metrics">
<div class="metric"><span>Status</span><strong id="status">warte</strong></div>
<div class="metric"><span>CAN</span><strong id="can">-</strong></div>
<div class="metric"><span>Abgas &deg;C</span><strong id="temp">- C</strong></div>
<div class="metric"><span>Speed (Reed)</span><strong id="liveSpeed">0.0 km/h</strong></div>
<div class="metric wide"><span>123 RPM / ADV&deg; / kPa</span><strong id="main123">0 / 0.0 / 0</strong></div>
<div class="metric"><span>123 Volt</span><strong id="liveTuneVolt">- V</strong></div>
<div class="metric"><span>123 AMP</span><strong id="liveTuneAmp">- A</strong></div>
<div class="metric"><span>Verteiler &deg;C</span><strong id="liveTuneTemp">- C</strong></div>
<div class="metric"><span>123 BLE</span><strong id="liveTuneConn">scan</strong></div>
<div class="metric"><span>Sonde h</span><strong id="liveHours">0.00</strong></div>
</div>
<p class="hint">Messstelle hinten im Auspuff: Lambda kann bei Falschluft magerer wirken als der Motor wirklich laeuft.</p>
</div>
</div><!-- /tab live -->
<div class="tab-section" data-tab="g123" hidden>
<div class="card g123-card">
<svg width="0" height="0" style="position:absolute" aria-hidden="true"><defs>
<radialGradient id="g123chrome" cx="50%" cy="32%" r="75%">
<stop offset="0%" stop-color="#fcfcfc"/><stop offset="38%" stop-color="#9a9a9a"/>
<stop offset="55%" stop-color="#eaeaea"/><stop offset="78%" stop-color="#7d7d7d"/><stop offset="100%" stop-color="#4a4a4a"/>
</radialGradient>
<radialGradient id="g123face" cx="50%" cy="28%" r="80%">
<stop offset="0%" stop-color="#363636"/><stop offset="55%" stop-color="#161616"/><stop offset="100%" stop-color="#000"/>
</radialGradient>
<linearGradient id="g123needle" x1="0" y1="0" x2="0" y2="1">
<stop offset="0%" stop-color="#f08a22"/><stop offset="100%" stop-color="#8f4408"/>
</linearGradient>
<radialGradient id="g123hub" cx="42%" cy="35%" r="70%">
<stop offset="0%" stop-color="#f0a04a"/><stop offset="100%" stop-color="#7a3c08"/>
</radialGradient>
</defs></svg>
<div class="g123-clock" id="g123Clock">--:--</div>
<div class="g123-gauges">
<svg id="g123gAdv" class="g123-gv" viewBox="0 0 240 240"></svg>
<svg id="g123gMap" class="g123-gv" viewBox="0 0 240 240"></svg>
<svg id="g123gRpm" class="g123-gv big" viewBox="0 0 240 240"></svg>
<svg id="g123gVA" class="g123-gv" viewBox="0 0 240 240"></svg>
<svg id="g123gTemp" class="g123-gv" viewBox="0 0 240 240"></svg>
</div>
<div class="g123-tunebar"><span class="g123-lock" id="g123Lock" onclick="g123ToggleLock()"><i class="g123-lockico"></i></span><button type="button" class="g123-tunebtn" id="g123TuneBtn" onclick="g123TuneMode()" disabled>Tune</button></div>
<div class="g123-tunepanel" id="g123TunePanel" hidden>
<button type="button" class="g123-adv" id="g123AdvDown" onclick="g123Adv('down')">&minus;</button>
<div class="g123-off"><span id="g123Off">0</span><small>Z&uuml;ndung-Offset</small></div>
<button type="button" class="g123-adv" id="g123AdvUp" onclick="g123Adv('up')">+</button>
<button type="button" class="g123-reset" id="g123Reset" onclick="g123Adv('reset')">0</button>
</div>
<div id="g123Conn" class="g123-conn" style="margin-top:14px">123 &mdash;</div>
</div>
</div><!-- /tab g123 -->
<div class="tab-section" data-tab="diag" hidden>
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
<details class="setup">
<summary>Live Meta</summary>
<div class="inside">
<div class="row"><span>WLAN / IP</span><strong id="liveWifiMeta" class="mono">-</strong></div>
<div class="row"><span>Speed Hz / Pulse</span><strong id="liveSpeedMeta">0.00 / 0</strong></div>
<div class="row"><span>123 Alter / RX</span><strong id="liveTuneMeta">0 ms / 0</strong></div>
<div class="row"><span>Geraet / Motor / Sonde</span><strong id="liveHoursMeta">0 / 0 / 0 h</strong></div>
</div>
</details>
<details class="setup" open>
<summary>Zeit / NTP</summary>
<div class="inside">
<p class="hint">NTP startet automatisch, wenn der Hub per STA (Z00-Station oder Handy-Hotspot) Internet hat. AP-only bleibt auf Bootzeit.</p>
<div class="row"><span>Sync</span><strong id="ntpSynced">-</strong></div>
<div class="row"><span>Aktuelle Zeit</span><strong id="ntpTime">-</strong></div>
<div class="row"><span>Zeitzone</span><strong id="ntpTz">-</strong></div>
<div class="row"><span>Server</span><strong id="ntpServer" class="mono">-</strong></div>
<div class="row"><span>Letzter Sync</span><strong id="ntpLastSync">-</strong></div>
<button type="button" class="secondary" id="ntpSyncBtn" onclick="ntpSyncNow()">Jetzt synchronisieren</button>
</div>
</details>
<details class="setup" open>
<summary>WiFi AP Clients</summary>
<div class="inside">
<p class="hint">Stationen am Hub-Hotspot Spartan3-Setup (192.168.4.x). MAC und RSSI kommen vom ESP32 AP, IP aus DHCP falls verfuegbar.</p>
<div class="row"><span>AP SSID</span><strong id="wifiApSsid" class="mono">-</strong></div>
<div class="row"><span>AP IP</span><strong id="wifiApIp" class="mono">-</strong></div>
<div class="row"><span>Stationen</span><strong id="wifiApCount">0</strong></div>
<div id="wifiApTable" class="mono">-</div>
</div>
</details>
<details class="setup" open>
<summary>WiFi HTTP Poll Clients</summary>
<div class="inside">
<p class="hint">Geraete, die /state oder /api/status pollen (Browser, Waveshare 2.8, Skripte). Device-ID aus X-Device Header oder ?client= Query.</p>
<div class="row"><span>Poll Clients</span><strong id="wifiHttpCount">0</strong></div>
<div id="wifiHttpTable" class="mono">-</div>
</div>
</details>
<details class="setup" open>
<summary>Event-Log (Ringbuffer)</summary>
<div class="inside">
<p class="hint">State-Transitions fuer 123TUNE, BLE, BM6, WiFi/HTTP und ESP-NOW. Auto-Refresh alle 2 s.</p>
<div class="row"><span>Events</span><strong id="eventCount">0</strong></div>
<div class="row"><span>123 State</span><strong id="tuneLinkState">-</strong></div>
<form action="/log/events_clear" method="post" style="margin-top:8px"><button class="secondary" type="submit">Event-Log leeren</button></form>
<pre id="eventLog" style="max-height:280px;margin-top:12px">-</pre>
</div>
</details>
</div><!-- /tab diag -->
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
<p class="hint">Zeit ist immer Pflicht: ms, epoch und time. Wenn NTP noch fehlt, bleibt epoch 0 und time zeigt boot+ms.</p>
<div class="row"><span>Zeit</span><strong id="logtime">-</strong></div>
<form action="/log_columns" method="post">
<label><input type="checkbox" name="spartan" id="colSpartan" value="1"> Spartan Lambda/Temp/Status</label>
<label><input type="checkbox" name="tune" id="colTune" value="1"> 123 RPM/ADV/MAP/Volt</label>
<label><input type="checkbox" name="bm6" id="colBm6" value="1"> BM6 Batterie</label>
<label><input type="checkbox" name="speed" id="colSpeed" value="1"> Speed/Reed</label>
<label><input type="checkbox" name="heater" id="colHeater" value="1"> Heater Analog</label>
<label><input type="checkbox" name="hours" id="colHours" value="1"> Betriebsstunden</label>
<button type="submit">Spalten speichern</button>
</form>
<p class="hint">Aendern der Spalten startet die Current-CSV neu, damit Header und Daten zusammenpassen.</p>
</div>
</details>
</div><!-- /tab log -->
<div class="tab-section" data-tab="setup" hidden>
<details class="setup" open>
<summary>OTA Firmware Update</summary>
<div class="inside">
<p class="hint">Firmware-BIN aus PlatformIO hochladen. Nach erfolgreichem Upload startet der Hub neu. Waehrend OTA sind Live-Polls kurz blockiert.</p>
<p class="hint" id="otaLockHint">OTA-Status: -</p>
<div class="row" style="gap:6px"><input id="otaTok" type="password" placeholder="OTA-Token" autocomplete="off" style="flex:1;min-width:0"><button type="button" id="otaTokSave">Token speichern</button></div>
<p class="hint">Ohne gesetzten Token ist OTA <b>gesperrt</b> (Schutz vor versehentlichem Fremd-Flash, z.B. Display-FW ueber den mDNS-Namen). Token einmal setzen; zum Hochladen im Feld lassen.</p>
<form id="otaForm" method="POST" action="/update" enctype="multipart/form-data">
<input class="file" type="file" name="update" accept=".bin,application/octet-stream" required>
<button type="submit" id="otaBtn">Firmware hochladen</button>
<div id="otaProgress" hidden>
<div class="ota-track"><div class="ota-bar" id="otaBar"></div></div>
<p class="hint" id="otaStatus">Upload laeuft...</p>
</div>
</form>
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
<details class="setup" open>
<summary>WLAN / Hotspot</summary>
<div class="inside">
<div class="row"><span>Aktueller Modus</span><strong id="wifiProfLabel">-</strong></div>
<div class="row"><span>Verbindung</span><strong id="wifi">nicht eingerichtet</strong></div>
<div class="row"><span>Hub-IP (externes Netz)</span><strong id="lanip">-</strong></div>
<div class="row"><span>Gespeichert</span><strong id="wifisaved">-</strong></div>

<p class="hint" style="margin:14px 0 6px"><b>1. Verbindungsmodus</b> &mdash; ein Tipp schaltet um:</p>
<div id="wifiProfBtns"></div>

<details style="margin:12px 0" open><summary><b>2. Zugangsdaten eintragen</b> (einmalig)</summary>
<p class="hint">Netz w&auml;hlen, SSID + Passwort eintragen, speichern &mdash; der Hub verbindet dann <b>automatisch</b> (kein extra Modus-Tipp). &#9888; ESP32 kann nur <b>2,4&nbsp;GHz</b>: z.B. <code>Z00-Station</code>, <b>NICHT</b> <code>...-5G</code>.</p>
<label>Netz</label>
<select id="credFor" onchange="credShow()">
<option value="1">Zuhause (Heim-WLAN)</option>
<option value="2">S24 (Handy-Hotspot)</option>
</select>
<form id="credForm1" action="/wifi_profile_save" method="post" style="margin:6px 0">
<input type="hidden" name="slot" value="1">
<label>Zuhause SSID</label><input name="ssid" id="profSsid1">
<label>Zuhause Passwort</label><input name="pass" type="password" id="profPass1">
<button type="submit">Zuhause speichern &amp; verbinden</button>
</form>
<form id="credForm2" action="/wifi_profile_save" method="post" style="margin:6px 0" hidden>
<input type="hidden" name="slot" value="2">
<label>S24 Hotspot SSID</label><input name="ssid" id="profSsid2">
<label>S24 Hotspot Passwort</label><input name="pass" type="password" id="profPass2">
<button type="submit">S24 speichern &amp; verbinden</button>
</form>
</details>

<details style="margin:8px 0"><summary>Anderes Netz suchen (Scan)</summary>
<p class="hint">Nur f&uuml;r ein NEUES Netz n&ouml;tig &mdash; f&uuml;r Zuhause/S24 nicht erforderlich. Beim Scan kann der AP kurz aussetzen; falls die Seite h&auml;ngt, neu laden.</p>
<div style="margin:6px 0"><button type="button" onclick="wifiScan()">Netzwerke scannen</button> <span id="wifiScanInfo" class="hint"></span></div>
<label>Gefundene Netzwerke</label>
<select id="wifiScanSel" onchange="document.getElementById('wcSsid').value=this.value"><option value="">— erst scannen —</option></select>
<label>SSID</label><input id="wcSsid" placeholder="Netzwerkname">
<label>Passwort</label><input id="wcPass" type="password" placeholder="WLAN-Passwort">
<button type="button" onclick="wifiConnect()">Verbinden &amp; speichern (Reboot)</button>
</details>

<p class="hint">In JEDEM Modus l&auml;uft der Hub-AP parallel weiter (Name/Passwort/IP frei einstellbar unter „SoftAP Name / Passwort / IP"). Der Hub ist immer unter <b>http://spartanhub.local</b> erreichbar.</p>
</div>
</details>
<details class="setup" open>
<summary>Zeitzone</summary>
<div class="inside">
<p class="hint">Standard ist Europe/Berlin (CET/CEST). Aenderung speichert in NVS und fordert einen NTP-Neusync an.</p>
<form action="/timezone" method="post">
<label for="tzSelect">Zeitzone</label>
<select id="tzSelect" name="tz_idx">
<option value="0">Europe/Berlin (CET/CEST)</option>
<option value="1">UTC</option>
<option value="2">Europe/London (GMT/BST)</option>
<option value="3">America/New_York (EST/EDT)</option>
<option value="4">America/Los_Angeles (PST/PDT)</option>
<option value="5">Asia/Tokyo (JST)</option>
</select>
<button type="submit">Speichern</button>
</form>
</div>
</details>
<details class="setup" open>
<summary>BLE Zielgeraete</summary>
<div class="inside">
<p class="hint">Festes 123-Profil. Manuell bleibt moeglich, damit wir spaeter weitere Geraete schnell aufnehmen koennen.</p>
<div class="row"><span>123 aktuell</span><strong id="taddrsetup" class="mono">-</strong></div>
<form action="/ble_target" method="post">
<label for="tunePreset">123 Profil</label><select id="tunePreset">
<option value="">Manuell</option>
<option value="ef:a8:b2:de:e0:9e">Echte 123TUNE+ (ef:a8:b2:de:e0:9e)</option>
<option value="10:20:ba:57:49:b1">Emu COM22 (10:20:ba:57:49:b1)</option>
</select>
<label for="tune_mac">123 BLE-Adresse</label><input id="tune_mac" name="tune_mac" placeholder="aa:bb:cc:dd:ee:ff">
<button type="submit">123 Ziel speichern</button>
</form>
<div class="row"><span>Scanliste</span><strong id="blescancount">-</strong></div>
<div id="blescan" class="mono"></div>
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
</div><!-- /tab setup -->
<div class="tab-section" data-tab="dev" hidden>
<div class="card">
<h3>BLE-Scan</h3>
<p class="hint">Einmal 10s scannen — gefundene Geräte als Ziel setzen.</p>
<div style="display:flex;gap:8px;align-items:center;margin-bottom:8px">
<button type="button" onclick="devBleScan()">Scan starten (10s)</button>
<span id="dev_scan_status" class="hint"></span>
</div>
<div id="dev_scan_results" style="font-family:monospace;font-size:11px"></div>
</div>
<div class="card">
<h3>Schalter</h3>
<div class="row"><span>123 BLE</span><span><button type="button" onclick="devFeat('ble123','on')">AN</button> <button type="button" onclick="devFeat('ble123','off')">AUS</button> <strong id="dev_ble123" style="margin-left:8px">-</strong></span></div>
<div class="row"><span>Logging</span><span><button type="button" onclick="devFeat('log','on')">AN</button> <button type="button" onclick="devFeat('log','off')">AUS</button> <strong id="dev_log" style="margin-left:8px">-</strong></span></div>
</div>
<div class="card">
<h3>Lambda-Demo (Tischtest)</h3>
<div class="row"><span>Modus</span><strong id="dev_lambda">-</strong></div>
<div style="display:flex;gap:8px;margin-top:8px">
<button type="button" onclick="devLambda('off')">AUS</button>
<button type="button" onclick="devLambda('fixed')">Fest 1.000</button>
<button type="button" onclick="devLambda('sweep')">Sweep 0.85-1.15</button>
</div>
</div>
<div class="card">
<h3>Anzeigefrequenz</h3>
<div class="row"><span>Aktuell</span><strong id="dev_poll_freq">5 Hz</strong></div>
<div style="display:flex;gap:8px;margin-top:8px">
<button type="button" onclick="setPollInterval(100);document.getElementById('dev_poll_freq').textContent='10 Hz'">10 Hz</button>
<button type="button" onclick="setPollInterval(200);document.getElementById('dev_poll_freq').textContent='5 Hz'">5 Hz</button>
<button type="button" onclick="setPollInterval(500);document.getElementById('dev_poll_freq').textContent='2 Hz'">2 Hz</button>
<button type="button" onclick="setPollInterval(1000);document.getElementById('dev_poll_freq').textContent='1 Hz'">1 Hz</button>
</div>
<p class="hint">10 Hz = sehr flüssig (wie 123TUNE+ App). 2 Hz = sparsam für Langzeit.</p>
</div>
<div class="card">
<h3>Access Point / mDNS</h3>
<form action="/ap_config" method="post" id="apConfigForm">
<label for="ap_ssid">AP Name (SSID)</label><input id="ap_ssid" name="ssid" value="">
<label for="ap_pass">AP Passwort</label><input id="ap_pass" name="pass" type="text" value="">
<label for="ap_ip">AP IP / Gateway (IP-Range)</label><input id="ap_ip" name="ip" value="">
<label for="ap_mask">Netzmaske</label><input id="ap_mask" name="mask" value="">
<label for="mdns_host">DNS-Name (mDNS)</label><input id="mdns_host" name="mdns" value="" placeholder="spartanhub">
<p class="hint">Passwort leer = offener AP, sonst &ge;8 Zeichen. DNS-Name ergibt <b>http://&lt;name&gt;.local</b> (nur a-z, 0-9, „-"). Nach Speichern starten AP &amp; mDNS neu.</p>
<button type="submit">AP &amp; mDNS speichern</button>
</form>
</div>
<div class="card">
<h3>System</h3>
<div class="row"><span>Firmware (Stand)</span><strong id="fwbuild">-</strong></div>
<div class="row"><span>CAN State / TX / RX</span><strong id="candiag">-</strong></div>
<div class="row"><span>CAN Fehler</span><strong id="canerr">0</strong></div>
<div class="row"><span>Heap frei</span><strong id="heap">-</strong></div>
<div class="row"><span>Ext. Flash (W25Q)</span><strong id="extflash">-</strong></div>
<div class="row"><span>Config-Backup (W25Q)</span><strong id="cfgbackup">-</strong></div>
<div class="hint" style="font-size:11px;margin:-2px 0 4px">WLAN-Daten &amp; Setup werden zusätzlich auf dem 16-MB-Chip gesichert und nach einem Firmware-Flash automatisch wiederhergestellt (überlebt sogar <code>erase_flash</code>).</div>
<div class="row"><span>WLAN-Kanal (STA / AP)</span><strong id="wifichan">-</strong></div>
<div class="row"><span>Fahrt-Gate (Variante A)</span><strong id="vargate">-</strong></div>
<div class="hint" style="font-size:11px;margin:-2px 0 4px">Variante&nbsp;A: Heim-/S24-WLAN verbindet nur im Stand. Fährt der Wagen (Motor läuft / Reed-Speed), bleibt der Hub-AP für die Displays stabil — kein Reconnect, kein Scan, kein Kanalwechsel.</div>
<div class="row"><span>Betriebsstunden</span><strong id="hours">-</strong></div>
<div style="display:flex;gap:8px;margin-top:8px">
<button class="secondary" type="button" onclick="copyJson()">JSON kopieren</button>
<form action="/restart" method="post" style="margin:0"><button class="danger" type="submit">Neustart</button></form>
</div>
<details style="margin-top:10px"><summary class="hint">JSON Rohdaten</summary>
<pre id="jsondump" style="font-size:10px;overflow:auto;max-height:200px">{}</pre>
</details>
</div>
</div><!-- /tab dev -->
<script>
async function wifiScan(){
  const info=document.getElementById('wifiScanInfo'), sel=document.getElementById('wifiScanSel');
  info.textContent=' scanne...';
  try{
    const nets=await (await fetch('/wifi_scan',{cache:'no-store'})).json();
    nets.sort((a,b)=>b.rssi-a.rssi);
    sel.innerHTML='<option value="">— '+nets.length+' Netze —</option>'+nets.map(n=>'<option value="'+(n.ssid||'').replace(/"/g,'')+'">'+(n.lock?'🔒 ':'')+(n.ssid||'')+' ('+n.rssi+' dBm)</option>').join('');
    info.textContent=' '+nets.length+' gefunden';
  }catch(e){ info.textContent=' Scan fehlgeschlagen — Seite neu laden'; }
}
async function wifiConnect(){
  const ssid=(document.getElementById('wcSsid').value||'').trim(), pass=document.getElementById('wcPass').value||'';
  if(!ssid){alert('Bitte SSID eingeben oder aus der Liste wählen');return;}
  try{ await fetch('/wifi_connect',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)});
    alert('Verbinde mit "'+ssid+'" — Gerät startet neu. Danach im Heimnetz (192.168.0.x) erreichbar.');
  }catch(e){ alert('Verbinden fehlgeschlagen'); }
}
async function devFeat(name,val){try{await fetch('/hub_feat?name='+name+'&val='+val);}catch(e){}}
async function devLambda(mode){try{await fetch('/lambda_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'mode='+mode});}catch(e){}}
async function devBleScan(){
  const status=document.getElementById('dev_scan_status');
  const results=document.getElementById('dev_scan_results');
  status.textContent=' scanne 10s...'; results.innerHTML='';
  try {
    // Scan via bestehender /api/status ble_scan Liste — frisch nach 10s
    await new Promise(r=>setTimeout(r,10000));
    const d=await(await fetch('/api/status',{cache:'no-store'})).json();
    const devs=d.ble_scan||[];
    if(devs.length===0){status.textContent=' nichts gefunden';return;}
    status.textContent=' '+devs.length+' Geräte';
    results.innerHTML=devs.map(x=>{
      const name=x.name||'---';
      const is123=x.tune?'✓123':'';
      const isBm6=x.bm6?'✓BM6':'';
      return `<div style="padding:3px 0;border-bottom:1px solid #2a3a2e">
        <span style="color:#9ed85b">${x.addr}</span> ${name} rssi=${x.rssi} ${is123}${isBm6}
        ${x.tune?`<button type="button" style="margin-left:6px;font-size:10px" onclick="devSetTune('${x.addr}')">als 123 setzen</button>`:''}
        ${x.bm6?`<button type="button" style="margin-left:4px;font-size:10px" onclick="devSetBm6('${x.addr}')">als BM6 setzen</button>`:''}
      </div>`;
    }).join('');
  } catch(e){status.textContent=' Fehler';}
}
async function devSetTune(mac){
  try{const fd=new FormData();fd.set('tune_mac',mac);await fetch('/ble_target',{method:'POST',body:fd});alert('123-Ziel gesetzt: '+mac);}catch(e){}
}
let lastJson = {};
function showTab(name) {
  document.querySelectorAll('.tab-section').forEach(s => {
    s.hidden = s.dataset.tab !== name;
  });
  document.getElementById('tabLive').classList.toggle('on', name === 'live');
  document.getElementById('tab123').classList.toggle('on', name === 'g123');
  document.getElementById('tabDiag').classList.toggle('on', name === 'diag');
  document.getElementById('tabLog').classList.toggle('on', name === 'log');
  document.getElementById('tabSetup').classList.toggle('on', name === 'setup');
  document.getElementById('tabDev').classList.toggle('on', name === 'dev');
  try { localStorage.setItem('spartanTab', name); } catch (e) {}
}
try {
  const saved = localStorage.getItem('spartanTab');
  if (saved === 'setup' || saved === 'log' || saved === 'diag' || saved === 'dev' || saved === 'g123') showTab(saved);
} catch (e) {}
function credShow() {
  const v = (document.getElementById('credFor') || {}).value || '1';
  const f1 = document.getElementById('credForm1'), f2 = document.getElementById('credForm2');
  if (f1) f1.hidden = (v !== '1');
  if (f2) f2.hidden = (v !== '2');
}
credShow();
document.getElementById('tunePreset')?.addEventListener('change', (e) => {
  document.getElementById('tune_mac').value = e.target.value || '';
});
async function saveBleTarget(kind, addr) {
  const isBm6 = kind === 'bm6';
  const target = isBm6 ? 'bm6_mac' : 'tune_mac';
  const endpoint = isBm6 ? '/bm6_target' : '/ble_target';
  const field = isBm6 ? 'bm6_mac' : 'tune_mac';
  const input = document.getElementById(target);
  input.value = addr;
  try {
    const fd = new FormData();
    fd.set(field, addr);
    await fetch(endpoint, { method: 'POST', body: fd, cache: 'no-store' });
    await refresh();
  } catch (err) {}
}
document.getElementById('blescan')?.addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-addr]');
  if (!btn) return;
  saveBleTarget(btn.dataset.kind, btn.dataset.addr);
});
document.getElementById('wifiPreset')?.addEventListener('change', (e) => {
  const option = e.target.selectedOptions[0];
  const ssid = option.value || '';
  const s = document.getElementById('ssid'); if (s) s.value = ssid;
  const p = document.getElementById('pass'); if (p) p.value = option.dataset.pass || '';
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
function escHtml(v) {
  return String(v ?? '').replace(/[&<>"']/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  }[c]));
}
function syncBlePresetOptions(selectId, devices, kind, savedAddr) {
  const sel = document.getElementById(selectId);
  if (!sel) return;
  sel.querySelectorAll('option[data-scan=\"1\"]').forEach(o => o.remove());
  const seen = new Set(Array.from(sel.options).map(o => String(o.value || '').toLowerCase()));
  devices.forEach(x => {
    const addr = String(x.addr || '').toLowerCase();
    if (!addr || seen.has(addr)) return;
    if (kind === 'bm6' && !x.bm6 && addr !== String(savedAddr || '').toLowerCase()) return;
    if (kind === 'tune' && !x.tune && addr !== String(savedAddr || '').toLowerCase()) return;
    const opt = document.createElement('option');
    opt.value = addr;
    opt.dataset.scan = '1';
    const name = x.name ? ' ' + x.name : '';
    opt.textContent = (kind === 'bm6' ? 'Scan BM6' : 'Scan 123') + name + ' ' + addr + ' ' + (x.rssi ?? 0) + ' dBm';
    sel.appendChild(opt);
    seen.add(addr);
  });
  const current = String(savedAddr || '').toLowerCase();
  if (current) sel.value = current;
}
async function copyJson() {
  try { await navigator.clipboard.writeText(JSON.stringify(lastJson, null, 2)); } catch(e) {}
}
async function ntpSyncNow() {
  try {
    await fetch('/ntp_sync', { method: 'POST', cache: 'no-store' });
    await refresh();
  } catch (e) {}
}
function otaShow(pct, msg) {
  const box = document.getElementById('otaProgress');
  const bar = document.getElementById('otaBar');
  const st = document.getElementById('otaStatus');
  if (box) box.hidden = false;
  if (bar) bar.style.width = Math.max(0, Math.min(100, pct)) + '%';
  if (st) st.textContent = msg || 'Upload laeuft...';
}
const otaForm = document.getElementById('otaForm');
if (otaForm) {
  otaForm.addEventListener('submit', (ev) => {
    ev.preventDefault();
    const fd = new FormData(otaForm);
    const file = fd.get('update');
    if (!file || !file.size) return;
    const btn = document.getElementById('otaBtn');
    if (btn) btn.disabled = true;
    otaShow(0, 'Upload laeuft...');
    let poll = setInterval(async () => {
      try {
        const r = await fetch('/api/ota/progress', { cache: 'no-store' });
        const d = await r.json();
        if (d.active && d.total > 0) {
          const n = Math.min(99, Math.round(100 * (d.written || d.rx) / d.total));
          otaShow(n, 'Flash ' + n + '%');
        }
      } catch (e) {}
    }, 500);
    const xhr = new XMLHttpRequest();
    xhr.open('POST', '/update');
    xhr.upload.onprogress = (e) => {
      if (e.lengthComputable) {
        const n = Math.round(100 * e.loaded / e.total);
        otaShow(Math.min(92, n), 'Upload ' + n + '%');
      }
    };
    xhr.onload = () => {
      clearInterval(poll);
      if (xhr.status === 200 && xhr.responseText.trim() === 'OK') {
        otaShow(100, 'Erfolg, Hub startet neu...');
      } else {
        otaShow(0, 'Fehler (' + xhr.status + '): ' + (xhr.responseText || 'FAIL'));
        if (btn) btn.disabled = false;
      }
    };
    xhr.onerror = () => {
      clearInterval(poll);
      otaShow(0, 'Netzwerkfehler');
      if (btn) btn.disabled = false;
    };
    xhr.setRequestHeader('X-OTA-Token', (document.getElementById('otaTok')||{}).value || '');
    xhr.send(fd);
  });
}
const otaTokSave = document.getElementById('otaTokSave');
if (otaTokSave) {
  otaTokSave.addEventListener('click', async () => {
    const t = (document.getElementById('otaTok')||{}).value || '';
    otaTokSave.disabled = true;
    try {
      const r = await fetch('/api/ota/token', { method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded'},
        body:'token=' + encodeURIComponent(t) });
      const d = await r.json();
      otaShow(0, d.ota_locked ? 'OTA-Token geloescht -> gesperrt' : 'OTA-Token gesetzt -> entsperrt');
    } catch (e) { otaShow(0, 'Token-Fehler'); }
    otaTokSave.disabled = false;
  });
}
async function refreshEvents() {
  try {
    const r = await fetch('/log/events?limit=40', { cache: 'no-store' });
    const events = await r.json();
    document.getElementById('eventCount').textContent = events.length;
    document.getElementById('eventLog').textContent = events.length ? events.map(ev => {
      const ts = ev.epoch ? new Date(ev.epoch * 1000).toISOString().slice(11, 19) : ('+' + ev.ms + 'ms');
      return ts + '  ' + (ev.type || '-') + '  ' + (ev.detail || '');
    }).join('\n') : '-';
  } catch (e) {}
}
function setFeatBadge(id, label, on) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = label + ' ' + (on ? 'AN' : 'AUS');
  cls(el, on ? 'tag ok' : 'tag bad');
}
let hubFeaturesEditing = false;
let apConfigEditing = false;
document.addEventListener('change', (ev) => {
  if (ev.target && ev.target.closest && ev.target.closest('#hubFeaturesForm')) hubFeaturesEditing = true;
  if (ev.target && ev.target.closest && ev.target.closest('#apConfigForm')) apConfigEditing = true;
});
document.addEventListener('focusin', (ev) => {
  if (ev.target && ev.target.closest && ev.target.closest('#hubFeaturesForm')) hubFeaturesEditing = true;
  if (ev.target && ev.target.closest && ev.target.closest('#apConfigForm')) apConfigEditing = true;
});
const G123NS = 'http://www.w3.org/2000/svg';
function g123E(t, a) { const e = document.createElementNS(G123NS, t); for (const k in a) e.setAttribute(k, a[k]); return e; }
function g123P(r, deg) { const a = deg * Math.PI / 180; return [120 + r * Math.sin(a), 120 - r * Math.cos(a)]; }
function g123Build(id, cfg) {
  const s = document.getElementById(id); if (!s) return;
  while (s.firstChild) s.removeChild(s.firstChild);
  const A0 = -135, SW = 270;
  s.appendChild(g123E('circle', { cx: 120, cy: 120, r: 118, fill: 'url(#g123chrome)' }));
  s.appendChild(g123E('circle', { cx: 120, cy: 120, r: 104, fill: 'url(#g123face)' }));
  const div = cfg.minor || 40, me = cfg.majorEvery || 5;
  for (let i = 0; i <= div; i++) {
    const f = i / div, deg = A0 + f * SW, mj = (i % me === 0);
    const p1 = g123P(mj ? 85 : 94, deg), p2 = g123P(101, deg);
    s.appendChild(g123E('line', { x1: p1[0].toFixed(1), y1: p1[1].toFixed(1), x2: p2[0].toFixed(1), y2: p2[1].toFixed(1), stroke: '#fff', 'stroke-width': mj ? 3 : 1.3, 'stroke-linecap': 'round', opacity: mj ? '1' : '0.6' }));
  }
  (cfg.labels || []).forEach(l => {
    const f = (l - cfg.min) / (cfg.max - cfg.min), deg = A0 + f * SW, p = g123P(72, deg);
    const t = g123E('text', { x: p[0].toFixed(1), y: (p[1] + 5).toFixed(1), 'text-anchor': 'middle', fill: '#efefef', 'font-size': cfg.lbl || 15, 'font-weight': '600' }); t.textContent = l; s.appendChild(t);
  });
  (cfg.title || []).forEach((tl, i) => { const t = g123E('text', { x: 120, y: 78 + i * 15, 'text-anchor': 'middle', fill: '#d8d8d8', 'font-size': cfg.tsz || 13, 'font-style': 'italic' }); t.textContent = tl; s.appendChild(t); });
  (cfg.texts || []).forEach(o => { const t = g123E('text', { x: o.x, y: o.y, 'text-anchor': o.a || 'middle', fill: o.c || '#9a9a9a', 'font-size': o.s || 11, 'font-family': o.mono ? "Courier New, monospace" : 'Arial', 'font-weight': o.w || '400' }); if (o.id) t.setAttribute('id', o.id); t.textContent = o.t || ''; s.appendChild(t); });
  const g = g123E('g', { id: id + 'N', transform: 'rotate(-135 120 120)' });
  g.appendChild(g123E('polygon', { points: '120,40 123.5,124 116.5,124', fill: 'url(#g123needle)' }));
  g.appendChild(g123E('rect', { x: '116.5', y: '120', width: '7', height: '34', rx: '2.5', fill: 'url(#g123needle)' }));
  g.appendChild(g123E('circle', { cx: 120, cy: 120, r: 13, fill: 'url(#g123hub)' }));
  g.appendChild(g123E('circle', { cx: 120, cy: 120, r: 5, fill: '#2a1c0c' }));
  s.appendChild(g);
}
function g123Set(id, frac) { const n = document.getElementById(id + 'N'); if (!n) return; frac = Math.max(0, Math.min(1, frac || 0)); n.setAttribute('transform', 'rotate(' + (-135 + frac * 270).toFixed(1) + ' 120 120)'); }
// [TUNE-LIVE] Sicherheits-Interlock: erst Lock antippen (armieren), dann wird der
// Tune-Button aktiv. Verstellung wirkt LIVE auf die echte Zuendung -> bewusst
// zweistufig. Server prueft zusaetzlich streaming + Tuning-Modus.
var g123Armed = false;
async function g123TunePost(act) {
  try { const r = await fetch('/api/tune/live', { method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'act=' + act });
    return await r.json();
  } catch (e) { return { ok:false }; }
}
function g123ToggleLock() {
  const d = lastJson || {};
  if ((d.tune_link_state !== 'streaming')) { g123Armed = false; }
  else { g123Armed = !g123Armed; }
  const lk = document.getElementById('g123Lock'); if (lk) lk.classList.toggle('armed', g123Armed);
  // Beim Entsperren-Zuruecknehmen einen aktiven Tuning-Modus sicher beenden.
  if (!g123Armed && d.tune_mode) { g123TunePost('mode'); }
  const tb = document.getElementById('g123TuneBtn'); if (tb) tb.disabled = !(g123Armed && d.tune_link_state === 'streaming');
}
function g123TuneMode() { if (!g123Armed) return; g123TunePost('mode'); }
function g123Adv(dir) { g123TunePost(dir); }
function g123InitGauges() {
  g123Build('g123gAdv', { min: 0, max: 55, minor: 33, majorEvery: 3, labels: [10, 20, 30, 40, 50], lbl: 13, title: ['Kurbelwelle °', 'Vorverstellung'], tsz: 11,
    texts: [{ id: 'g123dAdv', t: '0', x: 150, y: 160, a: 'middle', c: '#7a7a7a', s: 24, mono: true, w: '700' }] });
  g123Build('g123gMap', { min: 0, max: 220, minor: 44, majorEvery: 4, labels: [20, 60, 100, 140, 180, 200], lbl: 12, title: ['kPa'],
    texts: [{ id: 'g123dMap', t: '0', x: 120, y: 162, a: 'middle', c: '#7a7a7a', s: 24, mono: true, w: '700' }] });
  g123Build('g123gRpm', { min: 0, max: 8, minor: 40, majorEvery: 5, labels: [1, 2, 3, 4, 5, 6, 7, 8], lbl: 17, title: ['U/min'], tsz: 15,
    texts: [
      { id: 'g123dRpm', t: '0', x: 110, y: 150, a: 'end', c: '#cfcfcf', s: 30, mono: true, w: '700' },
      { id: 'g123dSpeed', t: '0', x: 138, y: 142, a: 'start', c: '#54d273', s: 20, mono: true, w: '700' },
      { t: 'km/h', x: 138, y: 156, a: 'start', c: '#7a8579', s: 10 },
      { id: 'g123dLam', t: '-.--', x: 120, y: 184, a: 'middle', c: '#ff5630', s: 24, mono: true, w: '700' }
    ] });
  g123Build('g123gVA', { min: 11.5, max: 14.5, minor: 36, majorEvery: 6, labels: [12, 13, 14], lbl: 13, title: ['VOLT'],
    texts: [{ t: 'AMP', x: 120, y: 150, a: 'middle', c: '#cfcfcf', s: 12 }, { id: 'g123dVolt', t: '0', x: 120, y: 176, a: 'middle', c: '#7a7a7a', s: 18, mono: true, w: '700' }] });
  g123Build('g123gTemp', { min: -20, max: 100, minor: 36, majorEvery: 6, labels: [-20, 0, 20, 40, 60, 80, 100], lbl: 12, title: ['TEMP °C'],
    texts: [{ id: 'g123dTemp', t: '0', x: 120, y: 166, a: 'middle', c: '#7a7a7a', s: 22, mono: true, w: '700' }] });
}
try { g123InitGauges(); } catch (e) {}
async function refresh() {
  try {
    const r = await fetch('/state?client=hub-webgui', {cache:'no-store'});
    const d = await r.json();
    lastJson = d;
    document.getElementById('source').textContent = d.source;
    document.getElementById('lambda').textContent = d.valid ? d.lambda.toFixed(3) : '-.---';
    document.getElementById('status').textContent = d.status;
    const lts=document.getElementById('lambdaTestStatus'); if(lts) lts.textContent = d.lambda_test_mode || 'off';
    document.getElementById('temp').textContent = d.valid ? d.temperature + ' C' : '- C';
    document.getElementById('main123').textContent = (d.rpm ?? 0) + ' / ' + Number(d.advance ?? 0).toFixed(1) + ' / ' + (d.map ?? 0);
    document.getElementById('liveTuneVolt').textContent = Number(d.volt ?? 0).toFixed(1) + ' V';
    document.getElementById('liveTuneAmp').textContent = Number(d.tune_amp ?? 0).toFixed(2) + ' A';
    document.getElementById('liveTuneTemp').textContent = (d.tune_temp ?? 0) + ' C';
    // 123-Cockpit-Tab (VDO-Look) — Nadeln + Digital-Insets
    if (window.g123Set) {
      const rpm = Number(d.rpm ?? 0);
      g123Set('g123gAdv', Number(d.advance ?? 0) / 55);
      g123Set('g123gMap', Number(d.map ?? 0) / 220);
      g123Set('g123gRpm', rpm / 8000);
      g123Set('g123gVA', (Number(d.volt ?? 0) - 11.5) / 3);
      g123Set('g123gTemp', (Number(d.tune_temp ?? 0) + 20) / 120);
      const gid = id => document.getElementById(id);
      let e;
      if (e = gid('g123dAdv')) e.textContent = Number(d.advance ?? 0).toFixed(0);
      if (e = gid('g123dMap')) e.textContent = Math.round(Number(d.map ?? 0));
      if (e = gid('g123dRpm')) e.textContent = Math.round(rpm);
      if (e = gid('g123dSpeed')) e.textContent = Number(d.speed_kmh ?? 0).toFixed(0);
      if (e = gid('g123dLam')) e.textContent = d.valid ? Number(d.lambda).toFixed(2) : '-.--';
      if (e = gid('g123dVolt')) e.textContent = Number(d.volt ?? 0).toFixed(1);
      if (e = gid('g123dTemp')) e.textContent = Math.round(Number(d.tune_temp ?? 0));
      const ck = gid('g123Clock'); if (ck && d.time_text) ck.textContent = String(d.time_text).slice(11, 16);
      const gc = gid('g123Conn');
      if (gc) { const on = !!d.tune_connected; gc.textContent = on ? ('123 ' + (d.tune_link_state || 'verbunden')) : '123 getrennt'; gc.classList.toggle('on', on); }
      // [TUNE-LIVE] Live-Zuendwinkel: Zustand aus /api/status spiegeln
      const streaming = (d.tune_link_state === 'streaming');
      const tuneOn = !!d.tune_mode;
      const tbtn = gid('g123TuneBtn');
      if (tbtn) { tbtn.disabled = !(g123Armed && streaming); tbtn.classList.toggle('on', tuneOn); tbtn.textContent = tuneOn ? 'Tuning AN' : 'Tune'; }
      const panel = gid('g123TunePanel'); if (panel) panel.hidden = !tuneOn;
      const off = gid('g123Off'); if (off) { const s = Number(d.tune_adv_steps ?? 0); off.textContent = (s > 0 ? '+' : '') + s; }
      const dis = !(tuneOn && streaming);
      ['g123AdvDown','g123AdvUp','g123Reset'].forEach(id => { const b = gid(id); if (b) b.disabled = dis; });
    }
    document.getElementById('can').textContent = d.can_ready ? 'aktiv' : 'Fehler';
    document.getElementById('wifiTop').textContent = d.wifi_connected ? d.wifi_ip : (d.ap_ip || 'offline');
    cls(document.getElementById('source'), d.source === 'CAN' ? 'tag ok' : (d.source === 'DEMO' ? 'tag warn' : 'tag bad'));
    cls(document.getElementById('status'), d.status === 'OK' ? 'ok' : (d.status === 'HEAT' || d.source === 'DEMO' ? 'warn' : 'bad'));
    cls(document.getElementById('can'), d.can_ready && d.can_state === 1 ? 'ok' : 'bad');
    if (!hubFeaturesEditing) {
      const hfAp = document.getElementById('hfAp');
      if (hfAp) hfAp.checked = !!d.hub_feat_ap;
      const hfWifi = document.getElementById('hfWifi');
      if (hfWifi) hfWifi.checked = !!d.hub_feat_wifi;
      const hfLog = document.getElementById('hfLog');
      if (hfLog) hfLog.checked = !!d.hub_feat_log;
      const hf123 = document.getElementById('hfBle123');
      if (hf123) hf123.checked = !!d.hub_feat_ble123;
    }
    setFeatBadge('featAp', 'AP', d.hub_feat_ap);
    setFeatBadge('featWifi', 'WLAN', d.hub_feat_wifi);
    setFeatBadge('featBle123', '123', d.hub_feat_ble123);
    setFeatBadge('featLog', 'LOG', d.hub_feat_log);
    // Dev-Tab Schalterzustände spiegeln
    {
      const sd=(id,on)=>{const e=document.getElementById(id);if(e){e.textContent=on?'AN':'AUS';e.style.color=on?'#54d273':'#ff6a5a';}};
      sd('dev_ble123', d.hub_feat_ble123); sd('dev_log', d.hub_feat_log);
      const dl=document.getElementById('dev_lambda'); if(dl) dl.textContent=(d.lambda_test_mode||'off');
    }
    document.getElementById('tuneLinkState').textContent = d.tune_link_state ?? '-';
    const apStations = Array.isArray(d.wifi_ap_stations) ? d.wifi_ap_stations : [];
    document.getElementById('wifiApSsid').textContent = d.wifi_ap_ssid || '-';
    document.getElementById('wifiApIp').textContent = d.ap_ip || '-';
    if (!apConfigEditing) {
      const apSsid = document.getElementById('ap_ssid');
      const apPass = document.getElementById('ap_pass');
      const apIp = document.getElementById('ap_ip');
      const apMask = document.getElementById('ap_mask');
      const apMdns = document.getElementById('mdns_host');
      if (apSsid) apSsid.value = d.wifi_ap_ssid || '';
      if (apPass) apPass.value = d.wifi_ap_password || '';
      if (apIp) apIp.value = d.wifi_ap_ip || '192.168.4.1';
      if (apMask) apMask.value = d.wifi_ap_mask || '255.255.255.0';
      if (apMdns) apMdns.value = d.mdns_host || 'spartanhub';
    }
    document.getElementById('wifiApCount').textContent = d.wifi_ap_station_count ?? apStations.length;
    document.getElementById('wifiApTable').innerHTML = apStations.length ? apStations.map(s =>
      escHtml(s.ip || '-') + ' / ' + escHtml(s.mac || '-') + '<br>' +
      (s.rssi ?? 0) + ' dBm / seit ' + Math.round((s.age_ms ?? 0) / 1000) + ' s / zuletzt ' +
      Math.round((s.last_seen_ms ?? 0) / 1000) + ' s'
    ).join('<br><br>') : '-';
    const httpPollers = Array.isArray(d.wifi_http_pollers) ? d.wifi_http_pollers : [];
    document.getElementById('wifiHttpCount').textContent = httpPollers.length;
    document.getElementById('wifiHttpTable').innerHTML = httpPollers.length ? httpPollers.map(p =>
      escHtml(p.device || '-') + ' / ' + escHtml(p.ip || '-') + ' (' + escHtml(p.via || '-') + ')<br>' +
      escHtml(p.mac || '-') + ' / polls ' + (p.poll_count ?? 0) + ' / zuletzt ' +
      Math.round((p.age_ms ?? 0) / 1000) + ' s<br>' + escHtml(p.user_agent || '-')
    ).join('<br><br>') : '-';
    const scan = Array.isArray(d.ble_scan) ? d.ble_scan : [];
    syncBlePresetOptions('tunePreset', scan, 'tune', d.tune_saved_address);
    document.getElementById('blescancount').textContent = scan.length + ' Geraete';
    document.getElementById('blescan').innerHTML = scan.map(x => {
      const name = escHtml(x.name || '-');
      const addr = escHtml(x.addr || '');
      const tag = x.tune ? '123?' : 'BLE';
      return '<div class="row"><span>' + tag + ' ' + name + '<br>' + addr + '</span><strong>' +
             (x.rssi ?? 0) + ' dBm<br><button type="button" data-kind="tune" data-addr="' +
             addr + '">123</button></strong></div>';
    }).join('');
    document.getElementById('wifi').textContent = d.wifi_connected ? d.wifi_ssid : (d.wifi_prof===0 ? 'Hub-AP (nur eigener AP)' : (d.wifi_saved ? 'verbindet...' : 'nicht eingerichtet'));
    document.getElementById('wifiProfLabel').textContent = (d.wifi_prof_labels||['Hub-AP','Zuhause','S24'])[d.wifi_prof||0];
    (function(){
      const labels = d.wifi_prof_labels||['Hub-AP','Zuhause','S24'];
      const ssids = d.wifi_prof_ssids||['','',''];
      const desc = [
        'Nur eigener AP &mdash; kein externes WLAN n&ouml;tig. Handy/Display direkt auf den Hub.',
        'Verbindet sich mit dem Heim-WLAN.',
        'Verbindet sich mit dem Handy-Hotspot.'
      ];
      const btns = labels.map((lbl,i)=>{
        const net = (ssids[i]&&ssids[i].length) ? escHtml(ssids[i]) : (i===0?'Spartan3-TestHub':'(SSID nicht gesetzt)');
        const active = (d.wifi_prof===i);
        const sub = desc[i] + (i>0 ? ' Netz: '+net : '');
        return '<form action="/wifi_prof" method="post" style="margin:0 0 8px">'
          + '<input type="hidden" name="slot" value="'+i+'">'
          + '<button type="submit" style="width:100%;text-align:left;padding:12px 14px;'
          + (active?'background:#2e7d32;color:#fff':'background:#1a2922;color:#cbeaa7')+'">'
          + (active?'● ':'○ ')+'<b>'+escHtml(lbl)+'</b>'+(active?' &mdash; aktiv':'')
          + '<br><span style="font-weight:400;font-size:.8rem;opacity:.85">'+sub+'</span>'
          + '</button></form>';
      });
      document.getElementById('wifiProfBtns').innerHTML=btns.join('');
      const e1=document.getElementById('profSsid1');if(e1&&!e1.value)e1.value=ssids[1]||'';
      const e2=document.getElementById('profSsid2');if(e2&&!e2.value)e2.value=ssids[2]||'';
    })();
    document.getElementById('lanip').textContent = d.wifi_connected ? d.wifi_ip : '-';
    document.getElementById('wifisaved').textContent = d.wifi_saved_ssid || '-';
    document.getElementById('liveWifiMeta').textContent = d.wifi_connected ? ((d.wifi_ssid || '-') + ' / ' + (d.wifi_ip || '-')) : ('AP ' + (d.ap_ip || '-'));
    document.getElementById('logstatus').textContent = d.log_ready ? 'bereit' : 'Dateisystem fehlt';
    document.getElementById('logcurrent').textContent = fmtBytes(d.log_current_bytes);
    document.getElementById('logold').textContent = fmtBytes(d.log_old_bytes);
    document.getElementById('logtime').textContent = (d.time_valid ? 'NTP ' : 'Bootzeit ') + (d.time_text || '-');
    document.getElementById('ntpSynced').textContent = d.ntp_synced ? 'ja' : 'nein';
    cls(document.getElementById('ntpSynced'), d.ntp_synced ? 'ok' : 'warn');
    document.getElementById('ntpTime').textContent = d.ntp_time || d.time_text || '-';
    document.getElementById('ntpTz').textContent = d.timezone || '-';
    document.getElementById('ntpServer').textContent = d.ntp_server || '-';
    const ntpAge = Number(d.ntp_last_sync_age_s ?? 0);
    document.getElementById('ntpLastSync').textContent = d.ntp_synced
      ? (ntpAge > 0 ? 'vor ' + ntpAge + ' s' : 'gerade eben')
      : '-';
    const tzSel = document.getElementById('tzSelect');
    if (tzSel && document.activeElement !== tzSel) tzSel.value = String(d.timezone_idx ?? 0);
    const cols = Number(d.log_columns ?? 63);
    document.getElementById('colSpartan').checked = !!(cols & 1);
    document.getElementById('colTune').checked = !!(cols & 2);
    document.getElementById('colBm6').checked = !!(cols & 4);
    document.getElementById('colSpeed').checked = !!(cols & 8);
    document.getElementById('colHeater').checked = !!(cols & 16);
    document.getElementById('colHours').checked = !!(cols & 32);
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
    document.getElementById('liveTuneConn').textContent = d.tune_connected ? 'verbunden' : 'scan';
    cls(document.getElementById('liveTuneConn'), d.tune_connected ? 'ok' : 'warn');
    document.getElementById('trx').textContent = d.tune_rx ?? 0;
    document.getElementById('tage').textContent = (d.tune_age_ms ?? 0) + ' ms';
    document.getElementById('tscan').textContent = (d.tune_scan_seen ?? 0) + ' / ' + (d.tune_scan_candidates ?? 0);
    document.getElementById('terr').textContent = (d.tune_bad_length ?? 0) + ' / ' + (d.tune_unknown_opcode ?? 0);
    document.getElementById('tvals').textContent = (d.rpm ?? 0) + ' / ' + Number(d.advance ?? 0).toFixed(1) + ' / ' + (d.map ?? 0);
    document.getElementById('taddr').textContent = d.tune_saved_address || '-';
    document.getElementById('taddrsetup').textContent = d.tune_saved_address || '-';
    var tuneMac = document.getElementById('tune_mac');
    if (tuneMac && document.activeElement !== tuneMac) tuneMac.value = d.tune_saved_address || '';
    { const fb=document.getElementById('fwbuild'); if(fb) fb.textContent = (d.fw_role||'?') + ' · ' + (d.fw_build||'?'); }
    { const ol=document.getElementById('otaLockHint'); if(ol) ol.textContent = 'OTA-Status: ' + (d.ota_locked ? 'GESPERRT (kein Token gesetzt)' : 'entsperrt (Token gesetzt)'); }
    document.getElementById('candiag').textContent = (d.can_state ?? '-') + ' / ' + (d.can_tx_errors ?? 0) + ' / ' + (d.can_rx_errors ?? 0);
    document.getElementById('canerr').textContent = d.can_status_errors ?? 0;
    document.getElementById('heap').textContent = d.heap_free ? Math.round(d.heap_free / 1024) + ' KB' : '-';
    { const ef = document.getElementById('extflash');
      if (ef) { ef.textContent = d.flash_ext_detected
        ? ('OK ✓ ' + (d.flash_ext_mfg||'?') + ' ' + (d.flash_ext_mb||0) + ' MB (0x' + (d.flash_ext_jedec||'') + ')')
        : ('nicht erkannt (0x' + (d.flash_ext_jedec||'------') + ')');
        ef.style.color = d.flash_ext_detected ? '#54d273' : '#ff6a5a'; } }
    { const cb = document.getElementById('cfgbackup');
      if (cb) {
        if (!d.flash_ext_detected) { cb.textContent = 'kein Chip'; cb.style.color = '#ff6a5a'; }
        else if (d.cfg_backup) { cb.textContent = 'gesichert ✓ (übersteht Flash)'; cb.style.color = '#54d273'; }
        else { cb.textContent = 'Chip da, noch kein Backup'; cb.style.color = '#f0c020'; } } }
    { const wc = document.getElementById('wifichan');
      if (wc) {
        const staCh = d.wifi_sta_channel ?? 0, apCh = d.wifi_ap_channel ?? 0, rs = d.wifi_sta_reason ?? 0;
        if (d.wifi_connected) {
          wc.textContent = 'STA Kanal ' + staCh + ' (' + (d.wifi_sta_rssi ?? 0) + ' dBm) / AP Kanal ' + apCh + ' ✓';
          wc.style.color = '#54d273';
        } else {
          let why;
          if (rs === 201 || d.wifi_home_not_found) why = 'AP nicht gefunden (Reichweite/Band/Name?)';
          else if (rs === 202 || rs === 204 || rs === 15) why = 'Passwort? (Auth abgelehnt)';
          else if (rs === 8) why = 'Kanal/DHCP (assoziiert, keine IP)';
          else if (rs === 0) why = 'kein Versuch / kein Profil aktiv';
          else why = 'Grund ' + rs;
          wc.textContent = 'nicht verbunden - ' + why + ' / AP Kanal ' + apCh;
          wc.style.color = '#ff6a5a';
        } } }
    { const vg = document.getElementById('vargate');
      if (vg) {
        if (d.vehicle_active) {
          vg.textContent = 'aktiv → Heim-WLAN pausiert (Motor läuft / fährt)';
          vg.style.color = '#f0c020';
        } else {
          vg.textContent = (d.wifi_connected ? 'Stand → Heim-WLAN verbunden' : 'Stand → Heim-WLAN frei');
          vg.style.color = '#54d273';
        } } }
    document.getElementById('hours').textContent = Number(d.device_hours ?? 0).toFixed(2) + ' / ' + Number(d.engine_hours ?? 0).toFixed(2) + ' / ' + Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    document.getElementById('liveHours').textContent = Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    document.getElementById('liveHoursMeta').textContent = Number(d.device_hours ?? 0).toFixed(2) + ' / ' + Number(d.engine_hours ?? 0).toFixed(2) + ' / ' + Number(d.sensor_hours ?? 0).toFixed(2) + ' h';
    const apd=document.getElementById('apdiag'); if(apd) apd.textContent = (d.ap_ip || '-') + ' / ' + (d.ap_retry_count ?? 0);
    document.getElementById('liveTuneMeta').textContent = (d.tune_age_ms ?? 0) + ' ms / ' + (d.tune_rx ?? 0);
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
let pollIntervalMs = 200;  // Standard 200ms = 5 Hz, flüssig aber sparsam
function setPollInterval(ms) {
  pollIntervalMs = ms;
  clearInterval(window._pollTimer);
  window._pollTimer = setInterval(() => { if (!document.hidden) refresh(); }, pollIntervalMs);
}
window._pollTimer = setInterval(() => { if (!document.hidden) refresh(); }, pollIntervalMs);
setInterval(() => {
  if (!document.hidden && !document.querySelector('[data-tab=\"diag\"]').hidden) refreshEvents();
}, 2000);
</script></main></body></html>)HTML";
#endif  // ENABLE_WEB_GUI
