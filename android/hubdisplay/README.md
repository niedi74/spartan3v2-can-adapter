# Spartan Hub Display (Android)

Minimale WebView-App, die die Hub-Weboberfläche im Vollbild lädt und den
**Bildschirm dauerhaft anlässt** (wie die 123TUNE+ App) — löst das AOD/Timeout-Problem,
das die Wake-Lock-API auf reinem HTTP nicht lösen kann.

## Features
- Vollbild-WebView auf die Hub-GUI (Anzeige + „Schirm an" in einem).
- `FLAG_KEEP_SCREEN_ON` + `keepScreenOn` → Display bleibt an, solange die App offen ist.
- Startbildschirm mit IP-Eingabefeld + Presets **AP 192.168.8.1** und **Heim 192.168.0.87**;
  zuletzt genutzte URL wird gemerkt (SharedPreferences).
- **Schwebender „IP"-Button** oben rechts im WebView (v1.1) → jederzeit zurück zur
  Verbindungsseite, auch im Immersive-Vollbild (wo die Zurück-Geste kaum nutzbar ist).
- Zurück-Taste: erst In-Page-Navigation, dann zurück zum Startbildschirm (IP umschalten).
- Immersive Fullscreen, Cleartext-HTTP erlaubt (für `http://192.168.x`).
- min SDK 21 (Android 5.0+), target SDK 34.

## Installieren
1. `HubDisplay.apk` aufs Handy laden (GitHub-Raw-Link im Browser).
2. Antippen → bei „Unbekannte App installieren" die Quelle erlauben.
3. Öffnen → AP-Preset oder eigene IP → **Verbinden**.

Die APK ist **Debug-signiert** (zum Sideloaden ok, nicht für den Play Store).

## Selbst bauen (ohne Android Studio, nur SDK + JDK)
```
aapt2 link -o build/base.apk -I <android.jar> --manifest AndroidManifest.xml \
  --min-sdk-version 21 --target-sdk-version 34
javac -source 8 -target 8 -bootclasspath <android.jar> -classpath <android.jar> \
  -d build/classes src/com/spartan/hubdisplay/MainActivity.java
d8 --release --lib <android.jar> --output build build/classes/com/spartan/hubdisplay/*.class
jar uf build/base.apk -C build classes.dex
zipalign -f 4 build/base.apk build/aligned.apk
apksigner sign --ks debug.keystore --ks-pass pass:android --out HubDisplay.apk build/aligned.apk
```
