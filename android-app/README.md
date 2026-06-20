# Spartan Hub Live Android

Minimal native Android dashboard for the Spartan engine-bay hub.

## Behavior

- Polls `http://<hub>/api/status?client=android-live` every 500 ms.
- Shows RPM, Lambda and BM6 voltage prominently.
- Shows advance, MAP, BM6 temperature and hub log status.
- Stores the last hub address and provides a `192.168.4.1` AP preset.
- Keeps the screen awake while the activity is visible.
- Marks data offline after two seconds without a successful response.

## Build

Requirements: JDK 17 and Android SDK platform/build-tools 35.

```powershell
cd android-app
.\gradlew.bat assembleDebug
```

APK output:

`app\build\outputs\apk\debug\app-debug.apk`
