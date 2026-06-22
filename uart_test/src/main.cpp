#include <Arduino.h>
// C6 sendet kontinuierlich T,rpm,adv,map auf GPIO5 (TX), kein Loopback.
// Hub-S3 GPIO21 (RX) empfaengt und zeigt rpm/adv/map in Live-Seite.

HardwareSerial uart1(1);
uint32_t cnt = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== C6 Bridge TX GPIO5 ===");
  uart1.begin(115200, SERIAL_8N1, 4, 5); // RX=GPIO4, TX=GPIO5
}

void loop() {
  delay(500);
  int rpm = 700 + (cnt % 50) * 50;  // 700-3150 sweep
  float adv = 10.0 + (cnt % 30) * 0.8;
  int map = 40 + (cnt % 60);
  String msg = "T," + String(rpm) + "," + String(adv,1) + "," + String(map) + ",2.5,13.8,75\n";
  uart1.print(msg);
  Serial.printf("TX: %s", msg.c_str());
  cnt++;
}
