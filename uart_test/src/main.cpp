#include <Arduino.h>
// Systematischer Pin-Test: testet UART TX auf GPIO4, 5, 6, 7, 15 nacheinander.
// Jeder Pin sendet 10x, dann weiter. Loopback auf S3-GPIO38 zeigt welcher ankommt.

const int TX_PINS[] = {4, 5, 6, 7, 15, 0, 1, 2, 3};
const int N_PINS = 9;
int pinIdx = 0;
int sendCount = 0;

HardwareSerial testSer(1);  // UART1

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== C6 TX-Pin-Scan ===");
}

void loop() {
  int txPin = TX_PINS[pinIdx];
  if (sendCount == 0) {
    testSer.end();
    delay(50);
    testSer.begin(115200, SERIAL_8N1, -1, txPin); // nur TX, kein RX
    delay(50);
    Serial.printf("Testing TX on GPIO%d\n", txPin);
  }
  String msg = "P" + String(txPin) + "," + String(sendCount) + "\n";
  testSer.print(msg);
  testSer.flush();
  Serial.printf("SENT GPIO%d: %s", txPin, msg.c_str());
  delay(300);
  sendCount++;
  if (sendCount >= 6) {
    sendCount = 0;
    pinIdx = (pinIdx + 1) % N_PINS;
  }
}
