/**
 * 01_ScanPDOs.ino — Correct PDO decoder, works with PPS chargers like yours
 * ──────────────────────────────────────────────────────────────────────────
 * Your charger has 6 PPS PDOs: variable 3.3V–5/9/12/15/18/20V at ~3A each.
 * Use setVoltage() or setPPSPDO() to request any voltage in 100mV steps.
 * IMPORTANT: call pd.task() in loop() to maintain PPS voltage.
 */

#include <Wire.h>
#include "AP33772S.h"

// ── Board pin definitions ────────────────────────────────────────────────────
#ifdef ARDUINO_ARCH_RP2040
  #define PIN_SDA  20
  #define PIN_SCL  21
  #define PIN_INT   6
#else  // ESP32
  #define PIN_SDA  21
  #define PIN_SCL  22
  #define PIN_INT  32
#endif

AP33772S pd(Wire, PIN_INT);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println(F("\n══ AP33772S PDO Scanner v2 ══"));

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (pd.begin() != AP33772S_OK) {
        Serial.println(F("[ERR] Not found on I2C!")); while(true);
    }
    Serial.println(F("[OK]  AP33772S found"));

    Serial.print(F("[...] Waiting for cable... "));
    if (pd.waitForStartup(5000) != AP33772S_OK) {
        Serial.println(F("timeout")); while(true);
    }
    Serial.println(F("attached!"));

    Serial.print(F("[...] Waiting for PDOs... "));
    if (pd.waitForPDOs(8000) != AP33772S_OK)
        Serial.println(F("timeout — trying anyway"));
    else
        Serial.println(F("received!"));

    uint8_t n = pd.readAllPDOs();
    Serial.printf("[INFO] %u valid PDO(s):\n\n", n);
    pd.printPDOs(Serial);

    // ── Show charger type ──────────────────────────────────────────────────
    Serial.println();
    if (pd.isPDConnected())     Serial.println(F("[INFO] PD charger connected"));
    if (pd.isLegacyConnected()) Serial.println(F("[INFO] Legacy (5V only)"));
    if (pd.isCableFlipped())    Serial.println(F("[INFO] Cable flipped (CC2)"));

    // ── Show PPS/AVS availability ──────────────────────────────────────────
    int8_t ppsIdx = pd.getPPSIndex();
    int8_t avsIdx = pd.getAVSIndex();
    if (ppsIdx > 0) Serial.printf("[INFO] PPS available at PDO%d\n", ppsIdx);
    if (avsIdx > 0) Serial.printf("[INFO] AVS available at PDO%d\n", avsIdx);
    if (ppsIdx < 0 && avsIdx < 0)
        Serial.println(F("[INFO] Fixed PDOs only — no variable voltage"));

    Serial.println(F("\n[INFO] Register dump:"));
    pd.dumpRegisters(Serial);
}

void loop() {
    pd.task();  // Keep-alive for PPS/AVS — essential!
    delay(100);
}