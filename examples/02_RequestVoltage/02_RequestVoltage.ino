/**
 * 02_RequestVoltage.ino
 * Works with PPS chargers (like yours: 5/9/12/15/18/20V variable).
 * Change DESIRED_MV to any value your charger supports.
 * MUST call pd.task() in loop() for PPS to maintain voltage.
 */

#include <Wire.h>
#include "AP33772S.h"

#ifdef ARDUINO_ARCH_RP2040
  #define PIN_SDA  20
  #define PIN_SCL  21
  #define PIN_INT   6
#else
  #define PIN_SDA  21
  #define PIN_SCL  22
  #define PIN_INT  32
#endif

#define DESIRED_MV    12000  // Try: 5000 9000 12000 15000 18000 20000
#define MIN_MA         2000

AP33772S pd(Wire, PIN_INT);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.printf("\n══ Request %u mV via USB-PD ══\n", DESIRED_MV);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (pd.begin() != AP33772S_OK) {
        Serial.println(F("[ERR] AP33772S not found!")); while(true);
    }

    // Protection config
    pd.setOTPThreshold_C(85);
    pd.setDeratingThreshold_C(75);
    pd.setUVPThreshold(UVP_80PCT);
    pd.setOVPOffset_mV(2000);
    pd.setOCPThreshold_mA(0);  // auto

    Serial.print(F("[...] Startup... "));
    if (pd.waitForStartup(5000) != AP33772S_OK) {
        Serial.println(F("timeout")); while(true);
    }
    Serial.println(F("OK"));

    Serial.print(F("[...] PDOs... "));
    pd.waitForPDOs(8000);
    Serial.println(F("done"));

    pd.readAllPDOs();
    pd.printPDOs(Serial);

    // Auto-select best PDO
    Serial.printf("\n[...] Requesting %u mV / min %u mA\n", DESIRED_MV, MIN_MA);
    uint8_t chosen = pd.setVoltage(DESIRED_MV, MIN_MA);

    if (!chosen) {
        Serial.println(F("[WARN] Not achievable — requesting 5V PDO1"));
        pd.setFixPDO(1, 2000);
    } else {
        const AP33772S_PDO &p = pd.getPDO(chosen);
        const char *t = (p.type==PDO_TYPE_FIXED)?"Fixed":
                        (p.type==PDO_TYPE_PPS)  ?"PPS"  :"AVS";
        Serial.printf("[INFO] Using PDO%d (%s) maxV=%umV maxI=%umA\n",
                      chosen, t, p.maxVoltage_mV, p.maxCurrent_mA);
    }

    // Wait for negotiation result
    Serial.print(F("[...] Negotiating... "));
    if (pd.waitForNegotiation(3000) == AP33772S_OK) {
        Serial.printf("OK  →  VREQ=%umV  IREQ=%umA\n",
                      pd.getRequestedVoltage_mV(),
                      pd.getRequestedCurrent_mA());
    } else {
        Serial.println(F("FAILED — check charger compatibility"));
    }
}

void loop()
{
    pd.task();  // ← ESSENTIAL for PPS/AVS keep-alive!

    static uint32_t t = 0;
    if (millis() - t < 1000) return;
    t = millis();

    Serial.printf("V=%5umV  I=%4umA  P=%5umW  T=%3d°C%s%s\n",
                  pd.getVoltage_mV(),
                  pd.getCurrent_mA(),
                  pd.getPower_mW(),
                  pd.getTemperature_C(),
                  pd.isDerating() ? "  [DR]" : "",
                  pd.isFault()    ? ("  "+pd.getFaultString()).c_str() : "");

    if (pd.isFault()) {
        Serial.println(F("[FAULT] Clearing and retrying safe 5V..."));
        pd.clearInterrupt();
        delay(500);
        pd.setFixPDO(1, 1500);
        pd.waitForNegotiation(2000);
    }
}