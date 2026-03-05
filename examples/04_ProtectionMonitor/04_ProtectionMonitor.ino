/**
 * 04_ProtectionMonitor.ino
 * ──────────────────────────────────────────────────────────────────────────
 * Interrupt-driven protection monitoring with full fault recovery.
 * The INT pin fires on any STATUS change (fault, new PDO, ready, etc.)
 *
 * The ISR only sets a flag — all real work is done in loop() context,
 * which is the correct pattern for both ESP32 and RP2040.
 *
 * Wiring (RP2040 / PicoPD):   SDA=GP20  SCL=GP21  INT=GP6
 * Wiring (ESP32):              SDA=21    SCL=22     INT=32
 * ──────────────────────────────────────────────────────────────────────────
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

#define TARGET_MV  12000
#define MIN_MA      2000

AP33772S pd(Wire, PIN_INT);

volatile bool intFlag = false;

// ── ISR — keep it minimal ───────────────────────────────────────────────────
#ifdef ARDUINO_ARCH_RP2040
void pdISR() { intFlag = true; }
#else
void IRAM_ATTR pdISR() { intFlag = true; }
#endif

// ── Negotiation helper ───────────────────────────────────────────────────────
void negotiate(uint16_t targetMV, uint16_t minMA)
{
    pd.readAllPDOs();
    uint8_t chosen = pd.setVoltage(targetMV, minMA);

    if (!chosen) {
        Serial.printf("[NEGO] %u mV unavailable — falling back to 5V\n", targetMV);
        pd.setFixPDO(1, 1500);
        chosen = 1;
    } else {
        const AP33772S_PDO &p = pd.getPDO(chosen);
        const char *t = p.type == PDO_TYPE_FIXED ? "Fixed" :
                        p.type == PDO_TYPE_PPS   ? "PPS"   : "AVS";
        Serial.printf("[NEGO] PDO%d (%s) %umV / max %umA\n",
                      chosen, t, p.maxVoltage_mV, p.maxCurrent_mA);
    }

    if (pd.waitForNegotiation(3000) == AP33772S_OK)
        Serial.printf("[NEGO] OK  →  VREQ=%umV  IREQ=%umA\n",
                      pd.getRequestedVoltage_mV(), pd.getRequestedCurrent_mA());
    else
        Serial.println(F("[NEGO] Negotiation did not confirm"));
}

// ── Interrupt handler — called from loop(), NOT from ISR ────────────────────
void handleInterrupt()
{
    intFlag = false;

    // Reading STATUS clears the register AND de-asserts the INT pin
    uint8_t status = pd.clearInterrupt();
    if (!status) return;

    Serial.printf("[INT]  STATUS=0x%02X  →", status);
    if (status & STATUS_STARTED) Serial.print(" STARTED");
    if (status & STATUS_READY)   Serial.print(" READY");
    if (status & STATUS_NEWPDO)  Serial.print(" NEWPDO");
    if (status & STATUS_OVP)     Serial.print(" OVP!");
    if (status & STATUS_UVP)     Serial.print(" UVP!");
    if (status & STATUS_OCP)     Serial.print(" OCP!");
    if (status & STATUS_OTP)     Serial.print(" OTP!");
    Serial.println();

    // ── New PDO set (charger hot-swapped or cable re-inserted) ────────────
    if (status & STATUS_NEWPDO) {
        Serial.println(F("[INT]  New capabilities — re-negotiating"));
        negotiate(TARGET_MV, MIN_MA);
        return;
    }

    // ── Fault: hardware already disabled VOUT ─────────────────────────────
    if (status & STATUS_FAULTS) {
        Serial.printf("[INT]  %s  —  waiting 1s then recovering\n",
                      pd.getFaultString().c_str());
        delay(1000);

        // Step 1: return to safe 5V
        pd.setFixPDO(1, 1500);
        if (pd.waitForNegotiation(3000) == AP33772S_OK) {
            Serial.println(F("[RECV] 5V restored — waiting 3s before ramp-up"));
            delay(3000);
        }

        // Step 2: re-request original target
        negotiate(TARGET_MV, MIN_MA);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println(F("\n══ AP33772S Protection Monitor (Interrupt-Driven) ══"));
    Serial.printf(  "   Target: %u mV / min %u mA\n\n", TARGET_MV, MIN_MA);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (pd.begin() != AP33772S_OK) {
        Serial.println(F("[ERR] AP33772S not found!")); while(true);
    }

    // ── Protection thresholds ─────────────────────────────────────────────
    pd.setOVPOffset_mV(2000);        // OVP = VREQ + 2V
    pd.setUVPThreshold(UVP_80PCT);   // UVP at 80% of VREQ
    pd.setOCPThreshold_mA(3500);     // OCP at 3.5A
    pd.setOTPThreshold_C(80);        // OTP at 80°C
    pd.setDeratingThreshold_C(70);   // De-rate at 70°C
    pd.setProtectionConfig(true, true, true, true, true);

    // ── Enable all interrupt sources ──────────────────────────────────────
    pd.setInterruptMask(MASK_ALL);

    // ── Attach hardware interrupt ─────────────────────────────────────────
    pd.attachInterruptCallback(pdISR);

    // ── Startup sequence ──────────────────────────────────────────────────
    Serial.print(F("[...] Waiting for cable... "));
    if (pd.waitForStartup(5000) != AP33772S_OK) {
        Serial.println(F("timeout!")); while(true);
    }
    Serial.println(F("attached"));

    Serial.print(F("[...] Waiting for PDOs... "));
    if (pd.waitForPDOs(8000) != AP33772S_OK)
        Serial.println(F("timeout — continuing"));
    else
        Serial.println(F("received"));

    pd.readAllPDOs();
    pd.printPDOs(Serial);
    Serial.println();

    negotiate(TARGET_MV, MIN_MA);
}

void loop()
{
    pd.task();  // ← keep-alive for PPS/AVS

    // Handle interrupt flag in main loop context (safe for I2C calls)
    if (intFlag) handleInterrupt();

    // ── Periodic telemetry ────────────────────────────────────────────────
    static uint32_t lastTx = 0;
    if (millis() - lastTx < 2000) return;
    lastTx = millis();

    Serial.printf("  V=%5umV  I=%4umA  P=%5umW  T=%3d°C  "
                  "PD=%d Lgcy=%d Flip=%d DR=%d\n",
                  pd.getVoltage_mV(),
                  pd.getCurrent_mA(),
                  pd.getPower_mW(),
                  pd.getTemperature_C(),
                  pd.isPDConnected(),
                  pd.isLegacyConnected(),
                  pd.isCableFlipped(),
                  pd.isDerating());
}