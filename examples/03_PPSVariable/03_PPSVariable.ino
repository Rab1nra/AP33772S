/**
 * 03_PPSVariable.ino
 * ──────────────────────────────────────────────────────────────────────────
 * Sweeps through voltages using PPS or AVS — whichever your charger offers.
 * Your charger has PPS in SPR slots, so this will work perfectly.
 *
 * IMPORTANT: pd.task() MUST be called in loop() — PPS chargers require a
 * keep-alive RDO re-send every ~500ms or they revert to 5V.
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

// ── Sweep config ─────────────────────────────────────────────────────────────
#define SWEEP_STEP_MV   500    // Step size in mV (100mV min for PPS)
#define SWEEP_DWELL_MS  1500   // How long to hold each voltage
#define SWEEP_CURRENT_MA 2000  // Constant current request during sweep

AP33772S pd(Wire, PIN_INT);

// Sweep state
uint8_t  sweepPDOIndex  = 0;
uint16_t sweepMinV      = 0;
uint16_t sweepMaxV      = 0;
uint8_t  sweepType      = 0;   // PDO_TYPE_PPS or PDO_TYPE_AVS
uint16_t sweepCurrentV  = 0;
bool     sweepGoingUp   = true;

// ── Helpers ──────────────────────────────────────────────────────────────────
const char* pdoTypeName(uint8_t t) {
    return t == PDO_TYPE_FIXED ? "Fixed" : t == PDO_TYPE_PPS ? "PPS" : "AVS";
}

bool requestSweepVoltage(uint16_t v_mV)
{
    int8_t rc;
    if (sweepType == PDO_TYPE_PPS)
        rc = pd.setPPSPDO(sweepPDOIndex, v_mV, SWEEP_CURRENT_MA);
    else
        rc = pd.setAVSPDO(sweepPDOIndex, v_mV, SWEEP_CURRENT_MA);
    return (rc == AP33772S_OK);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println(F("\n══ AP33772S Variable Voltage Sweep ══"));

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (pd.begin() != AP33772S_OK) {
        Serial.println(F("[ERR] AP33772S not found!")); while(true);
    }

    // Conservative protection settings for sweep
    pd.setOTPThreshold_C(85);
    pd.setDeratingThreshold_C(75);
    pd.setUVPThreshold(UVP_75PCT);
    pd.setOVPOffset_mV(2000);

    Serial.print(F("[...] Waiting for cable... "));
    if (pd.waitForStartup(5000) != AP33772S_OK) {
        Serial.println(F("timeout!")); while(true);
    }
    Serial.println(F("attached"));

    Serial.print(F("[...] Waiting for PDOs... "));
    if (pd.waitForPDOs(8000) != AP33772S_OK)
        Serial.println(F("timeout — continuing anyway"));
    else
        Serial.println(F("received"));

    uint8_t n = pd.readAllPDOs();
    Serial.printf("[INFO] %u valid PDO(s) found:\n", n);
    pd.printPDOs(Serial);
    Serial.println();

    // ── Find best variable-voltage PDO ────────────────────────────────────
    // Strategy: prefer highest-current PPS, then highest-current AVS
    uint16_t bestI = 0;

    for (uint8_t i = 0; i < 13; i++) {
        const AP33772S_PDO &p = pd.getPDO(i + 1);
        if (!p.valid || p.type == PDO_TYPE_FIXED) continue;
        if (p.maxCurrent_mA < SWEEP_CURRENT_MA)  continue;
        if (sweepType == PDO_TYPE_AVS && p.type == PDO_TYPE_PPS) continue; // PPS > AVS
        if (p.maxCurrent_mA > bestI || p.type == PDO_TYPE_PPS) {
            bestI         = p.maxCurrent_mA;
            sweepPDOIndex = p.index;
            sweepMinV     = p.minVoltage_mV;
            sweepMaxV     = p.maxVoltage_mV;
            sweepType     = p.type;
        }
    }

    if (!sweepPDOIndex) {
        Serial.println(F("[ERR] No PPS or AVS PDO found with enough current!"));
        Serial.println(F("      Charger may be Fixed-only. Exiting sweep."));
        while(true) delay(5000);
    }

    Serial.printf("[INFO] Sweep using PDO%d (%s)\n",
                  sweepPDOIndex, pdoTypeName(sweepType));
    Serial.printf("[INFO] Range: %u mV → %u mV  in %u mV steps\n",
                  sweepMinV, sweepMaxV, SWEEP_STEP_MV);
    Serial.printf("[INFO] Current: %u mA  Dwell: %u ms\n\n",
                  SWEEP_CURRENT_MA, SWEEP_DWELL_MS);

    // Start at minimum voltage
    sweepCurrentV = sweepMinV;
    sweepGoingUp  = true;

    if (!requestSweepVoltage(sweepCurrentV)) {
        Serial.println(F("[ERR] Initial request failed!"));
    } else {
        if (pd.waitForNegotiation(3000) == AP33772S_OK)
            Serial.printf("[OK]  Started at %u mV\n\n", sweepCurrentV);
        else
            Serial.println(F("[WARN] Negotiation did not confirm — continuing"));
    }
}

void loop()
{
    pd.task();  // ← keep-alive — never remove this!

    static uint32_t dwellTimer = 0;
    static bool     firstRun   = true;

    if (firstRun) { dwellTimer = millis(); firstRun = false; }
    if (millis() - dwellTimer < SWEEP_DWELL_MS) return;
    dwellTimer = millis();

    // ── Read current measurements at this voltage ─────────────────────────
    uint16_t vout = pd.getVoltage_mV();
    uint16_t iout = pd.getCurrent_mA();
    int8_t   temp = pd.getTemperature_C();

    Serial.printf("%s %5u mV  →  VOUT=%5u mV  IOUT=%4u mA  T=%3d°C",
                  sweepGoingUp ? "▲" : "▼",
                  sweepCurrentV, vout, iout, temp);

    if (pd.isFault()) {
        String f = pd.getFaultString();
        Serial.printf("  !! %s !!", f.c_str());
        Serial.println();
        // On fault: clear and restart sweep from minimum
        Serial.println(F("[FAULT] Resetting to min voltage..."));
        pd.clearInterrupt();
        delay(500);
        sweepCurrentV = sweepMinV;
        sweepGoingUp  = true;
        requestSweepVoltage(sweepCurrentV);
        pd.waitForNegotiation(2000);
        return;
    }
    if (pd.isDerating()) Serial.print(F("  [DERATING]"));
    Serial.println();

    // ── Advance sweep ─────────────────────────────────────────────────────
    if (sweepGoingUp) {
        uint16_t next = sweepCurrentV + SWEEP_STEP_MV;
        if (next > sweepMaxV) {
            // Hit top — reverse
            sweepGoingUp  = false;
            sweepCurrentV = sweepMaxV;
        } else {
            sweepCurrentV = next;
        }
    } else {
        if (sweepCurrentV <= sweepMinV + SWEEP_STEP_MV) {
            // Hit bottom — reverse
            sweepGoingUp  = true;
            sweepCurrentV = sweepMinV;
            Serial.println(F("── End of sweep — reversing ──"));
        } else {
            sweepCurrentV -= SWEEP_STEP_MV;
        }
    }

    // Request new voltage
    if (!requestSweepVoltage(sweepCurrentV)) {
        Serial.printf("[WARN] Request for %u mV rejected (out of range?)\n",
                      sweepCurrentV);
        sweepCurrentV = sweepMinV;
        sweepGoingUp  = true;
        requestSweepVoltage(sweepCurrentV);
    }
}