/**
 * 05_FullSystem.ino
 * ──────────────────────────────────────────────────────────────────────────
 * Production-ready state machine with Serial command interface.
 *
 * State machine:
 *   INIT → WAIT_CABLE → WAIT_PDO → NEGOTIATE → RUNNING ↔ FAULT
 *
 * Serial commands (send via Serial Monitor at 115200 baud):
 *   V<mV>    — request voltage, e.g. V12000
 *   SCAN     — re-read and print all PDOs
 *   DUMP     — print all registers
 *   RESET    — issue PD hard reset
 *   STATUS   — print current state and measurements
 *   HELP     — list commands
 *
 * IMPORTANT: pd.task() is called every loop() iteration.
 *            Without it, PPS/AVS voltage will drop to 5V after ~5s.
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

// ── Defaults ─────────────────────────────────────────────────────────────────
#define DEFAULT_TARGET_MV  12000
#define DEFAULT_MIN_MA      2000
#define MAX_FAULT_RETRIES      5

// ── State machine ─────────────────────────────────────────────────────────────
enum class State : uint8_t {
    INIT,
    WAIT_CABLE,
    WAIT_PDO,
    NEGOTIATE,
    RUNNING,
    FAULT
};

AP33772S pd(Wire, PIN_INT);

State    state         = State::INIT;
uint16_t targetMV      = DEFAULT_TARGET_MV;
uint16_t minMA         = DEFAULT_MIN_MA;
uint8_t  activePDO     = 0;
uint8_t  faultRetries  = 0;
uint32_t faultTimer    = 0;

volatile bool intFlag  = false;

// ── ISR ───────────────────────────────────────────────────────────────────────
#ifdef ARDUINO_ARCH_RP2040
void pdISR() { intFlag = true; }
#else
void IRAM_ATTR pdISR() { intFlag = true; }
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────
const char* stateName(State s)
{
    switch(s) {
        case State::INIT:       return "INIT";
        case State::WAIT_CABLE: return "WAIT_CABLE";
        case State::WAIT_PDO:   return "WAIT_PDO";
        case State::NEGOTIATE:  return "NEGOTIATE";
        case State::RUNNING:    return "RUNNING";
        case State::FAULT:      return "FAULT";
        default:                return "?";
    }
}

const char* pdoTypeName(uint8_t t)
{
    return t == PDO_TYPE_FIXED ? "Fixed" :
           t == PDO_TYPE_PPS   ? "PPS"   : "AVS";
}

// ─────────────────────────────────────────────────────────────────────────────
//  doNegotiate() — shared negotiation logic
// ─────────────────────────────────────────────────────────────────────────────
bool doNegotiate()
{
    Serial.printf("[NEGO] Requesting %u mV / min %u mA\n", targetMV, minMA);

    pd.readAllPDOs();
    activePDO = pd.setVoltage(targetMV, minMA);

    if (!activePDO) {
        Serial.println(F("[NEGO] Target not achievable — using 5V"));
        pd.setFixPDO(1, 1500);
        activePDO = 1;
    } else {
        const AP33772S_PDO &p = pd.getPDO(activePDO);
        Serial.printf("[NEGO] PDO%d (%s) maxV=%umV maxI=%umA\n",
                      activePDO, pdoTypeName(p.type),
                      p.maxVoltage_mV, p.maxCurrent_mA);
    }

    if (pd.waitForNegotiation(3000) == AP33772S_OK) {
        Serial.printf("[NEGO] OK  →  VREQ=%umV  IREQ=%umA\n",
                      pd.getRequestedVoltage_mV(),
                      pd.getRequestedCurrent_mA());
        return true;
    }

    Serial.println(F("[NEGO] Failed — no confirmation from charger"));
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  handleInterrupt() — called from loop(), not ISR context
// ─────────────────────────────────────────────────────────────────────────────
void handleInterrupt()
{
    intFlag = false;
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

    if (status & STATUS_NEWPDO) {
        Serial.println(F("[INT]  New PDOs received — re-negotiating"));
        state = State::NEGOTIATE;
    }

    if (status & STATUS_FAULTS) {
        Serial.printf("[INT]  %s — entering fault recovery\n",
                      pd.getFaultString().c_str());
        state      = State::FAULT;
        faultTimer = millis();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  checkSerial() — Serial command parser
// ─────────────────────────────────────────────────────────────────────────────
void checkSerial()
{
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HELP") {
        Serial.println(F("Commands:"));
        Serial.println(F("  V<mV>  — request voltage  (e.g. V12000)"));
        Serial.println(F("  SCAN   — re-read PDO list"));
        Serial.println(F("  DUMP   — print all registers"));
        Serial.println(F("  RESET  — PD hard reset"));
        Serial.println(F("  STATUS — current state and measurements"));
        Serial.println(F("  HELP   — this list"));

    } else if (cmd.startsWith("V")) {
        int32_t v = cmd.substring(1).toInt();
        if (v >= 3300 && v <= 28000) {
            targetMV = (uint16_t)v;
            Serial.printf("[CMD]  New target: %u mV\n", targetMV);
            state = State::NEGOTIATE;
        } else {
            Serial.println(F("[CMD]  Range: V3300 to V28000"));
        }

    } else if (cmd == "SCAN") {
        pd.readAllPDOs();
        pd.printPDOs(Serial);

    } else if (cmd == "DUMP") {
        pd.dumpRegisters(Serial);

    } else if (cmd == "RESET") {
        Serial.println(F("[CMD]  Hard reset issued"));
        pd.issueHardReset();
        state = State::WAIT_CABLE;
        faultRetries = 0;

    } else if (cmd == "STATUS") {
        Serial.printf("[STATUS] State=%-12s  V=%umV  I=%umA  P=%umW  T=%d°C\n",
                      stateName(state),
                      pd.getVoltage_mV(), pd.getCurrent_mA(),
                      pd.getPower_mW(),   pd.getTemperature_C());
        Serial.printf("         VREQ=%umV  IREQ=%umA  PDO=%d\n",
                      pd.getRequestedVoltage_mV(),
                      pd.getRequestedCurrent_mA(),
                      activePDO);
        Serial.printf("         PD=%d  Legacy=%d  Flip=%d  DR=%d  Fault=%d\n",
                      pd.isPDConnected(), pd.isLegacyConnected(),
                      pd.isCableFlipped(), pd.isDerating(), pd.isFault());

    } else if (cmd.length() > 0) {
        Serial.printf("[CMD]  Unknown: '%s'  (type HELP)\n", cmd.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  runStateMachine()
// ─────────────────────────────────────────────────────────────────────────────
void runStateMachine()
{
    switch (state) {

    // ── INIT ──────────────────────────────────────────────────────────────
    case State::INIT: {
        Wire.begin(PIN_SDA, PIN_SCL);
        Wire.setClock(400000);

        if (pd.begin() != AP33772S_OK) {
            Serial.println(F("[INIT] I2C failed — retrying in 2s"));
            delay(2000);
            return;
        }

        // Protection thresholds
        pd.setOVPOffset_mV(2000);
        pd.setUVPThreshold(UVP_80PCT);
        pd.setOCPThreshold_mA(0);      // auto = 110% of PDO
        pd.setOTPThreshold_C(85);
        pd.setDeratingThreshold_C(75);
        pd.setProtectionConfig(true, true, true, true, true);

        // Interrupts
        pd.setInterruptMask(MASK_ALL);
        pd.attachInterruptCallback(pdISR);

        Serial.println(F("[INIT] AP33772S ready — waiting for cable"));
        Serial.println(F("[INIT] Type HELP for commands\n"));
        state = State::WAIT_CABLE;
        break;
    }

    // ── WAIT_CABLE ────────────────────────────────────────────────────────
    case State::WAIT_CABLE: {
        static uint32_t pollT = 0;
        if (millis() - pollT < 200) return;
        pollT = millis();

        int16_t s = pd.readReg8(CMD_STATUS);
        if (s >= 0 && (s & STATUS_STARTED)) {
            Serial.println(F("[CABLE] Cable attached — waiting for PDOs"));
            state = State::WAIT_PDO;
        }
        break;
    }

    // ── WAIT_PDO ──────────────────────────────────────────────────────────
    case State::WAIT_PDO: {
        static uint32_t pollT   = 0;
        static uint32_t enteredAt = 0;
        if (enteredAt == 0) enteredAt = millis();
        if (millis() - pollT < 200) return;
        pollT = millis();

        int16_t s = pd.readReg8(CMD_STATUS);
        bool timedOut = (millis() - enteredAt > 8000);

        if ((s >= 0 && (s & STATUS_NEWPDO) && (s & STATUS_READY)) || timedOut) {
            if (timedOut) Serial.println(F("[PDO]  Timeout — trying anyway"));
            else          Serial.println(F("[PDO]  Source capabilities received"));

            enteredAt = 0;
            pd.readAllPDOs();
            pd.printPDOs(Serial);
            Serial.println();

            // Show PPS/AVS availability
            int8_t pi = pd.getPPSIndex(), ai = pd.getAVSIndex();
            if (pi > 0) Serial.printf("[INFO] PPS at PDO%d\n", pi);
            if (ai > 0) Serial.printf("[INFO] AVS at PDO%d\n", ai);

            state = State::NEGOTIATE;
        }
        break;
    }

    // ── NEGOTIATE ─────────────────────────────────────────────────────────
    case State::NEGOTIATE: {
        if (doNegotiate()) {
            state = State::RUNNING;
            faultRetries = 0;
        } else {
            state      = State::FAULT;
            faultTimer = millis();
        }
        break;
    }

    // ── RUNNING — nothing to do here, task() handles keep-alive ──────────
    case State::RUNNING:
        break;

    // ── FAULT — exponential back-off recovery ─────────────────────────────
    case State::FAULT: {
        // Back-off: 1s, 2s, 4s, 8s, 16s, capped at 30s
        uint32_t backoff = (uint32_t)1000 << min(faultRetries, (uint8_t)5);
        backoff = min(backoff, (uint32_t)30000);

        if (millis() - faultTimer < backoff) return;

        faultRetries++;
        Serial.printf("[FAULT] Recovery attempt %d (backoff %lus)\n",
                      faultRetries, backoff / 1000);

        if (faultRetries > MAX_FAULT_RETRIES) {
            Serial.println(F("[FAULT] Max retries reached — issuing hard reset"));
            pd.issueHardReset();
            faultRetries = 0;
            state        = State::WAIT_CABLE;
            faultTimer   = millis();
            return;
        }

        // Step 1: safe 5V
        pd.clearInterrupt();
        pd.setFixPDO(1, 1500);
        if (pd.waitForNegotiation(3000) == AP33772S_OK) {
            Serial.println(F("[FAULT] 5V restored — waiting before ramp-up"));
            delay(1500);
            state = State::NEGOTIATE;  // Will re-request targetMV
        } else {
            // 5V also failed — wait for next retry cycle
            faultTimer = millis();
        }
        break;
    }
    } // end switch
}

// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry — only printed in RUNNING state
// ─────────────────────────────────────────────────────────────────────────────
void printTelemetry()
{
    if (state != State::RUNNING) return;
    static uint32_t last = 0;
    if (millis() - last < 1000) return;
    last = millis();

    Serial.printf("V=%5umV  I=%4umA  P=%5umW  T=%3d°C  VREQ=%5umV",
                  pd.getVoltage_mV(),
                  pd.getCurrent_mA(),
                  pd.getPower_mW(),
                  pd.getTemperature_C(),
                  pd.getRequestedVoltage_mV());

    if (pd.isDerating()) Serial.print(F("  [DR]"));
    if (pd.isFault())    Serial.printf("  [%s]", pd.getFaultString().c_str());
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup() / loop()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println(F("\n╔══════════════════════════════════╗"));
    Serial.println(F("║  AP33772S Full System  v2.0      ║"));
    Serial.println(F("║  Type HELP for commands           ║"));
    Serial.println(F("╚══════════════════════════════════╝\n"));
}

void loop()
{
    pd.task();          // ← keep-alive for PPS/AVS — ALWAYS first
    checkSerial();      // Handle Serial commands
    if (intFlag) handleInterrupt();  // Handle INT pin events
    runStateMachine();  // Advance state machine
    printTelemetry();   // Print measurements when running
}