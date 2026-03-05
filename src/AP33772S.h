/**
 * AP33772S.h  —  Arduino Library for AP33772S USB PD 3.1 Sink Controller
 * ============================================================
 * Based on official CentyLab PicoPD library bit field definitions.
 * Targets: RP2040 (Arduino-Pico), ESP32 (Arduino)
 * Datasheet: DS46176 Rev. 9-2, Feb 2026  |  I2C: 0x52
 * ============================================================
 */

#ifndef AP33772S_H
#define AP33772S_H

#include <Arduino.h>
#include <Wire.h>

// ── I2C ──────────────────────────────────────────────────────────────────────
#define AP33772S_ADDRESS     (0x52)

// ── Buffer sizes ─────────────────────────────────────────────────────────────
#define READ_BUFF_LEN        (128)
#define WRITE_BUFF_LEN       (6)
#define MAX_PDO_ENTRIES      (13)
#define SRCPDO_BYTES         (26)   // 13 PDOs × 2 bytes each

// ── Register addresses ────────────────────────────────────────────────────────
#define CMD_STATUS           (0x01)  // RC  — clears on read
#define CMD_MASK             (0x02)
#define CMD_OPMODE           (0x03)
#define CMD_CONFIG           (0x04)
#define CMD_PDCONFIG         (0x05)
#define CMD_SYSTEM           (0x06)
#define CMD_TR25             (0x0C)  // NTC table
#define CMD_TR50             (0x0D)
#define CMD_TR75             (0x0E)
#define CMD_TR100            (0x0F)
#define CMD_VOLTAGE          (0x11)  // LSB = 80 mV
#define CMD_CURRENT          (0x12)  // LSB = 24 mA
#define CMD_TEMP             (0x13)  // unit = °C
#define CMD_VREQ             (0x14)  // LSB = 50 mV
#define CMD_IREQ             (0x15)  // LSB = 10 mA
#define CMD_VSELMIN          (0x16)  // LSB = 200 mV, default 5000 mV
#define CMD_UVPTHR           (0x17)
#define CMD_OVPTHR           (0x18)  // LSB = 80 mV
#define CMD_OCPTHR           (0x19)  // LSB = 50 mA
#define CMD_OTPTHR           (0x1A)  // unit = °C
#define CMD_DRTHR            (0x1B)  // unit = °C
#define CMD_SRCPDO           (0x20)  // 26-byte burst read
#define CMD_PD_REQMSG        (0x31)
#define CMD_PD_CMDMSG        (0x32)
#define CMD_PD_MSGRLT        (0x33)

// ── STATUS bits ───────────────────────────────────────────────────────────────
#define STATUS_STARTED       (1 << 0)
#define STATUS_READY         (1 << 1)
#define STATUS_NEWPDO        (1 << 2)
#define STATUS_UVP           (1 << 3)
#define STATUS_OVP           (1 << 4)
#define STATUS_OCP           (1 << 5)
#define STATUS_OTP           (1 << 6)
#define STATUS_FAULTS        (STATUS_UVP|STATUS_OVP|STATUS_OCP|STATUS_OTP)

// ── MASK bits ─────────────────────────────────────────────────────────────────
#define MASK_STARTED         (1 << 0)
#define MASK_READY           (1 << 1)
#define MASK_NEWPDO          (1 << 2)
#define MASK_UVP             (1 << 3)
#define MASK_OVP             (1 << 4)
#define MASK_OCP             (1 << 5)
#define MASK_OTP             (1 << 6)
#define MASK_ALL             (0x7F)

// ── OPMODE bits ───────────────────────────────────────────────────────────────
#define OPMODE_LGCYMOD       (1 << 0)
#define OPMODE_PDMOD         (1 << 1)
#define OPMODE_DR            (1 << 6)
#define OPMODE_CCFLIP        (1 << 7)

// ── CONFIG bits ───────────────────────────────────────────────────────────────
#define CONFIG_UVP_EN        (1 << 3)
#define CONFIG_OVP_EN        (1 << 4)
#define CONFIG_OCP_EN        (1 << 5)
#define CONFIG_OTP_EN        (1 << 6)
#define CONFIG_DR_EN         (1 << 7)
#define CONFIG_ALL_EN        (0xF8)

// ── PDCONFIG bits ─────────────────────────────────────────────────────────────
#define PDCFG_EPR_EN         (1 << 0)
#define PDCFG_PPS_EN         (1 << 1)

// ── PD_MSGRLT ────────────────────────────────────────────────────────────────
#define MSGRLT_SUCCESS       (1 << 0)

// ── SYSTEM register values ───────────────────────────────────────────────────
#define SYSTEM_OUTPUT_OFF    (0b00010001)
#define SYSTEM_OUTPUT_ON     (0b00010010)

// ── UVP modes ────────────────────────────────────────────────────────────────
#define UVP_80PCT            (1)
#define UVP_75PCT            (2)
#define UVP_70PCT            (3)

// ── Keep-alive interval for PPS/AVS (ms) ─────────────────────────────────────
#define AVS_KEEPALIVE_MS     (500)

// ── PDO type constants ────────────────────────────────────────────────────────
#define PDO_TYPE_FIXED       (0)
#define PDO_TYPE_PPS         (1)   // type=1 in SPR slot (index 1–7)
#define PDO_TYPE_AVS         (2)   // type=1 in EPR slot (index 8–13)

// ── Return codes ─────────────────────────────────────────────────────────────
#define AP33772S_OK          (0)
#define AP33772S_ERR_I2C     (-1)
#define AP33772S_ERR_TIMEOUT (-2)
#define AP33772S_ERR_RANGE   (-3)
#define AP33772S_ERR_TYPE    (-4)
#define AP33772S_ERR_NEGO    (-5)

// ─────────────────────────────────────────────────────────────────────────────
//  PDO bit-field structure — matches official CentyLab / Diodes Inc. definition
//
//  16-bit compressed PDO layout (little-endian from I2C):
//    bit  15    : detect    — 1 = PDO slot is populated
//    bit  14    : type      — 0 = Fixed,  1 = PPS (SPR) or AVS (EPR)
//    bits 13:10 : current_max  (4-bit, maps via currentMap/currentDecode)
//    bits  9:8  : voltage_min  (2-bit indicator) or peak_current (Fixed)
//    bits  7:0  : voltage_max  (8-bit)
//                  SPR: voltage_max × 100 mV
//                  EPR: voltage_max × 200 mV
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    union {
        struct {
            unsigned int voltage_max  : 8;  // bits  7:0
            unsigned int peak_current : 2;  // bits  9:8  (Fixed PDO)
            unsigned int current_max  : 4;  // bits 13:10
            unsigned int type         : 1;  // bit  14
            unsigned int detect       : 1;  // bit  15
        } fixed;
        struct {
            unsigned int voltage_max  : 8;  // bits  7:0
            unsigned int voltage_min  : 2;  // bits  9:8
            unsigned int current_max  : 4;  // bits 13:10
            unsigned int type         : 1;  // bit  14
            unsigned int detect       : 1;  // bit  15
        } pps;
        struct {
            unsigned int voltage_max  : 8;  // bits  7:0
            unsigned int voltage_min  : 2;  // bits  9:8
            unsigned int current_max  : 4;  // bits 13:10
            unsigned int type         : 1;  // bit  14
            unsigned int detect       : 1;  // bit  15
        } avs;
        struct {
            uint8_t byte0;  // LSB (voltage_max + lower bits of byte1)
            uint8_t byte1;  // MSB (detect, type, current_max, voltage_min)
        };
    };
} PDO_DATA_T;

// ── Decoded, human-readable PDO ──────────────────────────────────────────────
struct AP33772S_PDO {
    uint8_t  index;          // 1-based (1–13)
    bool     valid;          // detect bit = 1
    bool     isEPR;          // true for index 8–13
    uint8_t  type;           // PDO_TYPE_FIXED / _PPS / _AVS
    uint16_t minVoltage_mV;  // 0 for Fixed; 3300 for PPS; 15000 for AVS
    uint16_t maxVoltage_mV;
    uint16_t maxCurrent_mA;  // Approximate upper bound of current range
    uint8_t  currentCode;    // Raw 4-bit current_max field (for CURRENT_SEL)
    uint16_t raw;            // Raw 16-bit register value
};

// ─────────────────────────────────────────────────────────────────────────────
//  Main class
// ─────────────────────────────────────────────────────────────────────────────
class AP33772S {
public:
    explicit AP33772S(TwoWire &wire = Wire, int8_t intPin = -1);

    // ── Init ────────────────────────────────────────────────────────────────
    int8_t  begin(bool enableEPR = true, bool enablePPS = true);
    int8_t  waitForStartup(uint32_t timeoutMs = 2000);
    int8_t  waitForPDOs(uint32_t timeoutMs = 8000);

    // ── PDO discovery ───────────────────────────────────────────────────────
    uint8_t             readAllPDOs();
    bool                readPDO(uint8_t index, AP33772S_PDO &pdo);
    const AP33772S_PDO& getPDO(uint8_t index) const;
    uint8_t             getValidPDOCount()  const { return _validPDOCount; }
    int8_t              getPPSIndex()       const; // First PPS slot, or -1
    int8_t              getAVSIndex()       const; // First AVS slot, or -1
    void                printPDOs(Stream &s = Serial);

    // ── Voltage requests ────────────────────────────────────────────────────
    // All mV and mA parameters are in those units

    int8_t  setFixPDO(uint8_t pdoIndex, uint16_t maxCurrent_mA);
    int8_t  setPPSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA);
    int8_t  setAVSPDO(uint8_t pdoIndex, uint16_t voltage_mV, uint16_t maxCurrent_mA);

    /**
     * Auto-select best PDO for target voltage and request it.
     * Prefers PPS/AVS for exact voltage, falls back to closest Fixed.
     * @return selected PDO index, 0 on failure
     */
    uint8_t setVoltage(uint16_t voltage_mV, uint16_t minCurrent_mA = 1000);

    int8_t  waitForNegotiation(uint32_t timeoutMs = 3000);
    int8_t  issueHardReset();

    /**
     * MUST be called in loop() when using PPS or AVS.
     * Re-sends the RDO every AVS_KEEPALIVE_MS to maintain the voltage.
     * Without this, the charger will revert to fixed 5V after ~5s.
     */
    void    task();

    // ── Output switch ───────────────────────────────────────────────────────
    bool    setOutput(bool on);   ///< Control NMOS VOUT switch via SYSTEM reg

    // ── Status ──────────────────────────────────────────────────────────────
    uint8_t getStatus();
    uint8_t getOpMode();
    uint8_t getMsgResult();
    bool    isPDConnected();
    bool    isLegacyConnected();
    bool    isCableFlipped();
    bool    isDerating();
    bool    isFault();
    String  getFaultString();

    // ── ADC readings ────────────────────────────────────────────────────────
    uint16_t getVoltage_mV();
    uint16_t getCurrent_mA();
    uint32_t getPower_mW();
    int8_t   getTemperature_C();
    uint16_t getRequestedVoltage_mV();
    uint16_t getRequestedCurrent_mA();

    // ── Protection thresholds ───────────────────────────────────────────────
    bool setOVPOffset_mV(uint16_t offset_mV);
    bool setUVPThreshold(uint8_t uvpMode);   // UVP_80PCT / 75PCT / 70PCT
    bool setOCPThreshold_mA(uint16_t mA);   // 0 = auto (110% PDO max)
    bool setOTPThreshold_C(uint8_t degC);
    bool setDeratingThreshold_C(uint8_t degC);
    bool setMinVoltage_mV(uint16_t mV);
    bool setProtectionConfig(bool uvp, bool ovp, bool ocp, bool otp, bool dr);

    // ── Interrupts ──────────────────────────────────────────────────────────
    bool    setInterruptMask(uint8_t mask);
    uint8_t clearInterrupt();
    void    attachInterruptCallback(void (*cb)());

    // ── NTC calibration ─────────────────────────────────────────────────────
    bool setNTC(uint16_t r25 = 10000, uint16_t r50 = 4161,
                uint16_t r75 = 1928,  uint16_t r100 = 974);

    // ── Raw register I/O ────────────────────────────────────────────────────
    int16_t  readReg8(uint8_t reg);
    int32_t  readReg16(uint8_t reg);
    bool     writeReg8(uint8_t reg, uint8_t val);
    bool     writeReg16(uint8_t reg, uint16_t val);
    bool     readBytes(uint8_t reg, uint8_t *buf, uint8_t len);
    void     dumpRegisters(Stream &s = Serial);

private:
    TwoWire     &_wire;
    int8_t       _intPin;
    PDO_DATA_T   _pdoRaw[MAX_PDO_ENTRIES];  // Raw bit-field structs
    AP33772S_PDO _pdoDecoded[MAX_PDO_ENTRIES];
    uint8_t      _validPDOCount;

    // Keep-alive state for PPS/AVS
    bool     _keepAliveActive;
    uint32_t _keepAliveTimer;
    uint8_t  _keepAlivePDOIndex;
    uint16_t _keepAliveVoltageSel;  // Already divided (raw VOLTAGE_SEL value)
    uint8_t  _keepAliveCurrentSel;
    bool     _keepAliveIsAVS;

    // Helpers
    void    _decodePDO(uint8_t idx);
    void    _sendRDO(uint8_t pdoIndex, uint8_t currentSel, uint8_t voltageSel);

    // Current encode/decode — matches official CentyLab currentMap exactly
    // Input: mA (e.g. 3000 for 3A), Output: 0–15 for CURRENT_SEL field
    uint8_t  _currentEncode(uint16_t mA);
    // Input: 0–15 code, Output: approximate lower-bound mA
    uint16_t _currentDecode(uint8_t code);
};

#endif // AP33772S_H