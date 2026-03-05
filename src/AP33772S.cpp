/**
 * AP33772S.cpp  —  Implementation
 * Bit field decoding matches official CentyLab PicoPD library.
 * Key corrections vs naive decode:
 *   - bits[7:0]   = voltage_max   (NOT bits[13:8])
 *   - bits[13:10] = current_max   (4-bit range code)
 *   - bit[14]     = type          (NOT bits[15:14])
 *   - bit[15]     = detect/valid  (NOT part of type)
 *   - SPR voltage = voltage_max × 100mV
 *   - EPR voltage = voltage_max × 200mV
 *   - PPS VOLTAGE_SEL = target_mV / 100
 *   - AVS VOLTAGE_SEL = target_mV / 200
 */

#include "AP33772S.h"

// ── Constructor ──────────────────────────────────────────────────────────────
AP33772S::AP33772S(TwoWire &wire, int8_t intPin)
    : _wire(wire),
      _intPin(intPin),
      _validPDOCount(0),
      _keepAliveActive(false),
      _keepAliveTimer(0),
      _keepAlivePDOIndex(0),
      _keepAliveVoltageSel(0),
      _keepAliveCurrentSel(0),
      _keepAliveIsAVS(false)
{
    memset(_pdoRaw,     0, sizeof(_pdoRaw));
    memset(_pdoDecoded, 0, sizeof(_pdoDecoded));
}

// ── begin() ──────────────────────────────────────────────────────────────────
int8_t AP33772S::begin(bool enableEPR, bool enablePPS)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    if (_wire.endTransmission() != 0) return AP33772S_ERR_I2C;

    if (_intPin >= 0) pinMode(_intPin, INPUT);

    uint8_t pdcfg = 0;
    if (enableEPR) pdcfg |= PDCFG_EPR_EN;
    if (enablePPS) pdcfg |= PDCFG_PPS_EN;
    if (!writeReg8(CMD_PDCONFIG, pdcfg))    return AP33772S_ERR_I2C;
    if (!writeReg8(CMD_CONFIG,   CONFIG_ALL_EN)) return AP33772S_ERR_I2C;
    if (!writeReg8(CMD_MASK,     MASK_ALL))  return AP33772S_ERR_I2C;

    return AP33772S_OK;
}

// ── waitForStartup() ─────────────────────────────────────────────────────────
int8_t AP33772S::waitForStartup(uint32_t timeoutMs)
{
    uint32_t dl = millis() + timeoutMs;
    while (millis() < dl) {
        int16_t s = readReg8(CMD_STATUS);
        if (s >= 0 && (s & STATUS_STARTED)) return AP33772S_OK;
        delay(10);
    }
    return AP33772S_ERR_TIMEOUT;
}

// ── waitForPDOs() ────────────────────────────────────────────────────────────
int8_t AP33772S::waitForPDOs(uint32_t timeoutMs)
{
    uint32_t dl = millis() + timeoutMs;
    while (millis() < dl) {
        int16_t s = readReg8(CMD_STATUS);
        if (s >= 0 && (s & STATUS_NEWPDO) && (s & STATUS_READY))
            return AP33772S_OK;
        delay(50);
    }
    return AP33772S_ERR_TIMEOUT;
}

// ── _currentEncode()  — matches official CentyLab currentMap ────────────────
// Input: milliamps (1000–5000), Output: 0–15 CURRENT_SEL register code
uint8_t AP33772S::_currentEncode(uint16_t mA)
{
    if (mA < 1250) return 0;
    int code = ((int)mA - 1250) / 250 + 1;
    return (code > 15) ? 15 : (uint8_t)code;
}

// ── _currentDecode()  — reverse map from code to approximate mA ─────────────
uint16_t AP33772S::_currentDecode(uint8_t code)
{
    if (code == 0)  return 1000;   // < 1.25A, show as ~1A
    if (code == 15) return 5000;   // ≥ 5A
    return (uint16_t)(1250 + (code - 1) * 250);
}

// ── _decodePDO()  — decode bit fields into human-readable struct ─────────────
void AP33772S::_decodePDO(uint8_t idx)
{
    // idx is 0-based internally (0–12 → PDO1–PDO13)
    PDO_DATA_T   &raw = _pdoRaw[idx];
    AP33772S_PDO &dec = _pdoDecoded[idx];

    dec.index    = idx + 1;
    dec.raw      = (uint16_t)raw.byte0 | ((uint16_t)raw.byte1 << 8);
    dec.valid    = (raw.fixed.detect == 1);
    dec.isEPR    = (idx >= 7);  // 0-based: 7–12 → PDO8–PDO13

    if (!dec.valid) {
        dec.type = dec.minVoltage_mV = dec.maxVoltage_mV =
        dec.maxCurrent_mA = dec.currentCode = 0;
        return;
    }

    dec.currentCode   = raw.fixed.current_max;
    dec.maxCurrent_mA = _currentDecode(dec.currentCode);

    if (raw.fixed.type == 0) {
        // ── Fixed PDO ─────────────────────────────────────────────────────
        dec.type = PDO_TYPE_FIXED;
        dec.maxVoltage_mV = dec.isEPR
            ? (uint16_t)raw.fixed.voltage_max * 200   // EPR: 200 mV/unit
            : (uint16_t)raw.fixed.voltage_max * 100;  // SPR: 100 mV/unit
        dec.minVoltage_mV = dec.maxVoltage_mV;  // Fixed = single point

    } else if (!dec.isEPR) {
        // ── PPS (type=1 in SPR slot, index 1–7) ──────────────────────────
        dec.type          = PDO_TYPE_PPS;
        dec.maxVoltage_mV = (uint16_t)raw.pps.voltage_max * 100;  // 100 mV/unit
        // Decode minimum voltage from 2-bit indicator
        switch (raw.pps.voltage_min) {
            case 1:  dec.minVoltage_mV = 3300;  break;
            case 2:  dec.minVoltage_mV = 5000;  break;  // 3300mV < min ≤ 5000mV
            default: dec.minVoltage_mV = 3300;  break;
        }

    } else {
        // ── AVS (type=1 in EPR slot, index 8–13) ─────────────────────────
        dec.type          = PDO_TYPE_AVS;
        dec.maxVoltage_mV = (uint16_t)raw.avs.voltage_max * 200;  // 200 mV/unit
        switch (raw.avs.voltage_min) {
            case 1:  dec.minVoltage_mV = 15000; break;
            case 2:  dec.minVoltage_mV = 20000; break;  // 15000mV < min ≤ 20000mV
            default: dec.minVoltage_mV = 15000; break;
        }
    }
}

// ── readAllPDOs() ────────────────────────────────────────────────────────────
uint8_t AP33772S::readAllPDOs()
{
    uint8_t buf[SRCPDO_BYTES];
    if (!readBytes(CMD_SRCPDO, buf, SRCPDO_BYTES)) return 0;

    _validPDOCount = 0;
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++) {
        _pdoRaw[i].byte0 = buf[i * 2];
        _pdoRaw[i].byte1 = buf[i * 2 + 1];
        _decodePDO(i);
        if (_pdoDecoded[i].valid) _validPDOCount++;
    }
    return _validPDOCount;
}

// ── readPDO() ────────────────────────────────────────────────────────────────
bool AP33772S::readPDO(uint8_t index, AP33772S_PDO &pdo)
{
    if (index < 1 || index > 13) return false;
    uint8_t reg = 0x21 + (index - 1);  // SPR_PDO1=0x21 … EPR_PDO13=0x2D
    uint8_t buf[2];
    if (!readBytes(reg, buf, 2)) return false;
    uint8_t i = index - 1;
    _pdoRaw[i].byte0 = buf[0];
    _pdoRaw[i].byte1 = buf[1];
    _decodePDO(i);
    pdo = _pdoDecoded[i];
    return true;
}

// ── getPDO() ─────────────────────────────────────────────────────────────────
const AP33772S_PDO& AP33772S::getPDO(uint8_t index) const
{
    if (index < 1 || index > 13) index = 13;
    return _pdoDecoded[index - 1];
}

// ── getPPSIndex() / getAVSIndex() ────────────────────────────────────────────
int8_t AP33772S::getPPSIndex() const
{
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++)
        if (_pdoDecoded[i].valid && _pdoDecoded[i].type == PDO_TYPE_PPS)
            return (int8_t)_pdoDecoded[i].index;
    return -1;
}

int8_t AP33772S::getAVSIndex() const
{
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++)
        if (_pdoDecoded[i].valid && _pdoDecoded[i].type == PDO_TYPE_AVS)
            return (int8_t)_pdoDecoded[i].index;
    return -1;
}

// ── printPDOs() ──────────────────────────────────────────────────────────────
void AP33772S::printPDOs(Stream &s)
{
    s.println(F("┌─────┬──────────┬────────────┬────────────┬────────────┬────────┐"));
    s.println(F("│ IDX │ TYPE     │ MIN V (mV) │ MAX V (mV) │ MAX I (mA) │  RAW   │"));
    s.println(F("├─────┼──────────┼────────────┼────────────┼────────────┼────────┤"));
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid) continue;
        const char *ts = (p.type == PDO_TYPE_FIXED) ? "Fixed" :
                         (p.type == PDO_TYPE_PPS)   ? "PPS"   : "AVS";
        char row[96];
        snprintf(row, sizeof(row),
                 "│ %-3d │ %-8s │ %-10u │ %-10u │ %-10u │ 0x%04X │",
                 p.index, ts,
                 p.minVoltage_mV, p.maxVoltage_mV, p.maxCurrent_mA, p.raw);
        s.println(row);
    }
    s.println(F("└─────┴──────────┴────────────┴────────────┴────────────┴────────┘"));
}

// ── _sendRDO() — write PD_REQMSG ─────────────────────────────────────────────
// Layout: bits[15:12]=PDO_INDEX, bits[11:8]=CURRENT_SEL, bits[7:0]=VOLTAGE_SEL
void AP33772S::_sendRDO(uint8_t pdoIndex, uint8_t currentSel, uint8_t voltageSel)
{
    uint8_t b0 = voltageSel;
    uint8_t b1 = (uint8_t)(((pdoIndex & 0x0F) << 4) | (currentSel & 0x0F));
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(CMD_PD_REQMSG);
    _wire.write(b0);
    _wire.write(b1);
    _wire.endTransmission();
}

// ── setFixPDO() ──────────────────────────────────────────────────────────────
int8_t AP33772S::setFixPDO(uint8_t pdoIndex, uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13) return AP33772S_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)              return AP33772S_ERR_RANGE;
    if (p.type != PDO_TYPE_FIXED) return AP33772S_ERR_TYPE;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) {
        // Requested more current than PDO offers — cap at PDO max
        curSel = p.currentCode;
    }

    _keepAliveActive = false;  // Cancel any running keep-alive
    _sendRDO(pdoIndex, curSel, 0x00);
    return AP33772S_OK;
}

// ── setPPSPDO() ──────────────────────────────────────────────────────────────
int8_t AP33772S::setPPSPDO(uint8_t pdoIndex, uint16_t voltage_mV,
                            uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13) return AP33772S_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)             return AP33772S_ERR_RANGE;
    if (p.type != PDO_TYPE_PPS) return AP33772S_ERR_TYPE;

    // Voltage range check
    if (voltage_mV < p.minVoltage_mV || voltage_mV > p.maxVoltage_mV)
        return AP33772S_ERR_RANGE;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) curSel = p.currentCode;

    // PPS VOLTAGE_SEL: 100 mV per unit
    uint8_t volSel = (uint8_t)(voltage_mV / 100);

    // Setup keep-alive (PPS must be refreshed periodically)
    _keepAliveActive     = true;
    _keepAliveTimer      = millis();
    _keepAlivePDOIndex   = pdoIndex;
    _keepAliveVoltageSel = volSel;
    _keepAliveCurrentSel = curSel;
    _keepAliveIsAVS      = false;

    _sendRDO(pdoIndex, curSel, volSel);
    return AP33772S_OK;
}

// ── setAVSPDO() ──────────────────────────────────────────────────────────────
int8_t AP33772S::setAVSPDO(uint8_t pdoIndex, uint16_t voltage_mV,
                            uint16_t maxCurrent_mA)
{
    if (pdoIndex < 1 || pdoIndex > 13) return AP33772S_ERR_RANGE;
    const AP33772S_PDO &p = _pdoDecoded[pdoIndex - 1];
    if (!p.valid)             return AP33772S_ERR_RANGE;
    if (p.type != PDO_TYPE_AVS) return AP33772S_ERR_TYPE;

    if (voltage_mV < p.minVoltage_mV || voltage_mV > p.maxVoltage_mV)
        return AP33772S_ERR_RANGE;

    uint8_t curSel = _currentEncode(maxCurrent_mA);
    if (curSel > p.currentCode) curSel = p.currentCode;

    // AVS VOLTAGE_SEL: 200 mV per unit
    uint8_t volSel = (uint8_t)(voltage_mV / 200);

    _keepAliveActive     = true;
    _keepAliveTimer      = millis();
    _keepAlivePDOIndex   = pdoIndex;
    _keepAliveVoltageSel = volSel;
    _keepAliveCurrentSel = curSel;
    _keepAliveIsAVS      = true;

    _sendRDO(pdoIndex, curSel, volSel);
    return AP33772S_OK;
}

// ── setVoltage() — auto-select best PDO ──────────────────────────────────────
uint8_t AP33772S::setVoltage(uint16_t voltage_mV, uint16_t minCurrent_mA)
{
    if (_validPDOCount == 0) readAllPDOs();
    if (_validPDOCount == 0) return 0;

    // Pass 1: look for exact Fixed match
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid || p.type != PDO_TYPE_FIXED) continue;
        if (p.maxVoltage_mV != voltage_mV)         continue;
        if (p.maxCurrent_mA < minCurrent_mA)        continue;
        if (setFixPDO(p.index, minCurrent_mA) == AP33772S_OK) return p.index;
    }

    // Pass 2: PPS/AVS that covers target voltage — pick highest current PDO
    uint8_t  bestIdx = 0;
    uint16_t bestI   = 0;
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid || p.type == PDO_TYPE_FIXED) continue;
        if (p.maxVoltage_mV < voltage_mV)          continue;
        if (p.minVoltage_mV > voltage_mV)           continue;
        if (p.maxCurrent_mA < minCurrent_mA)        continue;
        if (p.maxCurrent_mA > bestI) { bestI = p.maxCurrent_mA; bestIdx = p.index; }
    }
    if (bestIdx) {
        const AP33772S_PDO &p = _pdoDecoded[bestIdx - 1];
        int8_t rc = (p.type == PDO_TYPE_PPS)
            ? setPPSPDO(bestIdx, voltage_mV, minCurrent_mA)
            : setAVSPDO(bestIdx, voltage_mV, minCurrent_mA);
        if (rc == AP33772S_OK) return bestIdx;
    }

    // Pass 3: nearest Fixed (closest above target)
    uint8_t  nearestIdx  = 0;
    uint32_t nearestDiff = UINT32_MAX;
    for (uint8_t i = 0; i < MAX_PDO_ENTRIES; i++) {
        const AP33772S_PDO &p = _pdoDecoded[i];
        if (!p.valid || p.type != PDO_TYPE_FIXED) continue;
        if (p.maxVoltage_mV < voltage_mV)          continue;
        if (p.maxCurrent_mA < minCurrent_mA)        continue;
        uint32_t diff = p.maxVoltage_mV - voltage_mV;
        if (diff < nearestDiff) { nearestDiff = diff; nearestIdx = p.index; }
    }
    if (nearestIdx) {
        if (setFixPDO(nearestIdx, minCurrent_mA) == AP33772S_OK) return nearestIdx;
    }

    return 0;
}

// ── waitForNegotiation() ─────────────────────────────────────────────────────
int8_t AP33772S::waitForNegotiation(uint32_t timeoutMs)
{
    uint32_t dl = millis() + timeoutMs;
    while (millis() < dl) {
        int16_t r = readReg8(CMD_PD_MSGRLT);
        if (r >= 0 && (r & MSGRLT_SUCCESS)) return AP33772S_OK;
        delay(50);
    }
    return AP33772S_ERR_NEGO;
}

// ── issueHardReset() ─────────────────────────────────────────────────────────
int8_t AP33772S::issueHardReset()
{
    _keepAliveActive = false;
    return writeReg8(CMD_PD_CMDMSG, 0x01) ? AP33772S_OK : AP33772S_ERR_I2C;
}

// ── task() — keep-alive for PPS/AVS, call in loop() ─────────────────────────
void AP33772S::task()
{
    if (!_keepAliveActive) return;
    if (millis() - _keepAliveTimer >= AVS_KEEPALIVE_MS) {
        _keepAliveTimer = millis();
        _sendRDO(_keepAlivePDOIndex, _keepAliveCurrentSel, _keepAliveVoltageSel);
    }
}

// ── setOutput() ──────────────────────────────────────────────────────────────
bool AP33772S::setOutput(bool on)
{
    return writeReg8(CMD_SYSTEM, on ? SYSTEM_OUTPUT_ON : SYSTEM_OUTPUT_OFF);
}

// ── Status ───────────────────────────────────────────────────────────────────
uint8_t AP33772S::getStatus()      { int16_t v=readReg8(CMD_STATUS);  return v<0?0:(uint8_t)v; }
uint8_t AP33772S::getOpMode()      { int16_t v=readReg8(CMD_OPMODE);  return v<0?0:(uint8_t)v; }
uint8_t AP33772S::getMsgResult()   { int16_t v=readReg8(CMD_PD_MSGRLT);return v<0?0:(uint8_t)v;}
bool    AP33772S::isPDConnected()     { return getOpMode() & OPMODE_PDMOD;    }
bool    AP33772S::isLegacyConnected() { return getOpMode() & OPMODE_LGCYMOD;  }
bool    AP33772S::isCableFlipped()    { return getOpMode() & OPMODE_CCFLIP;   }
bool    AP33772S::isDerating()        { return getOpMode() & OPMODE_DR;       }
bool    AP33772S::isFault()           { return getStatus() & STATUS_FAULTS;   }

String AP33772S::getFaultString()
{
    uint8_t s = getStatus();
    if (!(s & STATUS_FAULTS)) return "";
    String out = "FAULT:";
    if (s & STATUS_OVP) out += " OVP";
    if (s & STATUS_UVP) out += " UVP";
    if (s & STATUS_OCP) out += " OCP";
    if (s & STATUS_OTP) out += " OTP";
    return out;
}

// ── ADC readings ─────────────────────────────────────────────────────────────
uint16_t AP33772S::getVoltage_mV()
{
    int32_t r = readReg16(CMD_VOLTAGE);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 80);
}
uint16_t AP33772S::getCurrent_mA()
{
    int16_t r = readReg8(CMD_CURRENT);
    return (r < 0) ? 0 : (uint16_t)((uint8_t)r * 24);
}
uint32_t AP33772S::getPower_mW()
{
    return (uint32_t)getVoltage_mV() * getCurrent_mA() / 1000UL;
}
int8_t AP33772S::getTemperature_C()
{
    int16_t r = readReg8(CMD_TEMP);
    return (r < 0) ? 25 : (int8_t)r;
}
uint16_t AP33772S::getRequestedVoltage_mV()
{
    int32_t r = readReg16(CMD_VREQ);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 50);
}
uint16_t AP33772S::getRequestedCurrent_mA()
{
    int32_t r = readReg16(CMD_IREQ);
    return (r < 0) ? 0 : (uint16_t)((uint16_t)r * 10);
}

// ── Protection thresholds ────────────────────────────────────────────────────
bool AP33772S::setOVPOffset_mV(uint16_t mv) { return writeReg8(CMD_OVPTHR, mv/80); }
bool AP33772S::setUVPThreshold(uint8_t m)   { return (m>=1&&m<=3) && writeReg8(CMD_UVPTHR, m); }
bool AP33772S::setOCPThreshold_mA(uint16_t ma){ return writeReg8(CMD_OCPTHR, ma/50); }
bool AP33772S::setOTPThreshold_C(uint8_t c)  { return writeReg8(CMD_OTPTHR, c); }
bool AP33772S::setDeratingThreshold_C(uint8_t c){ return writeReg8(CMD_DRTHR, c); }
bool AP33772S::setMinVoltage_mV(uint16_t mv) { return writeReg8(CMD_VSELMIN, mv/200); }

bool AP33772S::setProtectionConfig(bool uvp, bool ovp, bool ocp, bool otp, bool dr)
{
    uint8_t cfg = 0;
    if (uvp) cfg |= CONFIG_UVP_EN;
    if (ovp) cfg |= CONFIG_OVP_EN;
    if (ocp) cfg |= CONFIG_OCP_EN;
    if (otp) cfg |= CONFIG_OTP_EN;
    if (dr)  cfg |= CONFIG_DR_EN;
    return writeReg8(CMD_CONFIG, cfg);
}

// ── Interrupt ────────────────────────────────────────────────────────────────
bool    AP33772S::setInterruptMask(uint8_t mask){ return writeReg8(CMD_MASK, mask); }
uint8_t AP33772S::clearInterrupt()              { return getStatus(); }
void    AP33772S::attachInterruptCallback(void (*cb)())
{
    if (_intPin >= 0 && cb)
        ::attachInterrupt(digitalPinToInterrupt(_intPin), cb, RISING);
}

// ── NTC calibration ──────────────────────────────────────────────────────────
bool AP33772S::setNTC(uint16_t r25, uint16_t r50, uint16_t r75, uint16_t r100)
{
    return writeReg16(CMD_TR25, r25)  && writeReg16(CMD_TR50, r50) &&
           writeReg16(CMD_TR75, r75)  && writeReg16(CMD_TR100, r100);
}

// ── Raw I/O  (little-endian per datasheet Fig. 5) ────────────────────────────
int16_t AP33772S::readReg8(uint8_t reg)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return -1;
    if (_wire.requestFrom((uint8_t)AP33772S_ADDRESS, (uint8_t)1) < 1) return -1;
    return (int16_t)_wire.read();
}

int32_t AP33772S::readReg16(uint8_t reg)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return -1;
    if (_wire.requestFrom((uint8_t)AP33772S_ADDRESS, (uint8_t)2) < 2) return -1;
    uint8_t lo = _wire.read(), hi = _wire.read();
    return (int32_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

bool AP33772S::writeReg8(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(reg); _wire.write(val);
    return _wire.endTransmission() == 0;
}

bool AP33772S::writeReg16(uint8_t reg, uint16_t val)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(reg);
    _wire.write((uint8_t)(val & 0xFF));
    _wire.write((uint8_t)(val >> 8));
    return _wire.endTransmission() == 0;
}

bool AP33772S::readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire.beginTransmission(AP33772S_ADDRESS);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom((uint8_t)AP33772S_ADDRESS, len) < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _wire.read();
    return true;
}

// ── dumpRegisters() ──────────────────────────────────────────────────────────
void AP33772S::dumpRegisters(Stream &s)
{
    const struct { uint8_t a; uint8_t b; const char *n; } regs[] = {
        {0x01,1,"STATUS"},{0x02,1,"MASK"},{0x03,1,"OPMODE"},{0x04,1,"CONFIG"},
        {0x05,1,"PDCFG"},{0x06,1,"SYSTEM"},{0x11,2,"VOLTAGE"},{0x12,1,"CURRENT"},
        {0x13,1,"TEMP"},{0x14,2,"VREQ"},{0x15,2,"IREQ"},{0x16,1,"VSELMIN"},
        {0x17,1,"UVPTHR"},{0x18,1,"OVPTHR"},{0x19,1,"OCPTHR"},{0x1A,1,"OTPTHR"},
        {0x1B,1,"DRTHR"},{0x33,1,"MSGRLT"},
    };
    s.println(F("── Register Dump ──"));
    char buf[48];
    for (auto &r : regs) {
        if (r.b == 1) {
            int16_t v = readReg8(r.a);
            snprintf(buf, sizeof(buf), "  0x%02X %-8s = 0x%02X (%d)",
                     r.a, r.n, (uint8_t)(v<0?0xFF:v), (uint8_t)(v<0?0:v));
        } else {
            int32_t v = readReg16(r.a);
            snprintf(buf, sizeof(buf), "  0x%02X %-8s = 0x%04X (%d)",
                     r.a, r.n, (uint16_t)(v<0?0xFFFF:v), (uint16_t)(v<0?0:v));
        }
        s.println(buf);
    }
}