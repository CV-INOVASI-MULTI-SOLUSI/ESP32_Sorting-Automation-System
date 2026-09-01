#pragma once
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

// ============================================================
// IOExpander — wrapper 1 chip MCP23017, fault-tolerant (guard `ok` di tiap panggilan).
// BARU: recheck() -- ping ulang I2C secara independen dari cache Adafruit lib, dipanggil
// BERKALA (bukan cuma sekali di boot) supaya degradasi koneksi I2C di tengah jalan (chip
// sempat OK saat begin() tapi longgar/gagal belakangan) tetap terdeteksi, bukan diam-diam
// mengembalikan nilai basi (read() akan tetap kembalikan HIGH kalau ok jadi false).
// ============================================================
class IOExpander {
public:
  bool begin(uint8_t addr) {
    address = addr;
    ok = mcp.begin_I2C(addr);
    return ok;
  }
  void pinMode(uint8_t pin, uint8_t mode) { if (ok) mcp.pinMode(pin, mode); }
  void write(uint8_t pin, bool val)       { if (ok) mcp.digitalWrite(pin, val); }
  bool read(uint8_t pin) { return ok ? mcp.digitalRead(pin) : HIGH; }  // fail-safe: HIGH kalau chip mati
  bool isHealthy() const { return ok; }
  bool recheck() {   // BARU
    Wire.beginTransmission(address);
    ok = (Wire.endTransmission() == 0);
    return ok;
  }
private:
  Adafruit_MCP23X17 mcp;
  bool ok = false;
  uint8_t address = 0;
};

// ============================================================
// IOBank — gabungan 2x IOExpander (MCP1 + MCP2), diakses pakai 1 index channel (CH0-31).
// CH0-15 -> MCP1, CH16-31 -> MCP2 otomatis. begin() cetak diagnostik PER-CHIP ke Serial,
// supaya kalau salah satu chip tidak terdeteksi, langsung ketahuan yang mana (bukan cuma
// gabungan true/false yang tidak jelas chip mana yang bermasalah).
// ============================================================
class IOBank {
public:
  bool begin(uint8_t addr1, uint8_t addr2) {
    bool ok1 = mcp1.begin(addr1);
    Serial.printf("[MCP] Chip1 @0x%02X: %s\n", addr1, ok1 ? "OK terdeteksi" : "!!! TIDAK TERDETEKSI !!!");
    bool ok2 = mcp2.begin(addr2);
    Serial.printf("[MCP] Chip2 @0x%02X: %s\n", addr2, ok2 ? "OK terdeteksi" : "!!! TIDAK TERDETEKSI !!!");
    return ok1 && ok2;
  }
  void pinMode(uint8_t ch, uint8_t mode) { chip(ch).pinMode(local(ch), mode); }
  void write(uint8_t ch, bool val)       { chip(ch).write(local(ch), val); }
  bool read(uint8_t ch)                  { return chip(ch).read(local(ch)); }
  bool isHealthy() const { return mcp1.isHealthy() && mcp2.isHealthy(); }
  bool recheckHealth() {   // BARU -- panggil berkala dari loop(), BUKAN cuma sekali di boot
    bool h1 = mcp1.recheck();
    bool h2 = mcp2.recheck();
    return h1 && h2;
  }
private:
  IOExpander mcp1, mcp2;
  IOExpander& chip(uint8_t ch) { return (ch < 16) ? mcp1 : mcp2; }
  uint8_t local(uint8_t ch) const { return ch % 16; }
};