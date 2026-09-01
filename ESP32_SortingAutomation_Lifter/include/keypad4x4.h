#pragma once
#include <Wire.h>

// ============================================================
// Keypad4x4 — scan matrix via PCF8574, non-blocking.
// Baris = output (P0-P3), Kolom = input (P4-P7), quasi-bidirectional.
// Layout: 1 2 3 A / 4 5 6 B / 7 8 9 C / * 0 # D
// ============================================================
class Keypad4x4 {
public:
  explicit Keypad4x4(uint8_t i2cAddr) : addr(i2cAddr) {}

  void begin() { writeExpander(0xFF); }  // idle: semua baris HIGH, siap discan

  // Panggil tiap loop() tanpa delay — 1 baris discan per panggilan (pacing non-blocking)
  char scan() {
    uint32_t now = millis();
    if (now - lastScanMs < 5) return 0;   // ~5ms/baris -> full scan ~20ms, cukup responsif
    lastScanMs = now;

    uint8_t out = 0xFF;
    out &= ~(1 << curRow);                // aktifkan 1 baris (LOW), sisanya HIGH
    writeExpander(out);
    delayMicroseconds(50);                // settle time matrix, BUKAN delay() penuh siklus
    uint8_t in = readExpander();
    char key = 0;

    for (uint8_t c = 0; c < 4; c++) {
      bool pressed = !(in & (1 << (4 + c)));
      uint8_t idx = curRow * 4 + c;
      if (pressed && !lastState[idx] && (now - lastEdge[idx] > 50)) {  // debounce 50ms
        key = KEYMAP[curRow][c];
        lastEdge[idx] = now;
      }
      lastState[idx] = pressed;
    }
    curRow = (curRow + 1) % 4;
    return key;   // 0 jika tidak ada tombol baru pada scan ini
  }

  // BARU: cek apakah tombol SEDANG ditahan (bukan cuma "baru ditekan") -- utk jog kontinu
  bool isHeld(char key) {
    for (uint8_t r = 0; r < 4; r++)
      for (uint8_t c = 0; c < 4; c++)
        if (KEYMAP[r][c] == key) return lastState[r * 4 + c];
    return false;
  }

private:
  uint8_t addr;
  uint8_t curRow = 0;
  uint32_t lastScanMs = 0;
  bool lastState[16] = {false};
  uint32_t lastEdge[16] = {0};
  static constexpr char KEYMAP[4][4] = {
    {'1','2','3','A'}, {'4','5','6','B'}, {'7','8','9','C'}, {'*','0','#','D'}
  };

  void writeExpander(uint8_t val) {
    Wire.beginTransmission(addr); Wire.write(val); Wire.endTransmission();
  }
  uint8_t readExpander() {
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;  // fail-safe: anggap tidak ada tombol jika I2C gagal
  }
};