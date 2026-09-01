// ============================================================
// FEEDER — Dispenser box (Conveyor2 + push) + Modbus + Menu Kalibrasi LCD/Keypad
// Push sensor-confirmed 2 sisi. Menu LCD 3-kategori (sinkron pola SORTER/PICKER/STOCKER).
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include <Preferences.h>
#include "config.h"
#include "registers.h"
#include "keypad4x4.h"
#include "io_expander.h"

LiquidCrystal_I2C lcd(I2CAddr::LCD, LcdCfg::COLS, LcdCfg::ROWS);
Keypad4x4 keypad(I2CAddr::KEYPAD);
IOBank io;
ModbusRTU mb;
Preferences prefs;

constexpr char Keypad4x4::KEYMAP[4][4];

bool lcdPresent = false, keypadPresent = false;

void lcdPrint(uint8_t col, uint8_t row, String text) {
  if (!lcdPresent) return;
  uint8_t avail = (col < LcdCfg::COLS) ? (LcdCfg::COLS - col) : 0;
  if (text.length() > avail) text = text.substring(0, avail);
  while (text.length() < avail) text += " ";
  lcd.setCursor(col, row);
  lcd.print(text);
}

#define FW_VERSION "v.01.00.25082026.21.17"
constexpr const char* FW_BUILD = __DATE__ " " __TIME__;

NodeState currentState = NodeState::INIT;
uint16_t faultCode = 0;
// BARU: Lapis 3 diagnostik -- sinkron pola SORTER/PICKER/STOCKER
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
uint32_t lastRs485Rx = 0;
bool modbusEverUsed = false;

struct FeederConfig {
  uint8_t  conveyorSpeed = 160;
  uint16_t pushTimeoutMs = 800;
} cfg;

enum class RefillState { IDLE, CONVEYOR_RUN, PUSHING, RETRACT, DONE, FAULT, ESTOPPED };
RefillState refillState = RefillState::IDLE;
uint32_t stateEnteredAt = 0;
bool refillRequested = false;

constexpr int PWM_FREQ = 20000, PWM_RES = 8, LEDC_CH_CONV2 = 0;

void motorWrite(uint8_t ain1, uint8_t ain2, bool forward) { io.write(ain1, forward); io.write(ain2, !forward); }
void enterRefillState(RefillState s) { refillState = s; stateEnteredAt = millis(); }

// --- Menu state (dideklarasikan awal, dipakai onCmdWrite) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, JOG_PARAM, TEST_IO_LIST, TEST_IO_ITEM, TEST_CMD_LIST, CONFIRM_RESET };
MenuState menuState = MenuState::NONE;

void loadConfigFromNvs() {
  prefs.begin("feeder_cal", true);
  if (prefs.isKey("cfg")) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
void saveConfigToNvs() { prefs.begin("feeder_cal", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }
// BARU: reset ke default -- hapus NVS "feeder_cal" & kembalikan RAM ke default
void resetConfigToDefault() {
  prefs.begin("feeder_cal", false);
  prefs.clear();
  prefs.end();
  cfg = FeederConfig();
  Serial.println("[RESET] FeederConfig dikembalikan ke default & NVS dihapus");
}

// DIPERBAIKI (pola sama dgn temuan STOCKER B01): motorWrite() sebelumnya dipanggil
// di DALAM badan case (dieksekusi ulang TIAP LOOP selama state itu berlangsung),
// padahal arah motor KONSTAN selama 1 state -- redundant I2C write. Sekarang
// motorWrite() cuma dipanggil SEKALI, tepat saat transisi MASUK ke state itu.
// DIPERBAIKI (pola sama dgn temuan STOCKER B01): motorWrite() sebelumnya dipanggil
// di DALAM badan case (dieksekusi ulang TIAP LOOP selama state itu berlangsung),
// padahal arah motor KONSTAN selama 1 state -- redundant I2C write. Sekarang
// dijaga dgn flag "sudah ditulis" per-state, cuma tulis SEKALI saat pertama masuk
// state itu -- struktur asli (coast dulu di IDLE, baru drive di CONVEYOR_RUN) TETAP
// dipertahankan persis, tidak dihilangkan.
bool motorWriteDoneForState = false;
void handleRefillFSM() {
  uint32_t elapsed = millis() - stateEnteredAt;
  switch (refillState) {
    case RefillState::IDLE:
      if (refillRequested) {
        if (io.read(CH::LIM_STOCK_EMPTY) == LOW) {
          faultCode = (uint16_t)FaultCode::STOCK_EMPTY;
          enterRefillState(RefillState::FAULT);
          return;
        }
        refillRequested = false;
        io.write(CH::CONV2_BIN1, false); io.write(CH::CONV2_BIN2, false);
        io.write(CH::DISP_STBY, HIGH);
        enterRefillState(RefillState::CONVEYOR_RUN);
        motorWriteDoneForState = false;   // state baru -- izinkan 1x tulis motorWrite di case berikutnya
      }
      break;
    case RefillState::CONVEYOR_RUN:
      if (!motorWriteDoneForState) { motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, true); motorWriteDoneForState = true; }
      ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
      if (io.read(CH::LIM_BOX_ARRIVED) == LOW) {
        ledcWrite(LEDC_CH_CONV2, 0);
        enterRefillState(RefillState::PUSHING);
        motorWriteDoneForState = false;
      }
      else if (elapsed > 8000) { ledcWrite(LEDC_CH_CONV2, 0); faultCode = (uint16_t)FaultCode::BOX_NOT_ARRIVED; enterRefillState(RefillState::FAULT); }
      break;
    case RefillState::PUSHING:
      if (!motorWriteDoneForState) { motorWrite(CH::DISP_AIN1, CH::DISP_AIN2, true); motorWriteDoneForState = true; }
      if (io.read(CH::LIM_PUSH_EXTENDED) == LOW) { enterRefillState(RefillState::RETRACT); motorWriteDoneForState = false; }
      else if (elapsed > cfg.pushTimeoutMs) {
        io.write(CH::DISP_AIN1, LOW); io.write(CH::DISP_AIN2, LOW);
        faultCode = (uint16_t)FaultCode::PUSH_STUCK; enterRefillState(RefillState::FAULT);
      }
      break;
    case RefillState::RETRACT:
      if (!motorWriteDoneForState) { motorWrite(CH::DISP_AIN1, CH::DISP_AIN2, false); motorWriteDoneForState = true; }
      if (io.read(CH::LIM_PUSH_HOME) == LOW) { io.write(CH::DISP_AIN1, LOW); io.write(CH::DISP_AIN2, LOW); enterRefillState(RefillState::DONE); }
      else if (elapsed > cfg.pushTimeoutMs) { faultCode = (uint16_t)FaultCode::PUSH_STUCK; enterRefillState(RefillState::FAULT); }
      break;
    case RefillState::DONE:
      io.write(CH::DISP_STBY, LOW);
      currentState = NodeState::IDLE;
      enterRefillState(RefillState::IDLE);
      Serial.println("[FEEDER] Refill SELESAI");
      break;
    case RefillState::FAULT: case RefillState::ESTOPPED:
      io.write(CH::DISP_AIN1, LOW); io.write(CH::DISP_AIN2, LOW);
      ledcWrite(LEDC_CH_CONV2, 0);
      io.write(CH::DISP_STBY, LOW);
      if (refillState == RefillState::FAULT) currentState = NodeState::FAULT;
      break;
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == HIGH) {
    currentState = NodeState::ESTOPPED;
    enterRefillState(RefillState::ESTOPPED);
    return;
  }
  if (currentState == NodeState::ESTOPPED) {
    currentState = NodeState::IDLE;
    enterRefillState(RefillState::IDLE);
  }
}

const char* stateText(NodeState s) {
  switch (s) {
    case NodeState::INIT: return "INIT";
    case NodeState::IDLE: return "IDLE";
    case NodeState::RUNNING_OR_MOVING: return "RUNNING";
    case NodeState::FAULT: return "FAULT";
    case NodeState::ESTOPPED: return "ESTOPPED";
    default: return "?";
  }
}

// BARU: teks aktivitas spesifik -- refillState SUDAH persis memetakan ke tahapan refill
String activityText() {
  switch (refillState) {
    case RefillState::IDLE:      return "Diam";
    case RefillState::CONVEYOR_RUN: return "Conveyor ON";
    case RefillState::PUSHING:   return "Mendorong Box";
    case RefillState::RETRACT:   return "Tarik Kembali";
    case RefillState::DONE:      return "Selesai";
    case RefillState::FAULT:     return "FAULT!";
    case RefillState::ESTOPPED:  return "E-STOP!";
  }
  return "?";
}

// BARU: versi kode numerik dari activityText() -- dikirim ke register Modbus (Lapis 2)
ActivityCode activityCode() {
  switch (refillState) {
    case RefillState::IDLE:         return ActivityCode::DIAM;
    case RefillState::CONVEYOR_RUN: return ActivityCode::CONVEYOR_JALAN;
    case RefillState::PUSHING:      return ActivityCode::MENDORONG_BOX;
    case RefillState::RETRACT:      return ActivityCode::MENARIK_KEMBALI;
    case RefillState::DONE:         return ActivityCode::SELESAI;
    case RefillState::FAULT:        return ActivityCode::FAULT_AKTIF;
    case RefillState::ESTOPPED:     return ActivityCode::ESTOP_AKTIF;
  }
  return ActivityCode::DIAM;
}

// BARU: indikator universal (sama pola di semua 4 node) -- dideklarasikan di sini, dipakai
// setelahnya di loop(). Butuh menuState, jadi diletakkan setelah deklarasinya (di atas file ini).
// DIPERBAIKI (pola sama dgn temuan STOCKER B01/EN_STEPPERS): LED_RUN/LED_MANUAL
// sebelumnya ditulis via I2C TIAP LOOP tanpa cek perubahan -- redundant I2C write
// meski nilai sama persis dgn sebelumnya. Sekarang di-cache, cuma tulis saat berubah.
void updateUniversalIndicators() {
  static uint32_t lastHeartbeatBlink = 0;
  static bool heartbeatState = false;
  if (millis() - lastHeartbeatBlink > 500) {
    lastHeartbeatBlink = millis();
    heartbeatState = !heartbeatState;
    io.write(CH::LED_OPERATION, heartbeatState);
  }
  static bool lastLedRun = false, lastLedManual = false;
  bool ledRunNow = (currentState == NodeState::RUNNING_OR_MOVING);
  bool ledManualNow = (menuState != MenuState::NONE);
  if (ledRunNow != lastLedRun) { io.write(CH::LED_RUN, ledRunNow); lastLedRun = ledRunNow; }
  if (ledManualNow != lastLedManual) { io.write(CH::LED_MANUAL, ledManualNow); lastLedManual = ledManualNow; }
}

// DIPERBAIKI: guard FAULT/ESTOPPED -- REQUEST_REFILL sebelumnya TIDAK dicek sama sekali
// (kelas bug sama dgn SORTER/PICKER/STOCKER yang sudah ditemukan)
void applyCommand(uint16_t opcode, uint16_t arg) {
  switch ((Cmd)opcode) {
    case Cmd::REQUEST_REFILL:
      if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
        Serial.println("[CMD] REQUEST_REFILL ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
        return;
      }
      refillRequested = true;
      currentState = NodeState::RUNNING_OR_MOVING;
      break;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb sebelum di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE; enterRefillState(RefillState::IDLE);
      break;
    case Cmd::SET_CONVEYOR_SPEED:
      cfg.conveyorSpeed = (uint8_t)constrain(arg, 0, 255);
      break;
    default: Serial.printf("[CMD] opcode %u tidak dikenal\n", opcode); break;
  }
}

uint16_t onCmdWrite(TRegister* reg, uint16_t val) {
  if (menuState != MenuState::NONE) {
    Serial.println("[FEEDER] Command Modbus diabaikan -- mode kalibrasi aktif (§12.6)");
    return val;
  }
  lastRs485Rx = millis(); modbusEverUsed = true;
  uint16_t opcode = val;
  uint16_t arg = mb.Hreg(Reg::CMD_ARG);
  uint16_t seq = mb.Hreg(Reg::CMD_SEQ);
  uint16_t lastAcked = mb.Hreg(Reg::CMD_ACK_SEQ);
  if (seq == lastAcked) { Serial.printf("[FEEDER] CMD seq=%u sudah diproses -- diabaikan\n", seq); return val; }
  Serial.printf("[FEEDER] CMD (Modbus) opcode=%u arg=%u seq=%u\n", opcode, arg, seq);
  applyCommand(opcode, arg);
  mb.Hreg(Reg::CMD_ACK_SEQ, seq);
  return val;
}

// ============================================================
// MENU LCD+Keypad -- 3 KATEGORI: Setting Kalibrasi / Test I/O / Test Command
// ============================================================
uint8_t jogStepIdx = 0;
constexpr int16_t JOG_STEPS[4] = {5, 10, 20, 50};
uint8_t selParam = 0;

void drawListMenu(const char* title, const char* labels[], uint8_t count, uint8_t cursor) {
  lcd.clear();
  lcdPrint(0, 0, title);
  uint8_t viewStart = 0;
  if (cursor >= 2) viewStart = cursor - 1;
  if (count >= 3 && viewStart > count - 3) viewStart = count - 3;
  for (uint8_t row = 0; row < 3 && (viewStart + row) < count; row++) {
    uint8_t idx = viewStart + row;
    String prefix = (idx == cursor) ? "> " : "  ";
    lcdPrint(0, row + 1, prefix + char('a' + idx) + ". " + labels[idx]);
  }
}

// --- LEVEL 0: TOP MENU ---
constexpr uint8_t TOP_COUNT = 3;
const char* TOP_LABELS[TOP_COUNT] = { "Setting Kalibrasi", "Test I/O", "Test Command" };
uint8_t topCursor = 0;
void drawTopMenuFeeder() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
constexpr uint8_t CAL_COUNT = 3;
const char* CAL_LABELS[CAL_COUNT] = { "Conveyor Speed", "Push Timeout (ms)", "Reset ke Default" };
uint8_t calCursor = 0;
void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

void drawConfirmReset() {
  lcd.clear();
  lcdPrint(0, 0, "RESET KE DEFAULT?");
  lcdPrint(0, 1, "Speed & Timeout akan");
  lcdPrint(0, 2, "HILANG, TAK BS BATAL");
  lcdPrint(0, 3, "C=YA,RESET D=batal");
}
void handleConfirmResetKey(char key) {
  if (key == 'C') { resetConfigToDefault(); lcdPrint(0, 3, "SUDAH DIRESET!      "); menuState = MenuState::CAL_LIST; }
  else if (key == 'D') { Serial.println("[CAL] Reset dibatalkan"); menuState = MenuState::CAL_LIST; drawCalList(); }
}

void drawParamMenuFeeder() {
  lcdPrint(0, 0, selParam == 1 ? "CONVEYOR SPEED" : "PUSH TIMEOUT (ms)");
  String line1;
  if (selParam == 1) line1 = "Nilai:" + String(cfg.conveyorSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 2) line1 = "Nilai:" + String(cfg.pushTimeoutMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, "A+ B- C:step");
  lcdPrint(0, 3, "#=SIMPAN D=kembali");
}
void handleParamKeyFeeder(char key) {
  int16_t step = JOG_STEPS[jogStepIdx];
  if (selParam == 1) {
    if (key == 'A') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed + step, 0, 255);
    else if (key == 'B') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed - step, 0, 255);
  } else if (selParam == 2) {
    if (key == 'A') cfg.pushTimeoutMs = (uint16_t)constrain((int)cfg.pushTimeoutMs + step, 100, 5000);
    else if (key == 'B') cfg.pushTimeoutMs = (uint16_t)constrain((int)cfg.pushTimeoutMs - step, 100, 5000);
  }
  if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
  else if (key == '#') { saveConfigToNvs(); lcdPrint(0, 3, "TERSIMPAN ke NVS!"); Serial.println("[CAL] FeederConfig disimpan ke NVS"); return; }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawParamMenuFeeder();
}

void handleCalListKey(char key) {
  if (key == 'A') { calCursor = (calCursor == 0) ? CAL_COUNT - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % CAL_COUNT; drawCalList(); }
  else if (key == 'C') {
    if (calCursor == 2) { menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); }
    else { selParam = calCursor + 1; menuState = MenuState::JOG_PARAM; lcd.clear(); drawParamMenuFeeder(); }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}

// --- LEVEL 1b: TEST I/O ---
struct IOTestItem { const char* label; uint8_t ch; bool isOutput; bool isSpecial; bool autoControlled; };
constexpr uint8_t IO_TEST_COUNT = 14;
IOTestItem IO_TEST_ITEMS[IO_TEST_COUNT] = {
  {"I2C Scan",            0, false, true,  false},
  {"System Info",         0, false, true,  false},
  {"ESTOP (in)",          CH::ESTOP,           false, false, false},
  {"LED_RUN (auto)",      CH::LED_RUN,         true,  false, true},
  {"LED_OPERATION(auto)", CH::LED_OPERATION,   true,  false, true},
  {"LED_MANUAL (auto)",   CH::LED_MANUAL,      true,  false, true},
  {"LED_FAULT (out)",     CH::LED_FAULT,       true,  false, false},
  {"BUZZER (out)",        CH::BUZZER,          true,  false, false},
  {"DISP_AIN1 (out)",     CH::DISP_AIN1,       true,  false, false},
  {"DISP_AIN2 (out)",     CH::DISP_AIN2,       true,  false, false},
  {"DISP_STBY (out)",     CH::DISP_STBY,       true,  false, false},
  {"LIM_STOCK_EMPTY(in)", CH::LIM_STOCK_EMPTY, false, false, false},
  {"LIM_BOX_ARRIVED(in)", CH::LIM_BOX_ARRIVED, false, false, false},
  {"LIM_PUSH_HOME (in)",  CH::LIM_PUSH_HOME,   false, false, false},
};
uint8_t ioTestCursor = 0;
const char* IO_TEST_LABELS_ONLY[IO_TEST_COUNT];
void buildIoTestLabels() { for (uint8_t i = 0; i < IO_TEST_COUNT; i++) IO_TEST_LABELS_ONLY[i] = IO_TEST_ITEMS[i].label; }
void drawTestIoList() {
  buildIoTestLabels();
  // DIPERBAIKI: title tampilkan currentState -- lihat penjelasan di SORTER
  String title = "TEST I/O [" + String(stateText(currentState)) + "]";
  drawListMenu(title.c_str(), IO_TEST_LABELS_ONLY, IO_TEST_COUNT, ioTestCursor);
}

void runI2CScanFromMenu() {
  lcd.clear(); lcdPrint(0, 0, "I2C SCAN...");
  Serial.println("[TEST-IO] I2C Scan dari menu:");
  String found = ""; uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      char buf[6]; snprintf(buf, sizeof(buf), "0x%02X ", addr);
      found += buf; count++;
      Serial.printf("[TEST-IO]   0x%02X terdeteksi\n", addr);
    }
  }
  lcdPrint(0, 1, String(count) + " device:");
  lcdPrint(0, 2, found.substring(0, 20));
  lcdPrint(0, 3, "D=kembali");
}
void drawSystemInfoFromMenu() {
  lcd.clear();
  lcdPrint(0, 0, "SYSTEM INFO");
  lcdPrint(0, 1, "FW:" + String(FW_VERSION));
  lcdPrint(0, 2, "Up:" + String(millis() / 1000) + "s I2Cerr:" + String(i2cErrorCount));
  lcdPrint(0, 3, "D=kembali");
  Serial.printf("[SYSINFO] FW=%s build=%s uptime=%lus freeHeap=%u slaveID=%d MCP=%s LCD=%s KP=%s i2cErrCount=%u lastFault=%u\n",
                FW_VERSION, FW_BUILD, (unsigned long)(millis() / 1000), ESP.getFreeHeap(),
                Rs485Cfg::SLAVE_ID, io.isHealthy() ? "OK" : "GAGAL",
                lcdPresent ? "ADA" : "TIDAK", keypadPresent ? "ADA" : "TIDAK",
                i2cErrorCount, lastFaultCode);
}
bool testIoLastVal = false;
bool testIoFirstDraw = true;

void drawTestIoItem() {
  IOTestItem &item = IO_TEST_ITEMS[ioTestCursor];
  if (item.isSpecial) { if (ioTestCursor == 0) runI2CScanFromMenu(); else drawSystemInfoFromMenu(); return; }
  bool val = io.read(item.ch);
  if (testIoFirstDraw) {
    lcd.clear();
    lcdPrint(0, 0, "TEST: " + String(item.label));
    if (item.isOutput) lcdPrint(0, 2, "C=toggle" + String(item.autoControlled ? " (auto)" : ""));
    else lcdPrint(0, 2, "(input, baca saja)");
    lcdPrint(0, 3, "D=kembali");
    testIoFirstDraw = false;
    testIoLastVal = !val;
  }
  if (val != testIoLastVal) {
    testIoLastVal = val;
    lcdPrint(0, 1, "Nilai = " + String(val ? "HIGH" : "LOW") + "   ");
  }
}
void handleTestIoListKey(char key) {
  if (key == 'A') { ioTestCursor = (ioTestCursor == 0) ? IO_TEST_COUNT - 1 : ioTestCursor - 1; drawTestIoList(); }
  else if (key == 'B') { ioTestCursor = (ioTestCursor + 1) % IO_TEST_COUNT; drawTestIoList(); }
  else if (key == 'C') { menuState = MenuState::TEST_IO_ITEM; testIoFirstDraw = true; drawTestIoItem(); }
  else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}
void handleTestIoItemKey(char key) {
  IOTestItem &item = IO_TEST_ITEMS[ioTestCursor];
  if (key == 'D') { menuState = MenuState::TEST_IO_LIST; drawTestIoList(); return; }
  if (item.isSpecial) { drawTestIoItem(); return; }
  static uint32_t lastToggleMs = 0;
  constexpr uint32_t TOGGLE_MIN_INTERVAL_MS = 300;
  if (key == 'C' && item.isOutput && (item.autoControlled || currentState == NodeState::IDLE)) {
    if (millis() - lastToggleMs < TOGGLE_MIN_INTERVAL_MS) return;
    lastToggleMs = millis();
    bool cur = io.read(item.ch);
    io.write(item.ch, !cur);
    Serial.printf("[TEST-IO] CH%u (%s) di-toggle -> %s\n", item.ch, item.label, !cur ? "HIGH" : "LOW");
  } else if (key == 'C' && item.isOutput) {
    Serial.println("[TEST-IO] Toggle ditolak -- state harus IDLE dulu");
  }
  drawTestIoItem();
}

// --- LEVEL 1c: TEST COMMAND ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 3;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"REQUEST_REFILL",       Cmd::REQUEST_REFILL,     0},
  {"SET_SPEED (test=200)", Cmd::SET_CONVEYOR_SPEED, 200},
  {"RESET_FAULT",          Cmd::RESET_FAULT,        0},
};
const char* CMD_TEST_LABELS_ONLY[CMD_TEST_COUNT];
void buildCmdTestLabels() { for (uint8_t i = 0; i < CMD_TEST_COUNT; i++) CMD_TEST_LABELS_ONLY[i] = CMD_TEST_ITEMS[i].label; }
uint8_t cmdTestCursor = 0;
void drawTestCmdList() {
  buildCmdTestLabels();
  drawListMenu("TEST COMMAND", CMD_TEST_LABELS_ONLY, CMD_TEST_COUNT, cmdTestCursor);
  lcdPrint(0, 3, "C=kirim D=kembali");
}
void handleTestCmdListKey(char key) {
  if (key == 'A') { cmdTestCursor = (cmdTestCursor == 0) ? CMD_TEST_COUNT - 1 : cmdTestCursor - 1; drawTestCmdList(); }
  else if (key == 'B') { cmdTestCursor = (cmdTestCursor + 1) % CMD_TEST_COUNT; drawTestCmdList(); }
  else if (key == 'C') {
    CmdTestItem &item = CMD_TEST_ITEMS[cmdTestCursor];
    Serial.printf("[TEST-CMD] Simulasi command dari 'node lain': opcode=%u (%s) arg=%u -- amati aksi fisik SEKARANG\n",
                  (uint16_t)item.opcode, item.label, item.testArg);
    applyCommand((uint16_t)item.opcode, item.testArg);
    lcdPrint(0, 3, "Terkirim, amati aksi");
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}

void handleTopMenuKeyFeeder(char key) {
  if (key == 'A') { topCursor = (topCursor == 0) ? TOP_COUNT - 1 : topCursor - 1; drawTopMenuFeeder(); }
  else if (key == 'B') { topCursor = (topCursor + 1) % TOP_COUNT; drawTopMenuFeeder(); }
  else if (key == 'C') {
    switch (topCursor) {
      case 0: menuState = MenuState::CAL_LIST; calCursor = 0; drawCalList(); break;
      case 1: menuState = MenuState::TEST_IO_LIST; ioTestCursor = 0; drawTestIoList(); break;
      case 2: menuState = MenuState::TEST_CMD_LIST; cmdTestCursor = 0; drawTestCmdList(); break;
    }
  } else if (key == 'D') { menuState = MenuState::NONE; lcd.clear(); Serial.println("[CAL] Keluar mode kalibrasi"); }
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  int sp1 = line.indexOf(' ');
  String cmd = (sp1 == -1) ? line : line.substring(0, sp1);
  uint16_t arg = (sp1 == -1) ? 0 : line.substring(sp1 + 1).toInt();
  cmd.toUpperCase();

  if (cmd == "REFILL")           applyCommand((uint16_t)Cmd::REQUEST_REFILL, 0);
  else if (cmd == "RESET_FAULT") applyCommand((uint16_t)Cmd::RESET_FAULT, 0);
  else if (cmd == "SPEED")       applyCommand((uint16_t)Cmd::SET_CONVEYOR_SPEED, arg);
  else if (cmd == "TIMEOUT") { cfg.pushTimeoutMs = constrain((int)arg, 100, 5000); saveConfigToNvs(); Serial.printf("[TIMEOUT] pushTimeoutMs=%u\n", cfg.pushTimeoutMs); }
  else if (cmd == "RESET") resetConfigToDefault();
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s refillState=%d fault=%u ESTOP=%d speed=%u timeout=%u stockEmpty=%d\n",
                  stateText(currentState), (int)refillState, faultCode, io.read(CH::ESTOP),
                  cfg.conveyorSpeed, cfg.pushTimeoutMs, io.read(CH::LIM_STOCK_EMPTY) == LOW);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000));
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] REFILL | RESET_FAULT | SPEED <0-255> | TIMEOUT <100-5000> | RESET | STATUS | HELP");
  }
  else Serial.printf("[SERIAL] '%s' tidak dikenal -- ketik HELP\n", cmd.c_str());
}

void scanI2CAndDetectOptional() {
  Serial.println("[I2C-SCAN] Memindai bus I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C-SCAN]   Terdeteksi di 0x%02X\n", addr);
      if (addr == I2CAddr::LCD) lcdPresent = true;
      if (addr == I2CAddr::KEYPAD) keypadPresent = true;
    }
  }
  Serial.printf("[I2C-SCAN] LCD %s, Keypad %s\n", lcdPresent ? "ADA" : "TIDAK ADA", keypadPresent ? "ADA" : "TIDAK ADA");
}

// BARU: hot-plug LCD/Keypad DUA ARAH
void checkLcdKeypadHotplug() {
  static uint32_t lastHotplugCheck = 0;
  if (millis() - lastHotplugCheck < 3000) return;
  lastHotplugCheck = millis();

  Wire.beginTransmission(I2CAddr::LCD);
  bool lcdPing = (Wire.endTransmission() == 0);
  if (!lcdPresent && lcdPing) { lcdPresent = true; lcd.init(); lcd.backlight(); Serial.println("[HOTPLUG] LCD baru terdeteksi -- diinisialisasi live"); }
  else if (lcdPresent && !lcdPing) { lcdPresent = false; Serial.println("[HOTPLUG] !!! LCD TIDAK TERDETEKSI LAGI !!!"); }

  Wire.beginTransmission(I2CAddr::KEYPAD);
  bool kpPing = (Wire.endTransmission() == 0);
  if (!keypadPresent && kpPing) { keypadPresent = true; keypad.begin(); Serial.println("[HOTPLUG] Keypad baru terdeteksi -- diinisialisasi live"); }
  else if (keypadPresent && !kpPing) {
    keypadPresent = false;
    Serial.println("[HOTPLUG] !!! Keypad TIDAK TERDETEKSI LAGI !!!");
    if (menuState != MenuState::NONE) { menuState = MenuState::NONE; Serial.println("[HOTPLUG] Keluar OTOMATIS dari mode kalibrasi -- cegah node terjebak"); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] FEEDER mulai");

  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_FREQ_HZ);
  scanI2CAndDetectOptional();

  if (lcdPresent) { lcd.init(); lcd.backlight(); lcdPrint(0, 0, "FEEDER"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }

  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  io.pinMode(CH::DISP_AIN1, OUTPUT); io.pinMode(CH::DISP_AIN2, OUTPUT); io.pinMode(CH::DISP_STBY, OUTPUT);
  io.pinMode(CH::CONV2_BIN1, OUTPUT); io.pinMode(CH::CONV2_BIN2, OUTPUT);
  io.pinMode(CH::LIM_STOCK_EMPTY, INPUT_PULLUP);
  io.pinMode(CH::LIM_BOX_ARRIVED, INPUT_PULLUP);
  io.pinMode(CH::LIM_PUSH_HOME, INPUT_PULLUP);
  io.pinMode(CH::LIM_PUSH_EXTENDED, INPUT_PULLUP);
  Serial.printf("[BOOT] MCP23017: %s, pinMode selesai\n", ioOk ? "OK" : "GAGAL");

  ledcSetup(LEDC_CH_CONV2, PWM_FREQ, PWM_RES);
  ledcAttachPin(GP::CONV2_PWM, LEDC_CH_CONV2);
  Serial.println("[BOOT] LEDC PWM conveyor2 OK");

  loadConfigFromNvs();
  Serial.println("[BOOT] FeederConfig dimuat dari NVS");

  Serial2.begin(Rs485Cfg::BAUD, SERIAL_8N1, Rs485Cfg::RX_PIN, Rs485Cfg::TX_PIN);
  mb.begin(&Serial2);
  mb.slave(Rs485Cfg::SLAVE_ID);
  mb.addHreg(Reg::STATE, (uint16_t)currentState);
  mb.addHreg(Reg::FAULT_CODE, faultCode);
  mb.addHreg(Reg::CMD, 0); mb.addHreg(Reg::CMD_ARG, 0);
  mb.addHreg(Reg::CMD_SEQ, 0); mb.addHreg(Reg::CMD_ACK_SEQ, 0);
  mb.addHreg(Reg::HEARTBEAT, 0);
  mb.addHreg(Reg::STOCK_EMPTY_FLAG, 0);
  mb.addHreg(Reg::ACTIVITY_CODE, 0); mb.addHreg(Reg::I2C_ERROR_COUNT, 0);
  mb.addHreg(Reg::LAST_FAULT_CODE, 0); mb.addHreg(Reg::UPTIME_SEC, 0);
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  Serial.printf("[BOOT] Modbus siap, slave ID=%d\n", Rs485Cfg::SLAVE_ID);

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;
  Serial.println("[BOOT] setup SELESAI -- ketik HELP di serial monitor");
}

void loop() {
  mb.task();

  if (keypadPresent) {
    char key = keypad.scan();
    if (key) {
      if (menuState == MenuState::NONE && key == '*') {
        menuState = MenuState::TOP_SELECT;
        if (lcdPresent) drawTopMenuFeeder();
        Serial.println("[CAL] Masuk mode kalibrasi (command eksternal dijeda, logic fisik tetap jalan)");
      } else if (menuState == MenuState::TOP_SELECT) handleTopMenuKeyFeeder(key);
      else if (menuState == MenuState::CAL_LIST) handleCalListKey(key);
      else if (menuState == MenuState::JOG_PARAM) handleParamKeyFeeder(key);
      else if (menuState == MenuState::TEST_IO_LIST) handleTestIoListKey(key);
      else if (menuState == MenuState::TEST_IO_ITEM) handleTestIoItemKey(key);
      else if (menuState == MenuState::TEST_CMD_LIST) handleTestCmdListKey(key);
      else if (menuState == MenuState::CONFIRM_RESET) handleConfirmResetKey(key);
    }
  }

  mb.Hreg(Reg::STATE, (uint16_t)currentState);
  mb.Hreg(Reg::FAULT_CODE, faultCode);
  mb.Hreg(Reg::STOCK_EMPTY_FLAG, io.read(CH::LIM_STOCK_EMPTY) == LOW ? 1 : 0);
  // BARU: update register Lapis 2 + Lapis 3 tiap loop
  mb.Hreg(Reg::ACTIVITY_CODE, (uint16_t)activityCode());
  mb.Hreg(Reg::I2C_ERROR_COUNT, i2cErrorCount);
  mb.Hreg(Reg::LAST_FAULT_CODE, lastFaultCode);
  mb.Hreg(Reg::UPTIME_SEC, (uint16_t)(millis() / 1000));

  static uint32_t lastMcpHealthCheck = 0;
  if (millis() - lastMcpHealthCheck > 2000) {
    lastMcpHealthCheck = millis();
    bool healthy = io.recheckHealth();
    if (!healthy) i2cErrorCount++;   // BARU -- catat SETIAP kegagalan
    if (!healthy && currentState != NodeState::FAULT && currentState != NodeState::ESTOPPED) {
      faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING;
      currentState = NodeState::FAULT;
      Serial.println("[SAFETY] !!! MCP23017 berhenti merespons I2C -- FAULT dipicu !!!");
    }
  }

  checkLcdKeypadHotplug();
  bool pauseAutoIndicators = (menuState == MenuState::TEST_IO_ITEM && IO_TEST_ITEMS[ioTestCursor].autoControlled);
  if (!pauseAutoIndicators) updateUniversalIndicators();

  // BARU: refresh live Test I/O untuk item INPUT/auto-controlled -- lihat perubahan real-time
  if (menuState == MenuState::TEST_IO_ITEM && lcdPresent) {
    IOTestItem &curItem = IO_TEST_ITEMS[ioTestCursor];
    if (!curItem.isSpecial && (!curItem.isOutput || curItem.autoControlled)) {
      static uint32_t lastTestIoRefresh = 0;
      if (millis() - lastTestIoRefresh > 200) {
        lastTestIoRefresh = millis();
        drawTestIoItem();
      }
    }
  }
  handleSafety();
  if (currentState != NodeState::ESTOPPED) handleRefillFSM();

  if (menuState != MenuState::NONE) return;

  handleSerialCommand();

  if (modbusEverUsed && currentState == NodeState::RUNNING_OR_MOVING && millis() - lastRs485Rx > 5000) {
    currentState = NodeState::FAULT; faultCode = (uint16_t)FaultCode::COMM_TIMEOUT;
  }

  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    mb.Hreg(Reg::HEARTBEAT, (uint16_t)(millis() / 1000));
    Serial.printf("[LOOP] state=%s refillState=%d ESTOP=%d\n", stateText(currentState), (int)refillState, io.read(CH::ESTOP));
  }

  if (lcdPresent) {
    static uint32_t lastLcdRefresh = 0;
    if (millis() - lastLcdRefresh > 500) {
      lastLcdRefresh = millis();
      lcdPrint(0, 0, "[AUTO] " + activityText());
      lcdPrint(0, 1, "State:" + String(stateText(currentState)));
      lcdPrint(0, 2, "Refill:" + String((int)refillState));
      lcdPrint(0, 3, "Tahan* utk kalibrasi");
    }
  }
}