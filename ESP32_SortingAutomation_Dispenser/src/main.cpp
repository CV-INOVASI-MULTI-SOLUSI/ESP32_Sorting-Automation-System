// ============================================================
// FEEDER — Dispenser box (Conveyor2 + push) + Modbus + Menu Kalibrasi LCD/Keypad
// Push sensor-confirmed 2 sisi. Menu LCD 3-kategori (sinkron pola SORTER/PICKER/STOCKER).
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "registers.h"
#include "keypad4x4.h"
#include "io_expander.h"

// BARU: objek pwm global, dipakai Test Modul Servo -- SAMA PERSIS pola dgn SORTER/PICKER/STOCKER
Adafruit_PWMServoDriver pwm(I2CAddr::PCA9685);

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
  uint16_t pushTimeoutMs = 800;   // DIUBAH makna: sekarang batas waktu MAKSIMUM tiap gerakan servo (pengaman, servo tak punya sensor "sampai")
  uint16_t buzzerOnMs  = 500;
  uint16_t buzzerOffMs = 500;
  // BARU: 2 servo gerbang stack dispenser -- Servo1=gerbang BAWAH, Servo2=gerbang ATAS.
  // Titik Awal=tertutup/tertahan, Titik Akhir=terbuka/lepas. DIUBAH: Interval sekarang =
  // waktu tempuh (Awal->Akhir), TERPISAH per servo -- SAMA konsep dengan hopper SORTER,
  // BUKAN lagi step+interval speed yang shared. Waktu Tahan tetap terpisah dari Interval.
  uint16_t servo1StartUs = 1000, servo1EndUs = 2000, servo1HoldMs = 500, servo1IntervalMs = 500;
  uint16_t servo2StartUs = 1000, servo2EndUs = 2000, servo2HoldMs = 500, servo2IntervalMs = 500;
} cfg;

// BARU: buzzer notifikasi -- 1 siklus ON-OFF non-blocking per trigger event
bool buzzerBeeping = false, buzzerCurrentlyOn = false;
uint32_t buzzerStateChangedAt = 0;
void triggerBuzzerBeep() {
  buzzerBeeping = true; buzzerCurrentlyOn = true;
  io.write(CH::BUZZER, HIGH);
  buzzerStateChangedAt = millis();
}
void updateBuzzerBeep() {
  if (!buzzerBeeping) return;
  uint32_t elapsed = millis() - buzzerStateChangedAt;
  if (buzzerCurrentlyOn && elapsed >= cfg.buzzerOnMs) {
    io.write(CH::BUZZER, LOW); buzzerCurrentlyOn = false; buzzerStateChangedAt = millis();
  } else if (!buzzerCurrentlyOn && elapsed >= cfg.buzzerOffMs) {
    buzzerBeeping = false;
  }
}

enum class RefillState { IDLE, CONVEYOR_RUN, SERVO1_TO_END, SERVO1_HOLD, SERVO1_TO_START,
                          SERVO2_TO_END, SERVO2_HOLD, SERVO2_TO_START, DONE, FAULT, ESTOPPED };
RefillState refillState = RefillState::IDLE;
uint32_t stateEnteredAt = 0;
bool refillRequested = false;

// DIREDESIGN: trajectory servo DURATION-BASED (bukan step-speed lagi) -- SAMA konsep dgn
// hopper SORTER. Servo1=channel PCA9685 0, Servo2=channel 1. Tiap servo punya tracking
// gerak (moveFromUs/moveStartMs) TERPISAH karena interval-nya juga terpisah.
void pcaSetServoUs(uint8_t channel, uint16_t us);   // forward declaration -- didefinisikan di bawah (Test Modul)
constexpr uint8_t SERVO1_CH = 0, SERVO2_CH = 1;
uint16_t servo1CurrentUs = 1000, servo2CurrentUs = 1000;
uint32_t servo1MoveStartMs = 0, servo2MoveStartMs = 0;
uint16_t servo1MoveFromUs = 1000, servo2MoveFromUs = 1000;

// Panggil SEKALI saat MULAI gerakan baru -- catat titik awal & waktu mulai utk interpolasi
void startServoMove(uint8_t which, uint16_t fromUs) {
  if (which == 1) { servo1MoveFromUs = fromUs; servo1MoveStartMs = millis(); }
  else { servo2MoveFromUs = fromUs; servo2MoveStartMs = millis(); }
}
// Interpolasi linear non-blocking menuju targetUs, selama Interval servo yang bersangkutan.
// Return true kalau SUDAH SAMPAI (durasi habis) -- PASTI selesai tepat waktu, tidak perlu
// timeout pengaman terpisah lagi (beda dari desain lama yang menunggu sensor fisik).
bool updateServoTrajectory(uint8_t which, uint16_t targetUs) {
  uint16_t &currentUs   = (which == 1) ? servo1CurrentUs : servo2CurrentUs;
  uint32_t moveStartMs  = (which == 1) ? servo1MoveStartMs : servo2MoveStartMs;
  uint16_t moveFromUs   = (which == 1) ? servo1MoveFromUs : servo2MoveFromUs;
  uint16_t intervalMs   = (which == 1) ? cfg.servo1IntervalMs : cfg.servo2IntervalMs;
  uint8_t ch            = (which == 1) ? SERVO1_CH : SERVO2_CH;
  uint32_t elapsed = millis() - moveStartMs;
  if (elapsed >= intervalMs) { currentUs = targetUs; pcaSetServoUs(ch, currentUs); return true; }
  float fraction = (float)elapsed / (float)intervalMs;
  int32_t interpolated = (int32_t)moveFromUs + (int32_t)(((int32_t)targetUs - (int32_t)moveFromUs) * fraction);
  currentUs = (uint16_t)constrain(interpolated, 500, 2500);
  pcaSetServoUs(ch, currentUs);
  return false;
}

// BARU: Test Sequence -- 1x siklus maju-mundur (Titik Awal->Titik Akhir->tahan->Titik Awal)
// PAKAI NILAI KALIBRASI, dipicu dari LCD Test Command ATAU langsung dari Orange Pi (opcode).
// State TERPISAH dari RefillState produksi -- tidak boleh bentrok, makanya WAJIB currentState==IDLE.
enum class TestServoCycleStage { NONE, TO_END, HOLD, TO_START };
TestServoCycleStage testServoCycleStage = TestServoCycleStage::NONE;
uint8_t testServoCycleWhich = 0;   // 1 atau 2
uint32_t testServoCycleStateAt = 0;
void startTestServoCycle(uint8_t which) {
  if (currentState != NodeState::IDLE) { Serial.println("[TEST] Servo cycle ditolak -- node sedang tidak IDLE"); return; }
  testServoCycleWhich = which;
  testServoCycleStage = TestServoCycleStage::TO_END;
  testServoCycleStateAt = millis();
  uint16_t startUs = (which == 1) ? cfg.servo1StartUs : cfg.servo2StartUs;
  startServoMove(which, startUs);
  Serial.printf("[TEST] Servo%u cycle dimulai (pakai nilai kalibrasi, termasuk Interval)\n", which);
}
void updateTestServoCycle() {
  if (testServoCycleStage == TestServoCycleStage::NONE) return;
  uint32_t elapsed = millis() - testServoCycleStateAt;
  uint16_t startUs  = (testServoCycleWhich == 1) ? cfg.servo1StartUs : cfg.servo2StartUs;
  uint16_t endUs    = (testServoCycleWhich == 1) ? cfg.servo1EndUs   : cfg.servo2EndUs;
  uint16_t holdMs   = (testServoCycleWhich == 1) ? cfg.servo1HoldMs  : cfg.servo2HoldMs;
  switch (testServoCycleStage) {
    case TestServoCycleStage::TO_END:
      if (updateServoTrajectory(testServoCycleWhich, endUs)) { testServoCycleStage = TestServoCycleStage::HOLD; testServoCycleStateAt = millis(); }
      break;
    case TestServoCycleStage::HOLD:
      if (elapsed >= holdMs) {
        startServoMove(testServoCycleWhich, endUs);
        testServoCycleStage = TestServoCycleStage::TO_START;
        testServoCycleStateAt = millis();
      }
      break;
    case TestServoCycleStage::TO_START:
      if (updateServoTrajectory(testServoCycleWhich, startUs)) {
        testServoCycleStage = TestServoCycleStage::NONE;
        Serial.printf("[TEST] Servo%u cycle SELESAI\n", testServoCycleWhich);
      }
      break;
    default: break;
  }
}

constexpr int PWM_FREQ = 20000, PWM_RES = 8, LEDC_CH_CONV2 = 0;

void motorWrite(uint8_t ain1, uint8_t ain2, bool forward) { io.write(ain1, forward); io.write(ain2, !forward); }
void enterRefillState(RefillState s) { refillState = s; stateEnteredAt = millis(); }

// --- Menu state (dideklarasikan awal, dipakai onCmdWrite) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, JOG_PARAM,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_RELAY, TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET };
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
      // DIUBAH: LIM_BOX_ARRIVED sekarang berfungsi sbg trigger proximity -- package berhenti
      // di posisi yang diharapkan (sensor fisik boleh proximity ATAU limit switch, sinyalnya sama)
      if (io.read(CH::LIM_BOX_ARRIVED) == LOW) {
        ledcWrite(LEDC_CH_CONV2, 0);
        startServoMove(1, cfg.servo1StartUs);   // BARU -- catat titik awal gerak Servo1
        enterRefillState(RefillState::SERVO1_TO_END);
      }
      else if (elapsed > 8000) { ledcWrite(LEDC_CH_CONV2, 0); faultCode = (uint16_t)FaultCode::BOX_NOT_ARRIVED; enterRefillState(RefillState::FAULT); }
      break;

    // --- BARU: Servo 1 -- gerbang BAWAH, lepas package tingkat-2 turun ke conveyor ---
    case RefillState::SERVO1_TO_END:
      // DIHAPUS: timeout pengaman terpisah tidak perlu lagi -- durasi SUDAH eksplisit via
      // Interval, updateServoTrajectory() PASTI selesai tepat waktu (bukan menunggu sensor).
      if (updateServoTrajectory(1, cfg.servo1EndUs)) enterRefillState(RefillState::SERVO1_HOLD);
      break;
    case RefillState::SERVO1_HOLD:
      if (elapsed >= cfg.servo1HoldMs) {
        startServoMove(1, cfg.servo1EndUs);   // BARU -- mulai gerak balik dari titik akhir
        enterRefillState(RefillState::SERVO1_TO_START);
      }
      break;
    case RefillState::SERVO1_TO_START:
      if (updateServoTrajectory(1, cfg.servo1StartUs)) {
        startServoMove(2, cfg.servo2StartUs);   // BARU -- mulai gerak Servo2
        enterRefillState(RefillState::SERVO2_TO_END);
      }
      break;

    // --- BARU: Servo 2 -- gerbang ATAS, lepas package tingkat-3 turun isi ulang tingkat-2 ---
    case RefillState::SERVO2_TO_END:
      if (updateServoTrajectory(2, cfg.servo2EndUs)) enterRefillState(RefillState::SERVO2_HOLD);
      break;
    case RefillState::SERVO2_HOLD:
      if (elapsed >= cfg.servo2HoldMs) {
        startServoMove(2, cfg.servo2EndUs);   // BARU -- mulai gerak balik dari titik akhir
        enterRefillState(RefillState::SERVO2_TO_START);
      }
      break;
    case RefillState::SERVO2_TO_START:
      if (updateServoTrajectory(2, cfg.servo2StartUs)) { enterRefillState(RefillState::DONE); triggerBuzzerBeep(); }
      break;

    case RefillState::DONE:
      io.write(CH::DISP_STBY, LOW);
      currentState = NodeState::IDLE;
      enterRefillState(RefillState::IDLE);
      Serial.println("[FEEDER] Refill SELESAI");
      break;
    case RefillState::FAULT: case RefillState::ESTOPPED:
      ledcWrite(LEDC_CH_CONV2, 0);
      io.write(CH::DISP_STBY, LOW);
      if (refillState == RefillState::FAULT) currentState = NodeState::FAULT;
      break;
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH dari HIGH ke LOW
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
    case RefillState::SERVO1_TO_END:   return "Servo1 Buka";
    case RefillState::SERVO1_HOLD:     return "Servo1 Tahan";
    case RefillState::SERVO1_TO_START: return "Servo1 Tutup";
    case RefillState::SERVO2_TO_END:   return "Servo2 Buka";
    case RefillState::SERVO2_HOLD:     return "Servo2 Tahan";
    case RefillState::SERVO2_TO_START: return "Servo2 Tutup";
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
    case RefillState::SERVO1_TO_END:   return ActivityCode::SERVO1_BUKA;
    case RefillState::SERVO1_HOLD:     return ActivityCode::SERVO1_TAHAN;
    case RefillState::SERVO1_TO_START: return ActivityCode::SERVO1_TUTUP;
    case RefillState::SERVO2_TO_END:   return ActivityCode::SERVO2_BUKA;
    case RefillState::SERVO2_HOLD:     return ActivityCode::SERVO2_TAHAN;
    case RefillState::SERVO2_TO_START: return ActivityCode::SERVO2_TUTUP;
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
    case Cmd::TEST_SERVO1_CYCLE: startTestServoCycle(1); break;
    case Cmd::TEST_SERVO2_CYCLE: startTestServoCycle(2); break;
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
constexpr uint8_t CAL_COUNT = 13;
const char* CAL_LABELS[CAL_COUNT] = { "Conveyor Speed", "Push Timeout (ms)",
                                        "Servo1 Titik Awal", "Servo1 Titik Akhir", "Servo1 Waktu Tahan", "Servo1 Interval(ms)",
                                        "Servo2 Titik Awal", "Servo2 Titik Akhir", "Servo2 Waktu Tahan", "Servo2 Interval(ms)",
                                        "Buzzer On (ms)", "Buzzer Off (ms)", "Reset ke Default" };
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
  switch (selParam) {
    case 1: lcdPrint(0, 0, "CONVEYOR SPEED"); break;
    case 2: lcdPrint(0, 0, "PUSH TIMEOUT (ms)"); break;
    case 3: lcdPrint(0, 0, "SERVO1 TITIK AWAL"); break;
    case 4: lcdPrint(0, 0, "SERVO1 TITIK AKHIR"); break;
    case 5: lcdPrint(0, 0, "SERVO1 WAKTU TAHAN"); break;
    case 6: lcdPrint(0, 0, "SERVO1 INTERVAL(ms)"); break;
    case 7: lcdPrint(0, 0, "SERVO2 TITIK AWAL"); break;
    case 8: lcdPrint(0, 0, "SERVO2 TITIK AKHIR"); break;
    case 9: lcdPrint(0, 0, "SERVO2 WAKTU TAHAN"); break;
    case 10: lcdPrint(0, 0, "SERVO2 INTERVAL(ms)"); break;
    case 11: lcdPrint(0, 0, "BUZZER ON (ms)"); break;
    case 12: lcdPrint(0, 0, "BUZZER OFF (ms)"); break;
  }
  String line1;
  if (selParam == 1) line1 = "Nilai:" + String(cfg.conveyorSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 2) line1 = "Nilai:" + String(cfg.pushTimeoutMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 3) line1 = "us:" + String(cfg.servo1StartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 4) line1 = "us:" + String(cfg.servo1EndUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 5) line1 = "ms:" + String(cfg.servo1HoldMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 6) line1 = "ms:" + String(cfg.servo1IntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 7) line1 = "us:" + String(cfg.servo2StartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 8) line1 = "us:" + String(cfg.servo2EndUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 9) line1 = "ms:" + String(cfg.servo2HoldMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 10) line1 = "ms:" + String(cfg.servo2IntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 11) line1 = "ms:" + String(cfg.buzzerOnMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 12) line1 = "ms:" + String(cfg.buzzerOffMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
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
  } else if (selParam == 3) {
    if (key == 'A') cfg.servo1StartUs = (uint16_t)constrain((int)cfg.servo1StartUs + step, 500, 2500);
    else if (key == 'B') cfg.servo1StartUs = (uint16_t)constrain((int)cfg.servo1StartUs - step, 500, 2500);
    pcaSetServoUs(SERVO1_CH, cfg.servo1StartUs); servo1CurrentUs = cfg.servo1StartUs;   // preview live
  } else if (selParam == 4) {
    if (key == 'A') cfg.servo1EndUs = (uint16_t)constrain((int)cfg.servo1EndUs + step, 500, 2500);
    else if (key == 'B') cfg.servo1EndUs = (uint16_t)constrain((int)cfg.servo1EndUs - step, 500, 2500);
    pcaSetServoUs(SERVO1_CH, cfg.servo1EndUs); servo1CurrentUs = cfg.servo1EndUs;   // preview live
  } else if (selParam == 5) {
    if (key == 'A') cfg.servo1HoldMs = (uint16_t)constrain((int)cfg.servo1HoldMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.servo1HoldMs = (uint16_t)constrain((int)cfg.servo1HoldMs - step * 10, 50, 5000);
  } else if (selParam == 6) {
    if (key == 'A') cfg.servo1IntervalMs = (uint16_t)constrain((int)cfg.servo1IntervalMs + step * 10, 100, 10000);
    else if (key == 'B') cfg.servo1IntervalMs = (uint16_t)constrain((int)cfg.servo1IntervalMs - step * 10, 100, 10000);
  } else if (selParam == 7) {
    if (key == 'A') cfg.servo2StartUs = (uint16_t)constrain((int)cfg.servo2StartUs + step, 500, 2500);
    else if (key == 'B') cfg.servo2StartUs = (uint16_t)constrain((int)cfg.servo2StartUs - step, 500, 2500);
    pcaSetServoUs(SERVO2_CH, cfg.servo2StartUs); servo2CurrentUs = cfg.servo2StartUs;   // preview live
  } else if (selParam == 8) {
    if (key == 'A') cfg.servo2EndUs = (uint16_t)constrain((int)cfg.servo2EndUs + step, 500, 2500);
    else if (key == 'B') cfg.servo2EndUs = (uint16_t)constrain((int)cfg.servo2EndUs - step, 500, 2500);
    pcaSetServoUs(SERVO2_CH, cfg.servo2EndUs); servo2CurrentUs = cfg.servo2EndUs;   // preview live
  } else if (selParam == 9) {
    if (key == 'A') cfg.servo2HoldMs = (uint16_t)constrain((int)cfg.servo2HoldMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.servo2HoldMs = (uint16_t)constrain((int)cfg.servo2HoldMs - step * 10, 50, 5000);
  } else if (selParam == 10) {
    if (key == 'A') cfg.servo2IntervalMs = (uint16_t)constrain((int)cfg.servo2IntervalMs + step * 10, 100, 10000);
    else if (key == 'B') cfg.servo2IntervalMs = (uint16_t)constrain((int)cfg.servo2IntervalMs - step * 10, 100, 10000);
  } else if (selParam == 11) {
    if (key == 'A') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs - step * 10, 50, 5000);
  } else if (selParam == 12) {
    if (key == 'A') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs - step * 10, 50, 5000);
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
    if (calCursor == 12) { menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); }
    else { selParam = calCursor + 1; menuState = MenuState::JOG_PARAM; lcd.clear(); drawParamMenuFeeder(); }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}

// --- LEVEL 1b: TEST I/O ---
struct IOTestItem { const char* label; uint8_t ch; bool autoControlled; };
void drawTestIoCategory();

// --- Kategori: Test Output ---
constexpr uint8_t OUTPUT_TEST_COUNT = 7;
IOTestItem OUTPUT_TEST_ITEMS[OUTPUT_TEST_COUNT] = {
  {"OPR",    CH::LED_OPERATION, true},
  {"RUN",    CH::LED_RUN,       true},
  {"MANUAL", CH::LED_MANUAL,    true},
  {"FAULT",  CH::LED_FAULT,     false},
  {"BUZZER", CH::BUZZER,        false},
  {"DISP_AIN1", CH::DISP_AIN1,  false},
  {"DISP_AIN2", CH::DISP_AIN2,  false},
};
uint8_t outputTestCursor = 0;
bool testOutputLastVal = false, testOutputFirstDraw = true;
void drawTestOutputList() {
  static const char* labels[OUTPUT_TEST_COUNT];
  for (uint8_t i = 0; i < OUTPUT_TEST_COUNT; i++) labels[i] = OUTPUT_TEST_ITEMS[i].label;
  drawListMenu("TEST OUTPUT", labels, OUTPUT_TEST_COUNT, outputTestCursor);
}
void drawTestOutputItem() {
  IOTestItem &item = OUTPUT_TEST_ITEMS[outputTestCursor];
  bool val = io.read(item.ch);
  if (testOutputFirstDraw) {
    lcd.clear();
    lcdPrint(0, 0, "TEST OUT: " + String(item.label));
    lcdPrint(0, 2, "C=toggle" + String(item.autoControlled ? " (auto)" : ""));
    lcdPrint(0, 3, "D=kembali");
    testOutputFirstDraw = false;
    testOutputLastVal = !val;
  }
  if (val != testOutputLastVal) { testOutputLastVal = val; lcdPrint(0, 1, "Nilai = " + String(val ? "HIGH" : "LOW") + "   "); }
}
void handleTestOutputListKey(char key) {
  if (key == 'A') { outputTestCursor = (outputTestCursor == 0) ? OUTPUT_TEST_COUNT - 1 : outputTestCursor - 1; drawTestOutputList(); }
  else if (key == 'B') { outputTestCursor = (outputTestCursor + 1) % OUTPUT_TEST_COUNT; drawTestOutputList(); }
  else if (key == 'C') { menuState = MenuState::TEST_OUTPUT_ITEM; testOutputFirstDraw = true; drawTestOutputItem(); }
  else if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); }
}
void handleTestOutputItemKey(char key) {
  IOTestItem &item = OUTPUT_TEST_ITEMS[outputTestCursor];
  if (key == 'D') { menuState = MenuState::TEST_OUTPUT_LIST; drawTestOutputList(); return; }
  static uint32_t lastToggleMs = 0;
  constexpr uint32_t TOGGLE_MIN_INTERVAL_MS = 300;
  if (key == 'C' && (item.autoControlled || currentState == NodeState::IDLE)) {
    if (millis() - lastToggleMs < TOGGLE_MIN_INTERVAL_MS) return;
    lastToggleMs = millis();
    bool cur = io.read(item.ch);
    io.write(item.ch, !cur);
    Serial.printf("[TEST-OUT] CH%u (%s) di-toggle -> %s\n", item.ch, item.label, !cur ? "HIGH" : "LOW");
  } else if (key == 'C') {
    Serial.println("[TEST-OUT] Toggle ditolak -- state harus IDLE dulu");
  }
  drawTestOutputItem();
}

// --- Kategori: Test Input -- semua input tampil sekaligus (live) ---
constexpr uint8_t INPUT_TEST_COUNT = 5;
IOTestItem INPUT_TEST_ITEMS[INPUT_TEST_COUNT] = {
  {"IN1", CH::ESTOP,             false},
  {"IN2", CH::LIM_STOCK_EMPTY,   false},
  {"IN3", CH::LIM_BOX_ARRIVED,   false},
  {"IN4", CH::LIM_PUSH_HOME,     false},
  {"IN5", CH::LIM_PUSH_EXTENDED, false},
};
uint8_t inputTestScrollTop = 0;
bool testInputLiveLastVal[INPUT_TEST_COUNT];
uint8_t testInputLastScrollTop = 255;
void drawTestInputList() {
  bool scrollChanged = (inputTestScrollTop != testInputLastScrollTop);
  if (scrollChanged) { lcd.clear(); testInputLastScrollTop = inputTestScrollTop; }
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idx = inputTestScrollTop + row;
    if (idx >= INPUT_TEST_COUNT) continue;
    bool val = io.read(INPUT_TEST_ITEMS[idx].ch);
    if (scrollChanged || val != testInputLiveLastVal[idx]) {
      testInputLiveLastVal[idx] = val;
      lcdPrint(0, row, String(INPUT_TEST_ITEMS[idx].label) + " = " + String(val ? 1 : 0) + "         ");
    }
  }
}
void handleTestInputListKey(char key) {
  if (key == 'A') { if (inputTestScrollTop > 0) { inputTestScrollTop--; drawTestInputList(); } }
  else if (key == 'B') { if (inputTestScrollTop < (INPUT_TEST_COUNT > 4 ? INPUT_TEST_COUNT - 4 : 0)) { inputTestScrollTop++; drawTestInputList(); } }
  else if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); }
}

// --- Kategori: I2C Scan ---
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
  Serial.printf("[TEST-IO] Total %u device\n", count);
}
void handleTestIoI2CScanKey(char key) {
  if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); return; }
}

// --- Kategori: Test RS485 ---
String lcdVersionShort() {
  String v = FW_VERSION;
  if (v.length() >= 16) return v.substring(2, 16);
  return v;
}
void drawTestRs485() {
  lcd.clear();
  lcdPrint(0, 0, "TEST RS485");
  lcdPrint(0, 1, "Slave:" + String(Rs485Cfg::SLAVE_ID) + " Baud:" + String(Rs485Cfg::BAUD));
  if (modbusEverUsed) lcdPrint(0, 2, "RX terakhir:" + String((millis() - lastRs485Rx) / 1000) + "s lalu");
  else lcdPrint(0, 2, "Belum ada data masuk");
  lcdPrint(0, 3, "D=kembali");
  Serial.printf("[TEST-RS485] slaveID=%d baud=%lu pernahTerimaData=%d sejakRxMs=%lu FW=%s\n",
                Rs485Cfg::SLAVE_ID, (unsigned long)Rs485Cfg::BAUD, modbusEverUsed,
                modbusEverUsed ? (unsigned long)(millis() - lastRs485Rx) : 0UL, FW_VERSION);
}
void handleTestRs485Key(char key) {
  if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); return; }
  drawTestRs485();
}

// --- Kategori: Test Modul -- sub-menu pilihan JENIS modul, SAMA di semua 4 node (board universal) ---
bool testModFirstDraw = true;
String testModLine1 = "", testModLine2 = "";
// DIUBAH: Servo (Test Modul) sekarang lewat library resmi Adafruit_PWMServoDriver, SAMA
// PERSIS pola dgn SORTER/PICKER/STOCKER (menggantikan driver Wire mentah).
void pcaSetServoUs(uint8_t channel, uint16_t us) {
  us = constrain(us, (uint16_t)500, (uint16_t)2500);
  uint16_t duty = (uint32_t)us * 4096 / 20000;
  pwm.setPWM(channel, 0, duty);
}

constexpr uint8_t MODULE_TYPE_COUNT = 4;
const char* MODULE_TYPE_LABELS[MODULE_TYPE_COUNT] = { "Stepper", "Motor DC", "Relay", "Servo" };
uint8_t moduleTypeCursor = 0;
void drawModuleTypeSelect() { drawListMenu("TEST MODUL", MODULE_TYPE_LABELS, MODULE_TYPE_COUNT, moduleTypeCursor); }
void handleModuleTypeKey(char key);

uint8_t testModAxis = 0;
void testStepperPulse(uint8_t axisIdx, bool forward) {
  const uint8_t stepPins[3] = { GP::STEP_1, GP::STEP_2, GP::STEP_3 };
  const uint8_t dirPins[3]  = { CH::DIR_1_MCP, CH::DIR_2_MCP, CH::DIR_3_MCP };
  io.write(CH::EN_123, LOW);
  io.write(dirPins[axisIdx], forward ? HIGH : LOW);
  delayMicroseconds(50);
  for (uint16_t i = 0; i < 100; i++) {
    digitalWrite(stepPins[axisIdx], HIGH);
    delayMicroseconds(4);
    digitalWrite(stepPins[axisIdx], LOW);
    delayMicroseconds(800);
  }
  io.write(CH::EN_123, HIGH);
}
void drawTestModStepper() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: STEPPER");
    lcdPrint(0, 1, "(cek fisik manual)");
    lcdPrint(0, 3, "C=ax A/B=puls D=kmb");
    testModFirstDraw = false;
  }
  const char* axisName[3] = {"1", "2", "3"};
  lcdPrint(0, 2, "Axis:" + String(axisName[testModAxis]) + " STEP/DIR/EN");
}
void handleTestModStepperKey(char key) {
  if (key == 'C') { testModAxis = (testModAxis + 1) % 3; }
  else if (key == 'A') { testStepperPulse(testModAxis, true); }
  else if (key == 'B') { testStepperPulse(testModAxis, false); }
  else if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  drawTestModStepper();
}

// --- Modul: Motor DC -- CATATAN: di FEEDER, channel AIN1/AIN2/STBY SAMA dgn DISP_AIN1/2/STBY
// produksi (alias). Guard IDLE-only belum ditambahkan -- operator perlu pastikan dispenser
// sedang tidak REFILLING sebelum test modul ini.
uint8_t testModMotorAState = 0, testModMotorBState = 0;
void testModSetMotor(bool channelA, uint8_t dirState) {
  uint8_t ain1 = channelA ? CH::AIN1 : CH::BIN1, ain2 = channelA ? CH::AIN2 : CH::BIN2;
  if (channelA) testModMotorAState = dirState; else testModMotorBState = dirState;
  if (dirState == 0) { io.write(ain1, LOW); io.write(ain2, LOW); }
  else { io.write(ain1, dirState == 1); io.write(ain2, dirState != 1); }
  io.write(CH::STBY, (testModMotorAState != 0 || testModMotorBState != 0));
}
void drawTestModMotorDC() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: MOTOR DC");
    lcdPrint(0, 3, "A=chA B=chB D=kmb");
    testModFirstDraw = false; testModLine1 = "\x01";
  }
  String line1 = "ChA:" + String(testModMotorAState) + " ChB:" + String(testModMotorBState);
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
  lcdPrint(0, 2, "A/B=maju,lg=stop");
}
void handleTestModMotorDCKey(char key) {
  if (key == 'A') { testModSetMotor(true, testModMotorAState == 0 ? 1 : 0); }
  else if (key == 'B') { testModSetMotor(false, testModMotorBState == 0 ? 1 : 0); }
  else if (key == 'D') {
    testModSetMotor(true, 0); testModSetMotor(false, 0);
    menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return;
  }
  drawTestModMotorDC();
}

uint8_t testModRelaySel = 0;
void drawTestModRelay() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: RELAY");
    lcdPrint(0, 3, "C=pilih A=tgl D=kmb");
    testModFirstDraw = false; testModLine1 = "\x01";
  }
  uint8_t ch = (testModRelaySel == 0) ? CH::RLY1 : CH::RLY2;
  bool val = io.read(ch);
  String line1 = "RLY" + String(testModRelaySel + 1) + " = " + String(val ? "HIGH" : "LOW");
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
}
void handleTestModRelayKey(char key) {
  static uint32_t lastToggleMs = 0;
  uint8_t ch = (testModRelaySel == 0) ? CH::RLY1 : CH::RLY2;
  if (key == 'C') { testModRelaySel = (testModRelaySel + 1) % 2; }
  else if (key == 'A') {
    if (millis() - lastToggleMs < 300) return;
    lastToggleMs = millis();
    io.write(ch, !io.read(ch));
  } else if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  drawTestModRelay();
}

uint8_t testModServoCh = 0;
bool testModServoDetected = false;
void drawTestModServo() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: SERVO");
    Wire.beginTransmission(I2CAddr::PCA9685);
    testModServoDetected = (Wire.endTransmission() == 0);
    testModFirstDraw = false; testModLine1 = "\x01";
  }
  String line1 = testModServoDetected ? ("PCA9685 OK, CH:" + String(testModServoCh)) : "PCA9685 TIDAK ADA";
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
  lcdPrint(0, 3, testModServoDetected ? "C=ch A/B=gerak D=kmb" : "D=kembali");
}
void handleTestModServoKey(char key) {
  if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  if (testModServoDetected) {
    if (key == 'C') { testModServoCh = (testModServoCh + 1) % 16; }
    else if (key == 'A') { pcaSetServoUs(testModServoCh, 1700); }
    else if (key == 'B') { pcaSetServoUs(testModServoCh, 1300); }
  }
  drawTestModServo();
}

void handleModuleTypeKey(char key) {
  if (key == 'A') { moduleTypeCursor = (moduleTypeCursor == 0) ? MODULE_TYPE_COUNT - 1 : moduleTypeCursor - 1; drawModuleTypeSelect(); }
  else if (key == 'B') { moduleTypeCursor = (moduleTypeCursor + 1) % MODULE_TYPE_COUNT; drawModuleTypeSelect(); }
  else if (key == 'C') {
    testModFirstDraw = true;
    switch (moduleTypeCursor) {
      case 0: menuState = MenuState::TEST_MOD_STEPPER; drawTestModStepper(); break;
      case 1: menuState = MenuState::TEST_MOD_MOTORDC; drawTestModMotorDC(); break;
      case 2: menuState = MenuState::TEST_MOD_RELAY; drawTestModRelay(); break;
      case 3: menuState = MenuState::TEST_MOD_SERVO; drawTestModServo(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); }
}

// --- Selector kategori (LEVEL 1b utama) ---
constexpr uint8_t TEST_IO_CAT_COUNT = 5;
const char* TEST_IO_CAT_LABELS[TEST_IO_CAT_COUNT] = { "I2C Scan", "Test Output", "Test Input", "Test RS485", "Test Modul" };
uint8_t testIoCatCursor = 0;
void drawTestIoCategory() { drawListMenu("TEST I/O", TEST_IO_CAT_LABELS, TEST_IO_CAT_COUNT, testIoCatCursor); }
void handleTestIoCategoryKey(char key) {
  if (key == 'A') { testIoCatCursor = (testIoCatCursor == 0) ? TEST_IO_CAT_COUNT - 1 : testIoCatCursor - 1; drawTestIoCategory(); }
  else if (key == 'B') { testIoCatCursor = (testIoCatCursor + 1) % TEST_IO_CAT_COUNT; drawTestIoCategory(); }
  else if (key == 'C') {
    switch (testIoCatCursor) {
      case 0: menuState = MenuState::TEST_IO_I2CSCAN; runI2CScanFromMenu(); break;
      case 1: menuState = MenuState::TEST_OUTPUT_LIST; outputTestCursor = 0; drawTestOutputList(); break;
      case 2: menuState = MenuState::TEST_INPUT_LIST; inputTestScrollTop = 0; testInputLastScrollTop = 255; drawTestInputList(); break;
      case 3: menuState = MenuState::TEST_RS485; drawTestRs485(); break;
      case 4: menuState = MenuState::TEST_MODULE_SELECT; moduleTypeCursor = 0; drawModuleTypeSelect(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}

// --- LEVEL 1c: TEST COMMAND ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 5;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"REQUEST_REFILL",       Cmd::REQUEST_REFILL,     0},
  {"SET_SPEED (test=200)", Cmd::SET_CONVEYOR_SPEED, 200},
  {"RESET_FAULT",          Cmd::RESET_FAULT,        0},
  {"Test Srv1 M/M", Cmd::TEST_SERVO1_CYCLE, 0},
  {"Test Srv2 M/M", Cmd::TEST_SERVO2_CYCLE, 0},
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
      case 1: menuState = MenuState::TEST_IO_CATEGORY; testIoCatCursor = 0; drawTestIoCategory(); break;
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
  pwm.begin(); pwm.setPWMFreq(50);   // BARU -- utk Test Modul Servo (library resmi, sinkron node lain)
  scanI2CAndDetectOptional();

  if (lcdPresent) { lcd.init(); lcd.backlight(); lcdPrint(0, 0, "FEEDER"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }

  // DITAMBAHKAN (akar masalah ditemukan!): pin Output-Enable PCA9685 (aktif-LOW) TIDAK PERNAH
  // ditarik LOW sebelumnya -- ini penyebab sebenarnya servo diam meski I2C berhasil sempurna.
  // WAJIB setelah io.begin() -- io.pinMode()/io.write() butuh IOBank sudah siap dulu.
  io.pinMode(CH::OE_PCA, OUTPUT);
  io.write(CH::OE_PCA, LOW);

  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  io.pinMode(CH::DISP_AIN1, OUTPUT); io.pinMode(CH::DISP_AIN2, OUTPUT); io.pinMode(CH::DISP_STBY, OUTPUT);
  io.pinMode(CH::CONV2_BIN1, OUTPUT); io.pinMode(CH::CONV2_BIN2, OUTPUT);
  // DIPERBAIKI (bug ditemukan): FEEDER tidak punya channel produksi asli Stepper/Relay,
  // jadi channel ini TIDAK PERNAH di-pinMode OUTPUT -- Test Modul Stepper/Relay diam total.
  io.pinMode(CH::DIR_1_MCP, OUTPUT); io.pinMode(CH::DIR_2_MCP, OUTPUT); io.pinMode(CH::DIR_3_MCP, OUTPUT);
  io.pinMode(CH::EN_123, OUTPUT);
  io.pinMode(CH::RLY1, OUTPUT); io.pinMode(CH::RLY2, OUTPUT);
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

  // BARU: posisi awal servo1/servo2 saat boot -- SETELAH loadConfigFromNvs() supaya pakai
  // nilai kalibrasi tersimpan (bukan default compile-time -- pelajaran dari bug urutan SORTER)
  pcaSetServoUs(SERVO1_CH, cfg.servo1StartUs); servo1CurrentUs = cfg.servo1StartUs;
  pcaSetServoUs(SERVO2_CH, cfg.servo2StartUs); servo2CurrentUs = cfg.servo2StartUs;
  Serial.println("[BOOT] Servo1/Servo2 diposisikan ke titik awal");

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
      else if (menuState == MenuState::TEST_IO_CATEGORY) handleTestIoCategoryKey(key);
      else if (menuState == MenuState::TEST_IO_I2CSCAN) handleTestIoI2CScanKey(key);
      else if (menuState == MenuState::TEST_OUTPUT_LIST) handleTestOutputListKey(key);
      else if (menuState == MenuState::TEST_OUTPUT_ITEM) handleTestOutputItemKey(key);
      else if (menuState == MenuState::TEST_INPUT_LIST) handleTestInputListKey(key);
      else if (menuState == MenuState::TEST_RS485) handleTestRs485Key(key);
      else if (menuState == MenuState::TEST_MODULE_SELECT) handleModuleTypeKey(key);
      else if (menuState == MenuState::TEST_MOD_STEPPER) handleTestModStepperKey(key);
      else if (menuState == MenuState::TEST_MOD_MOTORDC) handleTestModMotorDCKey(key);
      else if (menuState == MenuState::TEST_MOD_RELAY) handleTestModRelayKey(key);
      else if (menuState == MenuState::TEST_MOD_SERVO) handleTestModServoKey(key);
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
  bool pauseAutoIndicators = (menuState == MenuState::TEST_OUTPUT_ITEM && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled);
  if (!pauseAutoIndicators) updateUniversalIndicators();

  // DIPERBAIKI: refresh live utk kategori Test Output (auto-controlled saja), Test Input,
  // Test RS485, dan Test Modul
  if (menuState == MenuState::TEST_OUTPUT_ITEM && lcdPresent && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled) {
    static uint32_t lastTestOutRefresh = 0;
    if (millis() - lastTestOutRefresh > 100) { lastTestOutRefresh = millis(); drawTestOutputItem(); }
  }
  if (menuState == MenuState::TEST_INPUT_LIST && lcdPresent) {
    static uint32_t lastTestInRefresh = 0;
    if (millis() - lastTestInRefresh > 100) { lastTestInRefresh = millis(); drawTestInputList(); }
  }
  if (menuState == MenuState::TEST_RS485 && lcdPresent) {
    static uint32_t lastTestRs485Refresh = 0;
    if (millis() - lastTestRs485Refresh > 500) { lastTestRs485Refresh = millis(); drawTestRs485(); }
  }
  if (menuState == MenuState::TEST_MOD_MOTORDC && lcdPresent) {
    static uint32_t lastTestModRefresh = 0;
    if (millis() - lastTestModRefresh > 150) { lastTestModRefresh = millis(); drawTestModMotorDC(); }
  }
  if (menuState == MenuState::TEST_MOD_RELAY && lcdPresent) {
    static uint32_t lastTestModRefresh2 = 0;
    if (millis() - lastTestModRefresh2 > 150) { lastTestModRefresh2 = millis(); drawTestModRelay(); }
  }
  handleSafety();
  if (currentState != NodeState::ESTOPPED) handleRefillFSM();
  updateTestServoCycle();   // BARU -- proses test sequence non-blocking (independen dari RefillState produksi)
  updateBuzzerBeep();   // BARU -- proses siklus ON-OFF buzzer non-blocking

  if (menuState != MenuState::NONE) return;

  handleSerialCommand();

  // DIPERBAIKI: timeout diperlebar dari 5000ms -- watchdog cuma reset saat command BARU
  // dikirim, TIDAK ikut ter-reset oleh pembacaan status. Testing manual (baca status
  // berulang sambil menunggu progres) wajar jeda lebih dari 5 detik -- nilai lama terlalu
  // ketat. 30 detik cukup toleran, tetap berfungsi sbg pengaman komunikasi terputus total.
  constexpr uint32_t COMM_TIMEOUT_MS = 30000;
  if (modbusEverUsed && currentState == NodeState::RUNNING_OR_MOVING && millis() - lastRs485Rx > COMM_TIMEOUT_MS) {
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