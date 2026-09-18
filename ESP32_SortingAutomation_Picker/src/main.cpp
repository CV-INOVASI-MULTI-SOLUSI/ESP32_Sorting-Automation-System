// ============================================================
// PICKER — Logic Produksi: 6DOF Servo (PCA9685) + Modbus + Serial
// Menu LCD 3-kategori: Setting Kalibrasi / Test I/O / Test Command (sinkron pola SORTER).
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "registers.h"
#include "keypad4x4.h"
#include "io_expander.h"
#include "wifi_credentials.h"

LiquidCrystal_I2C lcd(I2CAddr::LCD, LcdCfg::COLS, LcdCfg::ROWS);
Keypad4x4 keypad(I2CAddr::KEYPAD);
IOBank io;
ModbusRTU mb;
Adafruit_PWMServoDriver pwm(I2CAddr::PCA9685);
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

// BARU: animasi LOADING singkat saat boot -- kasih feedback visual operator + waktu settle
// I2C/PCA9685/MCP23017 sebelum masuk operasi normal. No-op total kalau LCD tidak terdeteksi.
constexpr uint8_t BOOT_STEPS_TOTAL = 4;
constexpr uint16_t BOOT_STEP_DELAY_MS = 150;
uint8_t bootStep = 0;
void lcdBootProgress(const char* stepLabel) {
  bootStep++;
  // DIPERBAIKI: delay() WAJIB tetap jalan meski LCD tidak terdeteksi -- tujuannya kasih waktu
  // settle inisialisasi hardware, animasi LCD cuma bonus visual di atasnya. Sebelumnya delay
  // ikut di-skip bareng LCD, jadi tanpa LCD = TIDAK ada jeda sama sekali.
  if (lcdPresent) {
    uint8_t filled = (uint8_t)min((uint32_t)10, (uint32_t)bootStep * 10 / BOOT_STEPS_TOTAL);
    String bar = "[";
    for (uint8_t i = 0; i < 10; i++) bar += (i < filled) ? '#' : '-';
    bar += "]";
    lcdPrint(0, 0, "PICKER BOOTING");
    lcdPrint(0, 1, bar);
    lcdPrint(0, 2, "LOADING " + String(bootStep) + "/" + String(BOOT_STEPS_TOTAL));
    lcdPrint(0, 3, stepLabel);
  }
  delay(BOOT_STEP_DELAY_MS);
}

NodeState currentState = NodeState::INIT;
uint16_t faultCode = 0;
// BARU: Lapis 3 diagnostik -- sinkron pola SORTER
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
uint32_t lastRs485Rx = 0;
bool modbusEverUsed = false;

struct Pose { uint16_t us[ServoCfg::NUM_JOINTS]; };
// Slot 0=home, 1=pass, 2=reject, 3=lift(clearance per-objek). BARU slot 4=PACKAGE_PICKUP
// (posisi ambil package berisi batch objek di ujung conveyor), 5=LIFT_LOAD (posisi taruh
// package ke Lift Load Position STOCKER) -- dipakai Cmd::MOVE_PACKAGE. Default slot 4/5
// SENGAJA disamakan dgn home (aman, tidak akan gerak ekstrem) sampai dikalibrasi manual
// via menu Pose / SAVEPOSE.
Pose POSES[6] = {
  {{1500,1500,1500,1500,1500,1000}},
  {{1200,1600,1400,1500,1500,1000}},
  {{1800,1600,1400,1500,1500,1000}},
  {{1500,1300,1700,1500,1500,1000}},
  {{1500,1500,1500,1500,1500,1000}},
  {{1500,1500,1500,1500,1500,1000}},
};
int16_t PICK_OFFSET[ServoCfg::NUM_JOINTS]      = {0,0,-200,0,0,600};
int16_t PLACE_OFFSET[ServoCfg::NUM_JOINTS]     = {0,0,-200,0,0,1000};
int16_t CLEARANCE_OFFSET[ServoCfg::NUM_JOINTS] = {0,0,300,0,0,0};
// BARU: gerakan tambahan setelah Place, sebelum kembali Home -- default 0 (TIDAK bergerak)
// sampai dikalibrasi manual via menu Jog, supaya tidak ada asumsi arah gerakan yang salah.
int16_t POST_PLACE_OFFSET[ServoCfg::NUM_JOINTS] = {0,0,0,0,0,0};

uint16_t currentUs[ServoCfg::NUM_JOINTS];
uint16_t targetUs[ServoCfg::NUM_JOINTS];
bool moving = false;
uint16_t trajStepUs = 20;          // kecepatan JELAJAH (setelah "pemanasan") -- BISA DIATUR
uint16_t trajStepIntervalMs = 20;  // jeda antar update -- BISA DIATUR
uint32_t lastTrajStepMs = 0;

// BARU: buzzer notifikasi -- 1 siklus ON-OFF non-blocking per trigger event
uint16_t buzzerOnMs = 500, buzzerOffMs = 500;
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
  if (buzzerCurrentlyOn && elapsed >= buzzerOnMs) {
    io.write(CH::BUZZER, LOW); buzzerCurrentlyOn = false; buzzerStateChangedAt = millis();
  } else if (!buzzerCurrentlyOn && elapsed >= buzzerOffMs) {
    buzzerBeeping = false;
  }
}

// BARU: ramp akselerasi/deselerasi per-joint -- sama pola dgn STOCKER. Mulai pelan
// (rampMinStepUs), naik ke trajStepUs selama rampSteps langkah pertama, pelan lagi
// selama rampSteps langkah terakhir mendekati target. Mengurangi sentakan mekanis +
// lonjakan arus saat servo mulai/berhenti gerak mendadak.
uint16_t rampMinStepUs = 4;     // ukuran step PALING KECIL di awal/akhir gerakan -- BISA DIATUR
uint16_t rampSteps = 15;        // jumlah "langkah update" utk naik/turun kecepatan -- BISA DIATUR
uint16_t moveStepCount[ServoCfg::NUM_JOINTS] = {0};   // hitung sudah berapa kali joint ini di-update sejak mulai gerak
bool jointWasMoving[ServoCfg::NUM_JOINTS] = {false};

enum class QCmd : uint8_t { GOTO_POSE, PICK, PLACE, CLEARANCE, POST_PLACE };
struct QItem { QCmd cmd; uint8_t poseIdx; };
QItem cmdQueue[8]; uint8_t qHead = 0, qTail = 0;
bool ackPending = false; uint16_t pendingAckSeq = 0;

// --- Menu state (dideklarasikan di sini, SEBELUM onCmdWrite, supaya urutan kompilasi benar) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, JOG_JOINT, WAIT_SAVE_SLOT, JOG_OFFSET, JOG_SPEED,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_CATEGORY, TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_RELAY, TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET };
MenuState menuState = MenuState::NONE;

void usToDuty(uint8_t ch, uint16_t us) {
  us = constrain(us, ServoCfg::MIN_US, ServoCfg::MAX_US);
  uint16_t duty = (uint32_t)us * 4096 / 20000;
  pwm.setPWM(ch, 0, duty);
}

void startMoveAbs(const uint16_t target[ServoCfg::NUM_JOINTS], uint16_t) {
  // DIPERBAIKI (bug kritis): target WAJIB dibatasi ke rentang servo SEBELUM disimpan --
  // sebelumnya cuma currentUs yang dibatasi (baris updateTrajectory), targetUs dibiarkan
  // bebas. Kalau target di luar rentang (misal dari offset kalibrasi terlalu besar),
  // diff TIDAK PERNAH mencapai 0 (currentUs mentok di batas, target tetap di luar) --
  // firmware TERUS mengira belum sampai, TERUS kirim sinyal dorong tanpa henti, servo
  // menekan gear internal melawan batas mekanis terus-menerus -- inilah yang membuat
  // gear MG996R aus. Constrain di sini memastikan diff BISA benar-benar mencapai 0.
  for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++)
    targetUs[i] = (uint16_t)constrain((int)target[i], (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US);
  moving = true;
  currentState = NodeState::RUNNING_OR_MOVING;
}
void startMoveDelta(const int16_t delta[ServoCfg::NUM_JOINTS], uint16_t durationMs) {
  // DIPERBAIKI (bug kedua): hitung di ranah SIGNED (int32_t) dulu, BARU clamp -- kalau
  // langsung disimpan ke uint16_t, hasil NEGATIF (currentUs kecil + delta negatif besar)
  // akan WRAP-AROUND jadi angka besar positif (mis. -300 -> 65236), membuat constrain()
  // di startMoveAbs() salah arah (mengira di ATAS batas, padahal harusnya DI BAWAH batas).
  uint16_t target[ServoCfg::NUM_JOINTS];
  for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) {
    int32_t raw = (int32_t)currentUs[i] + (int32_t)delta[i];
    target[i] = (uint16_t)constrain(raw, (int32_t)ServoCfg::MIN_US, (int32_t)ServoCfg::MAX_US);
  }
  startMoveAbs(target, durationMs);
}

// BARU: hitung step size ramped -- mirip computeRampedInterval() di STOCKER, tapi di sini
// yang berubah UKURAN STEP (bukan interval waktu), karena PICKER pakai step-tetap-per-tick.
uint16_t computeRampedStep(uint8_t jointIdx, int32_t stepsFromStart, int32_t stepsRemaining) {
  int32_t r = min(stepsFromStart, stepsRemaining);
  if (r >= rampSteps || rampSteps == 0) return trajStepUs;   // sudah di kecepatan jelajah penuh
  float t = (float)r / rampSteps;
  return rampMinStepUs + (uint16_t)((float)(trajStepUs - rampMinStepUs) * t);
}

void updateTrajectory() {
  if (!moving) return;
  uint32_t now = millis();
  if (now - lastTrajStepMs < trajStepIntervalMs) return;
  lastTrajStepMs = now;
  bool anyMoving = false;
  for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) {
    int32_t diff = (int32_t)targetUs[i] - (int32_t)currentUs[i];
    if (diff == 0) { jointWasMoving[i] = false; moveStepCount[i] = 0; continue; }
    anyMoving = true;
    if (!jointWasMoving[i]) { moveStepCount[i] = 0; jointWasMoving[i] = true; }   // joint baru mulai gerak
    uint16_t maxStep = computeRampedStep(i, moveStepCount[i], abs(diff) / max((uint16_t)1, trajStepUs));
    int16_t step = (abs(diff) < (int32_t)maxStep) ? diff : (diff > 0 ? (int16_t)maxStep : -(int16_t)maxStep);
    currentUs[i] = constrain((int)currentUs[i] + step, (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US);
    usToDuty(i, currentUs[i]);
    moveStepCount[i]++;
  }
  if (!anyMoving) {
    moving = false;
    if (currentState == NodeState::RUNNING_OR_MOVING) currentState = NodeState::IDLE;
  }
  static uint32_t moveTimingStartMs = 0;
  static bool wasMoving = false;
  if (moving && !wasMoving) moveTimingStartMs = millis();
  if (!moving && wasMoving) Serial.printf("[MOVE] selesai dalam %lums\n", (unsigned long)(millis() - moveTimingStartMs));
  wasMoving = moving;
}

void enqueue(QCmd c, uint8_t poseIdx = 0) {
  uint8_t next = (qTail + 1) % 8;
  if (next == qHead) return;
  cmdQueue[qTail] = {c, poseIdx}; qTail = next;
}
// BARU: lacak aksi yang SEDANG dieksekusi -- utk tampilan aktivitas spesifik di LCD
QCmd currentAction = QCmd::GOTO_POSE;
uint8_t currentActionPoseIdx = 0;

void processQueue() {
  if (moving || qHead == qTail) return;
  QItem item = cmdQueue[qHead]; qHead = (qHead + 1) % 8;
  currentAction = item.cmd; currentActionPoseIdx = item.poseIdx;   // BARU
  switch (item.cmd) {
    case QCmd::GOTO_POSE: startMoveAbs(POSES[item.poseIdx].us, 800); break;
    case QCmd::PICK:  startMoveDelta(PICK_OFFSET, 400); break;
    case QCmd::PLACE: startMoveDelta(PLACE_OFFSET, 400); triggerBuzzerBeep(); break;   // BARU -- notifikasi: package diletakkan
    case QCmd::CLEARANCE: startMoveDelta(CLEARANCE_OFFSET, 400); break;
    case QCmd::POST_PLACE: startMoveDelta(POST_PLACE_OFFSET, 400); break;
  }
}
void checkSequenceComplete() {
  if (ackPending && qHead == qTail && !moving) {
    ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq);
    triggerBuzzerBeep();   // BARU -- notifikasi: siklus selesai, sudah kembali Home
  }
}

void loadPosesFromNvs() {
  prefs.begin("picker_cal", true);
  if (prefs.isKey("poses")) prefs.getBytes("poses", POSES, sizeof(POSES));
  if (prefs.isKey("pick"))  prefs.getBytes("pick", PICK_OFFSET, sizeof(PICK_OFFSET));
  if (prefs.isKey("place")) prefs.getBytes("place", PLACE_OFFSET, sizeof(PLACE_OFFSET));
  if (prefs.isKey("clear")) prefs.getBytes("clear", CLEARANCE_OFFSET, sizeof(CLEARANCE_OFFSET));
  if (prefs.isKey("postplace")) prefs.getBytes("postplace", POST_PLACE_OFFSET, sizeof(POST_PLACE_OFFSET));
  if (prefs.isKey("trajStep")) trajStepUs = prefs.getUShort("trajStep", trajStepUs);
  if (prefs.isKey("trajIntv")) trajStepIntervalMs = prefs.getUShort("trajIntv", trajStepIntervalMs);
  if (prefs.isKey("rampMin")) rampMinStepUs = prefs.getUShort("rampMin", rampMinStepUs);   // BARU
  if (prefs.isKey("rampSteps")) rampSteps = prefs.getUShort("rampSteps", rampSteps);       // BARU
  if (prefs.isKey("buzzOn")) buzzerOnMs = prefs.getUShort("buzzOn", buzzerOnMs);           // BARU
  if (prefs.isKey("buzzOff")) buzzerOffMs = prefs.getUShort("buzzOff", buzzerOffMs);       // BARU
  prefs.end();
}
void savePosesToNvs() { prefs.begin("picker_cal", false); prefs.putBytes("poses", POSES, sizeof(POSES)); prefs.end(); }
void saveOffsetToNvs(const char* key, int16_t* arr, size_t len) {
  prefs.begin("picker_cal", false); prefs.putBytes(key, arr, len); prefs.end();
}
void saveSpeedToNvs() {
  prefs.begin("picker_cal", false);
  prefs.putUShort("trajStep", trajStepUs);
  prefs.putUShort("trajIntv", trajStepIntervalMs);
  prefs.end();
}
void saveRampToNvs() {   // BARU
  prefs.begin("picker_cal", false);
  prefs.putUShort("rampMin", rampMinStepUs);
  prefs.putUShort("rampSteps", rampSteps);
  prefs.end();
}
void saveBuzzerToNvs() {   // BARU
  prefs.begin("picker_cal", false);
  prefs.putUShort("buzzOn", buzzerOnMs);
  prefs.putUShort("buzzOff", buzzerOffMs);
  prefs.end();
}
// BARU: reset total -- hapus SEMUA kalibrasi NVS ("picker_cal" namespace) & kembalikan RAM ke default
void resetAllToDefault() {
  prefs.begin("picker_cal", false);
  prefs.clear();
  prefs.end();
  Pose defaultPoses[4] = {
    {{1500,1500,1500,1500,1500,1000}}, {{1200,1600,1400,1500,1500,1000}},
    {{1800,1600,1400,1500,1500,1000}}, {{1500,1300,1700,1500,1500,1000}},
  };
  memcpy(POSES, defaultPoses, sizeof(POSES));
  int16_t defPick[ServoCfg::NUM_JOINTS] = {0,0,-200,0,0,600};
  int16_t defPlace[ServoCfg::NUM_JOINTS] = {0,0,-200,0,0,1000};
  int16_t defClear[ServoCfg::NUM_JOINTS] = {0,0,300,0,0,0};
  int16_t defPostPlace[ServoCfg::NUM_JOINTS] = {0,0,0,0,0,0};
  memcpy(PICK_OFFSET, defPick, sizeof(PICK_OFFSET));
  memcpy(PLACE_OFFSET, defPlace, sizeof(PLACE_OFFSET));
  memcpy(CLEARANCE_OFFSET, defClear, sizeof(CLEARANCE_OFFSET));
  memcpy(POST_PLACE_OFFSET, defPostPlace, sizeof(POST_PLACE_OFFSET));
  trajStepUs = 20; trajStepIntervalMs = 20;
  rampMinStepUs = 4; rampSteps = 15;   // BARU
  buzzerOnMs = 500; buzzerOffMs = 500;   // BARU
  Serial.println("[RESET] Semua kalibrasi PICKER dikembalikan ke default & NVS dihapus");
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

// BARU: teks aktivitas spesifik -- terjemahkan currentAction jadi info jelas, bukan cuma "RUNNING"
String activityText() {
  if (currentState == NodeState::FAULT) return "FAULT!";
  if (currentState == NodeState::ESTOPPED) return "E-STOP!";
  if (!moving) return "Diam";
  switch (currentAction) {
    case QCmd::GOTO_POSE:
      switch (currentActionPoseIdx) {
        case 0: return "Menuju Home";
        case 1: return "Menuju Pass";
        case 2: return "Menuju Reject";
        case 3: return "Menuju Lift";
      }
      return "Bergerak";
    case QCmd::PICK: return "Mengambil (Pick)";
    case QCmd::PLACE: return "Meletakkan (Place)";
    case QCmd::CLEARANCE: return "Naik (Clearance)";
    case QCmd::POST_PLACE: return "Post-Place";
  }
  return "Bergerak";
}

// BARU: versi kode numerik dari activityText() -- dikirim ke register Modbus (Lapis 2)
ActivityCode activityCode() {
  if (currentState == NodeState::FAULT) return ActivityCode::FAULT_AKTIF;
  if (currentState == NodeState::ESTOPPED) return ActivityCode::ESTOP_AKTIF;
  if (!moving) return ActivityCode::DIAM;
  switch (currentAction) {
    case QCmd::GOTO_POSE:
      switch (currentActionPoseIdx) {
        case 0: return ActivityCode::MENUJU_HOME;
        case 1: return ActivityCode::MENUJU_PASS;
        case 2: return ActivityCode::MENUJU_REJECT;
        case 3: return ActivityCode::MENUJU_LIFT;
      }
      return ActivityCode::BERGERAK;
    case QCmd::PICK: return ActivityCode::MENGAMBIL;
    case QCmd::PLACE: return ActivityCode::MELETAKKAN;
    case QCmd::CLEARANCE: return ActivityCode::NAIK_CLEARANCE;
    case QCmd::POST_PLACE: return ActivityCode::POST_PLACE_GERAK;
  }
  return ActivityCode::BERGERAK;
}

// BARU: indikator universal (sama pola di semua 4 node)
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

// DIPERBAIKI: guard FAULT/ESTOPPED -- sebelumnya opcode gerak (RUN_SEQUENCE dkk) TIDAK dicek sama
// sekali, sama seperti bug START di SORTER yang baru ditemukan. Sekarang seragam diproteksi.
bool blockIfFaulted(const char* opName) {
  if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    Serial.printf("[CMD] %s ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu\n", opName);
    return true;
  }
  return false;
}

void applyCommand(uint16_t opcode, uint16_t arg) {
  switch ((Cmd)opcode) {
    case Cmd::RUN_SEQUENCE:
      if (blockIfFaulted("RUN_SEQUENCE")) return;
      enqueue(QCmd::GOTO_POSE, 0);
      enqueue(QCmd::CLEARANCE);
      triggerBuzzerBeep();   // BARU -- notifikasi: arm mulai menuju objek untuk diambil
      enqueue(QCmd::GOTO_POSE, arg == 1 ? 1 : 2);
      enqueue(QCmd::PICK);
      enqueue(QCmd::GOTO_POSE, 3);
      enqueue(QCmd::PLACE);
      enqueue(QCmd::POST_PLACE);   // BARU -- gerakan tambahan sebelum kembali Home
      enqueue(QCmd::GOTO_POSE, 0);
      break;
    case Cmd::GOTO_HOME:   if (blockIfFaulted("GOTO_HOME")) return;   enqueue(QCmd::GOTO_POSE, 0); break;
    case Cmd::GOTO_PASS:   if (blockIfFaulted("GOTO_PASS")) return;   enqueue(QCmd::GOTO_POSE, 1); break;
    case Cmd::GOTO_REJECT: if (blockIfFaulted("GOTO_REJECT")) return; enqueue(QCmd::GOTO_POSE, 2); break;
    case Cmd::PICK:  if (blockIfFaulted("PICK")) return;  enqueue(QCmd::PICK); break;
    case Cmd::PLACE: if (blockIfFaulted("PLACE")) return; enqueue(QCmd::PLACE); break;
    // BARU: angkat 1 package (batch objek) dari posisi ujung conveyor (pose4=PACKAGE_PICKUP)
    // ke Lift Load Position STOCKER (pose5=LIFT_LOAD). Dipicu Orange Pi saat SORTER.PASS_COUNT
    // capai batas batch -- Orange Pi kirim RUN_FULL_CYCLE ke STOCKER SETELAH ack command ini.
    case Cmd::MOVE_PACKAGE:
      if (blockIfFaulted("MOVE_PACKAGE")) return;
      enqueue(QCmd::GOTO_POSE, 0);
      enqueue(QCmd::CLEARANCE);
      triggerBuzzerBeep();
      enqueue(QCmd::GOTO_POSE, 4);
      enqueue(QCmd::PICK);
      enqueue(QCmd::GOTO_POSE, 5);
      enqueue(QCmd::PLACE);
      enqueue(QCmd::POST_PLACE);
      enqueue(QCmd::GOTO_POSE, 0);
      break;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb sebelum di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE;
      break;
    // BARU -- biar Orange Pi bisa tuning kecepatan trajectory langsung (berlaku sama ke semua
    // 6 joint), runtime-only (gak auto-save NVS -- simpan permanen tetap lewat LCD '#' / Serial
    // kalau mau bertahan setelah reboot, lihat saveSpeedToNvs()).
    case Cmd::SET_TRAJ_STEP:          trajStepUs = (uint16_t)constrain(arg, 1, 500); break;
    case Cmd::SET_TRAJ_STEP_INTERVAL: trajStepIntervalMs = (uint16_t)constrain(arg, 5, 200); break;
    default: Serial.printf("[CMD] opcode %u tidak dikenal\n", opcode); break;
  }
}

uint16_t onCmdWrite(TRegister* reg, uint16_t val) {
  if (menuState != MenuState::NONE) {
    Serial.println("[PICKER] Command Modbus diabaikan -- mode kalibrasi aktif (§12.6)");
    return val;
  }
  lastRs485Rx = millis(); modbusEverUsed = true;
  uint16_t opcode = val;
  uint16_t arg = mb.Hreg(Reg::CMD_ARG);
  uint16_t seq = mb.Hreg(Reg::CMD_SEQ);
  uint16_t lastAcked = mb.Hreg(Reg::CMD_ACK_SEQ);
  if (seq == lastAcked) { Serial.printf("[PICKER] CMD seq=%u sudah diproses -- diabaikan\n", seq); return val; }
  Serial.printf("[PICKER] CMD (Modbus) opcode=%u arg=%u seq=%u\n", opcode, arg, seq);
  applyCommand(opcode, arg);
  if ((Cmd)opcode == Cmd::RUN_SEQUENCE || (Cmd)opcode == Cmd::MOVE_PACKAGE) { ackPending = true; pendingAckSeq = seq; }
  else mb.Hreg(Reg::CMD_ACK_SEQ, seq);
  return val;
}

// ============================================================
// MENU LCD+Keypad -- 3 KATEGORI: Setting Kalibrasi / Test I/O / Test Command
// Skema tombol: layar LIST A=naik B=turun C=pilih D=kembali
//               layar EDIT   A=naik nilai B=turun nilai C=step D=kembali #=SIMPAN
// ============================================================
uint8_t jogStepIdx = 0;
constexpr int16_t JOG_STEPS[4] = {5, 10, 20, 50};
uint8_t selJoint = 0;
uint8_t selSpeedParam = 1;
int16_t* editingOffset = nullptr;
const char* editingOffsetName = "";

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

void drawTopMenu() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
constexpr uint8_t CAL_COUNT = 7;
const char* CAL_LABELS[CAL_COUNT] = { "Pose (Home/Pass/dll)", "Pick Offset", "Place Offset", "Clearance Offset", "Post-Place Offset", "Speed (Step/Interval)", "Reset ke Default" };
uint8_t calCursor = 0;

void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

void drawConfirmReset() {
  lcd.clear();
  lcdPrint(0, 0, "RESET KE DEFAULT?");
  lcdPrint(0, 1, "Pose & offset akan");
  lcdPrint(0, 2, "HILANG, TAK BS BATAL");
  lcdPrint(0, 3, "C=YA,RESET D=batal");
}
void handleConfirmResetKey(char key) {
  if (key == 'C') {
    resetAllToDefault();
    for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) currentUs[i] = POSES[0].us[i];
    lcdPrint(0, 3, "SUDAH DIRESET!      ");
    menuState = MenuState::CAL_LIST;
  } else if (key == 'D') {
    Serial.println("[CAL] Reset dibatalkan");
    menuState = MenuState::CAL_LIST;
    drawCalList();
  }
}

void drawCalibrationLcd() {
  lcdPrint(0, 0, "CAL Joint" + String(selJoint) + " us:" + String(currentUs[selJoint]));
  lcdPrint(0, 1, "Step:" + String(JOG_STEPS[jogStepIdx]) + "  A+ B- C:step");
  lcdPrint(0, 2, "0-5=joint D=set");
  lcdPrint(0, 3, "#=save D=kembali");
}
void handleCalibrationKey(char key) {
  if (key >= '0' && key <= '5') { selJoint = key - '0'; }
  else if (key == 'A') { currentUs[selJoint] = constrain((int)currentUs[selJoint] + JOG_STEPS[jogStepIdx], (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US); usToDuty(selJoint, currentUs[selJoint]); }
  else if (key == 'B') { currentUs[selJoint] = constrain((int)currentUs[selJoint] - JOG_STEPS[jogStepIdx], (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US); usToDuty(selJoint, currentUs[selJoint]); }
  else if (key == 'C') { jogStepIdx = (jogStepIdx + 1) % 4; }
  else if (key == '#') { menuState = MenuState::WAIT_SAVE_SLOT; lcdPrint(0, 3, "Slot? 0-5 (cek doc)"); return; }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawCalibrationLcd();
}
void handleSaveSlotKey(char key) {
  if (key >= '0' && key <= '5') {
    uint8_t slot = key - '0';
    memcpy(POSES[slot].us, currentUs, sizeof(currentUs));
    savePosesToNvs();
    lcdPrint(0, 3, "Tersimpan slot " + String(slot));
    Serial.printf("[CAL] Pose disimpan ke slot %u (NVS)\n", slot);
    menuState = MenuState::JOG_JOINT;
  } else if (key == 'D') { menuState = MenuState::JOG_JOINT; drawCalibrationLcd(); }
}

void drawOffsetMenu() {
  lcdPrint(0, 0, String(editingOffsetName) + " Joint" + String(selJoint));
  lcdPrint(0, 1, "Delta:" + String(editingOffset[selJoint]) + "  Step:" + String(JOG_STEPS[jogStepIdx]));
  lcdPrint(0, 2, "0-5=joint A+B-C:step");
  lcdPrint(0, 3, "#=SAVE D=kembali");
}
void previewOffsetOnServo() {
  uint16_t preview = constrain((int)POSES[0].us[selJoint] + editingOffset[selJoint], (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US);
  usToDuty(selJoint, preview);
}
void handleOffsetKey(char key) {
  if (key >= '0' && key <= '5') { selJoint = key - '0'; previewOffsetOnServo(); }
  else if (key == 'A') { editingOffset[selJoint] += JOG_STEPS[jogStepIdx]; previewOffsetOnServo(); }
  else if (key == 'B') { editingOffset[selJoint] -= JOG_STEPS[jogStepIdx]; previewOffsetOnServo(); }
  else if (key == 'C') { jogStepIdx = (jogStepIdx + 1) % 4; }
  else if (key == '#') {
    const char* nvsKey = (editingOffset == PICK_OFFSET) ? "pick" : (editingOffset == PLACE_OFFSET) ? "place"
                        : (editingOffset == CLEARANCE_OFFSET) ? "clear" : "postplace";
    saveOffsetToNvs(nvsKey, editingOffset, ServoCfg::NUM_JOINTS * sizeof(int16_t));
    lcdPrint(0, 3, String(editingOffsetName) + " tersimpan!");
    Serial.printf("[CAL] %s offset disimpan ke NVS\n", editingOffsetName);
    return;
  } else if (key == 'D') {
    menuState = MenuState::CAL_LIST;
    for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) usToDuty(i, currentUs[i]);   // batalkan preview
    drawCalList();
    return;
  }
  drawOffsetMenu();
}

void drawSpeedMenu() {
  lcdPrint(0, 0, "KECEPATAN GERAK");
  String line1;
  switch (selSpeedParam) {
    case 1: line1 = "1:Jelajah=" + String(trajStepUs) + "us"; break;
    case 2: line1 = "2:Interval=" + String(trajStepIntervalMs) + "ms"; break;
    case 3: line1 = "3:RampMin=" + String(rampMinStepUs) + "us"; break;
    case 4: line1 = "4:RampSteps=" + String(rampSteps); break;
    case 5: line1 = "5:BuzzOn=" + String(buzzerOnMs) + "ms"; break;
    case 6: line1 = "6:BuzzOff=" + String(buzzerOffMs) + "ms"; break;
  }
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, "1-6=pilih A+B-C:step");
  lcdPrint(0, 3, "#=SIMPAN D=kembali");
}
void handleSpeedKey(char key) {
  int16_t step = JOG_STEPS[jogStepIdx];
  if (key >= '1' && key <= '6') { selSpeedParam = key - '0'; }
  else if (key == 'A' || key == 'B') {
    int delta = (key == 'A') ? step : -step;
    switch (selSpeedParam) {
      case 1: trajStepUs = (uint16_t)constrain((int)trajStepUs + delta, 1, 500); break;
      case 2: trajStepIntervalMs = (uint16_t)constrain((int)trajStepIntervalMs + delta, 5, 200); break;
      case 3: rampMinStepUs = (uint16_t)constrain((int)rampMinStepUs + delta, 1, 500); break;
      case 4: rampSteps = (uint16_t)constrain((int)rampSteps + delta, 0, 200); break;
      case 5: buzzerOnMs = (uint16_t)constrain((int)buzzerOnMs + delta * 10, 50, 5000); break;
      case 6: buzzerOffMs = (uint16_t)constrain((int)buzzerOffMs + delta * 10, 50, 5000); break;
    }
  } else if (key == 'C') { jogStepIdx = (jogStepIdx + 1) % 4; }
  else if (key == '#') {
    saveSpeedToNvs(); saveRampToNvs(); saveBuzzerToNvs();
    lcdPrint(0, 3, "TERSIMPAN ke NVS!");
    Serial.println("[CAL] Step/Interval/Ramp/Buzzer disimpan");
    return;
  }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawSpeedMenu();
}

void handleCalListKey(char key) {
  if (key == 'A') { calCursor = (calCursor == 0) ? CAL_COUNT - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % CAL_COUNT; drawCalList(); }
  else if (key == 'C') {
    switch (calCursor) {
      case 0: menuState = MenuState::JOG_JOINT; lcd.clear(); drawCalibrationLcd(); break;
      case 1: editingOffset = PICK_OFFSET; editingOffsetName = "PICK"; menuState = MenuState::JOG_OFFSET; lcd.clear(); drawOffsetMenu(); break;
      case 2: editingOffset = PLACE_OFFSET; editingOffsetName = "PLACE"; menuState = MenuState::JOG_OFFSET; lcd.clear(); drawOffsetMenu(); break;
      case 3: editingOffset = CLEARANCE_OFFSET; editingOffsetName = "CLEARANCE"; menuState = MenuState::JOG_OFFSET; lcd.clear(); drawOffsetMenu(); break;
      case 4: editingOffset = POST_PLACE_OFFSET; editingOffsetName = "POSTPLACE"; menuState = MenuState::JOG_OFFSET; lcd.clear(); drawOffsetMenu(); break;
      case 5: menuState = MenuState::JOG_SPEED; lcd.clear(); drawSpeedMenu(); break;
      case 6: menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenu(); }
}

// --- LEVEL 1b: TEST I/O ---
struct IOTestItem { const char* label; uint8_t ch; bool autoControlled; };
void drawTestIoCategory();

// --- Kategori: Test Output ---
constexpr uint8_t OUTPUT_TEST_COUNT = 4;
IOTestItem OUTPUT_TEST_ITEMS[OUTPUT_TEST_COUNT] = {
  {"OPR",    CH::LED_OPERATION, true},
  {"RUN",    CH::LED_RUN,       true},
  {"MANUAL", CH::LED_MANUAL,    true},
  {"FAULT",  CH::LED_FAULT,     false},
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

// --- Kategori: Test Input -- 3 sub-kategori (Button/Proximity/Limit Switch), SAMA PERSIS
// channel-nya di semua 4 node (board PCB universal) -- test channel mentah CH::xxx, TERLEPAS
// dari alias/peran yang dipakai node ini.
constexpr uint8_t BUTTON_TEST_COUNT = 3;
IOTestItem BUTTON_TEST_ITEMS[BUTTON_TEST_COUNT] = {
  {"ESTOP", CH::ESTOP,    false},
  {"BTN2",  CH::BUTTON_2, false},
  {"BTN3",  CH::BUTTON_3, false},
};
constexpr uint8_t PROX_TEST_COUNT = 2;
IOTestItem PROX_TEST_ITEMS[PROX_TEST_COUNT] = {
  {"PROX1", CH::PROX_1, false},
  {"PROX2", CH::PROX_2, false},
};
constexpr uint8_t LIMIT_TEST_COUNT = 10;
IOTestItem LIMIT_TEST_ITEMS[LIMIT_TEST_COUNT] = {
  {"LIM1",  CH::LIM_1,  false}, {"LIM2",  CH::LIM_2,  false},
  {"LIM3",  CH::LIM_3,  false}, {"LIM4",  CH::LIM_4,  false},
  {"LIM5",  CH::LIM_5,  false}, {"LIM6",  CH::LIM_6,  false},
  {"LIM7",  CH::LIM_7,  false}, {"LIM8",  CH::LIM_8,  false},
  {"LIM9",  CH::LIM_9,  false}, {"LIM10", CH::LIM_10, false},
};

constexpr uint8_t INPUT_CAT_COUNT = 3;
const char* INPUT_CAT_LABELS[INPUT_CAT_COUNT] = { "Button", "Proximity", "Limit Switch" };
uint8_t inputCatCursor = 0;
void drawInputCategorySelect() { drawListMenu("TEST INPUT", INPUT_CAT_LABELS, INPUT_CAT_COUNT, inputCatCursor); }

IOTestItem* activeInputItems = BUTTON_TEST_ITEMS;
uint8_t activeInputCount = BUTTON_TEST_COUNT;
uint8_t inputTestScrollTop = 0;
constexpr uint8_t INPUT_TEST_MAX = LIMIT_TEST_COUNT;
bool testInputLiveLastVal[INPUT_TEST_MAX];
uint8_t testInputLastScrollTop = 255;
void drawTestInputList() {
  bool scrollChanged = (inputTestScrollTop != testInputLastScrollTop);
  if (scrollChanged) { lcd.clear(); testInputLastScrollTop = inputTestScrollTop; }
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idx = inputTestScrollTop + row;
    if (idx >= activeInputCount) continue;
    bool val = io.read(activeInputItems[idx].ch);
    if (scrollChanged || val != testInputLiveLastVal[idx]) {
      testInputLiveLastVal[idx] = val;
      lcdPrint(0, row, String(activeInputItems[idx].label) + " = " + String(val ? 1 : 0) + "         ");
    }
  }
}
void handleTestInputListKey(char key) {
  if (key == 'A') { if (inputTestScrollTop > 0) { inputTestScrollTop--; drawTestInputList(); } }
  else if (key == 'B') { if (inputTestScrollTop < (activeInputCount > 4 ? activeInputCount - 4 : 0)) { inputTestScrollTop++; drawTestInputList(); } }
  else if (key == 'D') { menuState = MenuState::TEST_INPUT_CATEGORY; drawInputCategorySelect(); }
}
void handleInputCategoryKey(char key) {
  if (key == 'A') { inputCatCursor = (inputCatCursor == 0) ? INPUT_CAT_COUNT - 1 : inputCatCursor - 1; drawInputCategorySelect(); }
  else if (key == 'B') { inputCatCursor = (inputCatCursor + 1) % INPUT_CAT_COUNT; drawInputCategorySelect(); }
  else if (key == 'C') {
    switch (inputCatCursor) {
      case 0: activeInputItems = BUTTON_TEST_ITEMS; activeInputCount = BUTTON_TEST_COUNT; break;
      case 1: activeInputItems = PROX_TEST_ITEMS;   activeInputCount = PROX_TEST_COUNT;   break;
      case 2: activeInputItems = LIMIT_TEST_ITEMS;  activeInputCount = LIMIT_TEST_COUNT;  break;
    }
    inputTestScrollTop = 0; testInputLastScrollTop = 255;
    menuState = MenuState::TEST_INPUT_LIST; drawTestInputList();
  }
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

// --- Kategori: Test Modul -- sub-menu pilihan JENIS modul, SAMA di semua 4 node (board universal).
// PICKER sudah punya driver PCA9685 RESMI (objek pwm) -- Test Modul Servo REUSE ini langsung,
// TIDAK bikin driver raw terpisah (beda dgn SORTER/STOCKER yang tidak punya driver resmi).
bool testModFirstDraw = true;
String testModLine1 = "";
constexpr uint8_t MODULE_TYPE_COUNT = 4;
const char* MODULE_TYPE_LABELS[MODULE_TYPE_COUNT] = { "Stepper", "Motor DC", "Relay", "Servo" };
uint8_t moduleTypeCursor = 0;
void drawModuleTypeSelect() { drawListMenu("TEST MODUL", MODULE_TYPE_LABELS, MODULE_TYPE_COUNT, moduleTypeCursor); }
void handleModuleTypeKey(char key);

// --- Modul: Stepper (STEP_1/2/3 + DIR_1/2/3_MCP + EN_123) -- PICKER tidak punya infrastruktur
// motion, pulsa manual sederhana. Blocking SESAAT (~80ms/tekan) -- pengecualian wajar, murni
// test 1x-tekan, bukan continuous production loop.
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

// --- Modul: Motor DC (AIN1/AIN2/BIN1/BIN2/STBY) -- PICKER tidak pakai channel ini di produksi,
// aman ditest kapan saja.
uint8_t testModMotorAState = 0, testModMotorBState = 0;
void testModSetMotor(bool channelA, uint8_t dirState) {
  uint8_t ain1 = channelA ? CH::AIN1 : CH::BIN1, ain2 = channelA ? CH::AIN2 : CH::BIN2;
  if (channelA) testModMotorAState = dirState; else testModMotorBState = dirState;
  if (dirState == 0) { io.write(ain1, LOW); io.write(ain2, LOW); }
  else { io.write(ain1, dirState == 1); io.write(ain2, dirState != 1); }
  io.write(CH::STBY, (testModMotorAState != 0 || testModMotorBState != 0));
}
// DIPERBAIKI (bug ditemukan): sebelumnya A/B cuma toggle stop<->maju, dirState=2 (mundur)
// TIDAK PERNAH bisa dicapai dari menu -- arah mundur gak bisa ditest/dikalibrasi sama sekali.
// Sekarang A/B cycle stop->maju->mundur->stop tiap ditekan.
const char* motorDirName(uint8_t s) { return s == 0 ? "STOP" : (s == 1 ? "FWD" : "REV"); }
void drawTestModMotorDC() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: MOTOR DC");
    lcdPrint(0, 3, "A=chA B=chB D=kmb");
    testModFirstDraw = false; testModLine1 = "\x01";
  }
  String line1 = "ChA:" + String(motorDirName(testModMotorAState)) + " ChB:" + String(motorDirName(testModMotorBState));
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
  lcdPrint(0, 2, "A/B=cycle arah");
}
void handleTestModMotorDCKey(char key) {
  if (key == 'A') { testModSetMotor(true, (testModMotorAState + 1) % 3); }
  else if (key == 'B') { testModSetMotor(false, (testModMotorBState + 1) % 3); }
  else if (key == 'D') {
    testModSetMotor(true, 0); testModSetMotor(false, 0);
    menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return;
  }
  drawTestModMotorDC();
}

// --- Modul: Relay (RLY1/RLY2) ---
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

// --- Modul: Servo -- REUSE driver PCA9685 RESMI (objek pwm) yang sudah aktif produksi ---
uint8_t testModServoCh = 0;
void drawTestModServo() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: SERVO");
    lcdPrint(0, 3, "C=ch A/B=gerak D=kmb");
    testModFirstDraw = false; testModLine1 = "\x01";
  }
  String line1 = "Joint:" + String(testModServoCh) + " us:" + String(currentUs[testModServoCh]);
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
}
void handleTestModServoKey(char key) {
  if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  else if (key == 'C') { testModServoCh = (testModServoCh + 1) % ServoCfg::NUM_JOINTS; }
  else if (key == 'A') { currentUs[testModServoCh] = constrain((int)currentUs[testModServoCh] + 100, (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US); usToDuty(testModServoCh, currentUs[testModServoCh]); }
  else if (key == 'B') { currentUs[testModServoCh] = constrain((int)currentUs[testModServoCh] - 100, (int)ServoCfg::MIN_US, (int)ServoCfg::MAX_US); usToDuty(testModServoCh, currentUs[testModServoCh]); }
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
      case 2: menuState = MenuState::TEST_INPUT_CATEGORY; inputCatCursor = 0; drawInputCategorySelect(); break;
      case 3: menuState = MenuState::TEST_RS485; drawTestRs485(); break;
      case 4: menuState = MenuState::TEST_MODULE_SELECT; moduleTypeCursor = 0; drawModuleTypeSelect(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenu(); }
}

// --- LEVEL 1c: TEST COMMAND ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 9;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"RUN_SEQUENCE(pass)",   Cmd::RUN_SEQUENCE, 1},
  {"RUN_SEQUENCE(reject)", Cmd::RUN_SEQUENCE, 2},
  {"GOTO_HOME",            Cmd::GOTO_HOME,    0},
  {"GOTO_PASS",            Cmd::GOTO_PASS,    0},
  {"GOTO_REJECT",          Cmd::GOTO_REJECT,  0},
  {"PICK",                 Cmd::PICK,         0},
  {"PLACE",                Cmd::PLACE,        0},
  {"MOVE_PACKAGE",         Cmd::MOVE_PACKAGE, 0},
  {"RESET_FAULT",          Cmd::RESET_FAULT,  0},
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
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenu(); }
}

void handleTopMenuKey(char key) {
  if (key == 'A') { topCursor = (topCursor == 0) ? TOP_COUNT - 1 : topCursor - 1; drawTopMenu(); }
  else if (key == 'B') { topCursor = (topCursor + 1) % TOP_COUNT; drawTopMenu(); }
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
  cmd.toUpperCase();

  if (cmd == "JOG") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t joint = line.substring(sp1 + 1, sp2).toInt();
    uint16_t us = line.substring(sp2 + 1).toInt();
    if (joint < ServoCfg::NUM_JOINTS) {
      currentUs[joint] = constrain(us, ServoCfg::MIN_US, ServoCfg::MAX_US);
      usToDuty(joint, currentUs[joint]);
      Serial.printf("[JOG] Joint %u -> %u us\n", joint, currentUs[joint]);
    } else Serial.println("[JOG] Joint harus 0-5");
  }
  else if (cmd == "SAVEPOSE") {
    uint8_t slot = line.substring(sp1 + 1).toInt();
    if (slot < 6) { memcpy(POSES[slot].us, currentUs, sizeof(currentUs)); savePosesToNvs(); Serial.printf("[SAVEPOSE] -> slot %u\n", slot); }
    else Serial.println("[SAVEPOSE] Slot harus 0-5");
  }
  else if (cmd == "GOTO") {
    uint8_t slot = line.substring(sp1 + 1).toInt();
    if (slot < 6) { enqueue(QCmd::GOTO_POSE, slot); Serial.printf("[GOTO] Pindah ke pose %u\n", slot); }
    else Serial.println("[GOTO] Slot harus 0-5");
  }
  else if (cmd == "SEQ") {
    uint16_t arg = line.substring(sp1 + 1).toInt();
    applyCommand((uint16_t)Cmd::RUN_SEQUENCE, arg);
    Serial.printf("[SEQ] RUN_SEQUENCE arg=%u dimulai\n", arg);
  }
  else if (cmd == "MOVEPKG") {
    applyCommand((uint16_t)Cmd::MOVE_PACKAGE, 0);
    Serial.println("[MOVEPKG] MOVE_PACKAGE dimulai");
  }
  else if (cmd == "STEP") { trajStepUs = constrain((int)line.substring(sp1 + 1).toInt(), 1, 500); Serial.printf("[STEP] %u us\n", trajStepUs); }
  else if (cmd == "INTERVAL") { trajStepIntervalMs = constrain((int)line.substring(sp1 + 1).toInt(), 5, 200); Serial.printf("[INTERVAL] %u ms\n", trajStepIntervalMs); }
  else if (cmd == "RAMPMIN") { rampMinStepUs = constrain((int)line.substring(sp1 + 1).toInt(), 1, 500); saveRampToNvs(); Serial.printf("[RAMPMIN] %u us\n", rampMinStepUs); }
  else if (cmd == "RAMPSTEPS") { rampSteps = constrain((int)line.substring(sp1 + 1).toInt(), 0, 200); saveRampToNvs(); Serial.printf("[RAMPSTEPS] %u\n", rampSteps); }
  else if (cmd == "RESET") { resetAllToDefault(); for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) currentUs[i] = POSES[0].us[i]; }
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s fault=%u ESTOP=%d moving=%d qDepth=%d step=%uus interval=%ums rampMin=%uus rampSteps=%u\n",
                  stateText(currentState), faultCode, io.read(CH::ESTOP), moving, (qTail - qHead + 8) % 8,
                  trajStepUs, trajStepIntervalMs, rampMinStepUs, rampSteps);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000));
    Serial.print("[STATUS] currentUs: ");
    for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) { Serial.print(currentUs[i]); Serial.print(" "); }
    Serial.println();
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] JOG <0-5> <us> | SAVEPOSE <0-5> | GOTO <0-5> | SEQ <1|2> | MOVEPKG | STEP <us> | INTERVAL <ms>");
    Serial.println("[HELP] RAMPMIN <us> | RAMPSTEPS <n> | RESET | STATUS | HELP");
  }
  else Serial.printf("[SERIAL] '%s' tidak dikenal -- ketik HELP\n", cmd.c_str());
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH dari HIGH ke LOW
    io.write(CH::PCA_OE, HIGH);
    currentState = NodeState::ESTOPPED; moving = false;
    qHead = qTail = 0; ackPending = false;
    return;
  }
  if (currentState == NodeState::ESTOPPED) {
    io.write(CH::PCA_OE, LOW);
    currentState = NodeState::IDLE;
  }
}

// ============================================================
// OTA (WiFi) -- update firmware tanpa colok-cabut USB. Lihat wifi_credentials.h
// (gitignored, isi asli SSID/password/OTA password per file .example di include/).
// ============================================================
bool otaReady = false;   // true == WiFi connect & ArduinoOTA siap terima update sekarang
bool otaBegun = false;   // ArduinoOTA.begin() sudah dipanggil sekali (callback ter-register)

// Dipanggil dari ArduinoOTA.onStart() -- proses tulis flash BLOCKING selama beberapa detik.
// Matikan output PCA9685 (sama seperti E-stop fisik) supaya arm tidak nyangkut nge-drive
// posisi lama selama CPU sibuk nulis flash.
void otaSafeStop() {
  io.write(CH::PCA_OE, HIGH);
  moving = false;
  qHead = qTail = 0; ackPending = false;
}

void beginOtaService() {
  if (otaBegun) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { Serial.println("[OTA] Update mulai -- stop semua actuator"); otaSafeStop(); });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] Update selesai, reboot..."); });
  ArduinoOTA.onError([](ota_error_t err) { Serial.printf("[OTA] Error [%u]\n", err); });
  ArduinoOTA.begin();
  otaBegun = true;
  Serial.println("[OTA] Siap terima update");
}

// Dipanggil SEKALI di setup() -- boleh nunggu (blocking) sebentar, ini masih fase boot.
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[OTA] Menyambung WiFi '%s'...\n", WIFI_SSID);
  uint32_t deadline = millis() + 8000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[OTA] WiFi OK, IP=%s\n", WiFi.localIP().toString().c_str());
    beginOtaService();
    otaReady = true;
  } else {
    Serial.println("[OTA] WiFi gagal connect (timeout) -- lanjut boot, auto-retry non-blocking di loop()");
  }
}

// Dipanggil TIAP loop() -- NON-BLOCKING (tidak ada delay()), supaya polling Modbus dari
// Orange Pi tidak pernah telat/timeout gara-gara WiFi reconnect.
void updateOTA() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaBegun) beginOtaService();
    otaReady = true;
    ArduinoOTA.handle();
    return;
  }
  otaReady = false;
  static uint32_t lastRetryMs = 0;
  if (millis() - lastRetryMs > 30000) {
    lastRetryMs = millis();
    Serial.println("[OTA] WiFi belum/tidak connect, retry (non-blocking)...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void scanI2C() {
  Serial.println("[I2C-SCAN] Memindai bus I2C...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C-SCAN]   Terdeteksi di 0x%02X\n", addr);
      found++;
      if (addr == I2CAddr::LCD)    lcdPresent = true;
      if (addr == I2CAddr::KEYPAD) keypadPresent = true;
    }
  }
  Serial.printf("[I2C-SCAN] Selesai, %u device (0x20 Keypad, 0x21 LCD, 0x22/0x23 MCP, 0x40 PCA9685)\n", found);
}

// BARU: hot-plug LCD/Keypad DUA ARAH -- lihat penjelasan lengkap di SORTER
void checkLcdKeypadHotplug() {
  static uint32_t lastHotplugCheck = 0;
  if (millis() - lastHotplugCheck < 3000) return;
  lastHotplugCheck = millis();

  Wire.beginTransmission(I2CAddr::LCD);
  bool lcdPing = (Wire.endTransmission() == 0);
  if (!lcdPresent && lcdPing) {
    lcdPresent = true; lcd.init(); lcd.backlight();
    Serial.println("[HOTPLUG] LCD baru terdeteksi -- diinisialisasi live");
  } else if (lcdPresent && !lcdPing) {
    lcdPresent = false;
    Serial.println("[HOTPLUG] !!! LCD TIDAK TERDETEKSI LAGI !!!");
  }

  Wire.beginTransmission(I2CAddr::KEYPAD);
  bool kpPing = (Wire.endTransmission() == 0);
  if (!keypadPresent && kpPing) {
    keypadPresent = true; keypad.begin();
    Serial.println("[HOTPLUG] Keypad baru terdeteksi -- diinisialisasi live");
  } else if (keypadPresent && !kpPing) {
    keypadPresent = false;
    Serial.println("[HOTPLUG] !!! Keypad TIDAK TERDETEKSI LAGI !!!");
    if (menuState != MenuState::NONE) {
      menuState = MenuState::NONE;
      Serial.println("[HOTPLUG] Keluar OTOMATIS dari mode kalibrasi -- keypad hilang, cegah node terjebak");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] PICKER (6DOF Servo) mulai");

  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_FREQ_HZ);
  scanI2C();

  if (lcdPresent) { lcd.init(); lcd.backlight(); lcdPrint(0, 0, "PICKER - Servo"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");
  lcdBootProgress("I2C + LCD/Keypad");

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }
  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  io.pinMode(CH::PCA_OE, OUTPUT); io.write(CH::PCA_OE, HIGH);
  // DIPERBAIKI (bug ditemukan): PICKER tidak punya channel produksi asli Stepper/Motor DC/Relay,
  // jadi channel ini TIDAK PERNAH di-pinMode OUTPUT -- Test Modul (semua kecuali Servo) diam total.
  io.pinMode(CH::DIR_1_MCP, OUTPUT); io.pinMode(CH::DIR_2_MCP, OUTPUT); io.pinMode(CH::DIR_3_MCP, OUTPUT);
  io.pinMode(CH::EN_123, OUTPUT);
  io.pinMode(CH::AIN1, OUTPUT); io.pinMode(CH::AIN2, OUTPUT);
  io.pinMode(CH::BIN1, OUTPUT); io.pinMode(CH::BIN2, OUTPUT);
  io.pinMode(CH::STBY, OUTPUT);
  io.pinMode(CH::RLY1, OUTPUT); io.pinMode(CH::RLY2, OUTPUT);
  // DIPERBAIKI (bug ditemukan): pin native STEP_1/2/3 dipakai testStepperPulse() (Test Modul
  // Stepper) via digitalWrite() langsung -- io.pinMode() di atas cuma berlaku utk channel
  // MCP23017, BUKAN pin native ESP32 ini. Tanpa pinMode(OUTPUT), default boot = INPUT, pulsa
  // STEP tidak pernah keluar elektrik meski DIR/EN (MCP) sudah benar.
  pinMode(GP::STEP_1, OUTPUT); pinMode(GP::STEP_2, OUTPUT); pinMode(GP::STEP_3, OUTPUT);
  Serial.printf("[BOOT] MCP23017: %s\n", ioOk ? "OK" : "GAGAL");
  lcdBootProgress("I/O Expander MCP23017");

  pwm.begin(); pwm.setPWMFreq(ServoCfg::FREQ_HZ);
  loadPosesFromNvs();
  for (uint8_t i = 0; i < ServoCfg::NUM_JOINTS; i++) currentUs[i] = POSES[0].us[i];
  io.write(CH::PCA_OE, LOW);
  Serial.println("[BOOT] PCA9685 + pose dari NVS OK");
  lcdBootProgress("Servo PCA9685 + Pose");

  Serial2.begin(Rs485Cfg::BAUD, SERIAL_8N1, Rs485Cfg::RX_PIN, Rs485Cfg::TX_PIN);
  mb.begin(&Serial2);
  mb.slave(Rs485Cfg::SLAVE_ID);
  mb.addHreg(Reg::STATE, 1); mb.addHreg(Reg::FAULT_CODE, 0);
  mb.addHreg(Reg::CMD, 0); mb.addHreg(Reg::CMD_ARG, 0);
  mb.addHreg(Reg::CMD_SEQ, 0); mb.addHreg(Reg::CMD_ACK_SEQ, 0);
  mb.addHreg(Reg::HEARTBEAT, 0); mb.addHreg(Reg::CURRENT_POSE, 0);
  mb.addHreg(Reg::ACTIVITY_CODE, 0); mb.addHreg(Reg::I2C_ERROR_COUNT, 0);
  mb.addHreg(Reg::LAST_FAULT_CODE, 0); mb.addHreg(Reg::UPTIME_SEC, 0);
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  Serial.printf("[BOOT] Modbus siap, slave ID=%d\n", Rs485Cfg::SLAVE_ID);
  lcdBootProgress("Modbus RS485");

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;

  setupOTA();
  lcdBootProgress("WiFi OTA");

  Serial.println("[BOOT] setup SELESAI -- ketik HELP utk kalibrasi via Serial");
}

void loop() {
  mb.task();
  updateOTA();

  if (keypadPresent) {
    char key = keypad.scan();
    if (key) {
      // DIUBAH: boleh masuk menu dari state APA PUN sekarang (bukan cuma IDLE) -- logic fisik
      // tetap jalan selagi menu aktif (lihat di bawah), konsisten dgn pola SORTER
      if (menuState == MenuState::NONE && key == '*') {
        menuState = MenuState::TOP_SELECT;
        if (lcdPresent) drawTopMenu();
        Serial.println("[CAL] Masuk mode kalibrasi (command eksternal dijeda, logic fisik TETAP jalan)");
      } else if (menuState == MenuState::TOP_SELECT) handleTopMenuKey(key);
      else if (menuState == MenuState::CAL_LIST) handleCalListKey(key);
      else if (menuState == MenuState::JOG_JOINT) handleCalibrationKey(key);
      else if (menuState == MenuState::WAIT_SAVE_SLOT) handleSaveSlotKey(key);
      else if (menuState == MenuState::JOG_OFFSET) handleOffsetKey(key);
      else if (menuState == MenuState::JOG_SPEED) handleSpeedKey(key);
      else if (menuState == MenuState::TEST_IO_CATEGORY) handleTestIoCategoryKey(key);
      else if (menuState == MenuState::TEST_IO_I2CSCAN) handleTestIoI2CScanKey(key);
      else if (menuState == MenuState::TEST_OUTPUT_LIST) handleTestOutputListKey(key);
      else if (menuState == MenuState::TEST_OUTPUT_ITEM) handleTestOutputItemKey(key);
      else if (menuState == MenuState::TEST_INPUT_CATEGORY) handleInputCategoryKey(key);
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
  if (menuState == MenuState::TEST_MOD_SERVO && lcdPresent) {
    static uint32_t lastTestModRefresh3 = 0;
    if (millis() - lastTestModRefresh3 > 150) { lastTestModRefresh3 = millis(); drawTestModServo(); }
  }
  // DIUBAH: logic fisik (safety, trajektori servo, antrian) SEKARANG SELALU JALAN, termasuk
  // saat menu kalibrasi aktif -- konsisten dgn pola SORTER. Hanya command EKSTERNAL yang dijeda.
  handleSafety();
  if (currentState != NodeState::ESTOPPED) {
    updateTrajectory();
    processQueue();
    checkSequenceComplete();
    updateBuzzerBeep();   // BARU -- proses siklus ON-OFF buzzer non-blocking
  }

  if (menuState != MenuState::NONE) return;   // command eksternal dijeda saat menu aktif (§12.6)

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
  }

  if (lcdPresent) {
    static uint32_t lastLcdRefresh = 0;
    if (millis() - lastLcdRefresh > 500) {
      lastLcdRefresh = millis();
      lcdPrint(0, 0, "[AUTO] " + activityText());
      lcdPrint(0, 2, "State:" + String(stateText(currentState)));
      lcdPrint(0, 3, "Tahan* utk kalibrasi");
    }
  }
}