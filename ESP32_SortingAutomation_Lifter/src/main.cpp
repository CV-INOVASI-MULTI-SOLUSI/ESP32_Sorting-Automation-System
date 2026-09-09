// ============================================================
// STOCKER — Lift XYZ + Modbus + Menu Kalibrasi LCD/Keypad
// Y-axis ITU SENDIRI mekanisme pusher. Z-axis 2 motor sinkron via wiring
// paralel STEP/DIR. Menu LCD 3-kategori (sinkron pola SORTER/PICKER).
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

// BARU: objek pwm global, dipakai Test Modul Servo -- SAMA PERSIS pola dgn SORTER/PICKER
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
// BARU: Lapis 3 diagnostik -- sinkron pola SORTER
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
// BARU: Test ke Rak -- dideklarasikan awal karena dipakai di updateHoming() (definisi lebih
// awal dari section menu tempat variabel ini logis berada)
bool testRackSequencePending = false;
uint8_t testRackSequenceTarget = 0;
uint32_t lastRs485Rx = 0;
bool modbusEverUsed = false;

constexpr uint16_t STEP_PULSE_US = 4;
constexpr uint32_t HOMING_TIMEOUT_US = 30UL * 1000000UL;   // 30 detik PER AXIS (bukan kumulatif -- lihat perbaikan di updateHoming())
uint16_t stepIntervalUs = 600;
uint16_t homingStepIntervalUs = 150;
int32_t curPos[3] = {0, 0, 0}, tgtPos[3] = {0, 0, 0};
bool homed[3] = {false, false, false};
uint32_t lastStepMicros[3] = {0, 0, 0};

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

struct RackPos { int32_t x, z; };
RackPos RACK[6];
// BARU: Load Position -- titik tunggal tempat lift "standby" menunggu robot arm
// meletakkan/mengambil package, BEDA dari Home (limit switch, titik nol fisik)
// dan Rack (6 slot penyimpanan akhir). Dikalibrasi manual, tersimpan NVS.
RackPos loadPos = {0, 0};
int32_t pushExtendSteps = 1000;

enum class LiftState { HOMING, IDLE, MOVING, FAULT, ESTOPPED };
LiftState state = LiftState::IDLE;
uint8_t homingAxis = 0;

bool yRetracting = false;
uint32_t yRetractStartMs = 0;
constexpr uint32_t Y_RETRACT_TIMEOUT_MS = 5000;

enum class CycleStage { NONE, MOVING_XZ, PUSHING_Y, RETRACT_Y, RETURNING_XZ, PUSHING_Y_STANDALONE, RETRACT_Y_STANDALONE };
CycleStage cycleStage = CycleStage::NONE;
bool ackPending = false; uint16_t pendingAckSeq = 0;

uint16_t rampMinIntervalUs = 1200;
uint16_t rampSteps = 300;
uint8_t microstepMode = 2;

// --- Menu state (dideklarasikan awal, dipakai onCmdWrite) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, MOVE_AXIS, WAIT_SAVE_SLOT, JOG_SPEED, TEST_RACK_SELECT,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_RELAY, TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET };
MenuState menuState = MenuState::NONE;

void loadRackFromNvs() {
  prefs.begin("stocker_cal", true);
  if (prefs.isKey("rack")) prefs.getBytes("rack", RACK, sizeof(RACK));
  if (prefs.isKey("loadPos")) prefs.getBytes("loadPos", &loadPos, sizeof(loadPos));   // BARU
  if (prefs.isKey("pushExt")) pushExtendSteps = prefs.getInt("pushExt", pushExtendSteps);
  if (prefs.isKey("stepIntv")) stepIntervalUs = prefs.getUShort("stepIntv", stepIntervalUs);
  if (prefs.isKey("homeIntv")) homingStepIntervalUs = prefs.getUShort("homeIntv", homingStepIntervalUs);
  if (prefs.isKey("rampMin")) rampMinIntervalUs = prefs.getUShort("rampMin", rampMinIntervalUs);
  if (prefs.isKey("rampSteps")) rampSteps = prefs.getUShort("rampSteps", rampSteps);
  if (prefs.isKey("mstep")) microstepMode = prefs.getUChar("mstep", microstepMode);
  if (prefs.isKey("buzzOn")) buzzerOnMs = prefs.getUShort("buzzOn", buzzerOnMs);       // BARU
  if (prefs.isKey("buzzOff")) buzzerOffMs = prefs.getUShort("buzzOff", buzzerOffMs);   // BARU
  prefs.end();
}
void saveRackToNvs() { prefs.begin("stocker_cal", false); prefs.putBytes("rack", RACK, sizeof(RACK)); prefs.end(); }
void saveLoadPosToNvs() { prefs.begin("stocker_cal", false); prefs.putBytes("loadPos", &loadPos, sizeof(loadPos)); prefs.end(); }   // BARU
void savePushExtendToNvs() { prefs.begin("stocker_cal", false); prefs.putInt("pushExt", pushExtendSteps); prefs.end(); }
void saveStepIntervalToNvs() { prefs.begin("stocker_cal", false); prefs.putUShort("stepIntv", stepIntervalUs); prefs.end(); }
void saveHomingIntervalToNvs() { prefs.begin("stocker_cal", false); prefs.putUShort("homeIntv", homingStepIntervalUs); prefs.end(); }
void saveRampToNvs() { prefs.begin("stocker_cal", false); prefs.putUShort("rampMin", rampMinIntervalUs); prefs.putUShort("rampSteps", rampSteps); prefs.end(); }
void saveMicrostepToNvs() { prefs.begin("stocker_cal", false); prefs.putUChar("mstep", microstepMode); prefs.end(); }
void saveBuzzerToNvsStocker() { prefs.begin("stocker_cal", false); prefs.putUShort("buzzOn", buzzerOnMs); prefs.putUShort("buzzOff", buzzerOffMs); prefs.end(); }   // BARU
// BARU: reset total -- hapus SEMUA kalibrasi NVS & kembalikan RAM ke default
void resetAllToDefault() {
  prefs.begin("stocker_cal", false);
  prefs.clear();
  prefs.end();
  for (uint8_t i = 0; i < 6; i++) RACK[i] = {0, 0};
  loadPos = {0, 0};   // BARU
  pushExtendSteps = 1000;
  stepIntervalUs = 600; homingStepIntervalUs = 150;
  rampMinIntervalUs = 1200; rampSteps = 300; microstepMode = 2;
  buzzerOnMs = 500; buzzerOffMs = 500;   // BARU
  Serial.println("[RESET] Semua kalibrasi STOCKER dikembalikan ke default & NVS dihapus");
}

uint16_t readRackOccupiedBitmask() {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < 6; i++) if (io.read(CH::RACK_LIM[i]) == LOW) mask |= (1 << i);
  return mask;
}

// DIPERBAIKI (M03/M04 + tambalan gap #3): sebelumnya readRackOccupiedBitmask()
// dan updateRackLeds() dipanggil TERPISAH tiap loop TANPA throttle -- 12 transaksi
// I2C per iterasi (6 baca dobel utk data yang SAMA + 6 tulis), bersaing langsung
// dengan timing STEP. Sekarang digabung jadi 1 fungsi terjadwal (100ms, cukup
// responsif utk indikator visual), 1x baca dipakai utk KEDUA tujuan (bitmask
// Modbus + LED), bukan dibaca 2x terpisah.
uint16_t lastRackBitmask = 0;
void updateRackStatusTask() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();
  uint16_t mask = 0;
  for (uint8_t i = 0; i < 6; i++) {
    // DIHAPUS: io.write(CH::RACK_LED[i], ...) -- fitur RACK_LED tidak dipakai lagi di board universal baru
    if (io.read(CH::RACK_LIM[i]) == LOW) mask |= (1 << i);
  }
  lastRackBitmask = mask;
}

// BARU: indikator universal (sama pola di semua 4 node) --
// LED_OPERATION = heartbeat, berkedip TERUS selama firmware hidup (bukti tidak hang)
// LED_RUN       = solid nyala saat RUNNING, mati saat IDLE
// LED_MANUAL    = nyala selama mode kalibrasi (menuState != NONE) aktif
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

// DIHAPUS: stepPulse() lama (blocking, delayMicroseconds) -- sudah tidak dipakai
// sama sekali, digantikan startStepPulse()/updateStepPulse() non-blocking (M02)

int32_t moveStartPos[3] = {0, 0, 0};
bool axisWasMoving[3] = {false, false, false};

// ============================================================
// DIPERBAIKI (M01/B01 -- temuan paling kritis): DIR sebelumnya ditulis via I2C
// (MCP23017) SETIAP kali step dieksekusi, walau arah TIDAK berubah. Overhead
// I2C 1x digitalWrite() MCP23017 CONFIRMED ~650-700us (source Adafruit_BusIO:
// read-modify-write, 1 baca + 1 tulis register) -- jauh melebihi stepIntervalUs
// manapun (bahkan 600us default), jadi kecepatan REAL selalu dibatasi overhead
// I2C, bukan oleh nilai yang di-set. Sekarang DIR di-cache, cuma ditulis SEKALI
// saat arah benar-benar berubah -- berlaku sama utk X/Y/Z (termasuk homing &
// Y-retract, bukan cuma gerak manual/rack seperti draft awal).
// EN_STEPPERS juga di-cache dgn pola sama (M04 -- indikator/EN jangan ditulis
// ulang kalau state-nya sudah sama).
// ============================================================
bool dirState[3] = {false, false, false};          // arah TERAKHIR yang ditulis ke hardware per axis
bool stepHigh[3] = {false, false, false};          // true = pulsa STEP sedang HIGH, menunggu diturunkan
uint32_t stepPulseStartMicros[3] = {0, 0, 0};      // kapan pulsa HIGH dimulai (non-blocking width)
bool steppersEnabled = false;                       // cache EN_STEPPERS -- hindari I2C write berulang

void setStepperEnabled(bool enable) {
  if (steppersEnabled == enable) return;   // sudah di state itu, tidak perlu I2C lagi
  steppersEnabled = enable;
  io.write(CH::EN_STEPPERS, enable ? LOW : HIGH);   // EN TMC2209 aktif-LOW
}

// DIREVISI: tidak ada pin native lain tersedia di ESP32 untuk DIR -- kembali sepenuhnya ke
// MCP23017 (CH::DIR_1_MCP/DIR_2_MCP/DIR_3_MCP). Cache arah tetap dipertahankan (skip I2C
// sepenuhnya kalau arah tidak berubah) -- ini yang paling berdampak ke performa, bukan
// pilihan native/MCP itu sendiri.
void setAxisDirection(uint8_t axis, bool forward) {
  if (axis >= 3) return;
  // BARU: terapkan invert per-axis SEBELUM dibandingkan/ditulis -- supaya cache (dirState)
  // tetap konsisten dengan arah FISIK sesungguhnya, bukan arah yang "diminta" sebelum inversi
  const bool invert[3] = {AxisInvert::X, AxisInvert::Y, AxisInvert::Z};
  bool actualForward = forward != invert[axis];   // XOR -- balik kalau invert[axis]==true
  if (dirState[axis] == actualForward) return;   // arah FISIK sama -- skip sepenuhnya
  dirState[axis] = actualForward;
  const uint8_t dirPinsMcp[3] = {CH::DIR_1_MCP, CH::DIR_2_MCP, CH::DIR_3_MCP};
  io.write(dirPinsMcp[axis], actualForward ? HIGH : LOW);
  lastStepMicros[axis] = micros();   // settling time -- TMC2209 butuh jeda setelah DIR berubah sebelum STEP pertama valid
}

// DIPERBAIKI (M02): STEP pulse non-blocking -- sebelumnya stepPulse() pakai
// delayMicroseconds(4) BLOCKING. Dampak absolut kecil (4us) dibanding overhead
// I2C di atas, TAPI setelah DIR di-cache, 4us ini jadi proporsi lebih berarti
// dari sisa waktu per-step yang sudah jauh lebih pendek. State machine 2-fase:
// startStepPulse() naikkan pin, updateStepPulse() turunkan setelah STEP_PULSE_US.
bool startStepPulse(uint8_t axis) {
  const uint8_t stepPins[3] = {GP::STEP_X, GP::STEP_Y, GP::STEP_Z};
  if (stepHigh[axis]) return false;   // pulsa sebelumnya belum selesai turun -- jangan tumpuk
  digitalWrite(stepPins[axis], HIGH);
  stepPulseStartMicros[axis] = micros();
  stepHigh[axis] = true;
  return true;
}
void updateStepPulse(uint8_t axis) {
  const uint8_t stepPins[3] = {GP::STEP_X, GP::STEP_Y, GP::STEP_Z};
  if (!stepHigh[axis]) return;
  if ((uint32_t)(micros() - stepPulseStartMicros[axis]) < STEP_PULSE_US) return;
  digitalWrite(stepPins[axis], LOW);
  stepHigh[axis] = false;
}

// DIPERBAIKI (M06/M07): integer arithmetic, bukan float -- hindari overhead
// kalkulasi float per-step yang tidak perlu (ESP32 punya FPU jadi relatif
// cepat, tapi integer tetap lebih murah, dan konsisten dgn computeHomingRampedInterval)
uint16_t computeRampedInterval(uint8_t i, int32_t diff) {
  if (rampSteps == 0) return stepIntervalUs;
  // BARU: pengaman -- kalau rampMin < stepInterval (kombinasi terbalik/tidak valid,
  // belum ada validasi di UI menu), delta negatif bisa wraparound jadi angka besar
  // saat di-cast ke uint32_t, berisiko overflow di kalkulasi reduction. Fallback aman.
  if (rampMinIntervalUs <= stepIntervalUs) return stepIntervalUs;
  uint32_t stepsFromStart = (uint32_t)abs(curPos[i] - moveStartPos[i]);
  uint32_t stepsRemaining = (uint32_t)abs(diff);
  uint32_t r = min(stepsFromStart, stepsRemaining);
  if (r >= rampSteps) return stepIntervalUs;
  uint32_t delta = (uint32_t)(rampMinIntervalUs - stepIntervalUs);
  uint32_t reduction = (delta * r) / rampSteps;
  return (uint16_t)(rampMinIntervalUs - reduction);
}

// BARU (B06): flag utk pindahkan Serial.printf() KELUAR dari updateSteppers() --
// motion-critical function sebaiknya tidak punya side-effect I/O tersembunyi,
// walau dampak performa aktualnya sama (tetap 1x per transisi, bukan per-step)
bool motionJustFinished = false;
uint32_t motionFinishedDurationMs = 0;

// DIPERBAIKI (M01/B01, M02): DIR & EN sekarang di-cache (lihat setAxisDirection/
// setStepperEnabled di atas), STEP pulse non-blocking (startStepPulse/updateStepPulse).
// Ini PERBAIKAN PALING BERDAMPAK dari seluruh audit -- overhead I2C per-step turun
// dari ~650-700us (tiap step) menjadi ~0 (cuma saat arah benar-benar berubah).
void updateSteppers() {
  uint32_t now = micros();
  for (uint8_t axis = 0; axis < 3; axis++) {
    updateStepPulse(axis);         // turunkan pulsa HIGH sebelumnya dulu (non-blocking)
    if (stepHigh[axis]) continue;  // masih menunggu pulsa turun, jangan mulai step baru

    int32_t diff = tgtPos[axis] - curPos[axis];
    if (diff == 0) { axisWasMoving[axis] = false; continue; }

    if (!axisWasMoving[axis]) {
      axisWasMoving[axis] = true;
      moveStartPos[axis] = curPos[axis];
      setStepperEnabled(true);
      setAxisDirection(axis, diff > 0);
      lastStepMicros[axis] = now;
      continue;   // 1 iterasi jeda sebelum step pertama (settling time arah)
    }

    uint16_t interval = computeRampedInterval(axis, diff);
    if ((uint32_t)(now - lastStepMicros[axis]) < interval) continue;

    setAxisDirection(axis, diff > 0);   // no-op kalau arah sudah sama -- TIDAK ada I2C

    if (startStepPulse(axis)) {
      curPos[axis] += (diff > 0) ? 1 : -1;
      lastStepMicros[axis] = now;
    }
  }

  bool anyMoving = false;
  for (uint8_t axis = 0; axis < 3; axis++)
    anyMoving |= (tgtPos[axis] != curPos[axis]) || stepHigh[axis];

  static uint32_t moveTimingStartMs = 0;
  static bool wasAnyMoving = false;
  if (anyMoving && !wasAnyMoving) moveTimingStartMs = millis();
  if (!anyMoving && wasAnyMoving) { motionJustFinished = true; motionFinishedDurationMs = millis() - moveTimingStartMs; }
  wasAnyMoving = anyMoving;
  if (!anyMoving && state == LiftState::MOVING) state = LiftState::IDLE;
}

uint32_t homingStepCount = 0;
bool moveToRackXZ(uint8_t rackIdx);   // forward declaration -- dipakai di updateHoming() (Test ke Rak), didefinisikan di bawah
// BARU (perbaikan M05/B02 + celah yang saya temukan di draft rekomendasi): timer
// PER AXIS (bukan 1 timer kumulatif utk 3 axis) -- kalau kumulatif, axis pertama
// yang lama bisa "menghabiskan" alokasi waktu axis berikutnya secara tidak adil.
uint32_t homingAxisStartMicros[3] = {0, 0, 0};

uint16_t computeHomingRampedInterval() {
  if (rampSteps == 0) return homingStepIntervalUs;
  if (rampMinIntervalUs <= homingStepIntervalUs) return homingStepIntervalUs;   // pengaman kombinasi terbalik
  if (homingStepCount >= rampSteps) return homingStepIntervalUs;
  uint32_t delta = (uint32_t)(rampMinIntervalUs - homingStepIntervalUs);
  uint32_t reduction = (delta * homingStepCount) / rampSteps;
  return (uint16_t)(rampMinIntervalUs - reduction);
}

// BARU: inisialisasi homing terpusat -- dipanggil dari applyCommand() (HOME_ALL)
// dan menu kalibrasi (AutoHome), memastikan timer per-axis & cache DIR/pulsa
// direset konsisten di kedua jalur (sebelumnya duplikasi manual di 2 tempat).
void startHomingInternal() {
  homingAxis = 0;
  homingStepCount = 0;
  uint32_t now = micros();
  for (uint8_t i = 0; i < 3; i++) { homed[i] = false; stepHigh[i] = false; lastStepMicros[i] = now; }
  homingAxisStartMicros[0] = now;
  setStepperEnabled(true);
  setAxisDirection(0, false);   // arah homing selalu menuju limit (LOW), sama seperti sebelumnya
  state = LiftState::HOMING;
}

// DIPERBAIKI (M01/B01, M02, M05/B02): DIR di-cache, STEP pulse non-blocking,
// DAN timeout diukur dari SAAT AXIS INI MULAI (bukan dari step terakhir --
// sebelumnya lastStepMicros terus di-refresh tiap step berhasil, sehingga
// timeout TIDAK PERNAH tercapai selama motor terus menghasilkan step. Kalau
// limit switch rusak/lepas + axis kepanjangan, homing bisa berjalan TANPA
// BATAS WAKTU -- berisiko motor menabrak ujung mekanis).
void updateHoming() {
  const uint8_t limPins[3] = {CH::LIM_X, CH::LIM_Y, CH::LIM_Z};
  if (homingAxis >= 3) {
    state = LiftState::IDLE;
    for (uint8_t i = 0; i < 3; i++) { curPos[i] = 0; tgtPos[i] = 0; }
    if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); }
    Serial.println("[STOCKER] Homing SELESAI");
    // BARU: kalau ini bagian dari "Test ke Rak", lanjutkan otomatis ke rak tujuan
    if (testRackSequencePending) {
      testRackSequencePending = false;
      if (moveToRackXZ(testRackSequenceTarget)) {
        currentState = NodeState::RUNNING_OR_MOVING;
        Serial.printf("[TEST-RACK] Homing selesai, menuju Rak %u\n", testRackSequenceTarget);
      } else {
        Serial.println("[TEST-RACK] Gagal menuju rak setelah homing (cek log FaultCode)");
      }
    }
    return;
  }

  updateStepPulse(homingAxis);
  if (stepHigh[homingAxis]) return;   // tunggu pulsa turun sebelum lanjut

  if (io.read(limPins[homingAxis]) == LOW) {
    homed[homingAxis] = true;
    homingAxis++;
    homingStepCount = 0;
    if (homingAxis < 3) {
      uint32_t now = micros();
      homingAxisStartMicros[homingAxis] = now;   // reset timer utk axis BERIKUTNYA
      lastStepMicros[homingAxis] = now;
      setAxisDirection(homingAxis, false);
    }
    return;
  }

  uint32_t now = micros();
  if ((uint32_t)(now - homingAxisStartMicros[homingAxis]) >= HOMING_TIMEOUT_US) {
    faultCode = (uint16_t)FaultCode::HOMING_FAILED; state = LiftState::FAULT; currentState = NodeState::FAULT;
    setStepperEnabled(false);
    if (ackPending) ackPending = false;
    return;
  }

  uint16_t interval = computeHomingRampedInterval();
  if ((uint32_t)(now - lastStepMicros[homingAxis]) < interval) return;

  setAxisDirection(homingAxis, false);   // no-op kalau arah sudah sama
  if (startStepPulse(homingAxis)) {
    lastStepMicros[homingAxis] = now;
    homingStepCount++;
  }
}

// DIPERBAIKI (tambalan gap #2 -- Y-retract punya bug B01 yang SAMA PERSIS,
// tapi tidak disebutkan eksplisit di draft rekomendasi. DIR_Y sebelumnya
// ditulis via I2C tiap step retract juga): cache DIR + non-blocking pulse.
void startYRetract() {
  yRetracting = true; yRetractStartMs = millis();
  lastStepMicros[1] = micros();
  setStepperEnabled(true);
  setAxisDirection(1, false);
}
void updateYRetract() {
  if (!yRetracting) return;

  updateStepPulse(1);
  if (stepHigh[1]) return;

  if (io.read(CH::LIM_Y) == LOW) {
    yRetracting = false; curPos[1] = 0; tgtPos[1] = 0; homed[1] = true;
    Serial.println("[STOCKER] Y retract selesai");
    return;
  }
  if (millis() - yRetractStartMs > Y_RETRACT_TIMEOUT_MS) {
    yRetracting = false;
    faultCode = (uint16_t)FaultCode::PUSH_STUCK; state = LiftState::FAULT;
    Serial.println("[STOCKER] !!! Y retract GAGAL !!!");
    return;
  }
  uint32_t now = micros();
  if ((uint32_t)(now - lastStepMicros[1]) < stepIntervalUs) return;
  setAxisDirection(1, false);   // no-op kalau arah sudah sama
  if (startStepPulse(1)) lastStepMicros[1] = now;
}

bool moveToRackXZ(uint8_t rackIdx) {
  if (rackIdx >= 6 || !homed[0] || !homed[1] || !homed[2]) {
    faultCode = rackIdx >= 6 ? (uint16_t)FaultCode::RACK_IDX_INVALID : (uint16_t)FaultCode::NOT_HOMED;
    return false;
  }
  tgtPos[0] = RACK[rackIdx].x; tgtPos[2] = RACK[rackIdx].z;
  state = LiftState::MOVING;
  mb.Hreg(Reg::CURRENT_RACK_IDX, rackIdx);
  return true;
}

void handleCycle() {
  switch (cycleStage) {
    case CycleStage::NONE: break;
    case CycleStage::MOVING_XZ:
      if (state == LiftState::IDLE) {
        tgtPos[1] = pushExtendSteps; state = LiftState::MOVING; cycleStage = CycleStage::PUSHING_Y;
        triggerBuzzerBeep();   // BARU -- notifikasi: sudah masuk rack yang benar
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; }
      break;
    case CycleStage::PUSHING_Y:
      if (state == LiftState::IDLE) {
        startYRetract(); cycleStage = CycleStage::RETRACT_Y;
        triggerBuzzerBeep();   // BARU -- notifikasi: selesai dorong package
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; }
      break;
    case CycleStage::RETRACT_Y:
      if (!yRetracting) {
        if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; return; }
        // DIUBAH: setelah RUN_FULL_CYCLE selesai, lift kembali ke LOAD POSITION
        // (bukan Home lagi) -- siap langsung terima package berikutnya dari robot arm
        // tanpa perlu re-homing tiap siklus. Home tetap ada, cuma dipakai referensi kalibrasi.
        tgtPos[0] = loadPos.x; tgtPos[2] = loadPos.z; state = LiftState::MOVING;
        mb.Hreg(Reg::CURRENT_RACK_IDX, 0xFF);
        cycleStage = CycleStage::RETURNING_XZ;
      }
      break;
    case CycleStage::RETURNING_XZ:
      if (state == LiftState::IDLE) {
        cycleStage = CycleStage::NONE;
        if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); }
        triggerBuzzerBeep();   // BARU -- notifikasi: sudah kembali ke Load Position
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; }
      break;
    case CycleStage::PUSHING_Y_STANDALONE:
      if (state == LiftState::IDLE) { startYRetract(); cycleStage = CycleStage::RETRACT_Y_STANDALONE; }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; }
      break;
    case CycleStage::RETRACT_Y_STANDALONE:
      if (!yRetracting) { cycleStage = CycleStage::NONE; if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); } }
      break;
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH dari HIGH ke LOW
    // SENGAJA unconditional (BUKAN lewat setStepperEnabled()/cache) -- ini jalur
    // keselamatan, tidak boleh berisiko ke-skip walau cache software kebetulan
    // tidak sinkron dengan hardware. Fail-safe: selalu tulis ulang paksa.
    io.write(CH::EN_STEPPERS, HIGH);
    steppersEnabled = false;   // tetap sinkronkan cache setelahnya
    currentState = NodeState::ESTOPPED; state = LiftState::ESTOPPED;
    cycleStage = CycleStage::NONE; ackPending = false; yRetracting = false;
    return;
  }
  if (currentState == NodeState::ESTOPPED) {
    setStepperEnabled(true);   // aman dioptimasi -- bukan jalur darurat
    currentState = NodeState::IDLE; state = LiftState::IDLE;
    for (uint8_t i = 0; i < 3; i++) homed[i] = false;
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

// BARU: teks aktivitas spesifik -- cycleStage SUDAH persis memetakan ke tahapan "menyimpan box"
String activityText() {
  if (currentState == NodeState::FAULT) return "FAULT!";
  if (currentState == NodeState::ESTOPPED) return "E-STOP!";
  if (state == LiftState::HOMING) return "Homing Axis " + String(homingAxis);
  switch (cycleStage) {
    case CycleStage::NONE:
      return (state == LiftState::MOVING) ? "Gerak manual" : "Diam";
    case CycleStage::MOVING_XZ:        return "Menuju Rak";
    case CycleStage::PUSHING_Y:        return "Mendorong Box";
    case CycleStage::RETRACT_Y:        return "Tarik Pusher";
    case CycleStage::RETURNING_XZ:     return "Kembali Home";
    case CycleStage::PUSHING_Y_STANDALONE: return "Test Dorong";
    case CycleStage::RETRACT_Y_STANDALONE: return "Test Tarik";
  }
  return "?";
}

// BARU: versi kode numerik dari activityText() -- dikirim ke register Modbus (Lapis 2)
ActivityCode activityCode() {
  if (currentState == NodeState::FAULT) return ActivityCode::FAULT_AKTIF;
  if (currentState == NodeState::ESTOPPED) return ActivityCode::ESTOP_AKTIF;
  if (state == LiftState::HOMING) return ActivityCode::HOMING;
  switch (cycleStage) {
    case CycleStage::NONE: return (state == LiftState::MOVING) ? ActivityCode::BERGERAK_MANUAL : ActivityCode::DIAM;
    case CycleStage::MOVING_XZ: return ActivityCode::MENUJU_RAK;
    case CycleStage::PUSHING_Y: return ActivityCode::MENDORONG_BOX;
    case CycleStage::RETRACT_Y: return ActivityCode::MENARIK_PUSHER;
    case CycleStage::RETURNING_XZ: return ActivityCode::KEMBALI_KE_HOME;
    case CycleStage::PUSHING_Y_STANDALONE: return ActivityCode::TEST_DORONG;
    case CycleStage::RETRACT_Y_STANDALONE: return ActivityCode::TEST_TARIK;
  }
  return ActivityCode::DIAM;
}

// DIPERBAIKI: guard FAULT/ESTOPPED -- opcode gerak sebelumnya TIDAK dicek sama sekali
// (kelas bug sama dgn SORTER `START` & PICKER `RUN_SEQUENCE` yang sudah ditemukan sebelumnya)
bool blockIfFaulted(const char* opName, uint16_t seq) {
  if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    Serial.printf("[CMD] %s ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu\n", opName);
    mb.Hreg(Reg::CMD_ACK_SEQ, seq);
    return true;
  }
  return false;
}

void applyCommand(uint16_t opcode, uint16_t arg, uint16_t seq) {
  switch ((Cmd)opcode) {
    case Cmd::HOME_ALL:
      if (blockIfFaulted("HOME_ALL", seq)) return;
      startHomingInternal();
      currentState = NodeState::RUNNING_OR_MOVING;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::RUN_FULL_CYCLE:
      if (blockIfFaulted("RUN_FULL_CYCLE", seq)) return;
      if (!homed[1] || curPos[1] != 0) { faultCode = (uint16_t)FaultCode::NOT_HOMED; mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      if (!moveToRackXZ((uint8_t)arg)) { mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      currentState = NodeState::RUNNING_OR_MOVING;
      cycleStage = CycleStage::MOVING_XZ;
      triggerBuzzerBeep();   // BARU -- notifikasi: package sudah di lift, cycle mulai
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::MOVE_TO_RACK:
      if (blockIfFaulted("MOVE_TO_RACK", seq)) return;
      if (!moveToRackXZ((uint8_t)arg)) { mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      currentState = NodeState::RUNNING_OR_MOVING;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::PUSH_BOX:
      if (blockIfFaulted("PUSH_BOX", seq)) return;
      tgtPos[1] = curPos[1] + pushExtendSteps;
      state = LiftState::MOVING;
      currentState = NodeState::RUNNING_OR_MOVING;
      cycleStage = CycleStage::PUSHING_Y_STANDALONE;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::GOTO_LOAD_POSITION:   // BARU -- manual, menuju titik standby terima package
      if (blockIfFaulted("GOTO_LOAD_POSITION", seq)) return;
      if (!homed[0] || !homed[2]) { faultCode = (uint16_t)FaultCode::NOT_HOMED; mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      tgtPos[0] = loadPos.x; tgtPos[2] = loadPos.z;
      state = LiftState::MOVING;
      currentState = NodeState::RUNNING_OR_MOVING;
      mb.Hreg(Reg::CURRENT_RACK_IDX, 0xFF);
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb sebelum di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE; state = LiftState::IDLE;
      break;
    default: break;
  }
  mb.Hreg(Reg::CMD_ACK_SEQ, seq);
}

uint16_t onCmdWrite(TRegister* reg, uint16_t val) {
  if (menuState != MenuState::NONE) {
    Serial.println("[STOCKER] Command Modbus diabaikan -- mode kalibrasi aktif (§12.6)");
    return val;
  }
  lastRs485Rx = millis(); modbusEverUsed = true;
  uint16_t opcode = val;
  uint16_t arg = mb.Hreg(Reg::CMD_ARG);
  uint16_t seq = mb.Hreg(Reg::CMD_SEQ);
  uint16_t lastAcked = mb.Hreg(Reg::CMD_ACK_SEQ);
  if (seq == lastAcked) { Serial.printf("[STOCKER] CMD seq=%u sudah diproses -- diabaikan\n", seq); return val; }
  Serial.printf("[STOCKER] CMD (Modbus) opcode=%u arg=%u seq=%u\n", opcode, arg, seq);
  applyCommand(opcode, arg, seq);
  return val;
}

// ============================================================
// MENU LCD+Keypad -- 3 KATEGORI: Setting Kalibrasi / Test I/O / Test Command
// ============================================================
uint8_t selAxis = 0;
uint8_t jogStepIdx = 0;
constexpr int32_t JOG_STEPS[7] = {5, 10, 50, 200, 1000, 1500, 2000};
uint8_t selSpeedParam = 1;

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
void drawTopMenuStocker() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
constexpr uint8_t CAL_COUNT = 8;
const char* CAL_LABELS[CAL_COUNT] = { "AutoHome (wajib dulu)", "Jog Posisi (X/Y/Z)", "Test ke Rak",
                                        "Simpan ke Slot Rak", "Simpan Load Position", "Simpan Jarak Dorong", "Kecepatan", "Reset ke Default" };
uint8_t calCursor = 0;
void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

// BARU: Test ke Rak -- selalu HOME dulu, baru menuju rak (sesuai permintaan: "test rack 1
// selalu urutannya ke home dulu baru ke tempat rack"). Homing + pindah-ke-rak digabung
// jadi 1 urutan otomatis, bukan 2 langkah manual terpisah.
constexpr uint8_t RACK_COUNT = 6;
const char* RACK_LABELS[RACK_COUNT] = { "Rak 0", "Rak 1", "Rak 2", "Rak 3", "Rak 4", "Rak 5" };
uint8_t testRackCursor = 0;

void drawTestRackSelect() { drawListMenu("TEST KE RAK", RACK_LABELS, RACK_COUNT, testRackCursor); }
void handleTestRackSelectKey(char key) {
  if (key == 'A') { testRackCursor = (testRackCursor == 0) ? RACK_COUNT - 1 : testRackCursor - 1; drawTestRackSelect(); }
  else if (key == 'B') { testRackCursor = (testRackCursor + 1) % RACK_COUNT; drawTestRackSelect(); }
  else if (key == 'C') {
    if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
      lcdPrint(0, 3, "Tak bisa:FAULT/ESTOP");
    } else {
      testRackSequenceTarget = testRackCursor;
      testRackSequencePending = true;
      startHomingInternal();
      currentState = NodeState::RUNNING_OR_MOVING;
      lcdPrint(0, 3, "Homing, lalu ke rak");
      Serial.printf("[TEST-RACK] Home dulu, lalu menuju Rak %u\n", testRackSequenceTarget);
    }
  }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); }
}

void drawConfirmReset() {
  lcd.clear();
  lcdPrint(0, 0, "RESET KE DEFAULT?");
  lcdPrint(0, 1, "Rak & kecepatan akan");
  lcdPrint(0, 2, "HILANG, TAK BS BATAL");
  lcdPrint(0, 3, "C=YA,RESET D=batal");
}
void handleConfirmResetKey(char key) {
  if (key == 'C') { resetAllToDefault(); lcdPrint(0, 3, "SUDAH DIRESET!      "); menuState = MenuState::CAL_LIST; }
  else if (key == 'D') { Serial.println("[CAL] Reset dibatalkan"); menuState = MenuState::CAL_LIST; drawCalList(); }
}

void drawMoveAxisMenu() {
  const char axisName[3] = {'X', 'Y', 'Z'};
  lcdPrint(0, 0, "MOVE " + String(axisName[selAxis]) + String(selAxis == 1 ? " (pusher)" : ""));
  lcdPrint(0, 1, "Pos:" + String(curPos[selAxis]));
  lcdPrint(0, 2, "1X 2Y 3Z tahan A/B");
  lcdPrint(0, 3, "D=kembali");
}
void handleMoveAxisKey(char key) {
  if (key == '1') selAxis = 0;
  else if (key == '2') selAxis = 1;
  else if (key == '3') selAxis = 2;
  else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 7;
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawMoveAxisMenu();
}

void drawSpeedMenuStocker() {
  lcdPrint(0, 0, "KECEPATAN STEP");
  String line1;
  switch (selSpeedParam) {
    case 1: line1 = "1:Jelajah=" + String(stepIntervalUs) + "us"; break;
    case 2: line1 = "2:Homing=" + String(homingStepIntervalUs) + "us"; break;
    case 3: line1 = "3:RampMin=" + String(rampMinIntervalUs) + "us"; break;
    case 4: line1 = "4:RampSteps=" + String(rampSteps); break;
    case 5: line1 = "5:Microstep=1/" + String(microstepMode) + " (jumper MS1/MS2 TMC2209)"; break;
    case 6: line1 = "6:BuzzOn=" + String(buzzerOnMs) + "ms"; break;
    case 7: line1 = "7:BuzzOff=" + String(buzzerOffMs) + "ms"; break;
  }
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, "1-7=pilih A+B-C:step");
  lcdPrint(0, 3, "#=SIMPAN D=kembali");
}
void handleSpeedKeyStocker(char key) {
  int16_t step = JOG_STEPS[jogStepIdx];
  // DIUBAH (migrasi A4988->TMC2209): tabel microstep TMC2209 pin-only BEDA dari A4988 --
  // cuma 4 kombinasi lewat MS1/MS2 (bukan 5 lewat MS1/MS2/MS3), TIDAK ADA full-step (1/1).
  // Tabel resmi TMC2209 (MS2,MS1): 00=1/8, 01=1/2, 10=1/4, 11=1/16
  constexpr uint8_t MSTEP_VALUES[4] = {2, 4, 8, 16};   // urut kasar->halus, logis utk tombol A(naik)/B(turun)
  if (key >= '1' && key <= '7') { selSpeedParam = key - '0'; }
  else if (key == 'A' || key == 'B') {
    if (selSpeedParam == 5) {
      int8_t idx = 0;
      for (uint8_t i = 0; i < 4; i++) if (MSTEP_VALUES[i] == microstepMode) idx = i;
      idx = constrain(idx + (key == 'A' ? 1 : -1), 0, 3);
      microstepMode = MSTEP_VALUES[idx];
    } else {
      int delta = (key == 'A') ? step : -step;
      switch (selSpeedParam) {
        case 1: stepIntervalUs = (uint16_t)constrain((int)stepIntervalUs + delta, 20, 5000); break;
        case 2: homingStepIntervalUs = (uint16_t)constrain((int)homingStepIntervalUs + delta, 20, 5000); break;
        case 3: rampMinIntervalUs = (uint16_t)constrain((int)rampMinIntervalUs + delta, 20, 5000); break;
        case 4: rampSteps = (uint16_t)constrain((int)rampSteps + delta, 0, 5000); break;
        case 6: buzzerOnMs = (uint16_t)constrain((int)buzzerOnMs + delta * 10, 50, 5000); break;
        case 7: buzzerOffMs = (uint16_t)constrain((int)buzzerOffMs + delta * 10, 50, 5000); break;
      }
    }
  }
  else if (key == 'C') { jogStepIdx = (jogStepIdx + 1) % 7; }
  else if (key == '#') {
    saveStepIntervalToNvs(); saveHomingIntervalToNvs(); saveRampToNvs(); saveMicrostepToNvs(); saveBuzzerToNvsStocker();
    lcdPrint(0, 3, "TERSIMPAN ke NVS!");
    Serial.println("[CAL] Kecepatan step + microstep + buzzer disimpan ke NVS");
    return;
  }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawSpeedMenuStocker();
}

void handleSaveSlotKeyStocker(char key) {
  if (key >= '0' && key <= '5') {
    uint8_t slot = key - '0';
    RACK[slot] = {curPos[0], curPos[2]};
    saveRackToNvs();
    lcdPrint(0, 3, "Tersimpan slot " + String(slot));
    Serial.printf("[CAL] X=%ld Z=%ld -> RACK[%u]\n", (long)curPos[0], (long)curPos[2], slot);
    menuState = MenuState::CAL_LIST;
  } else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); }
}

void handleCalListKey(char key) {
  if (key == 'A') { calCursor = (calCursor == 0) ? CAL_COUNT - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % CAL_COUNT; drawCalList(); }
  else if (key == 'C') {
    switch (calCursor) {
      case 0:   // AutoHome
        if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
          lcdPrint(0, 3, "Tak bisa:FAULT/ESTOP");
        } else {
          startHomingInternal();
          currentState = NodeState::RUNNING_OR_MOVING;
          lcdPrint(0, 3, "Homing berjalan...");
          Serial.println("[CAL] AUTO HOME dari menu dimulai");
        }
        break;
      case 1:   // Jog Posisi
        if (!homed[0] || !homed[1] || !homed[2]) { lcdPrint(0, 3, "Home dulu! (opsi a)"); }
        else { menuState = MenuState::MOVE_AXIS; lcd.clear(); drawMoveAxisMenu(); }
        break;
      case 2:   // BARU: Test ke Rak -- selalu home dulu, baru menuju rak
        menuState = MenuState::TEST_RACK_SELECT; testRackCursor = 0; lcd.clear(); drawTestRackSelect();
        break;
      case 3: menuState = MenuState::WAIT_SAVE_SLOT; lcdPrint(0, 3, "Simpan(X,Z)slot?0-5"); break;
      case 4:   // BARU: Simpan Load Position -- simpan X,Z SAAT INI, cuma 1 titik (bukan slot 0-5)
        loadPos = {curPos[0], curPos[2]};
        saveLoadPosToNvs();
        lcdPrint(0, 3, "LoadPos disimpan!");
        Serial.printf("[CAL] X=%ld Z=%ld -> loadPos\n", (long)curPos[0], (long)curPos[2]);
        break;
      case 5:
        if (curPos[1] <= 0) { lcdPrint(0, 3, "Y harus>0(jog dulu)"); }
        else { pushExtendSteps = curPos[1]; savePushExtendToNvs(); lcdPrint(0, 3, "PushExtend disimpan!"); Serial.printf("[CAL] pushExtendSteps=%ld\n", (long)pushExtendSteps); }
        break;
      case 6: menuState = MenuState::JOG_SPEED; lcd.clear(); drawSpeedMenuStocker(); break;
      case 7: menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuStocker(); }
}

// --- LEVEL 1b: TEST I/O -- direstruktur jadi 4 kategori (I2C Scan / Test Output / Test
// Input / Test RS485). DIR_X/DIR_Y/DIR_Z/EN_STEPPERS DIHAPUS dari daftar test -- sekarang
// di-cache oleh setAxisDirection()/setStepperEnabled(), toggle manual di sini tidak lagi
// merefleksikan perilaku motor sesungguhnya (tidak berpengaruh, sesuai catatan Anda).
struct IOTestItem { const char* label; uint8_t ch; bool autoControlled; };
void drawTestIoCategory();   // forward declaration -- dipakai di banyak handler 'D' sebelum definisinya di bawah

// --- Kategori: Test Output (semua output, penamaan disederhanakan) ---
constexpr uint8_t OUTPUT_TEST_COUNT = 5;
IOTestItem OUTPUT_TEST_ITEMS[OUTPUT_TEST_COUNT] = {
  {"OPR",    CH::LED_OPERATION, true},
  {"RUN",    CH::LED_RUN,       true},
  {"MANUAL", CH::LED_MANUAL,    true},
  {"FAULT",  CH::LED_FAULT,     false},
  {"BUZZER", CH::BUZZER,        false},
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
  constexpr uint32_t TOGGLE_MIN_INTERVAL_MS = 300;   // jeda beban induktif (temuan sesi EMI)
  if (key == 'C') {
    if (millis() - lastToggleMs < TOGGLE_MIN_INTERVAL_MS) return;
    lastToggleMs = millis();
    bool cur = io.read(item.ch);
    io.write(item.ch, !cur);
    Serial.printf("[TEST-OUT] CH%u (%s) di-toggle -> %s\n", item.ch, item.label, !cur ? "HIGH" : "LOW");
  }
  drawTestOutputItem();
}

// --- Kategori: Test Input (semua limit switch/sensor, IN1..IN10, baca saja) ---
// DIUBAH: sekarang tampil LANGSUNG di layar list (semua nilai sekaligus, format "INx = 0/1"),
// TIDAK perlu masuk ke item satu-satu -- cocok utk uji beberapa sensor bersamaan. 4 baris LCD
// dipakai semua utk data (bukan cuma 3 + judul), scroll dgn A/B, update HANYA saat nilai berubah.
constexpr uint8_t INPUT_TEST_COUNT = 10;
IOTestItem INPUT_TEST_ITEMS[INPUT_TEST_COUNT] = {
  {"IN1",  CH::ESTOP,       false},
  {"IN2",  CH::LIM_X,       false},
  {"IN3",  CH::LIM_Y,       false},
  {"IN4",  CH::LIM_Z,       false},
  {"IN5",  CH::RACK_LIM[0], false},
  {"IN6",  CH::RACK_LIM[1], false},
  {"IN7",  CH::RACK_LIM[2], false},
  {"IN8",  CH::RACK_LIM[3], false},
  {"IN9",  CH::RACK_LIM[4], false},
  {"IN10", CH::RACK_LIM[5], false},
};
uint8_t inputTestScrollTop = 0;               // indeks item PALING ATAS yang sedang tampil (viewport 4 baris)
bool testInputLiveLastVal[INPUT_TEST_COUNT];  // nilai TERAKHIR yang ditampilkan per item -- cegah kedip
uint8_t testInputLastScrollTop = 255;         // beda dari nilai valid manapun -- paksa redraw penuh pertama kali

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
  else if (key == 'B') { if (inputTestScrollTop < INPUT_TEST_COUNT - 4) { inputTestScrollTop++; drawTestInputList(); } }
  else if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); }
  // 'C' sengaja tidak melakukan apa-apa -- input baca-saja, tidak ada item terpisah lagi
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
}
void handleTestIoI2CScanKey(char key) {
  if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); return; }
  // tombol lain: diam saja (hasil scan statis sampai keluar)
}

// --- Kategori: Test RS485 (BARU) -- diagnostik komunikasi Modbus RTU ---
void drawTestRs485() {
  lcd.clear();
  lcdPrint(0, 0, "TEST RS485");
  lcdPrint(0, 1, "Slave:" + String(Rs485Cfg::SLAVE_ID) + " Baud:" + String(Rs485Cfg::BAUD));
  if (modbusEverUsed) lcdPrint(0, 2, "RX terakhir:" + String((millis() - lastRs485Rx) / 1000) + "s lalu");
  else lcdPrint(0, 2, "Belum ada data masuk");
  lcdPrint(0, 3, "D=kembali");
  Serial.printf("[TEST-RS485] slaveID=%d baud=%lu pernahTerimaData=%d sejakRxMs=%lu heartbeatSec=%lu\n",
                Rs485Cfg::SLAVE_ID, (unsigned long)Rs485Cfg::BAUD, modbusEverUsed,
                modbusEverUsed ? (unsigned long)(millis() - lastRs485Rx) : 0UL, (unsigned long)(millis() / 1000));
}
void handleTestRs485Key(char key) {
  if (key == 'D') { menuState = MenuState::TEST_IO_CATEGORY; drawTestIoCategory(); return; }
  drawTestRs485();   // refresh manual (tombol lain)
}

// --- Selector kategori (LEVEL 1b utama) ---
// --- BARU: Test Modul -- sub-menu pilihan JENIS modul, SAMA di semua 4 node (board universal --
// channel/pin sudah didefinisikan sama persis terlepas modul itu benar2 terpasang fisik atau tidak
// di board node ini). Stepper pakai infrastruktur startJog() yang sudah ada (non-blocking).
// Motor DC/Relay/Servo murni untuk keperluan test manual -- STOCKER secara role asli tidak
// pakai channel2 itu, tapi channel-nya TETAP ada di config.h karena board universal.
void startJog(uint8_t axis, int32_t delta);   // forward declaration -- didefinisikan di bawah

// --- Servo (Test Modul) -- DIUBAH ke library resmi Adafruit_PWMServoDriver, SAMA PERSIS
// pola dengan SORTER/PICKER (menggantikan driver Wire mentah yang berpotensi bug tersembunyi).
void pcaSetServoUs(uint8_t channel, uint16_t us) {
  us = constrain(us, (uint16_t)500, (uint16_t)2500);
  uint16_t duty = (uint32_t)us * 4096 / 20000;
  pwm.setPWM(channel, 0, duty);
}

// --- Sub-menu pilihan jenis modul ---
constexpr uint8_t MODULE_TYPE_COUNT = 4;
const char* MODULE_TYPE_LABELS[MODULE_TYPE_COUNT] = { "Stepper", "Motor DC", "Relay", "Servo" };
uint8_t moduleTypeCursor = 0;
void drawModuleTypeSelect() { drawListMenu("TEST MODUL", MODULE_TYPE_LABELS, MODULE_TYPE_COUNT, moduleTypeCursor); }
void handleModuleTypeKey(char key);   // forward declaration -- dipakai di handleTestIoCategoryKey() sebelum definisi

// --- Modul: Stepper (STEP_1/2/3 + DIR_1/2/3_MCP + EN_123) -- STOCKER pakai startJog() non-blocking ---
uint8_t testModAxis = 0;
bool testModFirstDraw = true;
String testModLine1 = "", testModLine2 = "";
void drawTestModStepper() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: STEPPER");
    lcdPrint(0, 3, "C=ax A/B=jog D=kmb");
    testModFirstDraw = false; testModLine1 = "\x01"; testModLine2 = "\x01";
  }
  String line1 = "X:" + String(homed[0] ? "Y" : "N") + " Y:" + String(homed[1] ? "Y" : "N")
               + " Z:" + String(homed[2] ? "Y" : "N") + " EN:" + String(steppersEnabled ? "ON" : "OFF");
  if (line1 != testModLine1) { testModLine1 = line1; lcdPrint(0, 1, line1 + "   "); }
  const char* axisName[3] = {"X", "Y", "Z"};
  String line2 = "Axis:" + String(axisName[testModAxis]) + " Pos:" + String(curPos[testModAxis]);
  if (line2 != testModLine2) { testModLine2 = line2; lcdPrint(0, 2, line2 + "   "); }
}
void handleTestModStepperKey(char key) {
  constexpr int32_t TEST_JOG_DELTA = 200;
  if (key == 'C') { testModAxis = (testModAxis + 1) % 3; }
  else if (key == 'A') { startJog(testModAxis, TEST_JOG_DELTA); }
  else if (key == 'B') { startJog(testModAxis, -TEST_JOG_DELTA); }
  else if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  drawTestModStepper();
}

// --- Modul: Motor DC (AIN1/AIN2/BIN1/BIN2/STBY) -- ON/OFF sederhana, non-blocking ---
uint8_t testModMotorAState = 0, testModMotorBState = 0;   // 0=stop,1=maju,2=mundur
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
    testModSetMotor(true, 0); testModSetMotor(false, 0);   // stop semua sebelum keluar -- safety
    menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return;
  }
  drawTestModMotorDC();
}

// --- Modul: Relay (RLY1/RLY2) -- toggle, rate-limit 300ms (beban induktif) ---
uint8_t testModRelaySel = 0;   // 0=RLY1, 1=RLY2
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
    if (millis() - lastToggleMs < 300) return;   // rate-limit -- beban induktif, sama alasan Test Output
    lastToggleMs = millis();
    io.write(ch, !io.read(ch));
  } else if (key == 'D') { menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return; }
  drawTestModRelay();
}

// --- Modul: Servo (PCA9685) -- cek deteksi I2C dulu, baru izinkan gerak ---
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
    else if (key == 'A') { pcaSetServoUs(testModServoCh, 1700); }   // nudge kanan dari center
    else if (key == 'B') { pcaSetServoUs(testModServoCh, 1300); }   // nudge kiri dari center
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
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuStocker(); }
}

// DIHAPUS: drawSystemInfoFromMenu() -- System Info bukan lagi bagian dari 4 kategori
// Test I/O baru. Infonya (FW/uptime/heap/i2cErr/lastFault) tetap tersedia via Serial STATUS.

// --- LEVEL 1c: TEST COMMAND ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 6;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"HOME_ALL",             Cmd::HOME_ALL,       0},
  {"RUN_FULL_CYCLE(rak0)", Cmd::RUN_FULL_CYCLE, 0},
  {"MOVE_TO_RACK(rak0)",   Cmd::MOVE_TO_RACK,   0},
  {"PUSH_BOX",             Cmd::PUSH_BOX,       0},
  {"GOTO_LOAD_POSITION",   Cmd::GOTO_LOAD_POSITION, 0},
  {"RESET_FAULT",          Cmd::RESET_FAULT,    0},
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
    applyCommand((uint16_t)item.opcode, item.testArg, 0);
    lcdPrint(0, 3, "Terkirim, amati aksi");
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuStocker(); }
}

void handleTopMenuKeyStocker(char key) {
  if (key == 'A') { topCursor = (topCursor == 0) ? TOP_COUNT - 1 : topCursor - 1; drawTopMenuStocker(); }
  else if (key == 'B') { topCursor = (topCursor + 1) % TOP_COUNT; drawTopMenuStocker(); }
  else if (key == 'C') {
    switch (topCursor) {
      case 0: menuState = MenuState::CAL_LIST; calCursor = 0; drawCalList(); break;
      case 1: menuState = MenuState::TEST_IO_CATEGORY; testIoCatCursor = 0; drawTestIoCategory(); break;
      case 2: menuState = MenuState::TEST_CMD_LIST; cmdTestCursor = 0; drawTestCmdList(); break;
    }
  } else if (key == 'D') { menuState = MenuState::NONE; lcd.clear(); Serial.println("[CAL] Keluar mode kalibrasi"); }
}

// DIPERBAIKI: doAxisStep() lama BLOCKING, diganti startJog() non-blocking
void startJog(uint8_t axis, int32_t delta) {
  setStepperEnabled(true);
  tgtPos[axis] = curPos[axis] + delta;
  state = LiftState::MOVING;
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  int sp1 = line.indexOf(' ');
  String cmd = (sp1 == -1) ? line : line.substring(0, sp1);
  cmd.toUpperCase();

  if (cmd == "HOME") { applyCommand((uint16_t)Cmd::HOME_ALL, 0, 0); Serial.println("[SERIAL] HOME_ALL dimulai"); }
  else if (cmd == "JOG") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t axis = line.substring(sp1 + 1, sp2).toInt();
    int32_t steps = constrain(line.substring(sp2 + 1).toInt(), -20000, 20000);
    if (axis < 3) { startJog(axis, steps); Serial.printf("[JOG] Axis %u mulai bergerak %ld step\n", axis, (long)steps); }
    else Serial.println("[JOG] Axis harus 0(X)/1(Y=pusher)/2(Z)");
  }
  else if (cmd == "SETRACK") {
    uint8_t slot = line.substring(sp1 + 1).toInt();
    if (slot < 6) { RACK[slot] = {curPos[0], curPos[2]}; saveRackToNvs(); Serial.printf("[SETRACK] X=%ld Z=%ld -> RACK[%u]\n", (long)curPos[0], (long)curPos[2], slot); }
    else Serial.println("[SETRACK] Slot harus 0-5");
  }
  else if (cmd == "SETPUSH") {
    if (curPos[1] > 0) { pushExtendSteps = curPos[1]; savePushExtendToNvs(); Serial.printf("[SETPUSH] pushExtendSteps = %ld\n", (long)pushExtendSteps); }
    else Serial.println("[SETPUSH] Y harus >0 dulu (JOG 1 <steps>)");
  }
  else if (cmd == "MOVE") {
    uint8_t slot = line.substring(sp1 + 1).toInt();
    if (moveToRackXZ(slot)) { currentState = NodeState::RUNNING_OR_MOVING; Serial.printf("[MOVE] Menuju RACK[%u]\n", slot); }
    else Serial.println("[MOVE] Gagal -- cek homing/slot");
  }
  else if (cmd == "PUSH") { applyCommand((uint16_t)Cmd::PUSH_BOX, 0, 0); Serial.println("[PUSH] Extend+retract Y dimulai"); }
  else if (cmd == "LOADPOS") { applyCommand((uint16_t)Cmd::GOTO_LOAD_POSITION, 0, 0); Serial.println("[LOADPOS] Menuju Load Position dimulai"); }
  else if (cmd == "FULLCYCLE") {
    uint8_t slot = line.substring(sp1 + 1).toInt();
    applyCommand((uint16_t)Cmd::RUN_FULL_CYCLE, slot, 0);
    Serial.printf("[FULLCYCLE] RUN_FULL_CYCLE ke RACK[%u] dimulai\n", slot);
  }
  else if (cmd == "STEPSPEED") { stepIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 20, 5000); saveStepIntervalToNvs(); Serial.printf("[STEPSPEED] %u\n", stepIntervalUs); }
  else if (cmd == "HOMESPEED") { homingStepIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 20, 5000); saveHomingIntervalToNvs(); Serial.printf("[HOMESPEED] %u\n", homingStepIntervalUs); }
  else if (cmd == "RAMPMIN") { rampMinIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 20, 5000); saveRampToNvs(); Serial.printf("[RAMPMIN] %u\n", rampMinIntervalUs); }
  else if (cmd == "RAMPSTEPS") { rampSteps = constrain((int)line.substring(sp1 + 1).toInt(), 0, 5000); saveRampToNvs(); Serial.printf("[RAMPSTEPS] %u\n", rampSteps); }
  else if (cmd == "MICROSTEP") {
    uint8_t val = line.substring(sp1 + 1).toInt();
    // DIUBAH (migrasi TMC2209): 1/1 (full-step) TIDAK TERSEDIA lagi via pin MS1/MS2 -- beda dari A4988
    if (val == 2 || val == 4 || val == 8 || val == 16) { microstepMode = val; saveMicrostepToNvs(); Serial.printf("[MICROSTEP] Dicatat: 1/%u -- set jumper MS1/MS2 TMC2209 sesuai tabel (lihat HELP)\n", microstepMode); }
    else Serial.println("[MICROSTEP] Nilai harus 2, 4, 8, atau 16 (TMC2209 pin-mode, TIDAK ada 1=full-step)");
  }
  else if (cmd == "RESET") { resetAllToDefault(); }
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s fault=%u ESTOP=%d homed=%d,%d,%d pos=(X:%ld,Y:%ld,Z:%ld) pushExt=%ld rackIdx=%u bitmask=%u stepIntv=%u homeIntv=%u rampMin=%u rampSteps=%u microstep=1/%u\n",
                  stateText(currentState), faultCode, io.read(CH::ESTOP), homed[0], homed[1], homed[2],
                  (long)curPos[0], (long)curPos[1], (long)curPos[2], (long)pushExtendSteps,
                  mb.Hreg(Reg::CURRENT_RACK_IDX), readRackOccupiedBitmask(), stepIntervalUs, homingStepIntervalUs,
                  rampMinIntervalUs, rampSteps, microstepMode);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000));
    Serial.printf("[STATUS] FW=%s build=%s freeHeap=%u\n", FW_VERSION, FW_BUILD, ESP.getFreeHeap());
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] HOME | JOG <0=X,1=Y-pusher,2=Z> <steps> | SETRACK <0-5> | SETPUSH");
    Serial.println("[HELP] MOVE <0-5> | PUSH | FULLCYCLE <0-5> | STEPSPEED <us> | HOMESPEED <us>");
    Serial.println("[HELP] RAMPMIN <us> | RAMPSTEPS <n> | MICROSTEP <2|4|8|16 -- TMC2209, cek tabel MS1/MS2> | RESET | STATUS | HELP");
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
  Serial.println("\n[BOOT] STOCKER mulai (Y-axis = pusher)");

  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_FREQ_HZ);
  pwm.begin(); pwm.setPWMFreq(50);   // BARU -- utk Test Modul Servo (library resmi, sinkron SORTER/PICKER)
  scanI2CAndDetectOptional();

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }

  // DITAMBAHKAN (akar masalah ditemukan!): pin Output-Enable PCA9685 (aktif-LOW) TIDAK PERNAH
  // ditarik LOW sebelumnya -- ini penyebab sebenarnya servo diam meski I2C berhasil sempurna.
  // WAJIB setelah io.begin() -- io.pinMode()/io.write() butuh IOBank sudah siap dulu.
  io.pinMode(CH::OE_PCA, OUTPUT);
  io.write(CH::OE_PCA, LOW);

  if (lcdPresent) { lcd.init(); lcd.backlight(); lcdPrint(0, 0, "STOCKER"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");

  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  // DIREVISI: DIR selalu via MCP23017 -- tidak ada pin native lain tersedia di ESP32
  io.pinMode(CH::DIR_1_MCP, OUTPUT); io.pinMode(CH::DIR_2_MCP, OUTPUT); io.pinMode(CH::DIR_3_MCP, OUTPUT);
  Serial.println("[BOOT] DIR via MCP23017");
  io.pinMode(CH::EN_STEPPERS, OUTPUT); io.write(CH::EN_STEPPERS, HIGH);   // unconditional -- init boot, jangan lewat cache
  // DIPERBAIKI (bug ditemukan): STOCKER tidak punya channel produksi asli Motor DC/Relay,
  // jadi channel ini TIDAK PERNAH di-pinMode OUTPUT -- Test Modul Motor DC/Relay diam total.
  io.pinMode(CH::AIN1, OUTPUT); io.pinMode(CH::AIN2, OUTPUT);
  io.pinMode(CH::BIN1, OUTPUT); io.pinMode(CH::BIN2, OUTPUT);
  io.pinMode(CH::STBY, OUTPUT);
  io.pinMode(CH::RLY1, OUTPUT); io.pinMode(CH::RLY2, OUTPUT);
  steppersEnabled = false;   // sinkronkan cache
  io.pinMode(CH::LIM_X, INPUT_PULLUP); io.pinMode(CH::LIM_Y, INPUT_PULLUP); io.pinMode(CH::LIM_Z, INPUT_PULLUP);
  for (uint8_t i = 0; i < 6; i++) io.pinMode(CH::RACK_LIM[i], INPUT_PULLUP);
  // DIHAPUS: pinMode RACK_LED -- fitur tidak dipakai lagi di board universal baru
  Serial.printf("[BOOT] MCP23017: %s, pinMode selesai\n", ioOk ? "OK" : "GAGAL");

  pinMode(GP::STEP_X, OUTPUT); pinMode(GP::STEP_Y, OUTPUT); pinMode(GP::STEP_Z, OUTPUT);
  Serial.println("[BOOT] pinMode STEP_X/Y/Z (native) OK");

  loadRackFromNvs();
  io.write(CH::EN_STEPPERS, LOW);   // unconditional -- init boot, jangan lewat cache
  steppersEnabled = true;   // sinkronkan cache
  Serial.println("[BOOT] Kalibrasi dimuat dari NVS");

  Serial2.begin(Rs485Cfg::BAUD, SERIAL_8N1, Rs485Cfg::RX_PIN, Rs485Cfg::TX_PIN);
  mb.begin(&Serial2);
  mb.slave(Rs485Cfg::SLAVE_ID);
  mb.addHreg(Reg::STATE, (uint16_t)currentState);
  mb.addHreg(Reg::FAULT_CODE, faultCode);
  mb.addHreg(Reg::CMD, 0); mb.addHreg(Reg::CMD_ARG, 0);
  mb.addHreg(Reg::CMD_SEQ, 0); mb.addHreg(Reg::CMD_ACK_SEQ, 0);
  mb.addHreg(Reg::HEARTBEAT, 0);
  mb.addHreg(Reg::CURRENT_RACK_IDX, 0xFF);
  mb.addHreg(Reg::ALL_HOMED_FLAG, 0);
  mb.addHreg(Reg::RACK_OCCUPIED_BITMASK, 0);
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
      // DIUBAH: boleh masuk menu dari state APA PUN (bukan cuma IDLE) -- logic fisik tetap jalan
      if (menuState == MenuState::NONE && key == '*') {
        menuState = MenuState::TOP_SELECT;
        if (lcdPresent) drawTopMenuStocker();
        Serial.println("[CAL] Masuk mode kalibrasi (command eksternal dijeda, logic fisik TETAP jalan)");
      } else if (menuState == MenuState::TOP_SELECT) handleTopMenuKeyStocker(key);
      else if (menuState == MenuState::CAL_LIST) handleCalListKey(key);
      else if (menuState == MenuState::MOVE_AXIS) handleMoveAxisKey(key);
      else if (menuState == MenuState::WAIT_SAVE_SLOT) handleSaveSlotKeyStocker(key);
      else if (menuState == MenuState::JOG_SPEED) handleSpeedKeyStocker(key);
      else if (menuState == MenuState::TEST_RACK_SELECT) handleTestRackSelectKey(key);
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

    // Jog KONTINU saat A/B ditahan di menu MOVE_AXIS
    if (menuState == MenuState::MOVE_AXIS) {
      constexpr int32_t HOLD_LOOKAHEAD = 20000;
      static bool wasHeldA = false, wasHeldB = false;
      bool heldA = keypad.isHeld('A'), heldB = keypad.isHeld('B');
      if (heldA) { tgtPos[selAxis] = curPos[selAxis] + HOLD_LOOKAHEAD; state = LiftState::MOVING; setStepperEnabled(true); }
      else if (heldB) { tgtPos[selAxis] = curPos[selAxis] - HOLD_LOOKAHEAD; state = LiftState::MOVING; setStepperEnabled(true); }
      else if (wasHeldA || wasHeldB) { tgtPos[selAxis] = curPos[selAxis]; }
      wasHeldA = heldA; wasHeldB = heldB;
    }
  }

  mb.Hreg(Reg::STATE, (uint16_t)currentState);
  mb.Hreg(Reg::FAULT_CODE, faultCode);
  mb.Hreg(Reg::ALL_HOMED_FLAG, (homed[0] && homed[1] && homed[2]) ? 1 : 0);
  updateRackStatusTask();   // DIPERBAIKI: gabung baca+tulis rak, throttled 100ms, 1x baca dipakai bersama
  mb.Hreg(Reg::RACK_OCCUPIED_BITMASK, lastRackBitmask);   // pakai cache, TIDAK baca ulang I2C
  // BARU: update register Lapis 2 + Lapis 3 tiap loop
  mb.Hreg(Reg::ACTIVITY_CODE, (uint16_t)activityCode());
  mb.Hreg(Reg::I2C_ERROR_COUNT, i2cErrorCount);
  mb.Hreg(Reg::LAST_FAULT_CODE, lastFaultCode);
  mb.Hreg(Reg::UPTIME_SEC, (uint16_t)(millis() / 1000));
  bool pauseAutoIndicators = (menuState == MenuState::TEST_OUTPUT_ITEM && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled);
  if (!pauseAutoIndicators) updateUniversalIndicators();

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

  handleSafety();
  if (currentState != NodeState::ESTOPPED) {
    switch (state) {
      case LiftState::HOMING: updateHoming(); break;
      case LiftState::MOVING: updateSteppers(); break;
      default: break;
    }
    updateYRetract();
    handleCycle();
    updateBuzzerBeep();   // BARU -- proses siklus ON-OFF buzzer non-blocking
    if (ackPending && cycleStage == CycleStage::NONE && state == LiftState::IDLE && !yRetracting) {
      ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq);
    }
    if (state == LiftState::IDLE && !yRetracting && cycleStage == CycleStage::NONE
        && currentState == NodeState::RUNNING_OR_MOVING) currentState = NodeState::IDLE;
  }

  // BARU (B06): Serial.printf() dipindah keluar dari updateSteppers() -- motion-critical
  // function sebaiknya tidak punya side-effect I/O tersembunyi
  if (motionJustFinished) {
    motionJustFinished = false;
    Serial.printf("[MOVE] selesai dalam %lums\n", (unsigned long)motionFinishedDurationMs);
  }

  if (menuState == MenuState::MOVE_AXIS && lcdPresent) {
    static uint32_t lastMoveAxisRefresh = 0;
    if (millis() - lastMoveAxisRefresh > 200) { lastMoveAxisRefresh = millis(); drawMoveAxisMenu(); }
  }

  // DIPERBAIKI: refresh live utk kategori Test Output (auto-controlled saja) dan Test Input
  // (semua input, selalu live) -- struktur baru, 2 kategori terpisah menggantikan 1 list lama
  if (menuState == MenuState::TEST_OUTPUT_ITEM && lcdPresent && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled) {
    static uint32_t lastTestOutRefresh = 0;
    if (millis() - lastTestOutRefresh > 200) { lastTestOutRefresh = millis(); drawTestOutputItem(); }
  }
  if (menuState == MenuState::TEST_INPUT_LIST && lcdPresent) {
    static uint32_t lastTestInRefresh = 0;
    if (millis() - lastTestInRefresh > 200) { lastTestInRefresh = millis(); drawTestInputList(); }
  }
  if (menuState == MenuState::TEST_RS485 && lcdPresent) {
    static uint32_t lastTestRs485Refresh = 0;
    if (millis() - lastTestRs485Refresh > 500) { lastTestRs485Refresh = millis(); drawTestRs485(); }
  }
  if (menuState == MenuState::TEST_MOD_STEPPER && lcdPresent) {
    static uint32_t lastTestModRefresh = 0;
    if (millis() - lastTestModRefresh > 150) { lastTestModRefresh = millis(); drawTestModStepper(); }
  }
  if (menuState == MenuState::TEST_MOD_MOTORDC && lcdPresent) {
    static uint32_t lastTestModRefresh2 = 0;
    if (millis() - lastTestModRefresh2 > 150) { lastTestModRefresh2 = millis(); drawTestModMotorDC(); }
  }
  if (menuState == MenuState::TEST_MOD_RELAY && lcdPresent) {
    static uint32_t lastTestModRefresh3 = 0;
    if (millis() - lastTestModRefresh3 > 150) { lastTestModRefresh3 = millis(); drawTestModRelay(); }
  }

  if (menuState != MenuState::NONE) return;

  handleSerialCommand();

  // DIPERBAIKI: timeout diperlebar dari 5000ms -- watchdog ini cuma reset saat command
  // BARU dikirim (lastRs485Rx di-update di onCmdWrite()), TIDAK ikut ter-reset oleh
  // pembacaan status (Modbus read register) yang tidak lewat callback itu. Testing manual
  // via menu interaktif (baca status berulang sambil menunggu progres) WAJAR jeda lebih
  // dari 5 detik antar command baru -- nilai lama terlalu ketat, sering false-trigger.
  // 30 detik cukup toleran utk homing/gerak fisik + jeda manual, TAPI tetap berfungsi
  // sbg pengaman kalau komunikasi BENAR-BENAR terputus total (kabel RS485 lepas, dst).
  constexpr uint32_t COMM_TIMEOUT_MS = 30000;
  if (modbusEverUsed && currentState == NodeState::RUNNING_OR_MOVING && millis() - lastRs485Rx > COMM_TIMEOUT_MS) {
    currentState = NodeState::FAULT; faultCode = (uint16_t)FaultCode::COMM_TIMEOUT;
  }

  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    mb.Hreg(Reg::HEARTBEAT, (uint16_t)(millis() / 1000));
    Serial.printf("[LOOP] state=%s liftState=%d ESTOP=%d pos=(X:%ld,Y:%ld,Z:%ld)\n",
                  stateText(currentState), (int)state, io.read(CH::ESTOP),
                  (long)curPos[0], (long)curPos[1], (long)curPos[2]);
  }

  if (lcdPresent) {
    static uint32_t lastLcdRefresh = 0;
    if (millis() - lastLcdRefresh > 500) {
      lastLcdRefresh = millis();
      lcdPrint(0, 0, "[AUTO] " + activityText());
      lcdPrint(0, 1, "State:" + String(stateText(currentState)));
      // DIPERBAIKI: format dipersingkat (tanpa spasi/titik dua) -- posisi microstep tinggi +
      // jarak jauh bisa 5-6 digit, format lama ("X:... Y:... Z:...") overflow 20 kolom LCD
      // untuk kasus itu. Batas praktis TAMPILAN LCD (bukan batas nilai internal curPos).
      lcdPrint(0, 2, "X" + String(curPos[0]) + "Y" + String(curPos[1]) + "Z" + String(curPos[2]));
      lcdPrint(0, 3, "Tahan* utk kalibrasi");
    }
  }
}