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
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "registers.h"
#include "keypad4x4.h"
#include "io_expander.h"
#include "wifi_credentials.h"

// BARU: objek pwm global, dipakai Test Modul Servo -- SAMA PERSIS pola dgn SORTER/PICKER
Adafruit_PWMServoDriver pwm(I2CAddr::PCA9685);

LiquidCrystal_I2C lcd(I2CAddr::LCD, LcdCfg::COLS, LcdCfg::ROWS);
Keypad4x4 keypad(I2CAddr::KEYPAD);
IOBank io;
ModbusRTU mb;
Preferences prefs;

constexpr char Keypad4x4::KEYMAP[4][4];

bool lcdPresent = false, keypadPresent = false;

String lcdCacheTeks[LcdCfg::ROWS];
uint8_t lcdCacheCol[LcdCfg::ROWS] = {0};
bool lcdCacheValid[LcdCfg::ROWS] = {false};

void lcdPrint(uint8_t col, uint8_t row, String text) {
  if (!lcdPresent) return;
  uint8_t avail = (col < LcdCfg::COLS) ? (LcdCfg::COLS - col) : 0;
  if (text.length() > avail) text = text.substring(0, avail);
  while (text.length() < avail) text += " ";
  // BARU (2026-09-22): lewati penulisan kalau isi baris ini memang tidak berubah.
  //
  // Menulis 20 karakter ke LCD lewat I2C memakan beberapa milidetik, dan selama itu loop()
  // tertahan. Layar-layar test menggambar ulang seluruh baris tiap 100-250 ms walau isinya
  // sama persis, sehingga gerakan yang timing-nya diatur per-tick (servo/stepper) kehilangan
  // tick dan terasa tersendat -- padahal justru layar itulah yang dipakai mengamatinya.
  // Tick yang hilang tidak pernah dibayar belakangan, jadi dampaknya langsung terlihat.
  if (row < LcdCfg::ROWS && lcdCacheValid[row] && lcdCacheCol[row] == col && lcdCacheTeks[row] == text) return;
  if (row < LcdCfg::ROWS) { lcdCacheValid[row] = true; lcdCacheCol[row] = col; lcdCacheTeks[row] = text; }
  lcd.setCursor(col, row);
  lcd.print(text);
}

// Wajib dipakai menggantikan lcdClear() -- layar kosong membuat seluruh cache basi.
void lcdClear() {
  if (lcdPresent) lcd.clear();
  for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false;
}

#define FW_VERSION "v.01.00.25082026.21.17"
constexpr const char* FW_BUILD = __DATE__ " " __TIME__;

// BARU: animasi LOADING singkat saat boot -- kasih feedback visual operator + waktu settle
// I2C/PCA9685/MCP23017 sebelum masuk operasi normal. No-op total kalau LCD tidak terdeteksi.
constexpr uint8_t BOOT_STEPS_TOTAL = 5;
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
    lcdPrint(0, 0, "STOCKER BOOTING");
    lcdPrint(0, 1, bar);
    lcdPrint(0, 2, "LOADING " + String(bootStep) + "/" + String(BOOT_STEPS_TOTAL));
    lcdPrint(0, 3, stepLabel);
  }
  delay(BOOT_STEP_DELAY_MS);
}

NodeState currentState = NodeState::INIT;
uint16_t faultCode = 0;
// BARU: pemisah MAIN/TEST -- default FALSE (fail-safe, boot-IDLE). RUN_FULL_CYCLE (produksi)
// butuh MAIN aktif; MOVE_TO_RACK/PUSH_BOX (manual) butuh TEST mode. HOME_ALL/GOTO_LOAD_POSITION
// sengaja TIDAK ikut di-gate (dipakai bareng produksi & Test Rak).
bool mainModeActive = false;
// BARU: Lapis 3 diagnostik -- sinkron pola SORTER
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
// DIUBAH: Test ke Rak sekarang urutannya Load Position -> Rak -> Push -> Tarik -> Load Position
// lagi (BUKAN lagi Home -> Rak doang) -- reuse cycleStage produksi (MOVING_XZ..RETURNING_XZ)
// buat bagian push/tarik/balik, jadi behavior test 100% sama kayak RUN_FULL_CYCLE asli.
bool testRackGoingToLoad = false;
uint8_t testRackSequenceTarget = 0;
// BARU: ukur waktu 1 siklus penuh Test Rak (Load Position -> Rak -> Push -> Tarik -> Load
// Position lagi) -- mulai dihitung begitu berangkat menuju rak, berhenti begitu kembali ke
// Load Position (bareng buzzer selesai), ditampilkan ke LCD + Serial.
bool testRackTimingActive = false;
uint32_t testRackCycleStartMs = 0;
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
uint8_t microstepMode = 8;   // 8 (MS1=GND,MS2=GND) = default fisik TMC2209 & paling cepat lewat jumper

// --- Menu state (dideklarasikan awal, dipakai onCmdWrite) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, MOVE_AXIS, WAIT_SAVE_SLOT, JOG_SPEED, TEST_RACK_SELECT,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_CATEGORY, TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_RELAY, TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET , SYS_INFO};
MenuState menuState = MenuState::NONE;

// BARU (2026-09-22): layar Info Sistem didefinisikan tepat sebelum loop() -- di titik itu
// otaReady/i2cErrorCount/lastFaultCode sudah terdeklarasi. Menu atas memanggilnya lebih
// awal, jadi butuh dua baris pengenalan ini.
uint8_t sysInfoPage = 0;
void drawSysInfo();
void handleSysInfoKey(char key);


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
  rampMinIntervalUs = 1200; rampSteps = 300; microstepMode = 8;
  buzzerOnMs = 500; buzzerOffMs = 500;   // BARU
  Serial.println("[RESET] Semua kalibrasi STOCKER dikembalikan ke default & NVS dihapus");
}

// DIPERBAIKI (bug ditemukan 2026-09-20, TERPARAH di STOCKER): penomoran bit GESER 1 dari
// yang dibaca Orange Pi. Firmware dulu pakai bit = INDEX ARRAY (RACK_LIM[0] -> bit0), padahal
// RACK_LIM[0] itu limit switch fisik RAK 1 (array RACK[] sendiri sudah 1-based, RACK[0] gak
// dipakai). Orange Pi find_free_rack() baca bit 1-4 -- jadi bit RAK 1 GAK PERNAH DIBACA
// (selalu kelihatan kosong -> semua box ditumpuk ke Rak 1 terus), sementara bit 4 yang dibaca
// itu LIM_8 yang gak ada fisiknya. SEKARANG bit N = RAK N (1-based, bit0 sengaja selalu 0),
// dan cuma 4 rak fisik yang di-scan (LIM_8/LIM_9 gak dipakai, floating INPUT_PULLUP).
constexpr uint8_t RACK_PHYSICAL_COUNT = 4;   // Rak 1-4, dikonfirmasi user
uint16_t readRackOccupiedBitmask() {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < RACK_PHYSICAL_COUNT; i++)
    if (io.read(CH::RACK_LIM[i]) == LOW) mask |= (1 << (i + 1));
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
  // DIPERBAIKI: penomoran bit SAMA dgn readRackOccupiedBitmask() di atas (bit N = Rak N,
  // 1-based) -- dulu dua fungsi ini sama-sama salah, sekarang dua-duanya konsisten.
  uint16_t mask = 0;
  for (uint8_t i = 0; i < RACK_PHYSICAL_COUNT; i++) {
    // DIHAPUS: io.write(CH::RACK_LED[i], ...) -- fitur RACK_LED tidak dipakai lagi di board universal baru
    if (io.read(CH::RACK_LIM[i]) == LOW) mask |= (1 << (i + 1));
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
  // BARU (2026-09-20): LED_FAULT. Channel ini di-pinMode OUTPUT di setup() dan terdaftar di
  // menu Test Output, TAPI tidak pernah ditulis satu kali pun secara otomatis -- lampu fault
  // praktis mati permanen selama produksi di KEEMPAT node. Sekarang ikut dikelola di sini.
  static bool lastLedFault = false;
  bool ledFaultNow = (currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED);
  if (ledFaultNow != lastLedFault) { io.write(CH::LED_FAULT, ledFaultNow); lastLedFault = ledFaultNow; }
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
  io.write(CH::EN_STEPPERS, enable ? LOW : HIGH);   // EN aktif-LOW (berlaku sama di TMC2209 & DRV8825)
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
  lastStepMicros[axis] = micros();   // settling time -- driver stepper butuh jeda setelah DIR berubah sebelum STEP pertama valid
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

// ============================================================
// BARU (2026-09-20) -- penghentian gerak terpusat + pemicu FAULT yang KONSISTEN.
//
// Masalah yang diperbaiki:
//   1. FAULT TIDAK menghentikan gerakan. loop() cuma menjaga `currentState != ESTOPPED`,
//      dan cabang FAULT di handleCycle() mengecek `state` (LiftState) bukan `currentState`.
//      Akibatnya COMM_TIMEOUT / MCP23017 mati di tengah RUN_FULL_CYCLE: stepper tetap jalan
//      dan siklus tetap selesai sampai habis sambil melaporkan FAULT ke master. Kalau yang
//      mati MCP-nya, limit switch dibaca ngawur tapi homing tetap lanjut sampai timeout 30s.
//   2. faultCode di-set TANPA currentState/state ikut FAULT (moveToRackXZ, updateYRetract).
//      Register STATE melaporkan IDLE/RUNNING padahal FAULT_CODE bukan 0, dan activityCode()
//      tidak pernah mengembalikan FAULT_AKTIF.
//   3. ackPending menggantung kalau gerak non-cycle (MOVE_TO_RACK/GOTO_LOAD_POSITION) kena
//      fault -- baru terkirim belakangan saat RESET_FAULT, sehingga CMD_ACK_SEQ ketimpa
//      seq LAMA dan master mengira RESET_FAULT tidak di-ack.
//
// CATATAN KESELAMATAN (disengaja): haltMotion() TIDAK mematikan driver stepper. Axis Z itu
// vertikal -- mematikan driver berarti melepas torsi tahan dan beban bisa jatuh karena
// gravitasi. Jadi saat FAULT, langkah dihentikan (target disamakan dengan posisi sekarang)
// tapi driver tetap energized supaya Z tertahan. Mematikan driver hanya dilakukan oleh
// E-stop, yang memang tujuannya memutus daya secara sengaja.
// ============================================================
void haltMotion() {
  for (uint8_t i = 0; i < 3; i++) { tgtPos[i] = curPos[i]; stepHigh[i] = false; }
  yRetracting = false;
  cycleStage = CycleStage::NONE;
  testRackGoingToLoad = false;
  testRackTimingActive = false;
}

// Pemicu FAULT tunggal: set kode + state + hentikan gerak + tuntaskan ack yang menggantung.
// Ack tetap dikirim karena konvensi protokol di node ini "ack = command diterima & selesai
// diproses", BUKAN "command berhasil" -- master membedakan berhasil/gagal lewat FAULT_CODE.
void raiseFault(uint16_t code, const char* why) {
  if (faultCode == 0) faultCode = code;
  currentState = NodeState::FAULT;
  state = LiftState::FAULT;
  haltMotion();
  if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); }
  Serial.printf("[FAULT] %s (code=%u) -- gerak dihentikan, driver stepper TETAP aktif (Z tertahan)\n",
                why, faultCode);
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
    // DIUBAH: lewat raiseFault() -- dulu di sini ackPending cuma di-CLEAR tanpa pernah
    // dikirim, jadi master menunggu ack yang tidak akan pernah datang sampai timeout.
    // setStepperEnabled(false) juga dihapus: lihat catatan keselamatan di haltMotion().
    raiseFault((uint16_t)FaultCode::HOMING_FAILED, "Homing timeout 30s (limit switch tidak pernah kena)");
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
    // DIUBAH: dulu cuma set faultCode + state(LiftState), currentState TETAP RUNNING --
    // register STATE bohong (lapor RUNNING padahal fault) dan activityCode() tidak pernah
    // balikin FAULT_AKTIF. raiseFault() menyetel ketiganya sekaligus.
    raiseFault((uint16_t)FaultCode::PUSH_STUCK, "Y retract GAGAL (limit Y tidak kena dalam 5s)");
    return;
  }
  uint32_t now = micros();
  if ((uint32_t)(now - lastStepMicros[1]) < stepIntervalUs) return;
  setAxisDirection(1, false);   // no-op kalau arah sudah sama
  if (startStepPulse(1)) lastStepMicros[1] = now;
}

// DIUBAH: dulu batas rackIdx < 6 (RACK[6] penuh) -- sekarang fisik cuma ada 4 rack (Rak 1-4,
// dikonfirmasi user). rackIdx 0 dan 5 SENGAJA gak valid lagi (walau slot array RACK[6] masih
// ada, index 0/5 gak dipakai) -- cegah Orange Pi/test script kirim rack_idx ke rak yang gak
// ada fisiknya.
bool moveToRackXZ(uint8_t rackIdx) {
  bool idxValid = (rackIdx >= 1 && rackIdx <= 4);
  if (!idxValid || !homed[0] || !homed[1] || !homed[2]) {
    // DIUBAH: dulu cuma set faultCode dan return -- currentState tetap IDLE, jadi dari sisi
    // master kelihatan "IDLE tapi semua command ditolak" (blockIfFaulted cek faultCode != 0).
    raiseFault(!idxValid ? (uint16_t)FaultCode::RACK_IDX_INVALID : (uint16_t)FaultCode::NOT_HOMED,
               !idxValid ? "rack_idx di luar 1-4" : "belum homing (X/Y/Z)");
    return false;
  }
  tgtPos[0] = RACK[rackIdx].x; tgtPos[2] = RACK[rackIdx].z;
  state = LiftState::MOVING;
  mb.Hreg(Reg::CURRENT_RACK_IDX, rackIdx);
  return true;
}

// BARU: diagnostik -- timestamp mulai stage cycleStage SEKARANG, buat ukur durasi per-tahap
// (MOVING_XZ/PUSHING_Y/RETRACT_Y/RETURNING_XZ terpisah) dan bandingin ke kalkulasi teori
// (steps x stepIntervalUs), cari tau di tahap mana ada waktu ekstra yang gak kejelasin.
uint32_t cycleStageStartMs = 0;

void handleCycle() {
  switch (cycleStage) {
    case CycleStage::NONE: break;
    case CycleStage::MOVING_XZ:
      if (state == LiftState::IDLE) {
        Serial.printf("[CYCLE-TIMING] MOVING_XZ selesai dalam %lums\n", (unsigned long)(millis() - cycleStageStartMs));
        tgtPos[1] = pushExtendSteps; state = LiftState::MOVING; cycleStage = CycleStage::PUSHING_Y;
        cycleStageStartMs = millis();
        triggerBuzzerBeep();   // BARU -- notifikasi: sudah masuk rack yang benar
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; testRackTimingActive = false; }
      break;
    case CycleStage::PUSHING_Y:
      if (state == LiftState::IDLE) {
        Serial.printf("[CYCLE-TIMING] PUSHING_Y selesai dalam %lums\n", (unsigned long)(millis() - cycleStageStartMs));
        startYRetract(); cycleStage = CycleStage::RETRACT_Y;
        cycleStageStartMs = millis();
        triggerBuzzerBeep();   // BARU -- notifikasi: selesai dorong package
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; testRackTimingActive = false; }
      break;
    case CycleStage::RETRACT_Y:
      if (!yRetracting) {
        if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; testRackTimingActive = false; return; }
        Serial.printf("[CYCLE-TIMING] RETRACT_Y selesai dalam %lums\n", (unsigned long)(millis() - cycleStageStartMs));
        // DIUBAH: setelah RUN_FULL_CYCLE selesai, lift kembali ke LOAD POSITION
        // (bukan Home lagi) -- siap langsung terima package berikutnya dari robot arm
        // tanpa perlu re-homing tiap siklus. Home tetap ada, cuma dipakai referensi kalibrasi.
        tgtPos[0] = loadPos.x; tgtPos[2] = loadPos.z; state = LiftState::MOVING;
        mb.Hreg(Reg::CURRENT_RACK_IDX, 0xFF);
        cycleStage = CycleStage::RETURNING_XZ;
        cycleStageStartMs = millis();
      }
      break;
    case CycleStage::RETURNING_XZ:
      if (state == LiftState::IDLE) {
        Serial.printf("[CYCLE-TIMING] RETURNING_XZ selesai dalam %lums\n", (unsigned long)(millis() - cycleStageStartMs));
        cycleStage = CycleStage::NONE;
        if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); }
        triggerBuzzerBeep();   // BARU -- notifikasi: sudah kembali ke Load Position
        // BARU: kalau ini bagian dari Test Rak, tampilkan lama 1 siklus penuh (Load->Rak->
        // Push->Tarik->Load) ke LCD + Serial, bareng buzzer selesai.
        if (testRackTimingActive) {
          testRackTimingActive = false;
          float elapsedSec = (millis() - testRackCycleStartMs) / 1000.0f;
          Serial.printf("[TEST-RACK] 1 siklus (Rak->Push->Tarik->Load) selesai dalam %.2f detik\n", elapsedSec);
          if (menuState == MenuState::TEST_RACK_SELECT) {
            char buf[21];
            snprintf(buf, sizeof(buf), "Siklus: %.2f detik!", elapsedSec);
            lcdPrint(0, 3, buf);
          }
        }
      }
      else if (state == LiftState::FAULT) { cycleStage = CycleStage::NONE; ackPending = false; testRackTimingActive = false; }
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

// BARU (2026-09-20): pembatalan siklus yang SOPAN -- dipakai Cmd::STOP_MAIN.
// Dulu STOP_MAIN cuma mematikan flag mainModeActive; siklus RUN_FULL_CYCLE yang sedang
// berjalan TERUS sampai selesai, dan tidak ada satu pun command untuk menghentikan lift
// selain E-stop. Sekarang STOP_MAIN benar-benar menghentikan.
//
// Kenapa tidak langsung berhenti total: kalau dibatalkan saat pusher Y sedang menjulur,
// Y akan tertinggal di luar -- lift lalu bergerak di sumbu X/Z dengan pusher masih
// menjulur dan bisa menabrak rak. Jadi X/Z dihentikan di tempat, lalu Y ditarik balik
// dulu sampai limit sebelum node dinyatakan IDLE.
void abortCycle() {
  bool sedangGerak = (cycleStage != CycleStage::NONE) || yRetracting || (state == LiftState::MOVING);
  if (!sedangGerak) return;

  for (uint8_t i = 0; i < 3; i++) tgtPos[i] = curPos[i];   // hentikan semua axis di posisi sekarang
  testRackGoingToLoad = false;
  testRackTimingActive = false;
  if (ackPending) { ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq); }

  if (yRetracting) {
    cycleStage = CycleStage::RETRACT_Y_STANDALONE;
    Serial.println("[ABORT] Siklus dibatalkan -- retract Y yang sedang jalan dibiarkan selesai dulu");
  } else if (curPos[1] != 0) {
    startYRetract();
    cycleStage = CycleStage::RETRACT_Y_STANDALONE;
    Serial.println("[ABORT] Siklus dibatalkan -- pusher Y ditarik balik dulu sebelum berhenti");
  } else {
    cycleStage = CycleStage::NONE;
    state = LiftState::IDLE;
    currentState = NodeState::IDLE;
    Serial.println("[ABORT] Siklus dibatalkan -- lift berhenti di tempat (pusher sudah tertarik)");
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
    // DIPERBAIKI (2026-09-20): tgtPos TIDAK PERNAH direset di sini. Kalau E-stop ditekan saat
    // PUSHING_Y, tgtPos[1] tetap berisi pushExtendSteps -- begitu E-stop dilepas dan command
    // gerak berikutnya menyetel state=MOVING, updateSteppers() ikut melanjutkan dorongan Y
    // yang tertinggal itu, padahal tidak ada yang memintanya. Samakan target = posisi sekarang.
    for (uint8_t i = 0; i < 3; i++) { tgtPos[i] = curPos[i]; stepHigh[i] = false; }
    cycleStage = CycleStage::NONE; ackPending = false; yRetracting = false;
    testRackGoingToLoad = false;   // BARU -- cegah chain Test Rak resume aneh setelah E-stop dilepas
    testRackTimingActive = false;
    return;
  }
  if (currentState == NodeState::ESTOPPED) {
    setStepperEnabled(true);   // aman dioptimasi -- bukan jalur darurat
    currentState = NodeState::IDLE; state = LiftState::IDLE;
    for (uint8_t i = 0; i < 3; i++) homed[i] = false;
  }
}

// ============================================================
// OTA (WiFi) -- update firmware tanpa colok-cabut USB. Lihat wifi_credentials.h
// (gitignored, isi asli SSID/password/OTA password per file .example di include/).
// ============================================================
bool otaReady = false;   // true == WiFi connect & ArduinoOTA siap terima update sekarang
bool otaBegun = false;   // ArduinoOTA.begin() sudah dipanggil sekali (callback ter-register)

// Dipanggil dari ArduinoOTA.onStart() -- proses tulis flash BLOCKING selama beberapa detik,
// matikan driver stepper dulu (unconditional, sama pola fail-safe dgn handleSafety()) supaya
// tidak ada step nyasar selama CPU sibuk nulis flash.
void otaSafeStop() {
  io.write(CH::EN_STEPPERS, HIGH);
  steppersEnabled = false;
  for (uint8_t i = 0; i < 3; i++) { tgtPos[i] = curPos[i]; stepHigh[i] = false; }   // BARU -- sama alasannya dgn E-stop
  cycleStage = CycleStage::NONE; ackPending = false; yRetracting = false;
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
    case Cmd::START_MAIN:
      if (blockIfFaulted("START_MAIN", seq)) return;
      mainModeActive = true;
      Serial.println("[CMD] START_MAIN -- MAIN aktif, MOVE_TO_RACK/PUSH_BOX diblokir sampai STOP_MAIN");
      mb.Hreg(Reg::CMD_ACK_SEQ, seq); return;
    case Cmd::STOP_MAIN:
      mainModeActive = false;
      // DIUBAH (2026-09-20): STOP_MAIN sekarang BENAR-BENAR menghentikan. Dulu cuma mematikan
      // flag, siklus RUN_FULL_CYCLE yang sedang jalan tetap lanjut sampai habis -- padahal
      // shutdown_sequence() Orange Pi mengirim STOP_MAIN justru untuk menghentikan node.
      abortCycle();
      Serial.println("[CMD] STOP_MAIN -- MAIN mati, siklus dibatalkan, MOVE_TO_RACK/PUSH_BOX boleh dipakai lagi");
      mb.Hreg(Reg::CMD_ACK_SEQ, seq); return;
    case Cmd::HOME_ALL:
      if (blockIfFaulted("HOME_ALL", seq)) return;
      startHomingInternal();
      currentState = NodeState::RUNNING_OR_MOVING;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::RUN_FULL_CYCLE:
      if (blockIfFaulted("RUN_FULL_CYCLE", seq)) return;
      if (!mainModeActive) { Serial.println("[CMD] RUN_FULL_CYCLE ditolak -- MAIN belum aktif, kirim START_MAIN dulu"); mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      if (!homed[1] || curPos[1] != 0) { raiseFault((uint16_t)FaultCode::NOT_HOMED, "RUN_FULL_CYCLE: axis Y belum homing / pusher belum di posisi 0"); mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      if (!moveToRackXZ((uint8_t)arg)) { mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      currentState = NodeState::RUNNING_OR_MOVING;
      cycleStage = CycleStage::MOVING_XZ;
      cycleStageStartMs = millis();
      triggerBuzzerBeep();   // BARU -- notifikasi: package sudah di lift, cycle mulai
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::MOVE_TO_RACK:
      if (blockIfFaulted("MOVE_TO_RACK", seq)) return;
      if (mainModeActive) { Serial.println("[CMD] MOVE_TO_RACK ditolak -- MAIN aktif, STOP_MAIN dulu"); mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      if (!moveToRackXZ((uint8_t)arg)) { mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      currentState = NodeState::RUNNING_OR_MOVING;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::PUSH_BOX:
      if (blockIfFaulted("PUSH_BOX", seq)) return;
      if (mainModeActive) { Serial.println("[CMD] PUSH_BOX ditolak -- MAIN aktif, STOP_MAIN dulu"); mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      tgtPos[1] = curPos[1] + pushExtendSteps;
      state = LiftState::MOVING;
      currentState = NodeState::RUNNING_OR_MOVING;
      cycleStage = CycleStage::PUSHING_Y_STANDALONE;
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::GOTO_LOAD_POSITION:   // BARU -- manual, menuju titik standby terima package
      if (blockIfFaulted("GOTO_LOAD_POSITION", seq)) return;
      if (!homed[0] || !homed[2]) { raiseFault((uint16_t)FaultCode::NOT_HOMED, "GOTO_LOAD_POSITION: axis X/Z belum homing"); mb.Hreg(Reg::CMD_ACK_SEQ, seq); return; }
      tgtPos[0] = loadPos.x; tgtPos[2] = loadPos.z;
      state = LiftState::MOVING;
      currentState = NodeState::RUNNING_OR_MOVING;
      mb.Hreg(Reg::CURRENT_RACK_IDX, 0xFF);
      ackPending = true; pendingAckSeq = seq; return;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb sebelum di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE; state = LiftState::IDLE;
      break;
    // BARU -- biar Orange Pi bisa tuning kecepatan langsung, sama rentang/pola dgn Serial STEPSPEED/HOMESPEED.
    // DIUBAH: batas bawah 20->0 -- 0 = tanpa jeda sama sekali (step secepat loop() bisa jalan,
    // sama konvensi dgn hopperStepIntervalMs SORTER/servo*StepIntervalMs DISPENSER).
    case Cmd::SET_STEP_INTERVAL:
      stepIntervalUs = (uint16_t)constrain((int)arg, 0, 5000);
      saveStepIntervalToNvs();
      Serial.printf("[CMD] SET_STEP_INTERVAL -- stepIntervalUs=%u\n", stepIntervalUs);
      break;
    case Cmd::SET_HOMING_STEP_INTERVAL:
      homingStepIntervalUs = (uint16_t)constrain((int)arg, 20, 5000);
      saveHomingIntervalToNvs();
      Serial.printf("[CMD] SET_HOMING_STEP_INTERVAL -- homingStepIntervalUs=%u\n", homingStepIntervalUs);
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
  lcdClear();
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
constexpr uint8_t TOP_COUNT = 4;   // DIUBAH 3 -> 4, tambah "Info Sistem"
const char* TOP_LABELS[TOP_COUNT] = { "Setting Kalibrasi", "Test I/O", "Test Command", "Info Sistem" };
uint8_t topCursor = 0;
void drawTopMenuStocker() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
// BARU: "Reset Fault" jadi item pertama -- dulu cuma bisa dipicu lewat menu "Test Command"
// yang terkubur, gak gampang ditemukan operator pas node FAULT.
constexpr uint8_t CAL_COUNT = 9;
const char* CAL_LABELS[CAL_COUNT] = { "Reset Fault", "AutoHome (wajib dulu)", "Jog Posisi (X/Y/Z)", "Test ke Rak",
                                        "Simpan ke Slot Rak", "Simpan Load Position", "Simpan Jarak Dorong", "Kecepatan", "Reset ke Default" };
uint8_t calCursor = 0;
void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

// BARU: Test ke Rak -- selalu HOME dulu, baru menuju rak (sesuai permintaan: "test rack 1
// selalu urutannya ke home dulu baru ke tempat rack"). Homing + pindah-ke-rak digabung
// jadi 1 urutan otomatis, bukan 2 langkah manual terpisah.
// DIUBAH: fisik cuma ada 4 rack (Rak 1-4, dikonfirmasi user) -- dulu 6 (Rak 0-5). Label LCD
// LANGSUNG jadi nomor rackIdx asli (bukan lagi index-0 array RACK_LABELS), jadi testRackCursor
// disimpan sebagai rackIdx (1-4) langsung, bukan index 0-based lagi.
constexpr uint8_t RACK_COUNT = 4;
const char* RACK_LABELS[RACK_COUNT] = { "Rak 1", "Rak 2", "Rak 3", "Rak 4" };
uint8_t testRackCursor = 1;   // rackIdx asli (1-4), BUKAN index array

void drawTestRackSelect() { drawListMenu("TEST KE RAK", RACK_LABELS, RACK_COUNT, testRackCursor - 1); }
void handleTestRackSelectKey(char key) {
  if (key == 'A') { testRackCursor = (testRackCursor == 1) ? RACK_COUNT : testRackCursor - 1; drawTestRackSelect(); }
  else if (key == 'B') { testRackCursor = (testRackCursor == RACK_COUNT) ? 1 : testRackCursor + 1; drawTestRackSelect(); }
  else if (key == 'C') {
    if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
      lcdPrint(0, 3, "Tak bisa:FAULT/ESTOP");
    } else if (!homed[0] || !homed[2]) {
      lcdPrint(0, 3, "AutoHome dulu!");
      Serial.println("[TEST-RACK] Ditolak -- belum di-Home (AutoHome dulu, item menu #1)");
    } else {
      // DIUBAH: urutan sekarang Load Position -> Rak -> Push -> Tarik -> Load Position lagi
      // (BUKAN lagi Home -> Rak). Bagian push/tarik/balik REUSE cycleStage produksi yang sama
      // persis dengan RUN_FULL_CYCLE, dipicu begitu Load Position tercapai (lihat loop()).
      testRackSequenceTarget = testRackCursor;
      testRackGoingToLoad = true;
      tgtPos[0] = loadPos.x; tgtPos[2] = loadPos.z;
      state = LiftState::MOVING;
      currentState = NodeState::RUNNING_OR_MOVING;
      lcdPrint(0, 3, "Ke Load Pos dulu...");
      Serial.printf("[TEST-RACK] Menuju Load Position, lalu Rak %u (push+tarik+balik)\n", testRackSequenceTarget);
    }
  }
  else if (key == 'D') { menuState = MenuState::CAL_LIST; drawCalList(); }
}

void drawConfirmReset() {
  lcdClear();
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
  // Tabel resmi TMC2209 standalone pin-only (sumber: Watterott SilentStepStick TMC2209 docs):
  //   MS1=GND, MS2=GND -> 1/8   (PALING KASAR/CEPAT -- ini JUGA default kalau MS1/MS2 floating,
  //                               driver TMC2209 punya internal pulldown)
  //   MS1=VIO, MS2=GND -> 1/32
  //   MS1=GND, MS2=VIO -> 1/64  (PALING HALUS/LAMBAT)
  //   MS1=VIO, MS2=VIO -> 1/16
  // TIDAK ADA cara dapet LEBIH CEPAT dari 1/8 lewat jumper MS1/MS2 -- itu sudah yang PALING
  // KASAR yang bisa dipilih. Kalau butuh lebih cepat dari ini, satu-satunya jalan lain adalah
  // UART (TMC2209 bisa full-step/half-step via register CHOPCONF.MRES, TAPI itu butuh wiring
  // PDN_UART + library TMCStepper, BEDA dari pendekatan jumper pin murni di sini).
  constexpr uint8_t MSTEP_VALUES[4] = {8, 16, 32, 64};   // urut kasar(cepat)->halus(lambat), utk tombol A(naik)/B(turun)
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
        case 1: stepIntervalUs = (uint16_t)constrain((int)stepIntervalUs + delta, 0, 5000); break;   // DIUBAH: 20->0, 0=tanpa jeda
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
  if (key >= '1' && key <= '4') {   // DIUBAH: fisik cuma Rak 1-4 (dulu 0-5)
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
      case 0:   // BARU -- Reset Fault, langsung eksekusi (tidak destruktif, gak perlu konfirmasi)
        applyCommand((uint16_t)Cmd::RESET_FAULT, 0, 0);
        lcdPrint(0, 3, "Fault direset!      ");
        Serial.println("[CAL] Reset Fault dari menu LCD");
        break;
      case 1:   // AutoHome
        if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
          lcdPrint(0, 3, "Tak bisa:FAULT/ESTOP");
        } else {
          startHomingInternal();
          currentState = NodeState::RUNNING_OR_MOVING;
          lcdPrint(0, 3, "Homing berjalan...");
          Serial.println("[CAL] AUTO HOME dari menu dimulai");
        }
        break;
      case 2:   // Jog Posisi
        if (!homed[0] || !homed[1] || !homed[2]) { lcdPrint(0, 3, "Home dulu! (opsi a)"); }
        else { menuState = MenuState::MOVE_AXIS; lcdClear(); drawMoveAxisMenu(); }
        break;
      case 3:   // BARU: Test ke Rak -- selalu home dulu, baru menuju rak
        menuState = MenuState::TEST_RACK_SELECT; testRackCursor = 1; lcdClear(); drawTestRackSelect();
        break;
      case 4: menuState = MenuState::WAIT_SAVE_SLOT; lcdPrint(0, 3, "Simpan(X,Z)slot?1-4"); break;
      case 5:   // BARU: Simpan Load Position -- simpan X,Z SAAT INI, cuma 1 titik (bukan slot 0-5)
        loadPos = {curPos[0], curPos[2]};
        saveLoadPosToNvs();
        lcdPrint(0, 3, "LoadPos disimpan!");
        Serial.printf("[CAL] X=%ld Z=%ld -> loadPos\n", (long)curPos[0], (long)curPos[2]);
        break;
      case 6:
        if (curPos[1] <= 0) { lcdPrint(0, 3, "Y harus>0(jog dulu)"); }
        else { pushExtendSteps = curPos[1]; savePushExtendToNvs(); lcdPrint(0, 3, "PushExtend disimpan!"); Serial.printf("[CAL] pushExtendSteps=%ld\n", (long)pushExtendSteps); }
        break;
      case 7: menuState = MenuState::JOG_SPEED; lcdClear(); drawSpeedMenuStocker(); break;
      case 8: menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); break;
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
  {"FAULT",  CH::LED_FAULT,     true},   // DIUBAH -- sekarang auto-controlled (lihat updateUniversalIndicators)
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
    lcdClear();
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

// --- Kategori: Test Input -- 3 sub-kategori (Button/Proximity/Limit Switch), SAMA PERSIS
// channel-nya di semua 4 node (board PCB universal) -- test channel mentah CH::xxx, TERLEPAS
// dari alias/peran yang dipakai node ini (LIM_1..3 = LIM_X/Y/Z, LIM_4..9 = RACK_LIM[0..5] di
// node ini, tapi diuji sbg LIM1..LIM10 mentah di sini spy konsisten sama 3 node lain).
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
uint8_t inputTestScrollTop = 0;               // indeks item PALING ATAS yang sedang tampil (viewport 4 baris)
constexpr uint8_t INPUT_TEST_MAX = LIMIT_TEST_COUNT;
bool testInputLiveLastVal[INPUT_TEST_MAX];    // nilai TERAKHIR yang ditampilkan per item -- cegah kedip
uint8_t testInputLastScrollTop = 255;         // beda dari nilai valid manapun -- paksa redraw penuh pertama kali

void drawTestInputList() {
  bool scrollChanged = (inputTestScrollTop != testInputLastScrollTop);
  if (scrollChanged) { lcdClear(); testInputLastScrollTop = inputTestScrollTop; }
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
  // 'C' sengaja tidak melakukan apa-apa -- input baca-saja, tidak ada item terpisah lagi
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
  lcdClear(); lcdPrint(0, 0, "I2C SCAN...");
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
  lcdClear();
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
    lcdClear(); lcdPrint(0, 0, "MODUL: STEPPER");
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
// DIPERBAIKI (bug ditemukan): sebelumnya A/B cuma toggle stop<->maju, dirState=2 (mundur)
// TIDAK PERNAH bisa dicapai dari menu -- arah mundur gak bisa ditest/dikalibrasi sama sekali.
// Sekarang A/B cycle stop->maju->mundur->stop tiap ditekan.
const char* motorDirName(uint8_t s) { return s == 0 ? "STOP" : (s == 1 ? "FWD" : "REV"); }
void drawTestModMotorDC() {
  if (testModFirstDraw) {
    lcdClear(); lcdPrint(0, 0, "MODUL: MOTOR DC");
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
    testModSetMotor(true, 0); testModSetMotor(false, 0);   // stop semua sebelum keluar -- safety
    menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return;
  }
  drawTestModMotorDC();
}

// --- Modul: Relay (RLY1/RLY2) -- toggle, rate-limit 300ms (beban induktif) ---
uint8_t testModRelaySel = 0;   // 0=RLY1, 1=RLY2
void drawTestModRelay() {
  if (testModFirstDraw) {
    lcdClear(); lcdPrint(0, 0, "MODUL: RELAY");
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
    lcdClear(); lcdPrint(0, 0, "MODUL: SERVO");
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
      case 2: menuState = MenuState::TEST_INPUT_CATEGORY; inputCatCursor = 0; drawInputCategorySelect(); break;
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
      case 3: menuState = MenuState::SYS_INFO; sysInfoPage = 0; lcdClear(); drawSysInfo(); break;
    }
  } else if (key == 'D') { menuState = MenuState::NONE; lcdClear(); Serial.println("[CAL] Keluar mode kalibrasi"); }
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
  else if (cmd == "STEPSPEED") { stepIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 0, 5000); saveStepIntervalToNvs(); Serial.printf("[STEPSPEED] %u\n", stepIntervalUs); }
  else if (cmd == "HOMESPEED") { homingStepIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 20, 5000); saveHomingIntervalToNvs(); Serial.printf("[HOMESPEED] %u\n", homingStepIntervalUs); }
  else if (cmd == "RAMPMIN") { rampMinIntervalUs = constrain((int)line.substring(sp1 + 1).toInt(), 20, 5000); saveRampToNvs(); Serial.printf("[RAMPMIN] %u\n", rampMinIntervalUs); }
  else if (cmd == "RAMPSTEPS") { rampSteps = constrain((int)line.substring(sp1 + 1).toInt(), 0, 5000); saveRampToNvs(); Serial.printf("[RAMPSTEPS] %u\n", rampSteps); }
  else if (cmd == "MICROSTEP") {
    uint8_t val = line.substring(sp1 + 1).toInt();
    if (val == 8 || val == 16 || val == 32 || val == 64) { microstepMode = val; saveMicrostepToNvs(); Serial.printf("[MICROSTEP] Dicatat: 1/%u -- set jumper MS1/MS2 TMC2209 sesuai tabel (lihat HELP)\n", microstepMode); }
    else Serial.println("[MICROSTEP] Nilai harus 8, 16, 32, atau 64 (TMC2209 pin-mode, TIDAK ada 1=full-step, TIDAK ada 1/2 atau 1/4)");
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
    Serial.println("[HELP] RAMPMIN <us> | RAMPSTEPS <n> | MICROSTEP <8|16|32|64 -- TMC2209, cek tabel MS1/MS2, 8=paling cepat> | RESET | STATUS | HELP");
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
  if (!lcdPresent && lcdPing) { lcdPresent = true; lcd.init(); lcd.backlight(); for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false; Serial.println("[HOTPLUG] LCD baru terdeteksi -- diinisialisasi live"); }
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

  if (lcdPresent) { lcd.init(); lcd.backlight(); for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false; lcdPrint(0, 0, "STOCKER"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");
  lcdBootProgress("I2C+PCA+LCD/Keypad");

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
  lcdBootProgress("I/O Expander MCP23017");

  pinMode(GP::STEP_X, OUTPUT); pinMode(GP::STEP_Y, OUTPUT); pinMode(GP::STEP_Z, OUTPUT);
  Serial.println("[BOOT] pinMode STEP_X/Y/Z (native) OK");
  lcdBootProgress("Stepper STEP pinMode");

  loadRackFromNvs();
  io.write(CH::EN_STEPPERS, LOW);   // unconditional -- init boot, jangan lewat cache
  steppersEnabled = true;   // sinkronkan cache
  Serial.println("[BOOT] Kalibrasi dimuat dari NVS");
  lcdBootProgress("Kalibrasi NVS");

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
  mb.addHreg(Reg::MAIN_MODE_ACTIVE, 0);   // BARU -- status live MAIN vs TEST mode
  mb.addHreg(Reg::MENU_ACTIVE, 0);        // BARU -- 1 = operator di menu kalibrasi, command Modbus diabaikan
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  Serial.printf("[BOOT] Modbus siap, slave ID=%d\n", Rs485Cfg::SLAVE_ID);
  lcdBootProgress("Modbus RS485");

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;

  setupOTA();
  lcdBootProgress("WiFi OTA");

  Serial.println("[BOOT] setup SELESAI -- ketik HELP di serial monitor");
}


// ============================================================
// INFO SISTEM -- kategori menu utama ke-4 (BARU 2026-09-22)
//
// Sebelumnya semua data ini hanya terlihat lewat Serial USB: operator yang berdiri di panel
// harus mengambil laptop dan mencolok kabel hanya untuk tahu versi firmware atau alamat IP.
// Alamat IP yang paling sering dicari -- tanpa itu OTA tidak bisa dijalankan sama sekali.
//
// Tiga halaman, A/B berpindah, D keluar.
// ============================================================
constexpr uint8_t SYS_INFO_PAGES = 3;

String sysUptimeText() {
  uint32_t d = millis() / 1000;
  return String(d / 3600) + "j " + String((d % 3600) / 60) + "m " + String(d % 60) + "d";
}

void drawSysInfo() {
  lcdPrint(0, 0, "INFO SISTEM " + String(sysInfoPage + 1) + "/" + String(SYS_INFO_PAGES));
  switch (sysInfoPage) {
    case 0: {
      // FW_VERSION penuh 23 karakter, tidak muat di 20 kolom -- ambil bagian tengahnya yang
      // paling informatif. Versi lengkap tetap utuh di Serial STATUS.
      String v = FW_VERSION;
      lcdPrint(0, 1, "FW:" + (v.length() >= 16 ? v.substring(2, 16) : v));
      lcdPrint(0, 2, String(FW_BUILD));
      break;
    }
    case 1:
      lcdPrint(0, 1, "Hidup: " + sysUptimeText());
      lcdPrint(0, 2, "Heap: " + String(ESP.getFreeHeap()) + "B");
      break;
    case 2:
      if (otaReady) lcdPrint(0, 1, "IP:" + WiFi.localIP().toString());
      else          lcdPrint(0, 1, "WiFi: tidak connect");
      lcdPrint(0, 2, "I2Cerr:" + String(i2cErrorCount) + " Flt:" + String(lastFaultCode)
                     + " S" + String(Rs485Cfg::SLAVE_ID));
      break;
  }
  lcdPrint(0, 3, "A/B=halaman D=keluar");
}

void handleSysInfoKey(char key) {
  if (key == 'A') sysInfoPage = (sysInfoPage == 0) ? SYS_INFO_PAGES - 1 : sysInfoPage - 1;
  else if (key == 'B') sysInfoPage = (sysInfoPage + 1) % SYS_INFO_PAGES;
  else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuStocker(); return; }
  drawSysInfo();
}

void loop() {
  mb.task();
  updateOTA();

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
      else if (menuState == MenuState::SYS_INFO) handleSysInfoKey(key);
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
  mb.Hreg(Reg::MAIN_MODE_ACTIVE, mainModeActive ? 1 : 0);
  // BARU: master bisa bedain "node lagi dikalibrasi operator" vs "node mati/kabel putus" --
  // dua-duanya sama-sama TIDAK membalas CMD_ACK_SEQ, jadi sebelumnya tidak bisa dibedakan.
  mb.Hreg(Reg::MENU_ACTIVE, (menuState != MenuState::NONE) ? 1 : 0);
  bool pauseAutoIndicators = (menuState == MenuState::TEST_OUTPUT_ITEM && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled);
  if (!pauseAutoIndicators) updateUniversalIndicators();

  static uint32_t lastMcpHealthCheck = 0;
  if (millis() - lastMcpHealthCheck > 2000) {
    lastMcpHealthCheck = millis();
    bool healthy = io.recheckHealth();
    if (!healthy) i2cErrorCount++;   // BARU -- catat SETIAP kegagalan
    if (!healthy && currentState != NodeState::FAULT && currentState != NodeState::ESTOPPED) {
      // DIUBAH: lewat raiseFault() supaya gerakan benar-benar berhenti. Ini kasus paling
      // berbahaya dari bug lama: limit switch dibaca lewat MCP yang SAMA, jadi kalau MCP mati
      // sementara homing jalan, data limit tidak bisa dipercaya TAPI axis tetap melangkah.
      raiseFault((uint16_t)FaultCode::IO_EXPANDER_MISSING, "MCP23017 berhenti merespons I2C (data sensor tidak bisa dipercaya)");
    }
  }

  checkLcdKeypadHotplug();

  handleSafety();
  // DIPERBAIKI (2026-09-20): buzzer dipindah KELUAR dari blok bersyarat di bawah. Dulu ikut
  // di-skip saat ESTOPPED/FAULT -- kalau fault terjadi tepat di tengah bunyi, tidak ada lagi
  // yang mematikannya dan buzzer nyala terus tanpa henti.
  updateBuzzerBeep();

  // DIPERBAIKI (2026-09-20): FAULT ikut dijaga, dulu hanya ESTOPPED. Lihat penjelasan lengkap
  // di raiseFault()/haltMotion(). Tanpa ini, COMM_TIMEOUT atau MCP23017 mati di tengah siklus
  // tidak menghentikan apa pun -- stepper jalan terus sambil register melaporkan FAULT.
  if (currentState != NodeState::ESTOPPED && currentState != NodeState::FAULT) {
    switch (state) {
      case LiftState::HOMING: updateHoming(); break;
      case LiftState::MOVING: updateSteppers(); break;
      default: break;
    }
    updateYRetract();
    handleCycle();
    if (ackPending && cycleStage == CycleStage::NONE && state == LiftState::IDLE && !yRetracting) {
      ackPending = false; mb.Hreg(Reg::CMD_ACK_SEQ, pendingAckSeq);
    }
    if (state == LiftState::IDLE && !yRetracting && cycleStage == CycleStage::NONE
        && currentState == NodeState::RUNNING_OR_MOVING) {
      if (testRackGoingToLoad) {
        // Load Position tercapai -- lanjut ke Rak, REUSE cycleStage produksi buat push/tarik/balik.
        testRackGoingToLoad = false;
        if (moveToRackXZ(testRackSequenceTarget)) {
          cycleStage = CycleStage::MOVING_XZ;
          cycleStageStartMs = millis();
          testRackTimingActive = true;
          testRackCycleStartMs = millis();   // BARU -- mulai hitung 1 siklus (Rak->Push->Tarik->Load)
          Serial.printf("[TEST-RACK] Load Position tercapai, menuju Rak %u\n", testRackSequenceTarget);
        } else {
          // DIUBAH: JANGAN paksa currentState=IDLE di sini -- moveToRackXZ() yang gagal sudah
          // memanggil raiseFault() (currentState=FAULT). Menimpanya dgn IDLE menghapus jejak
          // fault dari register STATE, persis bug yang baru diperbaiki.
          Serial.println("[TEST-RACK] Gagal menuju rak dari Load Position (cek FaultCode)");
        }
      } else {
        currentState = NodeState::IDLE;
      }
    }
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
  // Uptime berjalan terus, jadi layar ini harus hidup sendiri tanpa menunggu tombol.
  // Aman dari masalah LCD-menahan-loop: lcdPrint() hanya menulis baris yang benar-benar
  // berubah, dan di halaman ini cuma baris detik yang bergerak.
  if (menuState == MenuState::SYS_INFO && lcdPresent) {
    static uint32_t lastSysRefresh = 0;
    if (millis() - lastSysRefresh > 500) { lastSysRefresh = millis(); drawSysInfo(); }
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
    // DIUBAH: lewat raiseFault() -- dulu cuma set register, gerakan tetap lanjut sampai selesai
    raiseFault((uint16_t)FaultCode::COMM_TIMEOUT, "tidak ada command Modbus baru selama 30 detik");
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