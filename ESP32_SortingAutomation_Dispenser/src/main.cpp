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
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "registers.h"
#include "keypad4x4.h"
#include "io_expander.h"
#include "wifi_credentials.h"

// BARU: objek pwm global, dipakai Test Modul Servo -- SAMA PERSIS pola dgn SORTER/PICKER/STOCKER
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
    lcdPrint(0, 0, "FEEDER BOOTING");
    lcdPrint(0, 1, bar);
    lcdPrint(0, 2, "LOADING " + String(bootStep) + "/" + String(BOOT_STEPS_TOTAL));
    lcdPrint(0, 3, stepLabel);
  }
  delay(BOOT_STEP_DELAY_MS);
}

NodeState currentState = NodeState::INIT;
uint16_t faultCode = 0;
// BARU: Lapis 3 diagnostik -- sinkron pola SORTER/PICKER/STOCKER
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
uint32_t lastRs485Rx = 0;
bool modbusEverUsed = false;

struct FeederConfig {
  uint8_t  conveyorSpeed = 160;
  bool     conveyorDir = true;   // BARU -- true=forward, false=reverse (dulu hardcoded forward)
  uint16_t pushTimeoutMs = 800;   // DIUBAH makna: sekarang batas waktu MAKSIMUM tiap gerakan servo (pengaman, servo tak punya sensor "sampai")
  uint16_t buzzerOnMs  = 500;
  uint16_t buzzerOffMs = 500;
  // BARU: 2 servo gerbang stack dispenser -- Servo1=gerbang BAWAH, Servo2=gerbang ATAS.
  // Titik Awal=tertutup/tertahan, Titik Akhir=terbuka/lepas. Waktu Tahan terpisah dari gerak.
  // DIUBAH TOTAL: *StepIntervalMs (dulu *IntervalMs = target durasi TETAP satu arah) sekarang
  // = jeda antar-tick step (SAMA pola dgn hopper SORTER/PICKER) -- durasi jadi HASIL
  // jarak/kecepatan, bukan target yang dipaksakan (menghilangkan bug "mundur sebelum sampai").
  uint16_t servo1StartUs = 1000, servo1EndUs = 2000, servo1HoldMs = 500, servo1StepIntervalMs = 20;
  uint16_t servo2StartUs = 1000, servo2EndUs = 2000, servo2HoldMs = 500, servo2StepIntervalMs = 20;
  // DIUBAH: servoStartDelayMs SUDAH GAK DIPAKAI di siklus produksi normal (REQUEST_REFILL) --
  // itu sekarang PROX_2-edge-triggered, sensor-confirmed. Field ini DIPAKAI LAGI, tapi KHUSUS
  // utk Cmd::FORCE_MIDDLE_REFILL (startup/recovery): conveyor jalan duluan, servo1+2 baru
  // dieksekusi setelah jeda ini.
  uint16_t servoStartDelayMs = 0;
  // BARU: besar lompatan tiap tick (us) -- pasangan servo*StepIntervalMs di atas.
  uint16_t servo1StepUs = 20, servo2StepUs = 20;
  // BARU: mekanisme 2-proximity -- batas waktu tunggu PACKAGE_MIDDLE_SENSOR konfirmasi LOW
  // (package baru beneran nyampe tengah) SETELAH servo selesai jatuhin -- kalau kelewat,
  // FAULT(MIDDLE_PACKAGE_MISSING) (macet/kehabisan stok tumpukan). DITARUH DI AKHIR struct.
  uint16_t middleConfirmTimeoutMs = 5000;
  // BARU: khusus TEST REFILL LOOP (mode test lokal, TERPISAH dari produksi) -- jeda antar
  // percobaan ulang jatuhin servo kalau TENGAH masih belum kedeteksi terisi. DITARUH DI AKHIR
  // struct (NVS-safe, jangan disisip di tengah).
  uint16_t testLoopRetryIntervalMs = 1000;
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

// DIUBAH: servo (gerbang stack) TIDAK LAGI bagian dari RefillState -- sekarang jalan PARALEL,
// state independen sendiri (lihat ServoRefillStage di bawah), bukan sub-tahapan berurutan
// setelah conveyor berhenti. RefillState sekarang murni soal conveyor2 doang.
enum class RefillState { IDLE, CONVEYOR_RUN, DONE, FAULT, ESTOPPED };
RefillState refillState = RefillState::IDLE;
uint32_t stateEnteredAt = 0;
bool refillRequested = false;

// BARU: pemisah MAIN/TEST -- default FALSE (fail-safe, boot-IDLE). Cmd::START_MAIN/STOP_MAIN
// (Orange Pi/master doang yang boleh ubah ini) nentuin node lagi mode produksi otomatis (MAIN)
// atau siap buat testing manual (TEST). Lihat guard di applyCommand() & handleMiddleSensor().
bool mainModeActive = false;

// DIUBAH TOTAL: trajectory servo STEP-RATE based (dulu duration-based -- bisa "mundur sebelum
// sampai" kalau Interval di-set lebih kecil dari kecepatan fisik servo yang sebenarnya sanggup).
// SAMA konsep dgn hopper SORTER/PICKER: tiap tick majukan posisi SEBESAR stepUs menuju target.
// Selesai ditentukan POSISI (currentUs==target), bukan waktu abis. Servo1=channel PCA9685 0,
// Servo2=channel 1.
void pcaSetServoUs(uint8_t channel, uint16_t us);   // forward declaration -- didefinisikan di bawah (Test Modul)
constexpr uint8_t SERVO1_CH = 0, SERVO2_CH = 1;
uint16_t servo1CurrentUs = 1000, servo2CurrentUs = 1000;
uint32_t servo1LastStepMs = 0, servo2LastStepMs = 0;

// Reset timer tick -- dipanggil saat MULAI gerakan baru, spy tick pertama gerakan baru nunggu
// 1 stepInterval penuh dulu (bukan langsung loncat). Param fromUs sudah TIDAK dipakai lagi
// (step-based gak butuh referensi "titik awal" terpisah, currentUs SENDIRI itu titik awalnya) --
// tetap dipertahankan di signature supaya semua caller lama gak perlu diubah.
void startServoMove(uint8_t which, uint16_t /*fromUs*/) {
  if (which == 1) servo1LastStepMs = millis();
  else servo2LastStepMs = millis();
}
bool updateServoTrajectory(uint8_t which, uint16_t targetUs) {
  uint16_t &currentUs     = (which == 1) ? servo1CurrentUs : servo2CurrentUs;
  uint32_t &lastStepMs    = (which == 1) ? servo1LastStepMs : servo2LastStepMs;
  uint16_t stepUs         = (which == 1) ? cfg.servo1StepUs : cfg.servo2StepUs;
  uint16_t stepIntervalMs = (which == 1) ? cfg.servo1StepIntervalMs : cfg.servo2StepIntervalMs;
  uint8_t ch              = (which == 1) ? SERVO1_CH : SERVO2_CH;
  if (currentUs == targetUs) return true;
  // BARU: stepIntervalMs=0 = "opsional/tanpa jeda" -- step tiap loop tick, secepat mungkin.
  if (stepIntervalMs > 0 && millis() - lastStepMs < stepIntervalMs) return false;
  lastStepMs = millis();
  int32_t diff = (int32_t)targetUs - (int32_t)currentUs;
  int16_t step = (abs(diff) < (int32_t)stepUs) ? (int16_t)diff : (diff > 0 ? (int16_t)stepUs : -(int16_t)stepUs);
  currentUs = (uint16_t)constrain((int32_t)currentUs + step, 500, 2500);
  pcaSetServoUs(ch, currentUs);
  return (currentUs == targetUs);
}

// BARU: guard mutual-exclusion -- forward declaration, definisi lengkap di bawah (butuh
// servoRefillStage yang dideklarasikan belakangan di file ini). DIPERLUKAN karena refill
// otomatis dari handleMiddleSensor() TIDAK PERNAH mengubah currentState (jalan diam-diam di
// servoRefillStage terpisah) -- guard currentState==IDLE SAJA gak cukup, servo bisa "direbut"
// command manual (TEST_SERVOx_CYCLE/MOVE_SERVOx_TO) di tengah refill otomatis jalan.
bool isServoRefillAutoActive();

// BARU: Test Sequence -- 1x siklus maju-mundur (Titik Awal->Titik Akhir->tahan->Titik Awal)
// PAKAI NILAI KALIBRASI, dipicu dari LCD Test Command ATAU langsung dari Orange Pi (opcode).
// State TERPISAH dari RefillState produksi -- tidak boleh bentrok, makanya WAJIB currentState==IDLE
// DAN servoRefillStage==NONE (lihat isServoRefillAutoActive()).
enum class TestServoCycleStage { NONE, TO_END, HOLD, TO_START };
TestServoCycleStage testServoCycleStage = TestServoCycleStage::NONE;
uint8_t testServoCycleWhich = 0;   // 1 atau 2
uint32_t testServoCycleStateAt = 0;
// DIPERBAIKI (bug ditemukan): layar kalibrasi "Servo1/2 Interval(ms)" sebelumnya cuma ubah
// angka doang, servo TIDAK PERNAH bergerak buat operator amati efeknya live -- beda sama
// Hopper SORTER yang punya mode test-live otomatis di layar Interval-nya. Sekarang sama pola:
// selama operator ada di layar Interval, cycle TO_END->HOLD->TO_START otomatis LOOP terus
// (bukan berhenti sekali), supaya perubahan Interval langsung kelihatan efeknya.
bool servoIntervalTestMode = false;
uint8_t servoIntervalTestWhich = 0;   // 1 atau 2, servo mana yang lagi di-loop
void startTestServoCycle(uint8_t which) {
  if (currentState != NodeState::IDLE) { Serial.println("[TEST] Servo cycle ditolak -- node sedang tidak IDLE"); return; }
  if (isServoRefillAutoActive()) { Serial.println("[TEST] Servo cycle ditolak -- refill otomatis (PROX_2) lagi pegang servo"); return; }
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
        if (servoIntervalTestMode && servoIntervalTestWhich == testServoCycleWhich) {
          // BARU: mode test-live -- ulangi cycle terus selama operator masih di layar Interval,
          // bukan berhenti sekali. Ini yang bikin "servo tidak bergerak" saat test interval.
          startServoMove(testServoCycleWhich, startUs);
          testServoCycleStage = TestServoCycleStage::TO_END;
          testServoCycleStateAt = millis();
        } else {
          testServoCycleStage = TestServoCycleStage::NONE;
          Serial.printf("[TEST] Servo%u cycle SELESAI\n", testServoCycleWhich);
        }
      }
      break;
    default: break;
  }
}

// BARU: jog manual 1 ARAH (BUKAN round-trip) -- Cmd::MOVE_SERVO1_TO/MOVE_SERVO2_TO. Servo
// gerak ke titik awal ATAU titik akhir (pilih via arg), berhenti DI SITU, gak balik sendiri --
// buat kebutuhan operator masukin package manual pertama kali (buka gerbang, taruh package,
// tutup lagi pakai command terpisah). TERPISAH dari testServoCycleStage produksi/test-cycle.
bool manualServo1Moving = false, manualServo2Moving = false;
uint16_t manualServo1Target = 0, manualServo2Target = 0;

void startMoveServoTo(uint8_t which, bool toEnd) {
  if (currentState != NodeState::IDLE) { Serial.printf("[CMD] MOVE_SERVO%u_TO ditolak -- node sedang tidak IDLE\n", which); return; }
  if (isServoRefillAutoActive()) { Serial.printf("[CMD] MOVE_SERVO%u_TO ditolak -- refill otomatis (PROX_2) lagi pegang servo\n", which); return; }
  uint16_t target = (which == 1)
      ? (toEnd ? cfg.servo1EndUs : cfg.servo1StartUs)
      : (toEnd ? cfg.servo2EndUs : cfg.servo2StartUs);
  startServoMove(which, target);
  if (which == 1) { manualServo1Target = target; manualServo1Moving = true; }
  else { manualServo2Target = target; manualServo2Moving = true; }
  Serial.printf("[CMD] MOVE_SERVO%u_TO -- menuju %s (%uus)\n", which, toEnd ? "titik akhir" : "titik awal", target);
}
void updateManualServoMove() {
  if (manualServo1Moving && updateServoTrajectory(1, manualServo1Target)) manualServo1Moving = false;
  if (manualServo2Moving && updateServoTrajectory(2, manualServo2Target)) manualServo2Moving = false;
}

constexpr int PWM_FREQ = 20000, PWM_RES = 8, LEDC_CH_CONV2 = 0, LEDC_CH_PUSH_TEST = 1;

void motorWrite(uint8_t ain1, uint8_t ain2, bool forward) { io.write(ain1, forward); io.write(ain2, !forward); }

// BARU: jog manual conveyor ON/OFF (Cmd::SET_CONVEYOR_ON_OFF) -- TIDAK terikat sensor apapun,
// murni nyala/mati sesuai perintah. Dipisah dari mekanisme REQUEST_REFILL/FORCE_MIDDLE_REFILL
// yang otomatis berhenti sendiri di sensor tertentu.
// BARU: status conveyor yang TERAKHIR DIPERINTAHKAN. Dicetak di baris [LOOP] supaya saat
// menelusuri masalah "package tidak bergerak" kelihatan langsung apakah belt memang sedang
// disuruh jalan atau tidak -- sebelumnya tidak ada cara tahu selain menebak dari gejala.
bool conveyorManualOn = false;

// BARU (2026-09-20): kapan terakhir kali keadaan motor conveyor BERUBAH. Saat motor mulai
// atau berhenti ada hentakan arus, dan hentakan itu sanggup mengganggu pembacaan I2C --
// gejalanya "conveyor sempat menyala sedikit lalu pembacaan jadi ngaco". Perubahan status
// sensor karena itu tidak diakui selama jendela pendek sesudahnya. Ini TIDAK membuang
// kedatangan yang nyata: package butuh waktu jauh lebih lama untuk menempuh jarak antar
// sensor daripada jendela ini.
uint32_t conveyorBerubahAt = 0;
constexpr uint32_t MOTOR_SETTLE_MS = 250;

inline bool motorBaruBerubah() {
  return (millis() - conveyorBerubahAt) < MOTOR_SETTLE_MS;
}

void setConveyorManual(bool on) {
  if (conveyorManualOn != on) conveyorBerubahAt = millis();
  conveyorManualOn = on;
  if (on) {
    motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, cfg.conveyorDir);
    io.write(CH::DISP_STBY, HIGH);
    ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
  } else {
    ledcWrite(LEDC_CH_CONV2, 0);
    io.write(CH::DISP_STBY, LOW);
  }
}
void enterRefillState(RefillState s) { refillState = s; stateEnteredAt = millis(); }

// ============================================================
// TEST REFILL LOOP -- mode test LOKAL, TERPISAH TOTAL dari state machine produksi
// (RefillState/ServoRefillStage/FORCE_MIDDLE_REFILL) -- gak nyentuh/gak diganggu logic
// itu sama sekali. Tujuan: uji mekanisme fisik TENGAH<->UJUNG tanpa Orange Pi/Modbus,
// cukup 1 tombol fisik (CH::BTN_TEST_BOX_FULL). Retry TANPA BATAS (sengaja gak ada
// fault) -- ini KHUSUS mode test, produksi normal TETAP pakai timeout+FAULT yang sudah ada.
// ============================================================
enum class TestLoopStage { OFF, FILL_CHECK, FILL_SERVO1_TO_END, FILL_SERVO1_HOLD, FILL_SERVO1_TO_START,
                            FILL_SERVO2_TO_END, FILL_SERVO2_HOLD, FILL_SERVO2_TO_START, FILL_RETRY_WAIT,
                            READY, MOVING_TO_END };
TestLoopStage testLoopStage = TestLoopStage::OFF;
uint32_t testLoopStageAt = 0;
bool lastBtnBoxFull = HIGH;

void testLoopStopOutputs() { ledcWrite(LEDC_CH_CONV2, 0); io.write(CH::DISP_STBY, LOW); }

void toggleTestRefillLoop() {
  if (testLoopStage != TestLoopStage::OFF) {
    testLoopStopOutputs();
    testLoopStage = TestLoopStage::OFF;
    Serial.println("[TESTLOOP] Dihentikan");
    return;
  }
  if (mainModeActive) { Serial.println("[TESTLOOP] Ditolak -- MAIN aktif, STOP_MAIN dulu"); return; }
  if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    Serial.println("[TESTLOOP] Ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
    return;
  }
  if (currentState == NodeState::RUNNING_OR_MOVING) {
    Serial.println("[TESTLOOP] Ditolak -- ada siklus produksi lain sedang jalan");
    return;
  }
  Serial.println("[TESTLOOP] AKTIF -- tekan tombol fisik BTN_TEST_BOX_FULL kapan saja utk simulasi 'box penuh'");
  testLoopStage = TestLoopStage::FILL_CHECK;
  testLoopStageAt = millis();
}

void updateTestRefillLoop() {
  if (testLoopStage == TestLoopStage::OFF) return;
  if (io.read(CH::ESTOP) == LOW) {   // safety -- E-stop fisik langsung matikan mode test ini juga
    testLoopStopOutputs();
    testLoopStage = TestLoopStage::OFF;
    return;
  }
  uint32_t elapsed = millis() - testLoopStageAt;
  switch (testLoopStage) {
    case TestLoopStage::FILL_CHECK:
      if (io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW) {
        triggerBuzzerBeep();
        Serial.println("[TESTLOOP] TENGAH sudah terisi -- siap, tekan tombol utk simulasi box penuh");
        testLoopStage = TestLoopStage::READY;
      } else {
        motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, cfg.conveyorDir);
        io.write(CH::DISP_STBY, HIGH);
        ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
        startServoMove(1, cfg.servo1StartUs);
        Serial.println("[TESTLOOP] TENGAH kosong -- conveyor jalan + servo coba jatuhkan package");
        testLoopStage = TestLoopStage::FILL_SERVO1_TO_END;
        testLoopStageAt = millis();
      }
      break;
    case TestLoopStage::FILL_SERVO1_TO_END:
      if (updateServoTrajectory(1, cfg.servo1EndUs)) { testLoopStage = TestLoopStage::FILL_SERVO1_HOLD; testLoopStageAt = millis(); }
      break;
    case TestLoopStage::FILL_SERVO1_HOLD:
      if (elapsed >= cfg.servo1HoldMs) { startServoMove(1, cfg.servo1EndUs); testLoopStage = TestLoopStage::FILL_SERVO1_TO_START; testLoopStageAt = millis(); }
      break;
    case TestLoopStage::FILL_SERVO1_TO_START:
      if (updateServoTrajectory(1, cfg.servo1StartUs)) { startServoMove(2, cfg.servo2StartUs); testLoopStage = TestLoopStage::FILL_SERVO2_TO_END; testLoopStageAt = millis(); }
      break;
    case TestLoopStage::FILL_SERVO2_TO_END:
      if (updateServoTrajectory(2, cfg.servo2EndUs)) { testLoopStage = TestLoopStage::FILL_SERVO2_HOLD; testLoopStageAt = millis(); }
      break;
    case TestLoopStage::FILL_SERVO2_HOLD:
      if (elapsed >= cfg.servo2HoldMs) { startServoMove(2, cfg.servo2EndUs); testLoopStage = TestLoopStage::FILL_SERVO2_TO_START; testLoopStageAt = millis(); }
      break;
    case TestLoopStage::FILL_SERVO2_TO_START:
      if (updateServoTrajectory(2, cfg.servo2StartUs)) {
        if (io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW) {
          testLoopStopOutputs();
          triggerBuzzerBeep();
          Serial.println("[TESTLOOP] TENGAH terkonfirmasi terisi -- siap, tekan tombol utk simulasi box penuh");
          testLoopStage = TestLoopStage::READY;
        } else {
          Serial.println("[TESTLOOP] TENGAH masih kosong -- tunggu retry interval, coba lagi");
          testLoopStage = TestLoopStage::FILL_RETRY_WAIT;
          testLoopStageAt = millis();
        }
      }
      break;
    case TestLoopStage::FILL_RETRY_WAIT:
      if (io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW) {
        testLoopStopOutputs();
        triggerBuzzerBeep();
        Serial.println("[TESTLOOP] TENGAH terkonfirmasi terisi -- siap, tekan tombol utk simulasi box penuh");
        testLoopStage = TestLoopStage::READY;
      } else if (elapsed >= cfg.testLoopRetryIntervalMs) {
        startServoMove(1, cfg.servo1StartUs);
        Serial.println("[TESTLOOP] Retry -- coba jatuhkan lagi");
        testLoopStage = TestLoopStage::FILL_SERVO1_TO_END;
        testLoopStageAt = millis();
      }
      break;
    case TestLoopStage::READY: {
      bool cur = io.read(CH::BTN_TEST_BOX_FULL);
      if (cur == LOW && lastBtnBoxFull == HIGH) {
        Serial.println("[TESTLOOP] Tombol ditekan -- simulasi box PENUH, conveyor jalan ke UJUNG");
        motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, cfg.conveyorDir);
        io.write(CH::DISP_STBY, HIGH);
        ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
        testLoopStage = TestLoopStage::MOVING_TO_END;
        testLoopStageAt = millis();
      }
      lastBtnBoxFull = cur;
      break;
    }
    case TestLoopStage::MOVING_TO_END:
      if (io.read(CH::PROX_BOX_ARRIVED) == LOW) {
        testLoopStopOutputs();
        triggerBuzzerBeep();
        Serial.println("[TESTLOOP] Package sampai UJUNG -- siap diambil arm (simulasi). Loop ulang, cek TENGAH lagi.");
        testLoopStage = TestLoopStage::FILL_CHECK;
        testLoopStageAt = millis();
      } else if (elapsed > 8000) {
        testLoopStopOutputs();
        Serial.println("[TESTLOOP] !! Package tidak sampai UJUNG dalam 8 detik -- cek sensor/wiring. Loop ulang, cek TENGAH lagi.");
        testLoopStage = TestLoopStage::FILL_CHECK;
        testLoopStageAt = millis();
      }
      break;
    default: break;
  }
}

// --- Menu state (dideklarasikan awal, dipakai onCmdWrite) ---
enum class MenuState { NONE, TOP_SELECT, CAL_LIST, JOG_PARAM,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_CATEGORY, TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
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

// DIUBAH TOTAL: mekanisme 2-proximity, TANPA limit switch sama sekali. PROX_BOX_ARRIVED
// (PROX_1, UJUNG) = package FULL sampai posisi ambil arm. PACKAGE_MIDDLE_SENSOR (PROX_2,
// TENGAH) = ada/tidaknya package di titik isi.
// Servo refill SEKARANG di-trigger PACKAGE_MIDDLE_SENSOR (LOW->HIGH = package baru saja
// ninggalin tengah krn didorong conveyor ke ujung) -- SENSOR-CONFIRMED, bukan delay tebakan lagi
// (servoStartDelayMs pensiun). Servo refill dideklarasikan SEBELUM handleRefillFSM/handleMiddleSensor
// karena dipakai di keduanya.
enum class ServoRefillStage { NONE, SERVO1_TO_END, SERVO1_HOLD, SERVO1_TO_START,
                               SERVO2_TO_END, SERVO2_HOLD, SERVO2_TO_START, WAIT_MIDDLE_CONFIRM };
ServoRefillStage servoRefillStage = ServoRefillStage::NONE;
uint32_t servoRefillStageAt = 0;

// BARU: definisi lengkap forward-declaration di atas -- dipakai guard mutual-exclusion
// TEST_SERVOx_CYCLE/MOVE_SERVOx_TO, cegah bentrok rebutan servo sama refill otomatis PROX_2.
bool isServoRefillAutoActive() { return servoRefillStage != ServoRefillStage::NONE; }

void handleRefillFSM() {
  uint32_t elapsed = millis() - stateEnteredAt;
  switch (refillState) {
    // DIPERBAIKI (bug ditemukan): semua guard (FAULT/ESTOPPED/PACKAGE_READY_FLAG) dipindah ke
    // applyCommand() -- REQUEST_REFILL SEKARANG cuma nyampe sini kalau memang SUDAH pasti mau
    // dieksekusi (refillRequested cuma true kalau lolos semua guard). Case ini murni eksekusi,
    // gak ada penolakan/penahanan diam-diam lagi.
    case RefillState::IDLE:
      if (refillRequested) {
        refillRequested = false;
        io.write(CH::CONV2_BIN1, false); io.write(CH::CONV2_BIN2, false);
        io.write(CH::DISP_STBY, HIGH);
        enterRefillState(RefillState::CONVEYOR_RUN);
        motorWriteDoneForState = false;   // state baru -- izinkan 1x tulis motorWrite di case berikutnya
      }
      break;
    case RefillState::CONVEYOR_RUN:
      if (!motorWriteDoneForState) { motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, cfg.conveyorDir); motorWriteDoneForState = true; }
      ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
      // PROX_BOX_ARRIVED (PROX_1, UJUNG) = package FULL sampai, siap diambil arm -- set
      // PACKAGE_READY_FLAG, Orange Pi WAJIB ACK (Cmd::ACK_PACKAGE_TAKEN) sebelum refill berikutnya.
      if (io.read(CH::PROX_BOX_ARRIVED) == LOW) {
        ledcWrite(LEDC_CH_CONV2, 0);
        mb.Hreg(Reg::PACKAGE_READY_FLAG, 1);
        Serial.println("[FEEDER] Package sampai UJUNG -- PACKAGE_READY_FLAG=1, siap diambil arm");
        enterRefillState(RefillState::DONE);
      }
      else if (elapsed > 8000) { ledcWrite(LEDC_CH_CONV2, 0); faultCode = (uint16_t)FaultCode::BOX_NOT_ARRIVED; enterRefillState(RefillState::FAULT); }
      break;
    case RefillState::DONE:
      io.write(CH::DISP_STBY, LOW);
      enterRefillState(RefillState::IDLE);
      Serial.println("[FEEDER] Conveyor SELESAI (servo refill nunggu PACKAGE_MIDDLE_SENSOR kosong)");
      break;
    case RefillState::FAULT: case RefillState::ESTOPPED:
      ledcWrite(LEDC_CH_CONV2, 0);
      io.write(CH::DISP_STBY, LOW);
      if (refillState == RefillState::FAULT) currentState = NodeState::FAULT;
      break;
  }
}

// BARU: pantau PACKAGE_MIDDLE_SENSOR (PROX_2) terus-menerus, TERPISAH dari RefillState conveyor.
// LOW->HIGH = package yang tadi duduk di tengah baru saja pergi (didorong conveyor ke ujung) --
// itu sinyal "tengah sekarang kosong, aman jatuhin package baru" -- trigger servo LANGSUNG,
// sensor-confirmed, bukan delay tebakan (servoStartDelayMs, pensiun).
bool lastMiddleSensorState = HIGH;
// BARU: counter latch buat MIDDLE_ARRIVAL_COUNT -- lihat komentar di registers.h. uint16_t
// SAMA lebar dgn register Modbus, wrap alami, gak perlu penanganan khusus.
uint16_t middleArrivalCount = 0;

// ============================================================
// DIPERBAIKI TOTAL (2026-09-20) — cara debounce-nya salah, bukan cuma angkanya.
//
// Gejala yang dilaporkan: "kalau servo masih cycle sedangkan package sudah melewati prox,
// tidak dianggap trigger -- prox baru trigger setelah proses refill selesai, padahal
// package-nya sudah lewat sensor."
//
// Versi lama BUKAN debounce, melainkan PEMBATAS LAJU: perubahan hanya boleh di-commit kalau
// sudah lewat 2 detik dari perubahan yang di-commit sebelumnya. Akibatnya dua-duanya buruk:
//
//   a) TERLAMBAT. Package yang datang kurang dari 2 detik setelah package sebelumnya pergi
//      baru dilaporkan setelah jatah 2 detik itu habis -- persis "baru trigger setelah
//      refill selesai" yang dilihat di lapangan.
//   b) HILANG SAMA SEKALI. Kalau package lewat cepat (hadir < 2 detik), kedatangan TIDAK
//      pernah di-commit, lalu saat package pergi nilainya sudah sama dengan yang terakhir
//      dilaporkan -- jadi tidak ada yang tertulis sama sekali. Satu package penuh lenyap
//      dari register, bukan cuma telat.
//
// Versi baru = debounce yang sebenarnya: sebuah nilai baru dianggap SAH setelah bertahan
// stabil selama MIDDLE_DEBOUNCE_MS. Tidak ada jendela buta. Setiap kedatangan yang benar-benar
// terjadi pasti terlaporkan, cuma tertunda selama waktu stabilisasi itu saja.
//
// Kenapa 2 detik boleh turun jauh ke 200 ms tanpa memunculkan lagi masalah "package tidak
// rata kebaca dua kali": jaminannya sekarang BUKAN dari lamanya waktu, tapi dari SYARAT
// URUTAN -- sebuah kedatangan baru hanya dihitung kalau sensor sempat kembali KOSONG secara
// stabil lebih dulu. Permukaan package yang tidak rata membuat sensor berkedip jauh lebih
// singkat dari 200 ms, jadi tetap tersaring, sementara celah antar package yang nyata selalu
// jauh lebih panjang. Ini justru lebih ketat daripada sekadar menunggu 2 detik.
// ============================================================
// DIUBAH (2026-09-20): debounce dibuat ASIMETRIS -- dua arah tepi punya tuntutan yang
// berlawanan, jadi memakai satu angka untuk keduanya selalu merugikan salah satunya.
//
//   DATANG (HIGH->LOW) menentukan JARAK BERHENTI. Sejak conveyor dihentikan oleh firmware,
//   angka ini praktis satu-satunya sisa keterlambatan. Package terus melaju selama firmware
//   masih menunggu kepastian, jadi makin kecil makin dekat berhentinya. 100 ms.
//
//   PERGI (LOW->HIGH) tidak memengaruhi jarak berhenti sama sekali -- tidak ada yang perlu
//   dihentikan saat package meninggalkan sensor. Justru DI SINILAH perlindungan terhadap
//   "package tidak rata" berada: celah pada permukaan package bisa terbaca sesaat sebagai
//   "package sudah pergi", dan kalau itu diterima, package yang sama akan terhitung DATANG
//   dua kali. Jadi arah ini sengaja dibuat lambat//konservatif. 600 ms.
//
// Hasilnya berhenti lebih dekat DAN lebih tahan noise sekaligus -- bukan tukar-tambah.
// Kalau masih terlalu jauh, turunkan DATANG dulu (50 ms masih wajar); kalau satu package
// mulai terhitung dua kali, naikkan PERGI, bukan DATANG.
constexpr uint32_t MIDDLE_DATANG_DEBOUNCE_MS = 100;
constexpr uint32_t MIDDLE_PERGI_DEBOUNCE_MS  = 600;

// ============================================================
// BARU (2026-09-20) — pembacaan sensor yang tahan gangguan bus I2C.
//
// Dugaan yang sedang diuji: "kalau servo sedang bergerak, PROX_1/PROX_2 tidak dianggap."
// Mekanisme yang mungkin: servo (PCA9685) dan kedua proximity (MCP23017 chip ke-2) memakai
// SATU bus I2C yang sama, berjalan 400 kHz. Tiap langkah servo menulis ke PCA9685. Kalau
// sebuah transaksi baca MCP terganggu di tengah keramaian itu, Adafruit_MCP23X17 tidak
// melaporkan kegagalan -- ia mengembalikan nilai begitu saja, dan yang keluar cenderung HIGH.
// Sensor ini aktif-LOW, jadi HIGH berarti "tidak ada package". Satu pembacaan meleset sudah
// cukup untuk menghapus kedatangan yang sebenarnya terjadi.
//
// Penanganannya: pembacaan hanya dipercaya kalau KONSISTEN. Selama nilainya sama dengan
// sampel sebelumnya, cukup satu kali baca (murah). Begitu nilainya BERUBAH -- justru saat
// yang menentukan -- perubahan itu diverifikasi dengan pembacaan ulang. Kalau kedua
// pembacaan tidak sepakat, perubahan itu dibuang dan dicatat sebagai anomali.
//
// Ini sekaligus alat ukur: kalau sensorAnomalyCount melonjak tepat ketika servo bergerak,
// dugaan di atas terbukti. Kalau tetap nol sementara gejalanya masih ada, berarti bukan bus
// I2C penyebabnya dan kita harus mencari ke arah lain.
// ============================================================
uint16_t sensorAnomalyCount = 0;

bool bacaSensorAndal(uint8_t channel, bool sampelSebelumnya) {
  bool v = io.read(channel);
  if (v == sampelSebelumnya) return v;         // tidak berubah -- tidak perlu diverifikasi
  bool v2 = io.read(channel);                   // berubah -- pastikan sekali lagi
  if (v2 == v) return v;                        // dua-duanya sepakat, perubahan sah
  sensorAnomalyCount++;
  return sampelSebelumnya;                      // tidak sepakat -- abaikan, pertahankan nilai lama
}

// Ambang yang berlaku untuk pembacaan mentah `raw`: LOW = sedang menuju "package ada"
// (kedatangan), HIGH = sedang menuju "package tidak ada" (kepergian).
inline uint32_t debounceUntuk(bool raw) {
  return (raw == LOW) ? MIDDLE_DATANG_DEBOUNCE_MS : MIDDLE_PERGI_DEBOUNCE_MS;
}

bool middleRawLast = HIGH;          // pembacaan mentah terakhir
uint32_t middleRawChangedAt = 0;    // kapan pembacaan mentah terakhir berubah
bool middleStable = HIGH;           // nilai yang sudah lolos uji kestabilan (inilah yang dipakai)
uint16_t conveyorAutoStopCount = 0; // berapa kali firmware menghentikan conveyor sendiri karena PROX_2

// BARU (2026-09-20): berapa package yang SUDAH meninggalkan PROX_2 tetapi BELUM sampai di
// PROX_1. Ini "izin" bagi PROX_1 untuk mengakui sebuah kedatangan. Lihat alasannya di
// updateUjungPresentDebounced(). Dibatasi 3 supaya tidak pernah lari liar kalau ada sesuatu
// yang tidak terduga di lapangan.
uint8_t paketMenujuUjung = 0;
uint16_t ujungArrivalCount = 0;     // kedatangan SAH di UJUNG (yang lolos guard urutan)

void handleMiddleSensor() {
  bool raw = bacaSensorAndal(CH::PACKAGE_MIDDLE_SENSOR, middleRawLast);
  if (raw != middleRawLast) { middleRawLast = raw; middleRawChangedAt = millis(); }
  if (raw == middleStable || millis() - middleRawChangedAt < debounceUntuk(raw)) return;
  if (motorBaruBerubah()) return;   // hentakan arus motor -- tunda pengakuan, jangan dibuang

  bool sebelumnya = middleStable;
  middleStable = raw;
  lastMiddleSensorState = raw;   // dipertahankan: dibaca di tempat lain (mis. tampilan STATUS)

  // Register live: SELALU ikut nilai stabil. Tidak ada lagi kondisi "tidak boleh commit".
  mb.Hreg(Reg::MIDDLE_PACKAGE_PRESENT, (middleStable == LOW) ? 1 : 0);

  if (sebelumnya == HIGH && middleStable == LOW) {
    // Package DATANG. Counter latch naik -- tidak di-gate mainModeActive (ini cuma pencatatan
    // pasif tanpa efek fisik) dan tidak lagi di-gate jendela waktu apa pun.
    middleArrivalCount++;
    mb.Hreg(Reg::MIDDLE_ARRIVAL_COUNT, middleArrivalCount);
    Serial.printf("[FEEDER] PROX_2: package DATANG (count=%u)\n", middleArrivalCount);

    // ============================================================
    // BARU (2026-09-20) — CONVEYOR DIHENTIKAN DI SINI, OLEH FIRMWARE.
    //
    // Keluhan yang diperbaiki: "refill masih jadi blok untuk PROX_2. Apapun kondisinya, jika
    // prox trigger maka stop sampai perintah berikutnya. Kalau harus menunggu cycle refill
    // servo selesai dulu, package keburu jauh."
    //
    // Selama ini yang menghentikan conveyor adalah Orange Pi: ia menunggu counter naik, lalu
    // mengirim perintah mati. Masalahnya master tidak selalu sedang memperhatikan -- pada
    // langkah refill ia sibuk mengirim rangkaian perintah servo yang makan beberapa detik.
    // Package yang menyentuh PROX_2 di tengah-tengah itu baru dihentikan setelah seluruh
    // rangkaian selesai, dan saat itu package sudah terlanjur jauh.
    //
    // Tidak ada nilai polling yang bisa memperbaiki ini: selalu ada jeda master + waktu
    // bolak-balik Modbus. Satu-satunya tempat yang bisa bereaksi seketika adalah firmware,
    // karena di sinilah tepi sensor itu terdeteksi. Jadi penghentian dipindah ke sini --
    // terjadi pada milidetik yang sama dengan terdeteksinya kedatangan.
    //
    // "Sampai perintah berikutnya": firmware TIDAK menyalakannya kembali sendiri. Conveyor
    // tetap mati sampai ada SET_CONVEYOR_ON_OFF(1) berikutnya dari master.
    //
    // Dibatasi ke TEST mode (mainModeActive == false) DENGAN SENGAJA. Selama MAIN, alur
    // produksi REQUEST_REFILL sendiri yang mengemudikan conveyor sampai package mencapai
    // UJUNG; kalau firmware ikut mematikannya di tengah, dua pihak akan berebut satu aktuator
    // yang sama -- persis kelas bug yang sudah kita temukan di Sorter (Motor A) dan sengaja
    // kita hindari. Alur produksi belum diubah ke pola pipeline ini.
    //
    // TEST REFILL LOOP juga dikecualikan karena ia mengemudikan conveyor sendiri.
    // ============================================================
    // DIUBAH (2026-09-20): gate !mainModeActive DICABUT, sama seperti di PROX_1. Sempat
    // tertinggal saat penyatuan jalur, dan akibatnya fatal: di produksi package TIDAK akan
    // berhenti di TENGAH, melainkan terus melaju melewati titik pengisian.
    if (testLoopStage == TestLoopStage::OFF) {
      setConveyorManual(false);
      conveyorAutoStopCount++;
      mb.Hreg(Reg::CONVEYOR_AUTOSTOP_COUNT, conveyorAutoStopCount);
      Serial.printf("[FEEDER] >>> CONVEYOR DIHENTIKAN OTOMATIS oleh PROX_2 (autostop=%u). "
                    "Menunggu perintah SET_CONVEYOR_ON_OFF(1) berikutnya.\n", conveyorAutoStopCount);
    }
  } else if (sebelumnya == LOW && middleStable == HIGH) {
    // Package MENINGGALKAN tengah, artinya ia sedang dalam perjalanan menuju UJUNG.
    // Inilah izin yang dipakai PROX_1 untuk mengakui kedatangan -- lihat penjelasan
    // lengkap di updateUjungPresentDebounced().
    if (paketMenujuUjung < 3) paketMenujuUjung++;
    Serial.printf("[FEEDER] PROX_2: package PERGI (tengah kosong, menuju ujung=%u)\n", paketMenujuUjung);
    // DIHAPUS (2026-09-20): dulu di sini ada pemicu otomatis servoRefillStage saat MAIN aktif.
    // Sekarang waktu gerak servo ditentukan oleh pipeline produksi (lihat updatePipeline()),
    // yang menjalankannya ketika conveyor BERHENTI -- bukan saat package sedang berjalan.
    // Kalau pemicu lama dibiarkan, akan ada dua pihak yang memerintah servo yang sama.
  }
}

// DIPERBAIKI (2026-09-20): PROX_1 punya cacat PERSIS SAMA dengan PROX_2 -- pembatas laju,
// bukan debounce. Package yang sampai di UJUNG kurang dari 2 detik setelah perubahan terakhir
// akan dilaporkan terlambat, atau hilang sama sekali kalau sempat pergi lebih dulu. Dipakaikan
// mekanisme yang sama: sebuah nilai dianggap sah setelah stabil MIDDLE_DEBOUNCE_MS.
bool ujungRawLast = HIGH;
uint32_t ujungRawChangedAt = 0;
bool ujungStable = HIGH;

void updateUjungPresentDebounced() {
  // Ambang asimetris yang sama dipakai di sini: kedatangan di UJUNG juga menentukan jarak
  // berhenti (langkah [14] mematikan conveyor begitu PROX_1 tersentuh), sedangkan kepergian
  // tidak menghentikan apa pun tapi rawan salah baca karena permukaan package.
  bool raw = bacaSensorAndal(CH::PROX_BOX_ARRIVED, ujungRawLast);
  if (raw != ujungRawLast) { ujungRawLast = raw; ujungRawChangedAt = millis(); }
  if (raw == ujungStable || millis() - ujungRawChangedAt < debounceUntuk(raw)) return;
  if (motorBaruBerubah()) return;   // hentakan arus motor -- tunda pengakuan, jangan dibuang
  bool sebelumnya = ujungStable;
  ujungStable = raw;
  mb.Hreg(Reg::UJUNG_PACKAGE_PRESENT, (ujungStable == LOW) ? 1 : 0);
  Serial.printf("[FEEDER] PROX_1 (UJUNG): package %s\n", (ujungStable == LOW) ? "SAMPAI" : "PERGI");

  // BARU (2026-09-20) — penghentian seketika di UJUNG, alasannya SAMA PERSIS dengan PROX_2.
  //
  // Keluhan: "saat package siap diambil arm robot, conveyor sempat menyala sedikit sehingga
  // pembacaan jadi ngaco." Sebabnya: yang menghentikan conveyor di langkah [14] adalah Orange
  // Pi. Package sudah menyentuh PROX_1, tetapi belt baru berhenti setelah perintah mati
  // menempuh perjalanan bolak-balik Modbus. Selama jeda itu package terus bergeser melewati
  // titik ambil, dan pembacaan berikutnya tidak lagi mencerminkan posisi sebenarnya.
  //
  // Waktu tempuh perintah itu tidak bisa dihilangkan dari sisi master, berapa pun rapatnya
  // polling. Maka penghentian dipindah ke sini -- pada milidetik yang sama dengan sampainya
  // package. Firmware tidak menyalakannya kembali; belt tetap mati sampai ada
  // SET_CONVEYOR_ON_OFF(1) berikutnya, yaitu setelah arm selesai mengambil.
  //
  // Batasan mode sama dengan PROX_2: hanya berlaku di TEST mode. Selama MAIN, alur produksi
  // REQUEST_REFILL sendiri yang mengemudikan conveyor sampai UJUNG dan menghentikannya di
  // sana, jadi kalau firmware ikut campur akan ada dua pihak merebut satu aktuator.
  if (sebelumnya == HIGH && ujungStable == LOW) {
    // ============================================================
    // GUARD URUTAN (permintaan user, 2026-09-20): kedatangan di PROX_1 hanya diakui kalau
    // memang ada package yang sebelumnya meninggalkan PROX_2.
    //
    // Alasannya fisik: satu-satunya jalan menuju UJUNG adalah melewati TENGAH lebih dulu.
    // Jadi PROX_1 yang menyala tanpa didahului kepergian dari PROX_2 pasti bukan package
    // yang sedang kita ikuti -- bisa tangan operator, bisa benda tersenggol, bisa pantulan
    // sensor. Kalau itu diakui, conveyor berhenti di saat yang salah dan seluruh urutan
    // langkah bergeser.
    //
    // Yang TIDAK dilakukan di sini: register live UJUNG_PACKAGE_PRESENT tetap ditulis apa
    // adanya di atas. Register itu memang laporan keadaan sensor, dan sebaiknya jujur --
    // yang ditahan cuma PENGAKUAN-nya (counter kedatangan sah + autostop).
    // ============================================================
    if (paketMenujuUjung == 0) {
      Serial.println("[FEEDER] PROX_1 DIABAIKAN -- tidak ada package yang tercatat "
                     "meninggalkan PROX_2. Harus lewat TENGAH dulu sebelum diakui sampai UJUNG.");
    } else {
      paketMenujuUjung--;
      ujungArrivalCount++;
      mb.Hreg(Reg::UJUNG_ARRIVAL_COUNT, ujungArrivalCount);
      Serial.printf("[FEEDER] PROX_1: kedatangan SAH (count=%u, sisa menuju ujung=%u)\n",
                    ujungArrivalCount, paketMenujuUjung);
      // DIUBAH (2026-09-20): gate !mainModeActive DICABUT. Penghentian seketika di sensor
      // sekarang berlaku SAMA di TEST maupun produksi -- inilah inti penyatuan dua jalur itu.
      // Dulu semua perbaikan (stop seketika, debounce, guard urutan) cuma hidup di TEST,
      // sementara produksi memakai pembacaan sensor mentah di handleRefillFSM.
      if (testLoopStage == TestLoopStage::OFF) {
        setConveyorManual(false);
        conveyorAutoStopCount++;
        mb.Hreg(Reg::CONVEYOR_AUTOSTOP_COUNT, conveyorAutoStopCount);
        Serial.printf("[FEEDER] >>> CONVEYOR DIHENTIKAN OTOMATIS oleh PROX_1 (autostop=%u). "
                      "Package berhenti di titik ambil.\n", conveyorAutoStopCount);
      }
      // Sinyal ke Orange Pi: package siap diambil arm. Di produksi inilah pemicu MOVE_PACKAGE
      // ke Picker. Dinaikkan DI SINI, pada kedatangan sah, bukan dari hasil polling master.
      if (mainModeActive) {
        mb.Hreg(Reg::PACKAGE_READY_FLAG, 1);
        Serial.println("[FEEDER] PACKAGE_READY_FLAG=1 -- package siap diambil arm robot");
      }
    }
  }
}

// ============================================================
// BARU (2026-09-20): counter LATCH untuk dua tombol fisik yang selama ini menganggur.
//
// Kebutuhannya: pada alur test yang baru, dua konfirmasi TIDAK BOLEH lagi berupa delay/timing
// tebakan -- harus dari kejadian nyata. Konfirmasi itu boleh datang dari Orange Pi (operator
// menekan Enter) ATAU dari tombol fisik di panel:
//   BUTTON_2 = "package sudah PENUH"
//   BUTTON_3 = "package sudah DIAMBIL robot"
//
// Kenapa counter latch, bukan register status biasa: master mem-polling dengan jeda, dan di
// sela-sela itu ada perintah lain yang makan waktu (servo bergerak beberapa detik). Status
// sesaat gampang terlewat; counter tidak pernah kehilangan kejadian. Pola ini SAMA PERSIS
// dengan MIDDLE_ARRIVAL_COUNT yang sudah terbukti untuk PROX_2 -- script cukup membandingkan
// nilai SEBELUM dan SESUDAH menunggu.
//
// BUTTON_2 tetap dipakai juga oleh TEST REFILL LOOP (updateTestRefillLoop) -- keduanya hanya
// MEMBACA pin dengan deteksi tepi masing-masing, jadi tidak saling mengganggu.
//
// Sengaja TIDAK di-gate mainModeActive: counter ini murni pelaporan, tidak menggerakkan
// aktuator apa pun, jadi aman aktif kapan saja.
// ============================================================
// DIPERBAIKI (2026-09-20): versi pertama memakai pola "tepi + jendela waktu" yang PERSIS SAMA
// cacatnya dengan sensor proximity sebelum diperbaiki -- sebuah tepi HIGH->LOW dihitung asal
// sudah lewat 50 ms dari tepi TERAKHIR YANG DIHITUNG. Selama pantulan kontak (bouncing) pin
// bergoyang naik-turun, dan setiap goyangan yang kebetulan jatuh lebih dari 50 ms sesudah
// hitungan sebelumnya ikut terhitung sebagai penekanan BARU.
//
// Terbukti di lapangan: dalam 8 detik BUTTON_2 naik 12 kali dan BUTTON_3 naik 14 kali.
// Akibatnya fatal untuk script: counter berubah terus dengan sendirinya, sehingga langkah
// "tunggu konfirmasi package penuh" langsung lolos tanpa menunggu siapa pun -- persis keluhan
// "package full tidak berfungsi walau sudah ditekan". Tombolnya justru TERLALU sensitif,
// bukan tidak terbaca.
//
// Versi ini memakai debounce kestabilan yang sama dengan sensor: sebuah nilai baru diakui
// setelah bertahan stabil, DAN penekanan berikutnya hanya dihitung kalau tombol sempat
// benar-benar DILEPAS secara stabil lebih dulu. Pantulan tidak bisa lolos lagi, dan tombol
// yang ditahan lama tetap terhitung satu kali.
constexpr uint32_t BTN_TEKAN_DEBOUNCE_MS = 80;    // seberapa lama harus stabil tertekan
constexpr uint32_t BTN_LEPAS_DEBOUNCE_MS = 150;   // seberapa lama harus stabil dilepas

uint16_t btnPackageFullCount = 0, btnPackageTakenCount = 0;

struct TombolDebounce {
  uint8_t channel;
  bool rawLast;
  uint32_t rawChangedAt;
  bool stabil;
  // Konstruktor eksplisit, bukan default member initializer -- proyek ini dikompilasi sebagai
  // C++11, dan di situ struct yang punya NSDMI berhenti menjadi aggregate sehingga tidak bisa
  // diinisialisasi dengan kurung kurawal.
  explicit TombolDebounce(uint8_t ch)
      : channel(ch), rawLast(HIGH), rawChangedAt(0), stabil(HIGH) {}
};

// Mengembalikan true TEPAT SEKALI per penekanan yang sah (transisi stabil lepas -> tekan).
bool tombolDitekan(TombolDebounce &t) {
  bool raw = io.read(t.channel);
  if (raw != t.rawLast) { t.rawLast = raw; t.rawChangedAt = millis(); }
  uint32_t ambang = (raw == LOW) ? BTN_TEKAN_DEBOUNCE_MS : BTN_LEPAS_DEBOUNCE_MS;
  if (raw == t.stabil || millis() - t.rawChangedAt < ambang) return false;
  bool sebelumnya = t.stabil;
  t.stabil = raw;
  return (sebelumnya == HIGH && t.stabil == LOW);
}

TombolDebounce tombolFull(CH::BUTTON_2);
TombolDebounce tombolTaken(CH::BUTTON_3);

void updateKonfirmasiButtons() {
  if (tombolDitekan(tombolFull)) {
    btnPackageFullCount++;
    mb.Hreg(Reg::BTN_PACKAGE_FULL_COUNT, btnPackageFullCount);
    Serial.printf("[TOMBOL] BUTTON_2 ditekan -- 'package PENUH' (count=%u)\n", btnPackageFullCount);
  }
  if (tombolDitekan(tombolTaken)) {
    btnPackageTakenCount++;
    mb.Hreg(Reg::BTN_PACKAGE_TAKEN_COUNT, btnPackageTakenCount);
    Serial.printf("[TOMBOL] BUTTON_3 ditekan -- 'package sudah DIAMBIL' (count=%u)\n", btnPackageTakenCount);
  }
}

// DIUBAH: FORCE_MIDDLE_REFILL -- utk startup/recovery kalau tengah kosong dari awal (gak ada
// edge yg bisa dideteksi handleMiddleSensor). SAMA pola dgn alur produksi LAMA (sebelum
// redesign PROX_2-edge-trigger): conveyor MAJU duluan, tunggu cfg.servoStartDelayMs, BARU
// servo1+servo2 dieksekusi -- BUKAN paralel bareng dari t=0. Field servoStartDelayMs yang
// tadinya pensiun (redesign PROX_2) dipakai lagi KHUSUS di sini. Keduanya (conveyor+servo)
// tuntas/berhenti begitu PACKAGE_MIDDLE_SENSOR = LOW. Timeout fault kalau kelewat batas waktu.
bool forceMiddleRefillActive = false;
bool forceServoScheduled = false;
uint32_t forceMiddleRefillAt = 0;
uint32_t forceServoStartAt = 0;
constexpr uint32_t FORCE_MIDDLE_REFILL_TIMEOUT_MS = 8000;

void startForceMiddleRefill() {
  if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    Serial.println("[CMD] FORCE_MIDDLE_REFILL ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
    return;
  }
  // DIHAPUS: guard LIM_STOCK_EMPTY -- Dispenser tidak punya limit switch deteksi stok habis lagi.
  if (refillState != RefillState::IDLE || servoRefillStage != ServoRefillStage::NONE || forceMiddleRefillActive) {
    Serial.println("[CMD] FORCE_MIDDLE_REFILL ditolak -- masih ada siklus lain jalan");
    return;
  }
  if (testLoopStage != TestLoopStage::OFF) {
    Serial.println("[CMD] FORCE_MIDDLE_REFILL ditolak -- TEST REFILL LOOP sedang aktif, matikan dulu");
    return;
  }
  forceMiddleRefillActive = true;
  forceMiddleRefillAt = millis();
  io.write(CH::CONV2_BIN1, false); io.write(CH::CONV2_BIN2, false);
  io.write(CH::DISP_STBY, HIGH);
  motorWrite(CH::CONV2_BIN1, CH::CONV2_BIN2, cfg.conveyorDir);
  // BARU: servo BELUM langsung jalan -- dijadwalkan cfg.servoStartDelayMs dari sekarang,
  // conveyor jalan duluan sendirian selama jeda itu.
  forceServoScheduled = true;
  forceServoStartAt = millis() + cfg.servoStartDelayMs;
  currentState = NodeState::RUNNING_OR_MOVING;
  Serial.println("[FEEDER] FORCE_MIDDLE_REFILL mulai -- conveyor MAJU duluan, servo nyusul setelah delay");
}

void updateForceMiddleRefill() {
  if (!forceMiddleRefillActive) return;
  ledcWrite(LEDC_CH_CONV2, cfg.conveyorSpeed);
  if (forceServoScheduled && (int32_t)(millis() - forceServoStartAt) >= 0) {
    forceServoScheduled = false;
    if (servoRefillStage == ServoRefillStage::NONE) {
      startServoMove(1, cfg.servo1StartUs);
      servoRefillStage = ServoRefillStage::SERVO1_TO_END;
      servoRefillStageAt = millis();
      Serial.println("[FEEDER] FORCE_MIDDLE_REFILL -- delay abis, servo1+servo2 mulai eksekusi");
    }
  }
  if (io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW) {
    ledcWrite(LEDC_CH_CONV2, 0);
    io.write(CH::DISP_STBY, LOW);
    forceMiddleRefillActive = false;
    forceServoScheduled = false;
    triggerBuzzerBeep();
    Serial.println("[FEEDER] FORCE_MIDDLE_REFILL SELESAI -- tengah terkonfirmasi terisi");
  } else if (millis() - forceMiddleRefillAt > FORCE_MIDDLE_REFILL_TIMEOUT_MS) {
    ledcWrite(LEDC_CH_CONV2, 0);
    io.write(CH::DISP_STBY, LOW);
    forceMiddleRefillActive = false;
    forceServoScheduled = false;
    servoRefillStage = ServoRefillStage::NONE;
    faultCode = (uint16_t)FaultCode::MIDDLE_PACKAGE_MISSING;
    currentState = NodeState::FAULT;
    Serial.println("[FEEDER] !!! FAULT: FORCE_MIDDLE_REFILL timeout -- tengah tetap tidak terisi !!!");
  }
}

// Siklus servo refill (gerbang bawah lalu atas), dipicu handleMiddleSensor() di atas. SETELAH
// gerbang atas selesai jatuhin package baru, TUNGGU PACKAGE_MIDDLE_SENSOR konfirmasi LOW
// (package baru BENERAN nyampe tengah) sebelum dianggap tuntas -- kalau kelewat batas waktu
// (cfg.middleConfirmTimeoutMs), FAULT(MIDDLE_PACKAGE_MISSING) (macet/kehabisan stok tumpukan).
void updateServoRefillStage() {
  if (servoRefillStage == ServoRefillStage::NONE) return;
  uint32_t elapsed = millis() - servoRefillStageAt;
  switch (servoRefillStage) {
    case ServoRefillStage::SERVO1_TO_END:
      if (updateServoTrajectory(1, cfg.servo1EndUs)) { servoRefillStage = ServoRefillStage::SERVO1_HOLD; servoRefillStageAt = millis(); }
      break;
    case ServoRefillStage::SERVO1_HOLD:
      if (elapsed >= cfg.servo1HoldMs) {
        startServoMove(1, cfg.servo1EndUs);
        servoRefillStage = ServoRefillStage::SERVO1_TO_START;
        servoRefillStageAt = millis();
      }
      break;
    case ServoRefillStage::SERVO1_TO_START:
      if (updateServoTrajectory(1, cfg.servo1StartUs)) {
        startServoMove(2, cfg.servo2StartUs);
        servoRefillStage = ServoRefillStage::SERVO2_TO_END;
        servoRefillStageAt = millis();
      }
      break;
    case ServoRefillStage::SERVO2_TO_END:
      if (updateServoTrajectory(2, cfg.servo2EndUs)) { servoRefillStage = ServoRefillStage::SERVO2_HOLD; servoRefillStageAt = millis(); }
      break;
    case ServoRefillStage::SERVO2_HOLD:
      if (elapsed >= cfg.servo2HoldMs) {
        startServoMove(2, cfg.servo2EndUs);
        servoRefillStage = ServoRefillStage::SERVO2_TO_START;
        servoRefillStageAt = millis();
      }
      break;
    case ServoRefillStage::SERVO2_TO_START:
      if (updateServoTrajectory(2, cfg.servo2StartUs)) {
        servoRefillStage = ServoRefillStage::WAIT_MIDDLE_CONFIRM;
        servoRefillStageAt = millis();
      }
      break;
    case ServoRefillStage::WAIT_MIDDLE_CONFIRM:
      if (io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW) {
        servoRefillStage = ServoRefillStage::NONE;
        triggerBuzzerBeep();
        Serial.println("[FEEDER] Servo refill SELESAI -- package baru terkonfirmasi di tengah");
      } else if (elapsed > cfg.middleConfirmTimeoutMs) {
        faultCode = (uint16_t)FaultCode::MIDDLE_PACKAGE_MISSING;
        currentState = NodeState::FAULT;
        servoRefillStage = ServoRefillStage::NONE;
        Serial.println("[FEEDER] !!! FAULT: package baru TIDAK terdeteksi di tengah setelah servo refill !!!");
      }
      break;
    default: break;
  }
}

// BARU: gabungkan 2 cabang paralel (conveyor + servo) -- currentState baru balik IDLE kalau
// KEDUANYA sudah tuntas, supaya REQUEST_REFILL berikutnya (atau Orange Pi yang polling STATE)
// tidak menyangka node sudah bebas padahal servo masih jalan.
// ============================================================
// PIPELINE PRODUKSI (BARU 2026-09-20) — urutan yang sama persis dengan yang sudah diuji
// lewat test-dispenser-pipeline, tapi dijalankan FIRMWARE, bukan script Orange Pi.
//
// Kenapa dipindah ke firmware: dua keputusan di alur ini harus terjadi seketika pada tepi
// sensor -- menghentikan conveyor di TENGAH dan di UJUNG. Master tidak bisa melakukannya
// tepat waktu; selalu ada jeda polling dan perjalanan Modbus, dan selama jeda itu package
// terlanjur bergeser. Semua sudah dibuktikan di lapangan sepanjang pengujian hari ini.
//
// Pembagian tugas jadi tegas:
//   FIRMWARE  : menghentikan conveyor di sensor, menggerakkan servo, menjaga urutan,
//               melaporkan kapan dirinya SIAP dan kapan package siap diambil.
//   ORANGE PI : memutuskan "package penuh" (REQUEST_REFILL) dan "package sudah diambil"
//               (ACK_PACKAGE_TAKEN). Dua itu saja.
//
// Padanan dengan tombol saat pengujian:
//   BUTTON_2 (package penuh)   -> Cmd::REQUEST_REFILL
//   BUTTON_3 (sudah diambil)   -> Cmd::ACK_PACKAGE_TAKEN
// ============================================================
enum class PipelineStage : uint8_t {
  MATI = 0,             // MAIN belum aktif
  INIT_SERVO1_BUKA,     // urutan awal: jatuhkan package pertama
  INIT_SERVO1_TAHAN,
  INIT_SERVO1_TUTUP,
  INIT_SERVO2_BUKA,
  INIT_SERVO2_TAHAN,
  INIT_SERVO2_TUTUP,
  INIT_TUNGGU_PROX2,    // tunggu package pertama sampai di TENGAH
  BUKA_GERBANG,         // servo1 dibuka supaya objek berikutnya bisa jatuh
  SIAP_ISI,             // == READY. Package di TENGAH, menunggu REQUEST_REFILL
  MAJU,                 // conveyor jalan menuju UJUNG
  REFILL_SERVO1_TUTUP,  // package sudah di UJUNG & berhenti -- gerbang dibereskan di sini
  REFILL_SERVO2_BUKA,
  REFILL_SERVO2_TAHAN,
  REFILL_SERVO2_TUTUP,
  TUNGGU_DIAMBIL,       // PACKAGE_READY_FLAG=1, menunggu ACK_PACKAGE_TAKEN
};
PipelineStage pipelineStage = PipelineStage::MATI;
uint32_t pipelineStageAt = 0;
uint16_t pipelineUjungAcuan = 0;   // nilai ujungArrivalCount saat mulai MAJU
constexpr uint32_t PIPELINE_MAJU_TIMEOUT_MS = 20000;

const char* pipelineText(PipelineStage s) {
  switch (s) {
    case PipelineStage::MATI:                return "MATI";
    case PipelineStage::INIT_SERVO1_BUKA:    return "INIT servo1 buka";
    case PipelineStage::INIT_SERVO1_TAHAN:   return "INIT servo1 tahan";
    case PipelineStage::INIT_SERVO1_TUTUP:   return "INIT servo1 tutup";
    case PipelineStage::INIT_SERVO2_BUKA:    return "INIT servo2 buka";
    case PipelineStage::INIT_SERVO2_TAHAN:   return "INIT servo2 tahan";
    case PipelineStage::INIT_SERVO2_TUTUP:   return "INIT servo2 tutup";
    case PipelineStage::INIT_TUNGGU_PROX2:   return "INIT tunggu PROX_2";
    case PipelineStage::BUKA_GERBANG:        return "buka gerbang";
    case PipelineStage::SIAP_ISI:            return "SIAP ISI (ready)";
    case PipelineStage::MAJU:                return "maju ke ujung";
    case PipelineStage::REFILL_SERVO1_TUTUP: return "refill servo1 tutup";
    case PipelineStage::REFILL_SERVO2_BUKA:  return "refill servo2 buka";
    case PipelineStage::REFILL_SERVO2_TAHAN: return "refill servo2 tahan";
    case PipelineStage::REFILL_SERVO2_TUTUP: return "refill servo2 tutup";
    case PipelineStage::TUNGGU_DIAMBIL:      return "tunggu diambil arm";
  }
  return "?";
}

void masukPipeline(PipelineStage s) {
  pipelineStage = s;
  pipelineStageAt = millis();
  mb.Hreg(Reg::PIPELINE_STAGE, (uint16_t)s);
  mb.Hreg(Reg::DISPENSER_READY, (s == PipelineStage::SIAP_ISI) ? 1 : 0);
  Serial.printf("[PIPELINE] -> %s\n", pipelineText(s));
}

// Dipanggil Cmd::START_MAIN. Kalau TENGAH sudah terisi, urutan awal dilewati -- tidak perlu
// menjatuhkan package baru ke tempat yang sudah ada isinya.
void mulaiPipeline() {
  paketMenujuUjung = 0;
  mb.Hreg(Reg::PACKAGE_READY_FLAG, 0);
  if (middleStable == LOW) {
    Serial.println("[PIPELINE] TENGAH sudah terisi -- urutan awal dilewati");
    startServoMove(1, cfg.servo1EndUs);
    masukPipeline(PipelineStage::BUKA_GERBANG);
  } else {
    Serial.println("[PIPELINE] TENGAH kosong -- jalankan urutan awal sampai package siap di TENGAH");
    setConveyorManual(true);
    startServoMove(1, cfg.servo1EndUs);
    masukPipeline(PipelineStage::INIT_SERVO1_BUKA);
  }
}

void hentikanPipeline(const char* alasan) {
  if (pipelineStage == PipelineStage::MATI) return;
  setConveyorManual(false);
  mb.Hreg(Reg::PACKAGE_READY_FLAG, 0);
  masukPipeline(PipelineStage::MATI);
  Serial.printf("[PIPELINE] dihentikan -- %s\n", alasan);
}

bool pipelineRefillDiminta = false;   // di-set Cmd::REQUEST_REFILL
bool pipelineAckDiterima = false;     // di-set Cmd::ACK_PACKAGE_TAKEN

void updatePipeline() {
  if (pipelineStage == PipelineStage::MATI) return;
  if (currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    hentikanPipeline("node FAULT/ESTOPPED");
    return;
  }
  uint32_t elapsed = millis() - pipelineStageAt;

  switch (pipelineStage) {
    // --- urutan awal: isi package pertama sampai duduk di TENGAH ---
    case PipelineStage::INIT_SERVO1_BUKA:
      if (updateServoTrajectory(1, cfg.servo1EndUs)) masukPipeline(PipelineStage::INIT_SERVO1_TAHAN);
      break;
    case PipelineStage::INIT_SERVO1_TAHAN:
      if (elapsed >= cfg.servo1HoldMs) { startServoMove(1, cfg.servo1StartUs); masukPipeline(PipelineStage::INIT_SERVO1_TUTUP); }
      break;
    case PipelineStage::INIT_SERVO1_TUTUP:
      if (updateServoTrajectory(1, cfg.servo1StartUs)) { startServoMove(2, cfg.servo2EndUs); masukPipeline(PipelineStage::INIT_SERVO2_BUKA); }
      break;
    case PipelineStage::INIT_SERVO2_BUKA:
      if (updateServoTrajectory(2, cfg.servo2EndUs)) masukPipeline(PipelineStage::INIT_SERVO2_TAHAN);
      break;
    case PipelineStage::INIT_SERVO2_TAHAN:
      if (elapsed >= cfg.servo2HoldMs) { startServoMove(2, cfg.servo2StartUs); masukPipeline(PipelineStage::INIT_SERVO2_TUTUP); }
      break;
    case PipelineStage::INIT_SERVO2_TUTUP:
      if (updateServoTrajectory(2, cfg.servo2StartUs)) masukPipeline(PipelineStage::INIT_TUNGGU_PROX2);
      break;
    case PipelineStage::INIT_TUNGGU_PROX2:
      // Conveyor sudah dimatikan sendiri oleh autostop PROX_2 begitu package sampai.
      if (middleStable == LOW) {
        startServoMove(1, cfg.servo1EndUs);
        masukPipeline(PipelineStage::BUKA_GERBANG);
      }
      else if (elapsed > 30000) {
        faultCode = (uint16_t)FaultCode::MIDDLE_PACKAGE_MISSING;
        currentState = NodeState::FAULT;
        hentikanPipeline("package pertama tidak pernah sampai di TENGAH dalam 30 detik");
      }
      break;

    // --- gerbang dibuka supaya objek berikutnya bisa jatuh, lalu menyatakan diri SIAP ---
    case PipelineStage::BUKA_GERBANG:
      if (updateServoTrajectory(1, cfg.servo1EndUs)) {
        masukPipeline(PipelineStage::SIAP_ISI);
        Serial.println("[PIPELINE] SIAP menampung objek -- menunggu REQUEST_REFILL dari Orange Pi");
      }
      break;

    case PipelineStage::SIAP_ISI:
      if (pipelineRefillDiminta) {
        pipelineRefillDiminta = false;
        pipelineUjungAcuan = ujungArrivalCount;
        setConveyorManual(true);
        masukPipeline(PipelineStage::MAJU);
      }
      break;

    case PipelineStage::MAJU:
      // Conveyor dihentikan oleh autostop PROX_1, dan PACKAGE_READY_FLAG ikut dinyalakan
      // di sana. Di sini cukup menunggu kedatangan sah itu tercatat.
      if (ujungArrivalCount != pipelineUjungAcuan) {
        startServoMove(1, cfg.servo1StartUs);
        masukPipeline(PipelineStage::REFILL_SERVO1_TUTUP);
      } else if (elapsed > PIPELINE_MAJU_TIMEOUT_MS) {
        // Sebab paling mungkin dibedakan di sini, supaya tidak berakhir sebagai fault umum
        // yang tidak menjelaskan apa-apa. Kalau conveyor sudah mati padahal package belum
        // sampai UJUNG, berarti autostop PROX_2 yang menghentikannya: package BERIKUTNYA
        // sampai di TENGAH lebih dulu daripada package ini sampai di UJUNG. Itu persoalan
        // JARAK FISIK antar sensor dibanding jarak antar package, bukan kesalahan program.
        if (!conveyorManualOn && middleStable == LOW) {
          Serial.println("[PIPELINE] !! Package berikutnya sampai di TENGAH sebelum package ini "
                         "sampai di UJUNG, sehingga conveyor keburu berhenti.");
          Serial.println("[PIPELINE] !! Jarak PROX_1-PROX_2 tidak cocok dengan jarak antar package. "
                         "Urutan langkah perlu disesuaikan, bukan sekadar diulang.");
        }
        faultCode = (uint16_t)FaultCode::BOX_NOT_ARRIVED;
        currentState = NodeState::FAULT;
        hentikanPipeline("package tidak sampai UJUNG dalam batas waktu");
      }
      break;

    // --- gerbang dibereskan SELAGI package menunggu diambil (conveyor diam) ---
    case PipelineStage::REFILL_SERVO1_TUTUP:
      if (updateServoTrajectory(1, cfg.servo1StartUs)) { startServoMove(2, cfg.servo2EndUs); masukPipeline(PipelineStage::REFILL_SERVO2_BUKA); }
      break;
    case PipelineStage::REFILL_SERVO2_BUKA:
      if (updateServoTrajectory(2, cfg.servo2EndUs)) masukPipeline(PipelineStage::REFILL_SERVO2_TAHAN);
      break;
    case PipelineStage::REFILL_SERVO2_TAHAN:
      if (elapsed >= cfg.servo2HoldMs) { startServoMove(2, cfg.servo2StartUs); masukPipeline(PipelineStage::REFILL_SERVO2_TUTUP); }
      break;
    case PipelineStage::REFILL_SERVO2_TUTUP:
      if (updateServoTrajectory(2, cfg.servo2StartUs)) masukPipeline(PipelineStage::TUNGGU_DIAMBIL);
      break;

    case PipelineStage::TUNGGU_DIAMBIL:
      if (pipelineAckDiterima) {
        pipelineAckDiterima = false;
        mb.Hreg(Reg::PACKAGE_READY_FLAG, 0);
        setConveyorManual(true);
        // Package berikutnya sedang menuju TENGAH; autostop PROX_2 yang akan
        // menghentikannya. Dari sini kembali ke pola buka gerbang -> SIAP ISI.
        masukPipeline(PipelineStage::INIT_TUNGGU_PROX2);
      }
      break;

    default: break;
  }
}

void updateRefillJoin() {
  if (currentState == NodeState::RUNNING_OR_MOVING &&
      refillState == RefillState::IDLE &&
      servoRefillStage == ServoRefillStage::NONE &&
      !forceMiddleRefillActive) {
    currentState = NodeState::IDLE;
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH dari HIGH ke LOW
    currentState = NodeState::ESTOPPED;
    enterRefillState(RefillState::ESTOPPED);
    // BARU: reset tracking servo refill juga -- cegah siklus lama "nyangkut" nge-resume aneh
    // begitu E-stop dilepas (servo diam di posisi terakhir, TIDAK otomatis lanjut).
    servoRefillStage = ServoRefillStage::NONE;
    forceMiddleRefillActive = false;
    forceServoScheduled = false;
    ledcWrite(LEDC_CH_CONV2, 0);
    return;
  }
  if (currentState == NodeState::ESTOPPED) {
    currentState = NodeState::IDLE;
    enterRefillState(RefillState::IDLE);
  }
}

// ============================================================
// OTA (WiFi) -- update firmware tanpa colok-cabut USB. Lihat wifi_credentials.h
// (gitignored, isi asli SSID/password/OTA password per file .example di include/).
// ============================================================
bool otaReady = false;   // true == WiFi connect & ArduinoOTA siap terima update sekarang
bool otaBegun = false;   // ArduinoOTA.begin() sudah dipanggil sekali (callback ter-register)

// Dipanggil dari ArduinoOTA.onStart() -- proses tulis flash BLOCKING selama beberapa detik.
// Servo PCA9685 aman (chip I2C eksternal, tahan posisi sendiri walau ESP32 sibuk), tapi
// conveyor (LEDC hardware PWM native) tetap jalan otonom kalau tidak dipaksa berhenti dulu.
void otaSafeStop() {
  ledcWrite(LEDC_CH_CONV2, 0);
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

// DIUBAH TOTAL: conveyor (refillState) & servo (servoRefillStage) sekarang PARALEL, jadi teks
// aktivitas harus GABUNGKAN keduanya -- bukan lagi 1:1 dari 1 enum aja.
const char* servoRefillStageText() {
  switch (servoRefillStage) {
    case ServoRefillStage::SERVO1_TO_END:   return "Srv1Buka";
    case ServoRefillStage::SERVO1_HOLD:     return "Srv1Tahan";
    case ServoRefillStage::SERVO1_TO_START: return "Srv1Tutup";
    case ServoRefillStage::SERVO2_TO_END:   return "Srv2Buka";
    case ServoRefillStage::SERVO2_HOLD:     return "Srv2Tahan";
    case ServoRefillStage::SERVO2_TO_START: return "Srv2Tutup";
    case ServoRefillStage::WAIT_MIDDLE_CONFIRM: return "TungguTengah";
    default: return nullptr;
  }
}
String activityText() {
  if (refillState == RefillState::FAULT) return "FAULT!";
  if (refillState == RefillState::ESTOPPED) return "E-STOP!";
  bool convOn = (refillState == RefillState::CONVEYOR_RUN);
  const char* srv = servoRefillStageText();
  if (convOn && srv) return "Conv+" + String(srv);
  if (convOn) return "Conveyor ON";
  if (srv) return String(srv);
  return "Diam";
}

// BARU: versi kode numerik -- dikirim ke register Modbus (Lapis 2). Servo diprioritaskan
// (lebih spesifik) kalau keduanya lagi jalan barengan -- STATE (RUNNING_OR_MOVING) tetap
// nunjukin node sedang sibuk terlepas dari detail aktivitas mana yang kepilih di sini.
ActivityCode activityCode() {
  // BARU: TEST REFILL LOOP prioritas tertinggi -- biar Orange Pi/skrip remote bisa TAU
  // kenapa command lain (SET_CONVEYOR_ON_OFF dkk) ditolak, tanpa perlu akses Serial USB device.
  if (testLoopStage != TestLoopStage::OFF) return ActivityCode::TEST_LOOP_AKTIF;
  switch (servoRefillStage) {
    case ServoRefillStage::SERVO1_TO_END:   return ActivityCode::SERVO1_BUKA;
    case ServoRefillStage::SERVO1_HOLD:     return ActivityCode::SERVO1_TAHAN;
    case ServoRefillStage::SERVO1_TO_START: return ActivityCode::SERVO1_TUTUP;
    case ServoRefillStage::SERVO2_TO_END:   return ActivityCode::SERVO2_BUKA;
    case ServoRefillStage::SERVO2_HOLD:     return ActivityCode::SERVO2_TAHAN;
    case ServoRefillStage::SERVO2_TO_START: return ActivityCode::SERVO2_TUTUP;
    case ServoRefillStage::WAIT_MIDDLE_CONFIRM: return ActivityCode::TUNGGU_KONFIRM_TENGAH;
    default: break;
  }
  // BARU (bug ditemukan): testServoCycleStage (dipicu Cmd::TEST_SERVO1/2_CYCLE via Modbus,
  // ATAU layar kalibrasi Servo Step/Interval) SEBELUMNYA gak pernah dicek di sini sama
  // sekali -- ACTIVITY_CODE tetap DIAM(0) walau servo BENERAN lagi gerak, bikin diagnosa
  // jarak jauh (tanpa Serial USB) keliatan "gak jalan" padahal firmware jalan normal. Reuse
  // kode ActivityCode SERVO1/2_BUKA/TAHAN/TUTUP yang sama -- aksi fisiknya identik, cuma beda
  // pemicu (test manual vs auto-refill), dan keduanya SALING EKSKLUSIF (lihat
  // isServoRefillAutoActive() -- gak akan dua-duanya aktif bareng).
  if (testServoCycleStage != TestServoCycleStage::NONE) {
    bool isServo1 = (testServoCycleWhich == 1);
    switch (testServoCycleStage) {
      case TestServoCycleStage::TO_END:   return isServo1 ? ActivityCode::SERVO1_BUKA  : ActivityCode::SERVO2_BUKA;
      case TestServoCycleStage::HOLD:     return isServo1 ? ActivityCode::SERVO1_TAHAN : ActivityCode::SERVO2_TAHAN;
      case TestServoCycleStage::TO_START: return isServo1 ? ActivityCode::SERVO1_TUTUP : ActivityCode::SERVO2_TUTUP;
      default: break;
    }
  }
  switch (refillState) {
    case RefillState::IDLE:         return ActivityCode::DIAM;
    case RefillState::CONVEYOR_RUN: return ActivityCode::CONVEYOR_JALAN;
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
  // BARU (2026-09-20): LED_FAULT. Channel ini di-pinMode OUTPUT di setup() dan terdaftar di
  // menu Test Output, TAPI tidak pernah ditulis satu kali pun secara otomatis -- lampu fault
  // praktis mati permanen selama produksi di KEEMPAT node. Sekarang ikut dikelola di sini.
  static bool lastLedFault = false;
  bool ledFaultNow = (currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED);
  if (ledFaultNow != lastLedFault) { io.write(CH::LED_FAULT, ledFaultNow); lastLedFault = ledFaultNow; }
}

// DIPERBAIKI: guard FAULT/ESTOPPED -- REQUEST_REFILL sebelumnya TIDAK dicek sama sekali
// (kelas bug sama dgn SORTER/PICKER/STOCKER yang sudah ditemukan)
void applyCommand(uint16_t opcode, uint16_t arg) {
  switch ((Cmd)opcode) {
    // DIPERBAIKI (bug ditemukan): cek PACKAGE_READY_FLAG dulunya ada di DALAM
    // handleRefillFSM (case IDLE), dieksekusi SATU LOOP TICK SETELAH currentState sudah kepalang
    // di-set RUNNING_OR_MOVING di sini -- kalau ditolak di situ, refillState TETAP IDLE tanpa
    // ada yg tau, lalu updateRefillJoin() langsung nge-balikin currentState ke IDLE lagi
    // SEKETIKA (refillRequested nyangkut true diam-diam di background). Orange Pi poll STATE
    // abis kirim command cuma lihat IDLE lagi -- kesannya command selesai/diabaikan, padahal
    // masih pending nunggu ACK_PACKAGE_TAKEN, TANPA sinyal FAULT/apapun. Sekarang SEMUA guard
    // dicek DI SINI, sebelum currentState berubah sama sekali -- reject SELALU eksplisit & instan.
    case Cmd::START_MAIN:
      if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
        Serial.println("[CMD] START_MAIN ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
        return;
      }
      if (testLoopStage != TestLoopStage::OFF) {
        Serial.println("[CMD] START_MAIN ditolak -- TEST REFILL LOOP masih aktif, matikan dulu");
        return;
      }
      mainModeActive = true;
      Serial.println("[CMD] START_MAIN -- mode produksi otomatis AKTIF, command TEST diblokir sampai STOP_MAIN");
      // BARU: langsung jalankan urutan awal sampai package duduk di TENGAH. Selesai itu
      // DISPENSER_READY jadi 1, dan barulah node ini pantas disebut siap produksi.
      mulaiPipeline();
      break;
    case Cmd::STOP_MAIN:
      mainModeActive = false;
      hentikanPipeline("STOP_MAIN diterima");
      Serial.println("[CMD] STOP_MAIN -- mode produksi otomatis MATI, command TEST boleh dipakai lagi");
      break;
    case Cmd::REQUEST_REFILL:
      if (!mainModeActive) {
        Serial.println("[CMD] REQUEST_REFILL ditolak -- MAIN belum aktif, kirim START_MAIN dulu");
        return;
      }
      if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
        Serial.println("[CMD] REQUEST_REFILL ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
        return;
      }
      if (testLoopStage != TestLoopStage::OFF) {
        Serial.println("[CMD] REQUEST_REFILL ditolak -- TEST REFILL LOOP sedang aktif, matikan dulu");
        return;
      }
      if (mb.Hreg(Reg::PACKAGE_READY_FLAG) != 0) {
        Serial.println("[CMD] REQUEST_REFILL ditolak -- package UJUNG belum di-ACK diambil (ACK_PACKAGE_TAKEN dulu)");
        return;
      }
      // DIUBAH (2026-09-20): REQUEST_REFILL tidak lagi menjalankan RefillState lama (yang
      // membaca sensor mentah dan menghentikan conveyor sendiri). Sekarang ia hanya menjadi
      // sinyal "package PENUH" bagi pipeline -- padanan langsung dari BUTTON_2 saat pengujian.
      if (pipelineStage != PipelineStage::SIAP_ISI) {
        Serial.printf("[CMD] REQUEST_REFILL ditolak -- Dispenser belum SIAP (tahap sekarang: %s)\n",
                      pipelineText(pipelineStage));
        return;
      }
      pipelineRefillDiminta = true;
      currentState = NodeState::RUNNING_OR_MOVING;
      Serial.println("[CMD] REQUEST_REFILL -- package dinyatakan PENUH, conveyor maju ke UJUNG");
      break;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb sebelum di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE; enterRefillState(RefillState::IDLE);
      // BARU: RESET_FAULT sekarang JUGA paksa keluar dari TEST REFILL LOOP kalau kesisa aktif
      // dari testing sebelumnya -- ini blocker tersembunyi yang gak keliatan dari Orange Pi
      // (pesan "ditolak" cuma tercetak ke Serial USB device, gak lewat Modbus), jadi tanpa
      // ini operator remote gak ada cara "bebasin paksa" node yang kesangkut testLoopStage.
      if (testLoopStage != TestLoopStage::OFF) {
        testLoopStopOutputs();
        testLoopStage = TestLoopStage::OFF;
        Serial.println("[CMD] RESET_FAULT -- TEST REFILL LOOP ikut dipaksa berhenti");
      }
      forceMiddleRefillActive = false;
      forceServoScheduled = false;
      break;
    case Cmd::SET_CONVEYOR_SPEED:
      cfg.conveyorSpeed = (uint8_t)constrain(arg, 0, 255);
      break;
    case Cmd::SET_CONVEYOR_DIR:
      cfg.conveyorDir = (arg != 0);
      break;
    // BARU: Orange Pi konfirmasi arm SUDAH ambil package di UJUNG -- clear PACKAGE_READY_FLAG,
    // baru REQUEST_REFILL berikutnya diizinkan lanjut (lihat guard di handleRefillFSM IDLE).
    case Cmd::ACK_PACKAGE_TAKEN:
      if (!mainModeActive) { Serial.println("[CMD] ACK_PACKAGE_TAKEN ditolak -- MAIN belum aktif"); return; }
      // DIUBAH: selain membersihkan flag, ini sekaligus izin melanjutkan siklus -- padanan
      // langsung dari BUTTON_3 saat pengujian. Conveyor dinyalakan lagi oleh pipeline,
      // bukan lewat perintah terpisah dari Orange Pi.
      if (pipelineStage != PipelineStage::TUNGGU_DIAMBIL) {
        Serial.printf("[CMD] ACK_PACKAGE_TAKEN -- tidak ada package yang menunggu diambil "
                      "(tahap sekarang: %s). Flag tetap dibersihkan.\n", pipelineText(pipelineStage));
        mb.Hreg(Reg::PACKAGE_READY_FLAG, 0);
        break;
      }
      pipelineAckDiterima = true;
      Serial.println("[CMD] ACK_PACKAGE_TAKEN -- arm sudah ambil, siklus dilanjutkan");
      break;
    case Cmd::FORCE_MIDDLE_REFILL:
      if (!mainModeActive) { Serial.println("[CMD] FORCE_MIDDLE_REFILL ditolak -- MAIN belum aktif"); return; }
      startForceMiddleRefill();
      break;
    // BARU -- biar Orange Pi bisa tuning kecepatan servo langsung, runtime-only (gak auto-save
    // NVS -- simpan permanen tetap lewat LCD '#' kalau mau bertahan setelah reboot).
    case Cmd::SET_SERVO1_STEP:          cfg.servo1StepUs = (uint16_t)constrain(arg, 1, 2500); break;
    case Cmd::SET_SERVO1_STEP_INTERVAL: cfg.servo1StepIntervalMs = (uint16_t)constrain(arg, 0, 500); break;
    case Cmd::SET_SERVO2_STEP:          cfg.servo2StepUs = (uint16_t)constrain(arg, 1, 2500); break;
    case Cmd::SET_SERVO2_STEP_INTERVAL: cfg.servo2StepIntervalMs = (uint16_t)constrain(arg, 0, 500); break;
    // BARU: semua command TEST/jog di bawah ini DITOLAK TOTAL selama mainModeActive -- Orange
    // Pi wajib STOP_MAIN dulu. Ini gantiin guard granular satu-satu yang sebelumnya rawan bolong
    // (lihat kasus MOVE_SERVOx_TO vs servoRefillStage otomatis yang baru ketemu & diperbaiki).
    case Cmd::SET_CONVEYOR_ON_OFF:
      if (mainModeActive) { Serial.println("[CMD] SET_CONVEYOR_ON_OFF ditolak -- MAIN aktif, STOP_MAIN dulu"); break; }
      if (refillState != RefillState::IDLE || testLoopStage != TestLoopStage::OFF || forceMiddleRefillActive) {
        Serial.println("[CMD] SET_CONVEYOR_ON_OFF ditolak -- ada siklus otomatis lain sedang jalan");
        break;
      }
      setConveyorManual(arg != 0);
      Serial.printf("[CMD] SET_CONVEYOR_ON_OFF -- conveyor %s\n", arg != 0 ? "ON" : "OFF");
      break;
    case Cmd::MOVE_SERVO1_TO:
      if (mainModeActive) { Serial.println("[CMD] MOVE_SERVO1_TO ditolak -- MAIN aktif, STOP_MAIN dulu"); break; }
      startMoveServoTo(1, arg != 0);
      break;
    case Cmd::MOVE_SERVO2_TO:
      if (mainModeActive) { Serial.println("[CMD] MOVE_SERVO2_TO ditolak -- MAIN aktif, STOP_MAIN dulu"); break; }
      startMoveServoTo(2, arg != 0);
      break;
    case Cmd::TEST_SERVO1_CYCLE:
      if (mainModeActive) { Serial.println("[CMD] TEST_SERVO1_CYCLE ditolak -- MAIN aktif, STOP_MAIN dulu"); break; }
      startTestServoCycle(1);
      break;
    case Cmd::TEST_SERVO2_CYCLE:
      if (mainModeActive) { Serial.println("[CMD] TEST_SERVO2_CYCLE ditolak -- MAIN aktif, STOP_MAIN dulu"); break; }
      startTestServoCycle(2);
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
constexpr uint8_t TOP_COUNT = 3;
const char* TOP_LABELS[TOP_COUNT] = { "Setting Kalibrasi", "Test I/O", "Test Command" };
uint8_t topCursor = 0;
void drawTopMenuFeeder() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
// DIUBAH: slot "Servo Start Delay(ms)" (pensiun -- servo sekarang trigger PACKAGE_MIDDLE_SENSOR,
// bukan delay lagi) di-reuse jadi "Middle Confirm Timeout(ms)" -- batas tunggu konfirmasi sensor
// tengah setelah servo jatuhin package baru.
// DIUBAH: "Servo1/2 Interval(ms)" (target durasi tetap) diganti "Step(us)" + "Step Interval(ms)"
// (kecepatan tetap, SAMA pola dgn hopper SORTER/PICKER) -- durasi jadi hasil jarak/kecepatan.
// BARU: "Force Servo Delay(ms)" -- pakai field servoStartDelayMs (dulu pensiun) KHUSUS utk
// FORCE_MIDDLE_REFILL (startup/recovery): conveyor jalan duluan, servo1+2 baru dieksekusi
// setelah jeda ini -- BEDA dari siklus produksi normal yang PROX_2-edge-triggered.
// BARU: "Reset Fault" jadi item PERTAMA -- dulu cuma bisa dipicu lewat menu "Test Command"
// yang terkubur. Item 1-18 (Conveyor Speed dst) TIDAK berubah nomornya terhadap selParam
// (lihat handleCalListKey -- selParam = calCursor, bukan calCursor+1).
constexpr uint8_t CAL_COUNT = 20;
const char* CAL_LABELS[CAL_COUNT] = { "Reset Fault",
                                        "Conveyor Speed", "Conveyor Dir", "Middle Confirm TO(ms)", "Push Timeout (ms)",
                                        "Servo1 Titik Awal", "Servo1 Titik Akhir", "Servo1 Waktu Tahan",
                                        "Servo1 Step (us)", "Servo1 Step Intv(ms)",
                                        "Servo2 Titik Awal", "Servo2 Titik Akhir", "Servo2 Waktu Tahan",
                                        "Servo2 Step (us)", "Servo2 Step Intv(ms)",
                                        "Buzzer On (ms)", "Buzzer Off (ms)", "Force Servo Delay(ms)",
                                        "Test Loop Retry(ms)", "Reset ke Default" };
uint8_t calCursor = 0;
void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

void drawConfirmReset() {
  lcdClear();
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
    case 2: lcdPrint(0, 0, "CONVEYOR DIR"); break;
    case 3: lcdPrint(0, 0, "MID CONFIRM TIMEOUT"); break;
    case 4: lcdPrint(0, 0, "PUSH TIMEOUT (ms)"); break;
    case 5: lcdPrint(0, 0, "SERVO1 TITIK AWAL"); break;
    case 6: lcdPrint(0, 0, "SERVO1 TITIK AKHIR"); break;
    case 7: lcdPrint(0, 0, "SERVO1 WAKTU TAHAN"); break;
    case 8: lcdPrint(0, 0, "SERVO1 STEP (us)"); break;
    case 9: lcdPrint(0, 0, "SERVO1 STEP INTV(ms)"); break;
    case 10: lcdPrint(0, 0, "SERVO2 TITIK AWAL"); break;
    case 11: lcdPrint(0, 0, "SERVO2 TITIK AKHIR"); break;
    case 12: lcdPrint(0, 0, "SERVO2 WAKTU TAHAN"); break;
    case 13: lcdPrint(0, 0, "SERVO2 STEP (us)"); break;
    case 14: lcdPrint(0, 0, "SERVO2 STEP INTV(ms)"); break;
    case 15: lcdPrint(0, 0, "BUZZER ON (ms)"); break;
    case 16: lcdPrint(0, 0, "BUZZER OFF (ms)"); break;
    case 17: lcdPrint(0, 0, "FORCE SERVO DELAY"); break;
    case 18: lcdPrint(0, 0, "TEST LOOP RETRY(ms)"); break;
  }
  String line1;
  if (selParam == 1) line1 = "Nilai:" + String(cfg.conveyorSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 2) line1 = "Arah:" + String(cfg.conveyorDir ? "FORWARD" : "REVERSE");
  else if (selParam == 3) line1 = "ms:" + String(cfg.middleConfirmTimeoutMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 4) line1 = "Nilai:" + String(cfg.pushTimeoutMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 5) line1 = "us:" + String(cfg.servo1StartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 6) line1 = "us:" + String(cfg.servo1EndUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 7) line1 = "ms:" + String(cfg.servo1HoldMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 8) line1 = "us:" + String(cfg.servo1StepUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 9) line1 = "ms:" + String(cfg.servo1StepIntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 10) line1 = "us:" + String(cfg.servo2StartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 11) line1 = "us:" + String(cfg.servo2EndUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 12) line1 = "ms:" + String(cfg.servo2HoldMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 13) line1 = "us:" + String(cfg.servo2StepUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 14) line1 = "ms:" + String(cfg.servo2StepIntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 15) line1 = "ms:" + String(cfg.buzzerOnMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 16) line1 = "ms:" + String(cfg.buzzerOffMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 17) line1 = "ms:" + String(cfg.servoStartDelayMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  else if (selParam == 18) line1 = "ms:" + String(cfg.testLoopRetryIntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]);
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, selParam == 2 ? "A=Forward B=Reverse" : "A+ B- C:step");
  lcdPrint(0, 3, "#=SIMPAN D=kembali");
}
void handleParamKeyFeeder(char key) {
  int16_t step = JOG_STEPS[jogStepIdx];
  if (selParam == 1) {
    if (key == 'A') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed + step, 0, 255);
    else if (key == 'B') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed - step, 0, 255);
  } else if (selParam == 2) {
    if (key == 'A') cfg.conveyorDir = true;
    else if (key == 'B') cfg.conveyorDir = false;
  } else if (selParam == 3) {
    if (key == 'A') cfg.middleConfirmTimeoutMs = (uint16_t)constrain((int)cfg.middleConfirmTimeoutMs + step * 10, 500, 20000);
    else if (key == 'B') cfg.middleConfirmTimeoutMs = (uint16_t)constrain((int)cfg.middleConfirmTimeoutMs - step * 10, 500, 20000);
  } else if (selParam == 4) {
    if (key == 'A') cfg.pushTimeoutMs = (uint16_t)constrain((int)cfg.pushTimeoutMs + step, 100, 5000);
    else if (key == 'B') cfg.pushTimeoutMs = (uint16_t)constrain((int)cfg.pushTimeoutMs - step, 100, 5000);
  } else if (selParam == 5) {
    if (key == 'A') cfg.servo1StartUs = (uint16_t)constrain((int)cfg.servo1StartUs + step, 500, 2500);
    else if (key == 'B') cfg.servo1StartUs = (uint16_t)constrain((int)cfg.servo1StartUs - step, 500, 2500);
    pcaSetServoUs(SERVO1_CH, cfg.servo1StartUs); servo1CurrentUs = cfg.servo1StartUs;   // preview live
  } else if (selParam == 6) {
    if (key == 'A') cfg.servo1EndUs = (uint16_t)constrain((int)cfg.servo1EndUs + step, 500, 2500);
    else if (key == 'B') cfg.servo1EndUs = (uint16_t)constrain((int)cfg.servo1EndUs - step, 500, 2500);
    pcaSetServoUs(SERVO1_CH, cfg.servo1EndUs); servo1CurrentUs = cfg.servo1EndUs;   // preview live
  } else if (selParam == 7) {
    if (key == 'A') cfg.servo1HoldMs = (uint16_t)constrain((int)cfg.servo1HoldMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.servo1HoldMs = (uint16_t)constrain((int)cfg.servo1HoldMs - step * 10, 50, 5000);
  } else if (selParam == 8) {
    // BARU: batas atas dinaikkan 500->2500 -- rentang servo cuma 2000us (500-2500), jadi
    // stepUs=2500 udah cukup lompat langsung ujung ke ujung dalam 1 tick kalau operator mau.
    if (key == 'A') cfg.servo1StepUs = (uint16_t)constrain((int)cfg.servo1StepUs + step, 1, 2500);
    else if (key == 'B') cfg.servo1StepUs = (uint16_t)constrain((int)cfg.servo1StepUs - step, 1, 2500);
  } else if (selParam == 9) {
    // BARU: batas bawah 1->0 -- 0 = tanpa jeda (step tiap loop tick, secepat mungkin). AWAS:
    // stepUs besar + interval 0 = "selesai" logika dalam 1 tick, servo fisik belum tentu
    // ngejar secepat itu. Kalibrasi hati-hati.
    if (key == 'A') cfg.servo1StepIntervalMs = (uint16_t)constrain((int)cfg.servo1StepIntervalMs + step, 0, 500);
    else if (key == 'B') cfg.servo1StepIntervalMs = (uint16_t)constrain((int)cfg.servo1StepIntervalMs - step, 0, 500);
  } else if (selParam == 10) {
    if (key == 'A') cfg.servo2StartUs = (uint16_t)constrain((int)cfg.servo2StartUs + step, 500, 2500);
    else if (key == 'B') cfg.servo2StartUs = (uint16_t)constrain((int)cfg.servo2StartUs - step, 500, 2500);
    pcaSetServoUs(SERVO2_CH, cfg.servo2StartUs); servo2CurrentUs = cfg.servo2StartUs;   // preview live
  } else if (selParam == 11) {
    if (key == 'A') cfg.servo2EndUs = (uint16_t)constrain((int)cfg.servo2EndUs + step, 500, 2500);
    else if (key == 'B') cfg.servo2EndUs = (uint16_t)constrain((int)cfg.servo2EndUs - step, 500, 2500);
    pcaSetServoUs(SERVO2_CH, cfg.servo2EndUs); servo2CurrentUs = cfg.servo2EndUs;   // preview live
  } else if (selParam == 12) {
    if (key == 'A') cfg.servo2HoldMs = (uint16_t)constrain((int)cfg.servo2HoldMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.servo2HoldMs = (uint16_t)constrain((int)cfg.servo2HoldMs - step * 10, 50, 5000);
  } else if (selParam == 13) {
    if (key == 'A') cfg.servo2StepUs = (uint16_t)constrain((int)cfg.servo2StepUs + step, 1, 2500);
    else if (key == 'B') cfg.servo2StepUs = (uint16_t)constrain((int)cfg.servo2StepUs - step, 1, 2500);
  } else if (selParam == 14) {
    if (key == 'A') cfg.servo2StepIntervalMs = (uint16_t)constrain((int)cfg.servo2StepIntervalMs + step, 0, 500);
    else if (key == 'B') cfg.servo2StepIntervalMs = (uint16_t)constrain((int)cfg.servo2StepIntervalMs - step, 0, 500);
  } else if (selParam == 15) {
    if (key == 'A') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs - step * 10, 50, 5000);
  } else if (selParam == 16) {
    if (key == 'A') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs + step * 10, 50, 5000);
    else if (key == 'B') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs - step * 10, 50, 5000);
  } else if (selParam == 17) {
    if (key == 'A') cfg.servoStartDelayMs = (uint16_t)constrain((int)cfg.servoStartDelayMs + step * 10, 0, 10000);
    else if (key == 'B') cfg.servoStartDelayMs = (uint16_t)constrain((int)cfg.servoStartDelayMs - step * 10, 0, 10000);
  } else if (selParam == 18) {
    if (key == 'A') cfg.testLoopRetryIntervalMs = (uint16_t)constrain((int)cfg.testLoopRetryIntervalMs + step * 10, 200, 10000);
    else if (key == 'B') cfg.testLoopRetryIntervalMs = (uint16_t)constrain((int)cfg.testLoopRetryIntervalMs - step * 10, 200, 10000);
  }
  if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
  else if (key == '#') { saveConfigToNvs(); lcdPrint(0, 3, "TERSIMPAN ke NVS!"); Serial.println("[CAL] FeederConfig disimpan ke NVS"); return; }
  else if (key == 'D') { servoIntervalTestMode = false; menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawParamMenuFeeder();
}

void handleCalListKey(char key) {
  if (key == 'A') { calCursor = (calCursor == 0) ? CAL_COUNT - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % CAL_COUNT; drawCalList(); }
  else if (key == 'C') {
    if (calCursor == 0) {   // BARU -- Reset Fault, langsung eksekusi (tidak destruktif, gak perlu konfirmasi)
      applyCommand((uint16_t)Cmd::RESET_FAULT, 0);
      lcdPrint(0, 3, "Fault direset!      ");
      Serial.println("[CAL] Reset Fault dari menu LCD");
    } else if (calCursor == 19) { menuState = MenuState::CONFIRM_RESET; drawConfirmReset(); }
    else {
      selParam = calCursor;
      // BARU: aktifkan test-live di layar Servo1/2 Step(8/13) ATAU Step Interval(9/14) --
      // sama pola dgn hopperIntervalTestMode SORTER, supaya servo keliatan gerak live pas
      // salah satu dari 2 parameter kecepatan ini diubah, bukan diam kayak sebelumnya.
      servoIntervalTestMode = (selParam == 8 || selParam == 9 || selParam == 13 || selParam == 14);
      servoIntervalTestWhich = (selParam == 8 || selParam == 9) ? 1 : 2;
      if (servoIntervalTestMode && testServoCycleStage == TestServoCycleStage::NONE) startTestServoCycle(servoIntervalTestWhich);
      menuState = MenuState::JOG_PARAM; lcdClear(); drawParamMenuFeeder();
    }
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
  {"FAULT",  CH::LED_FAULT,     true},   // DIUBAH -- sekarang auto-controlled (lihat updateUniversalIndicators)
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
// channel-nya di semua 4 node (board PCB universal) -- test channel mentah CH::xxx. Dispenser
// TIDAK PAKAI limit switch sama sekali di produksi (murni 2 proximity, PROX_BOX_ARRIVED +
// PACKAGE_MIDDLE_SENSOR), tapi kategori "Limit Switch" tetap ada di menu ini buat uji channel
// LIM1..LIM10 mentah (board universal, sama persis di semua node).
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
  lcdClear();
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
    lcdClear(); lcdPrint(0, 0, "MODUL: STEPPER");
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
constexpr uint8_t TEST_PUSH_SPEED = 200;   // BARU -- ChA (push) gak punya cfg speed produksi lagi (sekarang servo), pakai speed tetap khusus test
void testModSetMotor(bool channelA, uint8_t dirState) {
  // DIPERBAIKI: sebelumnya cuma set arah (AIN/BIN) + STBY, TANPA pernah ledcWrite ke PWMA/PWMB
  // -- TB6612FNG butuh sinyal PWM aktif spy motor benar-benar berputar, arah doang tidak cukup.
  uint8_t ain1 = channelA ? CH::AIN1 : CH::BIN1, ain2 = channelA ? CH::AIN2 : CH::BIN2;
  uint8_t pwmCh = channelA ? LEDC_CH_PUSH_TEST : LEDC_CH_CONV2;
  uint8_t speed = channelA ? TEST_PUSH_SPEED : cfg.conveyorSpeed;
  if (channelA) testModMotorAState = dirState; else testModMotorBState = dirState;
  if (dirState == 0) { io.write(ain1, LOW); io.write(ain2, LOW); ledcWrite(pwmCh, 0); }
  else { io.write(ain1, dirState == 1); io.write(ain2, dirState != 1); ledcWrite(pwmCh, speed); }
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
    testModSetMotor(true, 0); testModSetMotor(false, 0);
    menuState = MenuState::TEST_MODULE_SELECT; drawModuleTypeSelect(); return;
  }
  drawTestModMotorDC();
}

uint8_t testModRelaySel = 0;
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
      case 2: menuState = MenuState::TEST_INPUT_CATEGORY; inputCatCursor = 0; drawInputCategorySelect(); break;
      case 3: menuState = MenuState::TEST_RS485; drawTestRs485(); break;
      case 4: menuState = MenuState::TEST_MODULE_SELECT; moduleTypeCursor = 0; drawModuleTypeSelect(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuFeeder(); }
}

// --- LEVEL 1c: TEST COMMAND ---
// BARU: field isLocalToggle -- item dgn ini true BUKAN opcode Modbus asli (opcode/testArg
// diabaikan), tapi toggle fungsi lokal (TEST REFILL LOOP), lihat handleTestCmdListKey().
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; bool isLocalToggle; };
constexpr uint8_t CMD_TEST_COUNT = 9;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"REQUEST_REFILL",       Cmd::REQUEST_REFILL,     0, false},
  {"SET_SPEED (test=200)", Cmd::SET_CONVEYOR_SPEED, 200, false},
  {"SET_DIR (test=REV)",   Cmd::SET_CONVEYOR_DIR,   0, false},
  {"ACK_PACKAGE_TAKEN",    Cmd::ACK_PACKAGE_TAKEN,  0, false},
  {"FORCE_MIDDLE_REFILL",  Cmd::FORCE_MIDDLE_REFILL,0, false},
  {"RESET_FAULT",          Cmd::RESET_FAULT,        0, false},
  {"Test Srv1 M/M", Cmd::TEST_SERVO1_CYCLE, 0, false},
  {"Test Srv2 M/M", Cmd::TEST_SERVO2_CYCLE, 0, false},
  {"TEST REFILL LOOP (toggle)", Cmd::NONE, 0, true},
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
    if (item.isLocalToggle) {
      toggleTestRefillLoop();
      lcdPrint(0, 3, testLoopStage != TestLoopStage::OFF ? "AKTIF, tekan tombol" : "Dimatikan");
    } else {
      Serial.printf("[TEST-CMD] Simulasi command dari 'node lain': opcode=%u (%s) arg=%u -- amati aksi fisik SEKARANG\n",
                    (uint16_t)item.opcode, item.label, item.testArg);
      applyCommand((uint16_t)item.opcode, item.testArg);
      lcdPrint(0, 3, "Terkirim, amati aksi");
    }
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
  } else if (key == 'D') { menuState = MenuState::NONE; lcdClear(); Serial.println("[CAL] Keluar mode kalibrasi"); }
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
  else if (cmd == "ACK")         applyCommand((uint16_t)Cmd::ACK_PACKAGE_TAKEN, 0);
  else if (cmd == "FORCEFILL")   applyCommand((uint16_t)Cmd::FORCE_MIDDLE_REFILL, 0);
  else if (cmd == "TESTLOOP")    toggleTestRefillLoop();
  else if (cmd == "RESET_FAULT") applyCommand((uint16_t)Cmd::RESET_FAULT, 0);
  else if (cmd == "SPEED")       applyCommand((uint16_t)Cmd::SET_CONVEYOR_SPEED, arg);
  else if (cmd == "DIR")         applyCommand((uint16_t)Cmd::SET_CONVEYOR_DIR, arg);
  else if (cmd == "TIMEOUT") { cfg.pushTimeoutMs = constrain((int)arg, 100, 5000); saveConfigToNvs(); Serial.printf("[TIMEOUT] pushTimeoutMs=%u\n", cfg.pushTimeoutMs); }
  else if (cmd == "RESET") resetConfigToDefault();
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s refillState=%d fault=%u ESTOP=%d speed=%u dir=%s timeout=%u packageReady=%u\n",
                  stateText(currentState), (int)refillState, faultCode, io.read(CH::ESTOP),
                  cfg.conveyorSpeed, cfg.conveyorDir ? "FWD" : "REV", cfg.pushTimeoutMs,
                  mb.Hreg(Reg::PACKAGE_READY_FLAG));
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus servoRefillStage=%d middleSensor=%d middleTimeoutMs=%u forceRefillActive=%d\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000),
                  (int)servoRefillStage, io.read(CH::PACKAGE_MIDDLE_SENSOR) == LOW, cfg.middleConfirmTimeoutMs, forceMiddleRefillActive);
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] REFILL | ACK | FORCEFILL | RESET_FAULT | SPEED <0-255> | DIR <0|1> | TIMEOUT <100-5000> | RESET | STATUS | HELP");
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
    if (menuState != MenuState::NONE) { menuState = MenuState::NONE; servoIntervalTestMode = false; Serial.println("[HOTPLUG] Keluar OTOMATIS dari mode kalibrasi -- cegah node terjebak"); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] FEEDER mulai");

  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_FREQ_HZ);
  pwm.begin(); pwm.setPWMFreq(50);   // BARU -- utk Test Modul Servo (library resmi, sinkron node lain)
  scanI2CAndDetectOptional();

  if (lcdPresent) { lcd.init(); lcd.backlight(); for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false; lcdPrint(0, 0, "FEEDER"); Serial.println("[BOOT] LCD OK"); }
  else Serial.println("[BOOT] LCD dilewati -- kalibrasi via Serial (HELP)");

  if (keypadPresent) { keypad.begin(); Serial.println("[BOOT] Keypad OK"); }
  else Serial.println("[BOOT] Keypad dilewati");
  lcdBootProgress("I2C + LCD/Keypad");

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
  // DIPERBAIKI (bug ditemukan): pin native STEP_1/2/3 dipakai testStepperPulse() (Test Modul
  // Stepper) via digitalWrite() langsung -- io.pinMode() di atas cuma berlaku utk channel
  // MCP23017, BUKAN pin native ESP32 ini. Tanpa pinMode(OUTPUT), default boot = INPUT, pulsa
  // STEP tidak pernah keluar elektrik meski DIR/EN (MCP) sudah benar.
  pinMode(GP::STEP_1, OUTPUT); pinMode(GP::STEP_2, OUTPUT); pinMode(GP::STEP_3, OUTPUT);
  // DIHAPUS: pinMode LIM_STOCK_EMPTY/LIM_PUSH_HOME/LIM_PUSH_EXTENDED -- Dispenser tidak pakai
  // limit switch sama sekali, murni 2 proximity (UJUNG+TENGAH).
  io.pinMode(CH::PROX_BOX_ARRIVED, INPUT_PULLUP);
  io.pinMode(CH::PACKAGE_MIDDLE_SENSOR, INPUT_PULLUP);   // mekanisme 2-proximity (UJUNG+TENGAH)
  io.pinMode(CH::BTN_TEST_BOX_FULL, INPUT_PULLUP);   // BARU -- tombol fisik TEST REFILL LOOP
  // BARU (2026-09-20): BUTTON_3 sebelumnya TIDAK PERNAH di-pinMode di Dispenser -- terdaftar
  // di menu Test Input tapi tidak pernah disiapkan, jadi pembacaannya tidak bermakna.
  // (BUTTON_2 sudah disiapkan lewat alias BTN_TEST_BOX_FULL di baris atas -- channel sama.)
  io.pinMode(CH::BUTTON_3, INPUT_PULLUP);
  // DIUBAH (2026-09-20): sinkronkan kondisi AWAL kedua sensor ke keadaan fisik saat boot --
  // supaya tidak ada tepi palsu yang terhitung sebagai "kedatangan" hanya karena firmware
  // baru hidup sementara package memang sudah ada di sensor sejak sebelumnya.
  lastMiddleSensorState = io.read(CH::PACKAGE_MIDDLE_SENSOR);
  middleRawLast = middleStable = lastMiddleSensorState;
  middleRawChangedAt = millis();
  ujungRawLast = ujungStable = io.read(CH::PROX_BOX_ARRIVED);
  ujungRawChangedAt = millis();
  Serial.printf("[BOOT] MCP23017: %s, pinMode selesai\n", ioOk ? "OK" : "GAGAL");
  lcdBootProgress("I/O Expander MCP23017");

  ledcSetup(LEDC_CH_CONV2, PWM_FREQ, PWM_RES);
  ledcAttachPin(GP::CONV2_PWM, LEDC_CH_CONV2);
  // BARU: channel A (dulu push DC, sekarang push produksi sudah pindah ke servo) TIDAK PERNAH
  // di-setup PWM-nya sebelum ini -- Test Modul Motor DC ChA gak bisa muter chip TB6612 kalau
  // fisiknya masih terpasang. Setup terpisah, HANYA dipakai test menu, TIDAK ganggu produksi.
  ledcSetup(LEDC_CH_PUSH_TEST, PWM_FREQ, PWM_RES);
  ledcAttachPin(GP::PWMA_TB6612, LEDC_CH_PUSH_TEST);
  Serial.println("[BOOT] LEDC PWM conveyor2 OK");
  lcdBootProgress("PWM Conveyor2");

  loadConfigFromNvs();
  Serial.println("[BOOT] FeederConfig dimuat dari NVS");

  // BARU: posisi awal servo1/servo2 saat boot -- SETELAH loadConfigFromNvs() supaya pakai
  // nilai kalibrasi tersimpan (bukan default compile-time -- pelajaran dari bug urutan SORTER)
  pcaSetServoUs(SERVO1_CH, cfg.servo1StartUs); servo1CurrentUs = cfg.servo1StartUs;
  pcaSetServoUs(SERVO2_CH, cfg.servo2StartUs); servo2CurrentUs = cfg.servo2StartUs;
  Serial.println("[BOOT] Servo1/Servo2 diposisikan ke titik awal");
  lcdBootProgress("Kalibrasi NVS");

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
  mb.addHreg(Reg::PACKAGE_READY_FLAG, 0);   // BARU -- mekanisme 2-proximity
  // Nilai awal diambil dari kondisi stabil yang sudah disinkronkan di atas, jadi register
  // sudah benar sejak detik pertama -- tidak perlu menunggu perubahan pertama dulu.
  mb.addHreg(Reg::MIDDLE_PACKAGE_PRESENT, (middleStable == LOW) ? 1 : 0);   // status live PROX_2
  mb.addHreg(Reg::UJUNG_PACKAGE_PRESENT, (ujungStable == LOW) ? 1 : 0);     // status live PROX_1 (UJUNG)
  mb.addHreg(Reg::MAIN_MODE_ACTIVE, 0);         // BARU -- status live MAIN vs TEST mode
  mb.addHreg(Reg::MENU_ACTIVE, 0);              // BARU -- 1 = operator di menu kalibrasi, command Modbus diabaikan
  mb.addHreg(Reg::BTN_PACKAGE_FULL_COUNT, 0);   // BARU -- konfirmasi "package PENUH" dari tombol fisik
  mb.addHreg(Reg::BTN_PACKAGE_TAKEN_COUNT, 0);  // BARU -- konfirmasi "package DIAMBIL" dari tombol fisik
  mb.addHreg(Reg::CONVEYOR_AUTOSTOP_COUNT, 0);  // BARU -- berapa kali firmware stop conveyor sendiri krn PROX_2
  mb.addHreg(Reg::UJUNG_ARRIVAL_COUNT, 0);      // BARU -- kedatangan SAH di UJUNG (lolos guard urutan)
  mb.addHreg(Reg::DISPENSER_READY, 0);          // BARU -- 1 = siap menampung objek baru
  mb.addHreg(Reg::PIPELINE_STAGE, 0);           // BARU -- tahap pipeline produksi
  mb.addHreg(Reg::MIDDLE_ARRIVAL_COUNT, 0);     // BARU -- counter latch PROX_2, anti-kelewatan
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  Serial.printf("[BOOT] Modbus siap, slave ID=%d\n", Rs485Cfg::SLAVE_ID);
  lcdBootProgress("Modbus RS485");

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;

  setupOTA();
  lcdBootProgress("WiFi OTA");

  Serial.println("[BOOT] setup SELESAI -- ketik HELP di serial monitor");
}

// BARU: apakah SALAH SATU jalur servo sedang bergerak sekarang -- dipakai di baris [LOOP]
// supaya angka anomali dan waktu putaran bisa langsung dikaitkan dengan gerakan servo.
bool servoSedangBergerak() {
  return isServoRefillAutoActive()
      || manualServo1Moving || manualServo2Moving
      || testServoCycleStage != TestServoCycleStage::NONE
      || forceMiddleRefillActive;
}

uint32_t loopMaksMs = 0;   // putaran TERLAMA sejak laporan [LOOP] terakhir

void loop() {
  // Ukur lama satu putaran penuh. Kalau sebuah putaran memakan waktu lebih lama daripada
  // ambang debounce sensor, pulsa sensor yang pendek bisa terlewat sama sekali -- dan itu
  // akan kelihatan di sini sebagai angka, bukan sebagai tebakan.
  {
    static uint32_t putaranSebelumnya = 0;
    uint32_t sekarang = millis();
    if (putaranSebelumnya != 0) {
      uint32_t lama = sekarang - putaranSebelumnya;
      if (lama > loopMaksMs) loopMaksMs = lama;
    }
    putaranSebelumnya = sekarang;
  }
  mb.task();
  updateOTA();

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
  // DIHAPUS: update dinamis STOCK_EMPTY_FLAG -- Dispenser tidak punya limit switch deteksi stok
  // habis lagi, register ini sekarang selalu 0 (nilai default dari addHreg di setup()).
  // BARU: update register Lapis 2 + Lapis 3 tiap loop
  mb.Hreg(Reg::ACTIVITY_CODE, (uint16_t)activityCode());
  mb.Hreg(Reg::I2C_ERROR_COUNT, i2cErrorCount);
  mb.Hreg(Reg::LAST_FAULT_CODE, lastFaultCode);
  mb.Hreg(Reg::UPTIME_SEC, (uint16_t)(millis() / 1000));
  // BARU: status live PROX_BOX_ARRIVED (UJUNG), simetris dgn MIDDLE_PACKAGE_PRESENT --
  // sekarang lewat updateUjungPresentDebounced() (didebounce), bukan tulis mentah lagi.
  updateUjungPresentDebounced();
  mb.Hreg(Reg::MAIN_MODE_ACTIVE, mainModeActive ? 1 : 0);
  // BARU: master bisa bedain "node lagi dikalibrasi operator" vs "node mati/kabel putus" --
  // dua-duanya sama-sama TIDAK membalas CMD_ACK_SEQ, jadi sebelumnya tidak bisa dibedakan.
  mb.Hreg(Reg::MENU_ACTIVE, (menuState != MenuState::NONE) ? 1 : 0);

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
  // BARU -- counter tombol konfirmasi. Sengaja DI LUAR blok bersyarat: ini murni pelaporan,
  // tidak menggerakkan aktuator, dan justru berguna saat node sedang FAULT/ESTOPPED (operator
  // tetap bisa menandai "sudah diambil" tanpa harus mereset node dulu).
  updateKonfirmasiButtons();
  if (currentState != NodeState::ESTOPPED) {
    handleRefillFSM();
    updatePipeline();           // BARU -- pipeline produksi (urutan yang sama dgn pengujian)
    handleMiddleSensor();       // BARU -- trigger servo refill dari PACKAGE_MIDDLE_SENSOR, bukan delay lagi
    updateServoRefillStage();   // BARU -- siklus servo refill, PARALEL dgn conveyor (lihat komentar di atas)
    updateForceMiddleRefill();  // BARU -- utk startup/recovery, lihat komentar di definisinya
    updateRefillJoin();         // BARU -- currentState balik IDLE hanya kalau semua cabang SUDAH tuntas
  }
  updateTestServoCycle();   // BARU -- proses test sequence non-blocking (independen dari RefillState produksi)
  updateManualServoMove();  // BARU -- proses jog manual 1 arah MOVE_SERVO1_TO/MOVE_SERVO2_TO
  updateTestRefillLoop();   // BARU -- TEST REFILL LOOP, independen total dari RefillState produksi
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
    // DIPERLUAS: conveyor + kondisi kedua proximity ikut dicetak. Saat menelusuri "package
    // tidak bergerak" atau "PROX_2 tidak trigger", tiga hal inilah yang perlu dilihat
    // bersamaan, dan sebelumnya tidak satu pun tampil di sini.
    // DIPERLUAS lagi: anomali pembacaan sensor + waktu putaran TERLAMA sejak laporan
    // sebelumnya. Dua angka inilah yang membuktikan atau menggugurkan dugaan "servo bikin
    // prox tidak terbaca": anomali naik = bus I2C terganggu; loopMax besar = firmware
    // terlalu lama satu putaran sehingga pulsa sensor terlewat. Kalau dua-duanya tenang
    // sementara gejalanya tetap ada, penyebabnya bukan di sini.
    Serial.printf("[LOOP] state=%s refillState=%d ESTOP=%d conveyor=%s PROX_2=%s PROX_1=%s "
                  "arrival=%u autostop=%u anomali=%u loopMax=%lums servo=%s "
                  "ujungSah=%u menujuUjung=%u pipeline=%s\n",
                  stateText(currentState), (int)refillState, io.read(CH::ESTOP),
                  conveyorManualOn ? "ON" : "OFF",
                  (middleStable == LOW) ? "TERTUTUP" : "terbuka",
                  (ujungStable == LOW) ? "TERTUTUP" : "terbuka",
                  middleArrivalCount, conveyorAutoStopCount,
                  sensorAnomalyCount, (unsigned long)loopMaksMs,
                  servoSedangBergerak() ? "BERGERAK" : "diam",
                  ujungArrivalCount, paketMenujuUjung, pipelineText(pipelineStage));
    loopMaksMs = 0;   // reset jendela pengukuran
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