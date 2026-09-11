// ============================================================
// SORTER — Conveyor1 + Palang + Proximity + Menu Kalibrasi LCD/Keypad
// Hopper DINONAKTIFKAN sementara -- mekanisme fisiknya belum ditentukan.
//
// PRINSIP PENTING: LCD & Keypad HANYA untuk kalibrasi -- kalau tidak
// terpasang (device tidak terdeteksi di I2C scan), firmware TETAP JALAN
// NORMAL untuk produksi (conveyor/palang/proximity/Modbus tidak bergantung
// sama sekali pada LCD/Keypad). Semua panggilan lcd./keypad. dijaga oleh
// flag lcdPresent/keypadPresent, tidak akan macet/spam error kalau tidak ada.
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

LiquidCrystal_I2C lcd(I2CAddr::LCD, LcdCfg::COLS, LcdCfg::ROWS);

Keypad4x4 keypad(I2CAddr::KEYPAD);
IOBank io;
ModbusRTU mb;
// DIUBAH: servo hopper sekarang pakai library RESMI Adafruit_PWMServoDriver -- SAMA PERSIS
// pola dengan PICKER (yang terbukti bekerja), menggantikan driver Wire mentah sebelumnya.
Adafruit_PWMServoDriver pwm(I2CAddr::PCA9685);
Preferences prefs;

constexpr char Keypad4x4::KEYMAP[4][4];

bool lcdPresent = false, keypadPresent = false;   // BARU -- LCD/Keypad opsional, hanya utk kalibrasi

// BARU: helper LCD -- SELALU isi penuh sampai ujung baris (20 kolom), supaya sisa karakter
// dari teks sebelumnya yang lebih panjang tidak pernah nempel/menumpuk. Dipakai menggantikan
// lcd.print() manual + spasi tebakan di menu kalibrasi & status live.
void lcdPrint(uint8_t col, uint8_t row, String text) {
  if (!lcdPresent) return;
  uint8_t avail = (col < LcdCfg::COLS) ? (LcdCfg::COLS - col) : 0;
  if (text.length() > avail) text = text.substring(0, avail);
  while (text.length() < avail) text += " ";
  lcd.setCursor(col, row);
  lcd.print(text);
}

// BARU: animasi LOADING singkat saat boot -- kasih feedback visual operator + waktu settle
// I2C/PCA9685/MCP23017 sebelum masuk operasi normal. No-op total kalau LCD tidak terdeteksi
// (dipanggil SETELAH lcd.init() supaya benar-benar tampil).
constexpr uint8_t BOOT_STEPS_TOTAL = 6;
constexpr uint16_t BOOT_STEP_DELAY_MS = 150;
uint8_t bootStep = 0;
void lcdBootProgress(const char* stepLabel) {
  bootStep++;
  // DIPERBAIKI: delay() WAJIB tetap jalan meski LCD tidak terdeteksi -- tujuannya kasih waktu
  // settle inisialisasi hardware (I2C/PCA9685/MCP23017), animasi LCD cuma bonus visual di atasnya.
  // Sebelumnya delay ikut di-skip bareng LCD (return duluan), jadi tanpa LCD = TIDAK ada jeda sama
  // sekali -- gak sesuai maksud "beri waktu inisialisasi" yang harusnya berlaku terlepas dari LCD.
  if (lcdPresent) {
    uint8_t filled = (uint8_t)min((uint32_t)10, (uint32_t)bootStep * 10 / BOOT_STEPS_TOTAL);
    String bar = "[";
    for (uint8_t i = 0; i < 10; i++) bar += (i < filled) ? '#' : '-';
    bar += "]";
    lcdPrint(0, 0, "SORTER BOOTING");
    lcdPrint(0, 1, bar);
    lcdPrint(0, 2, "LOADING " + String(bootStep) + "/" + String(BOOT_STEPS_TOTAL));
    lcdPrint(0, 3, stepLabel);
  }
  delay(BOOT_STEP_DELAY_MS);
}

NodeState currentState = NodeState::INIT;
uint16_t faultCode = 0;
// BARU: Lapis 3 diagnostik -- bantu identifikasi masalah intermiten (mis. EMI relay) dari jarak jauh
uint16_t i2cErrorCount = 0;
uint16_t lastFaultCode = 0;
uint32_t lastRs485Rx = 0;
bool modbusEverUsed = false;   // BARU: watchdog COMM_TIMEOUT cuma aktif kalau memang pernah ada Modbus
                                 // master asli yang bicara -- sebelumnya selalu memicu FAULT palsu 5 detik
                                 // setelah RUNNING kalau diuji murni via Serial (Modbus tidak pernah dipakai)

struct SorterConfig {
  uint8_t  conveyorSpeed    = 180;
  bool     conveyorDir      = true;
  uint16_t palangPushMs     = 300;           // DIUBAH nama dari palangPulseMs -- palang sekarang
                                              // linear actuator (motor DC), bukan solenoid relay lagi.
                                              // Ini durasi drive motor MAJU (push), bukan lagi durasi relay ON.
  float    distMm           = 150.0f;        // O2: ukur jarak fisik scan->palang
  float    mmPerSecAtMaxPwm = 300.0f;        // O2: ukur kecepatan conveyor aktual
  uint8_t  palangSpeed      = 180;           // DIUBAH nama dari motorASpeed -- channel Motor A yang
                                              // tadinya spare SEKARANG dipakai buat actuator palang ini.
  // DIUBAH KEMBALI ke POSITIONAL -- rack-pinion Anda pendek (dalam 1 putaran servo), jadi
  // servo positional (spt MG996R) lebih sederhana & presisi drpd continuous-rotation.
  uint16_t hopperStartUs    = 1000;   // posisi diam/tertarik
  uint16_t hopperPushUs     = 2000;   // posisi dorong penuh
  uint16_t hopperIntervalMs = 1000;   // total waktu 1 siklus penuh (dorong+kembali)
  // BARU: buzzer notifikasi -- durasi ON/OFF (ms) saat trigger event (palang aktif/reject)
  uint16_t buzzerOnMs  = 500;
  uint16_t buzzerOffMs = 500;
  // BARU: palang linear actuator -- field DITARUH DI AKHIR struct supaya kalibrasi lama
  // tersimpan di NVS (blob lebih pendek, belum punya field ini) tetap kebaca aman, field baru
  // otomatis pakai nilai default ini kalau blob NVS lama belum punya byte-nya.
  bool     palangDir        = true;   // arah PUSH (true=forward via motorWrite). RETRACT = kebalikannya otomatis.
  uint16_t palangRetractMs  = 300;    // durasi drive motor MUNDUR (retract balik ke posisi awal)
  // DIUBAH TOTAL: hopper dari "target durasi tetap" (hopperIntervalMs, SEKARANG TIDAK DIPAKAI LAGI
  // tapi field dibiarkan supaya offset NVS di belakangnya tidak geser) jadi "kecepatan step tetap"
  // -- sama pola dgn trajStepUs/trajStepIntervalMs PICKER. Durasi jadi HASIL (jarak/kecepatan),
  // bukan target yang dipaksakan -- ini juga menghilangkan bug "mundur sebelum sampai" kalau
  // Interval di-set lebih kecil dari kecepatan fisik servo yang sebenarnya sanggup.
  uint16_t hopperStepUs         = 20;   // besar lompatan tiap tick (us)
  uint16_t hopperStepIntervalMs = 20;   // jeda antar tick (ms)
} cfg;

uint32_t passCount = 0, rejectCount = 0;
bool palangPending = false, palangActive = false;

// BARU: buzzer notifikasi -- 1 siklus ON-OFF non-blocking per trigger event (bukan alarm terus-menerus)
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
    buzzerBeeping = false;   // 1 siklus ON-OFF selesai
  }
}
uint32_t palangTriggerAt = 0;
bool lastProxState = HIGH;
uint32_t lastProxEdge = 0;

struct PendingClass { bool valid; uint32_t scanTimeMs; bool isReject; };
PendingClass pendingQ[4];
uint8_t qHead = 0, qTail = 0;

constexpr int PWM_FREQ = 20000, PWM_RES = 8, LEDC_CH_CONV1 = 0, LEDC_CH_MOTORA = 1;

// DIUBAH TOTAL: servo hopper sekarang pakai library RESMI Adafruit_PWMServoDriver (objek
// pwm global) -- SAMA PERSIS pola dengan PICKER yang terbukti bekerja, menggantikan driver
// Wire mentah sebelumnya yang mungkin punya bug tersembunyi.
constexpr uint8_t HOPPER_PCA_CHANNEL = 0;   // channel PCA9685 yang dipakai utk servo hopper
constexpr uint16_t HOPPER_MIN_US = 400, HOPPER_MAX_US = 2500;   // DIVERIFIKASI FISIK -- di bawah 400 servo tidak bergerak (meski masih bisa diputar manual)

// DIUBAH: fungsi generik -- dipakai BERSAMA oleh hopper produksi (channel tetap) dan Test
// Modul Servo (channel bisa diganti-ganti operator). SAMA PERSIS pola usToDuty() PICKER.
void pcaSetServoUs(uint8_t channel, uint16_t us) {
  us = constrain(us, HOPPER_MIN_US, HOPPER_MAX_US);
  uint16_t duty = (uint32_t)us * 4096 / 20000;
  pwm.setPWM(channel, 0, duty);
}
void hopperSetUs(uint16_t us) { pcaSetServoUs(HOPPER_PCA_CHANNEL, us); }

// Variabel gerak BERSAMA (dipakai FSM produksi & Test Sequence) -- dipindah ke atas supaya
// bisa dipakai Test Sequence yang didefinisikan lebih dulu dalam file.
// DIUBAH TOTAL: hopperCurrentUs = posisi FISIK SAAT INI yang di-track terus-menerus (SAMA pola
// dgn currentUs[] PICKER) -- gantikan hopperMoveFromUs/hopperMoveStartMs (interpolasi duration-based
// lama). Selesai-nya gerakan sekarang ditentukan POSISI (currentUs==target), bukan waktu abis.
uint16_t hopperCurrentUs = 1000;   // diinisialisasi ulang di setup() pakai cfg.hopperStartUs
uint32_t lastHopperStepMs = 0;     // kapan tick step TERAKHIR terjadi

// --- BARU: Test Sequence Hopper -- 1x siklus maju-mundur PAKAI NILAI KALIBRASI (Titik Awal/Dorong/
// Step/StepInterval), dipicu dari LCD Test Command ATAU langsung dari Orange Pi (opcode). REUSE
// trajectory helper YANG SAMA dengan produksi (updateHopperTrajectory, didefinisikan di bawah).
bool updateHopperTrajectory(uint16_t targetUs);   // forward declaration -- definisi lengkap di handleHopper()
enum class TestHopperCycleStage { NONE, MOVING_TO_PUSH, MOVING_TO_START };
TestHopperCycleStage testHopperCycleStage = TestHopperCycleStage::NONE;
void startTestHopperCycle() {
  if (currentState != NodeState::IDLE) { Serial.println("[TEST] Hopper cycle ditolak -- node sedang tidak IDLE"); return; }
  testHopperCycleStage = TestHopperCycleStage::MOVING_TO_PUSH;
  Serial.println("[TEST] Hopper cycle dimulai (pakai nilai kalibrasi Step/StepInterval)");
}
void updateTestHopperCycle() {
  switch (testHopperCycleStage) {
    case TestHopperCycleStage::MOVING_TO_PUSH:
      if (updateHopperTrajectory(cfg.hopperPushUs)) testHopperCycleStage = TestHopperCycleStage::MOVING_TO_START;
      break;
    case TestHopperCycleStage::MOVING_TO_START:
      if (updateHopperTrajectory(cfg.hopperStartUs)) {
        testHopperCycleStage = TestHopperCycleStage::NONE;
        Serial.println("[TEST] Hopper cycle SELESAI");
      }
      break;
    default: break;
  }
}

void motorWrite(uint8_t ain1, uint8_t ain2, bool forward) {
  io.write(ain1, forward);
  io.write(ain2, !forward);
}

uint32_t calculateTOF() {
  float speedMmS = (cfg.conveyorSpeed / 255.0f) * cfg.mmPerSecAtMaxPwm;
  if (speedMmS < 5.0f) speedMmS = 5.0f;
  return (uint32_t)((cfg.distMm / speedMmS) * 1000.0f);
}

void enqueueClassification(bool isReject, uint32_t scanTimeMs) {
  uint8_t next = (qTail + 1) % 4;
  if (next == qHead) return;
  pendingQ[qTail] = {true, scanTimeMs, isReject};
  qTail = next;
}

// --- BARU: push button uji manual PASS/REJECT -- panggil enqueueClassification() PERSIS SAMA
// dengan yang dipakai onClassifyWrite() (jalur Modbus asli dari OrangePi/HuskyLens), supaya
// perilaku TOF+palang yang diuji lewat tombol 100% identik dengan produksi sesungguhnya. ---
void handleTestButtons() {
  static bool lastPass = HIGH, lastReject = HIGH;
  static uint32_t lastPassEdge = 0, lastRejectEdge = 0;
  constexpr uint32_t DEBOUNCE_MS = 50;

  bool curPass = io.read(CH::BTN_TEST_PASS);
  if (curPass == LOW && lastPass == HIGH && millis() - lastPassEdge > DEBOUNCE_MS) {
    lastPassEdge = millis();
    enqueueClassification(false, millis());
    Serial.println("[TEST-BTN] Tombol PASS ditekan -- simulasi klasifikasi pass");
  }
  lastPass = curPass;

  bool curReject = io.read(CH::BTN_TEST_REJECT);
  if (curReject == LOW && lastReject == HIGH && millis() - lastRejectEdge > DEBOUNCE_MS) {
    lastRejectEdge = millis();
    enqueueClassification(true, millis());
    Serial.println("[TEST-BTN] Tombol REJECT ditekan -- simulasi klasifikasi reject");
  }
  lastReject = curReject;
}

uint16_t onClassifyWrite(TRegister* reg, uint16_t val) {
  enqueueClassification(val != 0, millis());
  Serial.printf("[SORTER] Klasifikasi diterima: %s, dijadwalkan trigger dlm %lu ms\n",
                val ? "REJECT" : "pass", (unsigned long)calculateTOF());
  return val;
}

// --- BARU: Hopper servo + rack-pinion -- FSM 2-tahap (Push -> Return), dirancang supaya
// TOTAL 1 siklus penuh = persis cfg.hopperIntervalMs yang di-set user (bukan waktu tunggu SAJA).
// DIREDESIGN TOTAL: servo hopper CONTINUOUS ROTATION -- konsep lama (posisi Awal/Dorong)
// TIDAK BERLAKU untuk servo jenis ini. Sekarang: kirim pulsa arah+kecepatan SELAMA durasi
// tertentu, LALU WAJIB kirim pulsa netral eksplisit (STOP) sebelum ganti arah -- servo
// continuous-rotation TIDAK PERNAH "diam sendiri" di suatu posisi seperti servo biasa,
// dia AKAN TERUS BERPUTAR sampai benar-benar diperintah netral.
// DIKEMBALIKAN ke POSITIONAL: rack-pinion pendek, servo positional (spt MG996R) lebih
// sederhana & presisi drpd continuous-rotation -- servo diam sendiri di posisi yang
// diperintah, tidak perlu STOP eksplisit / risiko putar tak terkendali.
// DIREDESIGN: Interval sekarang = WAKTU TEMPUH satu arah (Titik Awal->Titik Dorong), BUKAN
// lagi total 1 siklus. Servo bergerak BERTAHAP (ramped, non-blocking) selama durasi itu --
// bukan lompat instan seperti sebelumnya. Arah kembali (Dorong->Awal) pakai durasi SAMA.
enum class HopperState { AT_START, MOVING_TO_PUSH, AT_PUSH, MOVING_TO_START };
HopperState hopperState = HopperState::AT_START;
bool hopperIntervalTestMode = false;   // di-set true/false dari handleCalListKey()/handleParamKey()

// DIUBAH TOTAL: dulu interpolasi linear duration-based (target durasi TETAP, step dihitung
// mundur dari waktu tersisa -- bisa "mundur sebelum sampai" kalau Interval di-set lebih kecil
// dari kecepatan fisik servo yang sebenarnya). SEKARANG step-rate based, SAMA pola dgn PICKER:
// tiap tick (cfg.hopperStepIntervalMs) majukan posisi SEBESAR cfg.hopperStepUs menuju target.
// Selesai ditentukan POSISI (hopperCurrentUs == targetUs), bukan waktu abis -- gak mungkin lagi
// "declare selesai" sebelum benar-benar sampai. Durasi total jadi HASIL (jarak/kecepatan step).
bool updateHopperTrajectory(uint16_t targetUs) {
  if (hopperCurrentUs == targetUs) return true;
  // BARU: StepIntervalMs=0 = "opsional/tanpa jeda" -- step tiap loop tick, secepat mungkin
  // (cuma dibatasi kecepatan loop asli). Operator tanggung sendiri resiko fisiknya (lihat
  // komentar di handleParamKey case 11/12).
  if (cfg.hopperStepIntervalMs > 0 && millis() - lastHopperStepMs < cfg.hopperStepIntervalMs) return false;
  lastHopperStepMs = millis();
  int32_t diff = (int32_t)targetUs - (int32_t)hopperCurrentUs;
  int16_t step = (abs(diff) < (int32_t)cfg.hopperStepUs) ? (int16_t)diff
                 : (diff > 0 ? (int16_t)cfg.hopperStepUs : -(int16_t)cfg.hopperStepUs);
  hopperCurrentUs = (uint16_t)constrain((int32_t)hopperCurrentUs + step, (int32_t)HOPPER_MIN_US, (int32_t)HOPPER_MAX_US);
  hopperSetUs(hopperCurrentUs);
  return (hopperCurrentUs == targetUs);
}

void handleHopper() {
  // BARU: kalau operator SEDANG di layar kalibrasi Hopper Step/StepInterval, paksa FSM tetap
  // bersiklus LIVE walau currentState bukan RUNNING_OR_MOVING -- supaya kecepatan bisa
  // diamati/diverifikasi langsung sambil nilai disesuaikan, tanpa perlu START produksi.
  if (currentState != NodeState::RUNNING_OR_MOVING && !hopperIntervalTestMode) {
    if (hopperState != HopperState::AT_START) {
      hopperCurrentUs = cfg.hopperStartUs; hopperSetUs(hopperCurrentUs); hopperState = HopperState::AT_START;
    }
    return;
  }
  switch (hopperState) {
    case HopperState::AT_START:
      hopperState = HopperState::MOVING_TO_PUSH;
      break;
    case HopperState::MOVING_TO_PUSH:
      if (updateHopperTrajectory(cfg.hopperPushUs)) hopperState = HopperState::AT_PUSH;
      break;
    case HopperState::AT_PUSH:
      hopperState = HopperState::MOVING_TO_START;
      break;
    case HopperState::MOVING_TO_START:
      if (updateHopperTrajectory(cfg.hopperStartUs)) hopperState = HopperState::AT_START;
      break;
  }
}

// DIPERBAIKI: STBY dipakai BERSAMA oleh conveyor (channel B) & motor A (channel baru) --
// kalau ditulis terpisah di 2 tempat, salah satu akan menimpa yang lain (kelas bug yang sama
// dgn LED_RUN sebelumnya). Satu fungsi terpadu, dipanggil ulang di kedua sisi.
uint8_t motorAState = 0;   // 0=stop, 1=maju, 2=mundur
// DIPERBAIKI (pola sama dgn temuan STOCKER B01): STBY/BIN/AIN sebelumnya ditulis
// via I2C TIAP LOOP selagi motor jalan, walau arah/state TIDAK berubah -- redundant
// I2C write (~650-700us per panggilan, dikonfirmasi dari source Adafruit_BusIO).
// Sekarang di-cache, cuma tulis saat benar-benar berubah.
bool lastConv1StbyState = false;
void updateConv1Stby() {
  bool conveyorRunning = (currentState == NodeState::RUNNING_OR_MOVING);
  bool stbyNow = conveyorRunning || motorAState != 0;
  if (stbyNow != lastConv1StbyState) { io.write(CH::CONV1_STBY, stbyNow); lastConv1StbyState = stbyNow; }
}

bool lastConveyorDirWritten = false;
bool conveyorDirCacheValid = false;   // dirWritten belum pernah diisi -- paksa tulis pertama kali
void handleConveyor() {
  updateConv1Stby();
  if (currentState != NodeState::RUNNING_OR_MOVING) { ledcWrite(LEDC_CH_CONV1, 0); conveyorDirCacheValid = false; return; }
  if (!conveyorDirCacheValid || cfg.conveyorDir != lastConveyorDirWritten) {
    motorWrite(CH::CONV1_BIN1, CH::CONV1_BIN2, cfg.conveyorDir);
    lastConveyorDirWritten = cfg.conveyorDir;
    conveyorDirCacheValid = true;
  }
  ledcWrite(LEDC_CH_CONV1, cfg.conveyorSpeed);
}

// DIUBAH TOTAL: palang dulu solenoid relay (1 pulsa HIGH, pegas balik sendiri). SEKARANG linear
// actuator (motor DC di channel Motor A) -- harus di-drive AKTIF 2 arah: PUSH (maju, cfg.palangPushMs)
// lalu RETRACT (mundur, cfg.palangRetractMs), TIDAK ada pegas yang otomatis menariknya balik.
// Dipindah KE ATAS setMotorA() karena dipakai sbg guard di situ (lihat bug ditemukan di bawah).
enum class PalangState { IDLE, PUSHING, RETRACTING };
PalangState palangState = PalangState::IDLE;
uint32_t palangStateAt = 0;

// BARU: motor A -- channel yang tadinya menganggur di chip TB6612FNG yang sama dgn conveyor,
// SEKARANG dipakai jadi actuator palang (lihat handlePalangQueue) -- fungsi ini tetap dipakai
// utk jog/test manual (Test Command/Serial MOTORA), independen dari siklus otomatis palang.
// DIPERBAIKI (bug ditemukan): channel ini dipakai BARENG jog manual & siklus otomatis palang
// TANPA saling kunci -- jog manual bisa nyelonong di tengah PUSHING/RETRACTING dan nge-override
// arah motor sampai transisi state berikutnya (motorWrite() otomatis cuma ditulis SEKALI saat
// masuk state, bukan tiap loop), bikin reject sungguhan bisa gagal separuh jalan. Sekarang jog
// manual DITOLAK selama palang otomatis masih jalan -- produksi/safety menang drpd diagnostik.
void setMotorA(uint8_t dirState) {
  if (palangState != PalangState::IDLE) {
    Serial.println("[MOTORA] Ditolak -- palang otomatis sedang jalan (PUSHING/RETRACTING), tunggu selesai");
    return;
  }
  motorAState = dirState;
  if (dirState == 0) { io.write(CH::CONV1_AIN1, LOW); io.write(CH::CONV1_AIN2, LOW); ledcWrite(LEDC_CH_MOTORA, 0); }
  else { motorWrite(CH::CONV1_AIN1, CH::CONV1_AIN2, dirState == 1); ledcWrite(LEDC_CH_MOTORA, cfg.palangSpeed); }
  updateConv1Stby();
}

void handlePalangQueue() {
  uint32_t now = millis();
  if (qHead != qTail && !palangPending && palangState == PalangState::IDLE) {
    PendingClass &pc = pendingQ[qHead];
    if (pc.isReject) { palangTriggerAt = pc.scanTimeMs + calculateTOF(); palangPending = true; }
    qHead = (qHead + 1) % 4;
  }
  if (palangPending && palangState == PalangState::IDLE && now >= palangTriggerAt) {
    motorWrite(CH::CONV1_AIN1, CH::CONV1_AIN2, cfg.palangDir);   // arah PUSH sesuai kalibrasi
    ledcWrite(LEDC_CH_MOTORA, cfg.palangSpeed);
    motorAState = 1;   // BARU -- supaya updateConv1Stby() tau motor lagi jalan (STBY ikut ON)
    triggerBuzzerBeep();   // notifikasi audio saat objek REJECT ditolak
    palangActive = true;
    palangState = PalangState::PUSHING;
    palangStateAt = now;
    rejectCount++;
    mb.Hreg(Reg::REJECT_COUNT, (uint16_t)rejectCount);
    Serial.printf("[SORTER] Palang PUSH mulai! rejectCount=%lu\n", (unsigned long)rejectCount);
    palangPending = false;
  }
  if (palangState == PalangState::PUSHING && now - palangStateAt >= cfg.palangPushMs) {
    motorWrite(CH::CONV1_AIN1, CH::CONV1_AIN2, !cfg.palangDir);   // arah RETRACT -- kebalikan PUSH
    palangState = PalangState::RETRACTING;
    palangStateAt = now;
    Serial.println("[SORTER] Palang RETRACT mulai");
  }
  if (palangState == PalangState::RETRACTING && now - palangStateAt >= cfg.palangRetractMs) {
    ledcWrite(LEDC_CH_MOTORA, 0);
    motorAState = 0;
    updateConv1Stby();
    palangState = PalangState::IDLE;
    palangActive = false;
    Serial.println("[SORTER] Palang RETRACT selesai, siap siklus berikutnya");
  }
}

void handleSensors() {
  bool s = io.read(CH::PROX_PASS);
  uint32_t now = millis();
  if (s != lastProxState && now - lastProxEdge > 30) {
    lastProxEdge = now; lastProxState = s;
    if (s == LOW) { passCount++; mb.Hreg(Reg::PASS_COUNT, (uint16_t)passCount); }
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH ke aktif-LOW sesuai instruksi terbaru
    currentState = NodeState::ESTOPPED;
    io.write(CH::CONV1_STBY, LOW);
    ledcWrite(LEDC_CH_CONV1, 0);
    // DIUBAH: palang sekarang motor DC (bukan relay) -- E-stop WAJIB potong daya motor langsung,
    // BUKAN coba retract dulu (itu masih butuh motor jalan, kontradiktif sama tujuan E-stop).
    // State palang dipaksa balik IDLE -- kalau lagi di tengah PUSH/RETRACT saat E-stop ditekan,
    // siklus itu DIBATALKAN, bukan dilanjut otomatis setelah E-stop dilepas.
    ledcWrite(LEDC_CH_MOTORA, 0); motorAState = 0;
    palangState = PalangState::IDLE; palangActive = false; palangPending = false;
    return;
  }
  if (currentState == NodeState::ESTOPPED) currentState = NodeState::IDLE;   // diam, TIDAK auto-RUNNING (D11)
}

// ============================================================
// OTA (WiFi) -- update firmware tanpa colok-cabut USB. Lihat wifi_credentials.h
// (gitignored, isi asli SSID/password/OTA password per file .example di include/).
// ============================================================
bool otaReady = false;   // true == WiFi connect & ArduinoOTA siap terima update sekarang
bool otaBegun = false;   // ArduinoOTA.begin() sudah dipanggil sekali (callback ter-register)

// Dipanggil dari ArduinoOTA.onStart() -- proses tulis flash BLOCKING selama beberapa detik,
// jadi motor/actuator yang lagi jalan HARUS dipaksa berhenti dulu (LEDC hardware PWM tetap
// jalan otonom walau CPU sibuk nulis flash, jadi tidak otomatis berhenti sendiri).
void otaSafeStop() {
  ledcWrite(LEDC_CH_CONV1, 0);
  io.write(CH::CONV1_STBY, LOW);
  ledcWrite(LEDC_CH_MOTORA, 0); motorAState = 0;
  palangState = PalangState::IDLE; palangActive = false; palangPending = false;
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

// --- NVS: persist cfg (BARU -- sebelumnya cfg SELALU reset ke default tiap boot, tidak tersimpan) ---
void loadConfigFromNvs() {
  prefs.begin("sorter_cal", true);
  if (prefs.isKey("cfg")) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
void saveConfigToNvs() {
  prefs.begin("sorter_cal", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
// BARU: reset ke default -- hapus NVS "sorter_cal" (namespace ini HANYA menyimpan cfg, aman dibersihkan
// penuh) DAN reset variabel di RAM ke nilai default compile-time (SorterConfig{} = default member init)
void resetConfigToDefault() {
  prefs.begin("sorter_cal", false);
  prefs.clear();
  prefs.end();
  cfg = SorterConfig();
  Serial.println("[RESET] SorterConfig dikembalikan ke default & NVS dihapus");
}

// --- Logic command, DIPAKAI BERSAMA oleh jalur Modbus (onCmdWrite) DAN Serial (handleSerialCommand) ---
const char* stateText(NodeState s);

void applyCommand(uint16_t opcode, uint16_t arg) {
  switch ((Cmd)opcode) {
    case Cmd::START:
      if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
        Serial.println("[CMD] START ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
      } else {
        currentState = NodeState::RUNNING_OR_MOVING;
      }
      break;
    case Cmd::STOP:  currentState = NodeState::IDLE; break;
    case Cmd::RESET_FAULT:
      if (faultCode != 0) lastFaultCode = faultCode;   // BARU -- breadcrumb, simpan SEBELUM di-nol-kan
      faultCode = 0; currentState = NodeState::IDLE;
      break;
    // DIUBAH: hopperIntervalMs (target durasi tetap) sudah tidak dipakai lagi -- diganti
    // hopperStepUs/hopperStepIntervalMs (kecepatan step tetap). Opcode Modbus ini di-arahkan ulang
    // ke hopperStepIntervalMs (padanan paling dekat) -- hopperStepUs cuma bisa diatur via
    // LCD/Serial utk sekarang (1 opcode cuma bawa 1 arg, gak cukup utk 2 parameter baru).
    case Cmd::SET_HOPPER_INTERVAL: cfg.hopperStepIntervalMs = (uint16_t)constrain(arg, 0, 500); break;
    case Cmd::SET_CONVEYOR_SPEED:  cfg.conveyorSpeed = (uint8_t)constrain(arg, 0, 255); break;
    case Cmd::SET_CONVEYOR_DIR:    cfg.conveyorDir = (arg != 0); break;
    case Cmd::RESET_COUNTERS:
      passCount = 0; rejectCount = 0;
      mb.Hreg(Reg::PASS_COUNT, 0); mb.Hreg(Reg::REJECT_COUNT, 0);
      break;
    case Cmd::SET_MOTOR_A: setMotorA((uint8_t)constrain(arg, 0, 2)); break;
    case Cmd::TEST_HOPPER_CYCLE: startTestHopperCycle(); break;
    case Cmd::TEST_TRIGGER_PALANG:
      enqueueClassification(true, millis());
      Serial.println("[SORTER] TEST_TRIGGER_PALANG -- simulasi reject dikirim");
      break;
    case Cmd::TEST_FAULT:
      faultCode = 99; currentState = NodeState::FAULT;
      break;
    default: Serial.printf("[CMD] opcode %u tidak dikenal\n", opcode); break;
  }
}

// ============================================================
// MENU LCD+Keypad -- 3 KATEGORI UTAMA:
//   1. Setting Kalibrasi (nilai yg tersimpan ke NVS)
//   2. Test I/O (channel MCP23017 + I2C scan + System Info)
//   3. Test Command (simulasi command SEOLAH dikirim node lain via Modbus --
//      utk verifikasi opcode benar-benar menghasilkan aksi fisik yg sesuai)
//
// Skema tombol KONSISTEN di semua layar LIST: A=naik B=turun C=pilih D=kembali
// Layar EDIT NILAI (jog angka): A=naik nilai B=turun nilai C=ganti step D=kembali #=SIMPAN
// ============================================================
#define FW_VERSION "v.01.00.25082026.21.17"
constexpr const char* FW_BUILD = __DATE__ " " __TIME__;

enum class MenuState { NONE, TOP_SELECT, CAL_LIST, JOG_PARAM,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_CATEGORY, TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_RELAY, TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET };
MenuState menuState = MenuState::NONE;
uint8_t jogStepIdx = 0;
constexpr int16_t JOG_STEPS[4] = {5, 10, 20, 50};

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

// --- LEVEL 0: TOP MENU (3 kategori) ---
constexpr uint8_t TOP_COUNT = 3;
const char* TOP_LABELS[TOP_COUNT] = { "Setting Kalibrasi", "Test I/O", "Test Command" };
uint8_t topCursor = 0;

void drawTopMenuSorter() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI (list param, masing2 masuk ke JOG_PARAM) ---
// DIUBAH: palang sekarang linear actuator (motor DC), bukan relay -- "Palang Pulse" pecah jadi
// Speed/Dir/Push/Retract (4 parameter, dulu cuma 1 ON-OFF). Speed pakai channel Motor A yang sama.
// DIUBAH: "Hopper Interval(ms)" (target durasi tetap) diganti "Hopper Step(us)" + "Hopper Step
// Interval(ms)" (kecepatan tetap, SAMA pola dgn "Speed (Step/Interval)" PICKER) -- durasi total
// jadi hasil jarak/kecepatan, bukan dipaksa satu angka yang bisa gak realistis fisiknya.
constexpr uint8_t CAL_COUNT = 15;
const char* CAL_LABELS[CAL_COUNT] = { "Conveyor Speed", "Conveyor Dir", "Palang Speed", "Palang Dir",
                                        "Palang Push (ms)", "Palang Retract (ms)", "Dist (TOF mm)", "Mm/s Max",
                                        "Hopper Titik Awal", "Hopper Titik Dorong", "Hopper Step (us)", "Hopper Step Interval(ms)",
                                        "Buzzer On (ms)", "Buzzer Off (ms)", "Reset ke Default" };
uint8_t calCursor = 0;
uint8_t selParam = 0;

void drawCalList() { drawListMenu("SETTING KALIBRASI", CAL_LABELS, CAL_COUNT, calCursor); }

// --- BARU: konfirmasi Reset ke Default -- aksi merusak (hapus NVS), wajib konfirmasi 2 langkah ---
void drawConfirmReset() {
  lcd.clear();
  lcdPrint(0, 0, "RESET KE DEFAULT?");
  lcdPrint(0, 1, "Semua kalibrasi akan");
  lcdPrint(0, 2, "HILANG, TAK BS BATAL");
  lcdPrint(0, 3, "C=YA,RESET D=batal");
}

void handleConfirmResetKey(char key) {
  if (key == 'C') {
    resetConfigToDefault();
    lcdPrint(0, 3, "SUDAH DIRESET!      ");
    menuState = MenuState::CAL_LIST;
  } else if (key == 'D') {
    Serial.println("[CAL] Reset dibatalkan");
    menuState = MenuState::CAL_LIST;
    drawCalList();
  }
}

void drawParamMenu();   // DIPERBAIKI (bug pre-existing): forward declaration -- dipanggil di
                        // handleCalListKey() di bawah SEBELUM definisi lengkapnya, tanpa ini file
                        // tidak akan compile sama sekali ("drawParamMenu tidak dikenal di scope ini")

void handleCalListKey(char key) {
  if (key == 'A') { calCursor = (calCursor == 0) ? CAL_COUNT - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % CAL_COUNT; drawCalList(); }
  else if (key == 'C') {
    if (calCursor == 14) {   // "Reset ke Default" -- minta konfirmasi dulu, bukan langsung eksekusi
      menuState = MenuState::CONFIRM_RESET;
      drawConfirmReset();
    } else {
      selParam = calCursor + 1;
      // BARU: aktifkan test-live di KEDUA layar Hopper Step(11)/Step Interval(12) -- operator
      // perlu liat efeknya live pas ngatur salah satu dari dua parameter ini.
      hopperIntervalTestMode = (selParam == 11 || selParam == 12);
      menuState = MenuState::JOG_PARAM;
      lcd.clear(); drawParamMenu();
    }
  }
  else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

void drawParamMenu() {
  switch (selParam) {
    case 1: lcdPrint(0, 0, "CONVEYOR SPEED"); break;
    case 2: lcdPrint(0, 0, "CONVEYOR DIR"); break;
    case 3: lcdPrint(0, 0, "PALANG SPEED"); break;
    case 4: lcdPrint(0, 0, "PALANG DIR"); break;
    case 5: lcdPrint(0, 0, "PALANG PUSH (ms)"); break;
    case 6: lcdPrint(0, 0, "PALANG RETRACT(ms)"); break;
    case 7: lcdPrint(0, 0, "DIST_MM"); break;
    case 8: lcdPrint(0, 0, "MM_PER_SEC_MAX"); break;
    case 9: lcdPrint(0, 0, "HOPPER TITIK AWAL"); break;
    case 10: lcdPrint(0, 0, "HOPPER TITIK DORONG"); break;
    case 11: lcdPrint(0, 0, "HOPPER STEP (us)"); break;
    case 12: lcdPrint(0, 0, "HOPPER STEP INTV(ms)"); break;
    case 13: lcdPrint(0, 0, "BUZZER ON (ms)"); break;
    case 14: lcdPrint(0, 0, "BUZZER OFF (ms)"); break;
  }
  String line1;
  switch (selParam) {
    case 1: line1 = "Nilai:" + String(cfg.conveyorSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 2: line1 = "Arah:" + String(cfg.conveyorDir ? "FORWARD" : "REVERSE"); break;
    case 3: line1 = "Nilai:" + String(cfg.palangSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 4: line1 = "Arah PUSH:" + String(cfg.palangDir ? "FORWARD" : "REVERSE"); break;
    case 5: line1 = "Nilai:" + String(cfg.palangPushMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 6: line1 = "Nilai:" + String(cfg.palangRetractMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 7: line1 = "Nilai:" + String(cfg.distMm, 1) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 8: line1 = "Nilai:" + String(cfg.mmPerSecAtMaxPwm, 1) + " Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 9: line1 = "us:" + String(cfg.hopperStartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 10: line1 = "us:" + String(cfg.hopperPushUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 11: line1 = "us:" + String(cfg.hopperStepUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 12: line1 = "ms:" + String(cfg.hopperStepIntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 13: line1 = "ms:" + String(cfg.buzzerOnMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 14: line1 = "ms:" + String(cfg.buzzerOffMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
  }
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, (selParam == 2 || selParam == 4) ? "A=Forward B=Reverse" : "A+ B- C:step");
  lcdPrint(0, 3, "#=SIMPAN D=kembali");
}

void handleParamKey(char key) {
  int16_t step = JOG_STEPS[jogStepIdx];
  switch (selParam) {
    case 1:
      if (key == 'A') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed + step, 0, 255);
      else if (key == 'B') cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed - step, 0, 255);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 2:
      if (key == 'A') cfg.conveyorDir = true;
      else if (key == 'B') cfg.conveyorDir = false;
      break;
    case 3:
      if (key == 'A') cfg.palangSpeed = (uint8_t)constrain((int)cfg.palangSpeed + step, 0, 255);
      else if (key == 'B') cfg.palangSpeed = (uint8_t)constrain((int)cfg.palangSpeed - step, 0, 255);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      if (motorAState != 0) ledcWrite(LEDC_CH_MOTORA, cfg.palangSpeed);   // update live kalau sedang jalan
      break;
    case 4:
      if (key == 'A') cfg.palangDir = true;
      else if (key == 'B') cfg.palangDir = false;
      break;
    case 5:
      if (key == 'A') cfg.palangPushMs = (uint16_t)constrain((int)cfg.palangPushMs + step, 50, 3000);
      else if (key == 'B') cfg.palangPushMs = (uint16_t)constrain((int)cfg.palangPushMs - step, 50, 3000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 6:
      if (key == 'A') cfg.palangRetractMs = (uint16_t)constrain((int)cfg.palangRetractMs + step, 50, 3000);
      else if (key == 'B') cfg.palangRetractMs = (uint16_t)constrain((int)cfg.palangRetractMs - step, 50, 3000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 7:
      if (key == 'A') cfg.distMm += step;
      else if (key == 'B') cfg.distMm = max(0.0f, cfg.distMm - step);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 8:
      if (key == 'A') cfg.mmPerSecAtMaxPwm += step;
      else if (key == 'B') cfg.mmPerSecAtMaxPwm = max(5.0f, cfg.mmPerSecAtMaxPwm - step);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    // DIKEMBALIKAN ke POSITIONAL: preview live langsung untuk Titik Awal/Dorong (servo diam
    // sendiri di posisi yang diperintah, aman ditahan tanpa risiko putar tak terkendali).
    case 9:
      if (key == 'A') cfg.hopperStartUs = (uint16_t)constrain((int)cfg.hopperStartUs + step, (int)HOPPER_MIN_US, (int)HOPPER_MAX_US);
      else if (key == 'B') cfg.hopperStartUs = (uint16_t)constrain((int)cfg.hopperStartUs - step, (int)HOPPER_MIN_US, (int)HOPPER_MAX_US);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      hopperCurrentUs = cfg.hopperStartUs; hopperSetUs(hopperCurrentUs);   // preview live, sinkron tracking
      break;
    case 10:
      if (key == 'A') cfg.hopperPushUs = (uint16_t)constrain((int)cfg.hopperPushUs + step, (int)HOPPER_MIN_US, (int)HOPPER_MAX_US);
      else if (key == 'B') cfg.hopperPushUs = (uint16_t)constrain((int)cfg.hopperPushUs - step, (int)HOPPER_MIN_US, (int)HOPPER_MAX_US);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      hopperCurrentUs = cfg.hopperPushUs; hopperSetUs(hopperCurrentUs);   // preview live, sinkron tracking
      break;
    case 11:
      // BARU: batas atas dinaikkan 500->2500 -- rentang servo cuma 2000us (500-2500), jadi
      // stepUs=2500 udah cukup lompat langsung ujung ke ujung dalam 1 tick kalau operator mau.
      if (key == 'A') cfg.hopperStepUs = (uint16_t)constrain((int)cfg.hopperStepUs + step, 1, 2500);
      else if (key == 'B') cfg.hopperStepUs = (uint16_t)constrain((int)cfg.hopperStepUs - step, 1, 2500);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 12:
      // BARU: batas bawah 1->0 -- 0 = tanpa jeda (step tiap loop tick, secepat mungkin).
      // AWAS: stepUs besar + interval 0 = gerakan "selesai" logika dalam 1 tick, servo fisik
      // belum tentu ngejar secepat itu -- balik ke resiko yang sama kayak model duration-based
      // lama kalau dipaksa lebih cepat dari kemampuan mekanis servo. Kalibrasi hati-hati.
      if (key == 'A') cfg.hopperStepIntervalMs = (uint16_t)constrain((int)cfg.hopperStepIntervalMs + step, 0, 500);
      else if (key == 'B') cfg.hopperStepIntervalMs = (uint16_t)constrain((int)cfg.hopperStepIntervalMs - step, 0, 500);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 13:
      if (key == 'A') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs + step * 10, 50, 5000);
      else if (key == 'B') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs - step * 10, 50, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 14:
      if (key == 'A') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs + step * 10, 50, 5000);
      else if (key == 'B') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs - step * 10, 50, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
  }
  if (key == '#') {
    saveConfigToNvs();
    lcdPrint(0, 3, "TERSIMPAN ke NVS!");
    Serial.println("[CAL] SorterConfig disimpan ke NVS");
    return;
  }
  if (key == 'D') { hopperIntervalTestMode = false; menuState = MenuState::CAL_LIST; drawCalList(); return; }
  drawParamMenu();
}

// --- LEVEL 1b: TEST I/O (channel MCP + I2C Scan + System Info, dalam 1 list) ---
// autoControlled: channel yang DIKONTROL OTOMATIS oleh sistem (LED_RUN/LED_OPERATION/LED_MANUAL) --
// TIDAK BISA ditoggle manual (akan langsung ditimpa balik oleh updateUniversalIndicators() tiap
// loop, percuma/membingungkan kalau dicoba) -- cuma bisa DIBACA live, sama seperti channel input.
struct IOTestItem { const char* label; uint8_t ch; bool autoControlled; };
void drawTestIoCategory();   // forward declaration -- dipakai di banyak handler 'D' sebelum definisinya di bawah

// --- Kategori: Test Output (semua output, penamaan disederhanakan) ---
constexpr uint8_t OUTPUT_TEST_COUNT = 7;
IOTestItem OUTPUT_TEST_ITEMS[OUTPUT_TEST_COUNT] = {
  {"OPR",    CH::LED_OPERATION, true},
  {"RUN",    CH::LED_RUN,       true},
  {"MANUAL", CH::LED_MANUAL,    true},
  {"FAULT",  CH::LED_FAULT,     false},
  {"BUZZER", CH::BUZZER,        false},
  {"CONV1_STBY", CH::CONV1_STBY, false},
  // DIUBAH: dulu "PALANG" (relay reject) -- sekarang palang pakai motor DC (Motor A), RLY1
  // jadi channel spare, tetap bisa ditest manual raw di sini.
  {"RLY1(spare)", CH::RLY1,  false},
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
  // BARU: rate-limit toggle -- beban induktif (relay, motor DC) menghasilkan lonjakan tegangan
  // balik (back-EMF) tiap kali dimatikan. Toggle terlalu cepat berturut-turut bisa mengganggu
  // integritas sinyal I2C ke LCD/Keypad/MCP -- gejala: display acak, respons putus-putus
  // (temuan awal saat uji PALANG_RELAY). Mitigasi software; akar masalah tetap di hardware.
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
// dari alias/peran yang dipakai node ini (mis. LIM_1..10 tetap diuji semua walau node ini
// cuma pakai sebagian sebagai limit switch beneran).
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
constexpr uint8_t INPUT_TEST_MAX = LIMIT_TEST_COUNT;   // terbesar dari 3 kategori -- ukuran array cache nilai
bool testInputLiveLastVal[INPUT_TEST_MAX];    // nilai TERAKHIR yang ditampilkan per item -- cegah kedip
uint8_t testInputLastScrollTop = 255;         // beda dari nilai valid manapun -- paksa redraw penuh pertama kali

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

// --- Kategori: Test RS485 -- diagnostik komunikasi Modbus RTU ---
// BARU: LCD cuma 20 kolom -- FW_VERSION penuh (format "v.XX.XX.DDMMYYYY.HH.MM", 23 karakter)
// TIDAK MUAT. Tampilkan cuma MAJOR.MINOR.tanggal di LCD, versi lengkap TETAP utuh di Serial STATUS.
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

// --- Kategori: Test Modul -- sub-menu pilihan JENIS modul, SAMA di semua 4 node (board
// universal -- channel/pin sudah didefinisikan sama persis terlepas modul itu benar2
// terpasang fisik atau tidak di board node ini).
bool testModFirstDraw = true;
String testModLine1 = "", testModLine2 = "";

// --- PCA9685 minimal raw driver (Wire langsung, TANPA library) -- utk Test Modul Servo.
// --- PCA9685 (driver sudah didefinisikan di atas, dipakai bersama produksi hopper & test modul) ---

constexpr uint8_t MODULE_TYPE_COUNT = 4;
const char* MODULE_TYPE_LABELS[MODULE_TYPE_COUNT] = { "Stepper", "Motor DC", "Relay", "Servo" };
uint8_t moduleTypeCursor = 0;
void drawModuleTypeSelect() { drawListMenu("TEST MODUL", MODULE_TYPE_LABELS, MODULE_TYPE_COUNT, moduleTypeCursor); }
void handleModuleTypeKey(char key);

// --- Modul: Stepper (STEP_1/2/3 + DIR_1/2/3_MCP + EN_123) -- SORTER TIDAK punya infrastruktur
// motion (curPos/homed/startJog) spt STOCKER, jadi pulsa manual sederhana. SENGAJA blocking
// SESAAT (~100 pulsa x 800us ~ 80ms per tekan tombol) -- ini PENGECUALIAN yang wajar karena
// murni test manual 1x-tekan, BUKAN bagian continuous production loop (beda konteks dgn
// prinsip non-blocking yang kita jaga ketat utk motion produksi).
uint8_t testModAxis = 0;
void testStepperPulse(uint8_t axisIdx, bool forward) {
  const uint8_t stepPins[3] = { GP::STEP_1, GP::STEP_2, GP::STEP_3 };
  const uint8_t dirPins[3]  = { CH::DIR_1_MCP, CH::DIR_2_MCP, CH::DIR_3_MCP };
  io.write(CH::EN_123, LOW);   // aktifkan (aktif-LOW)
  io.write(dirPins[axisIdx], forward ? HIGH : LOW);
  delayMicroseconds(50);
  for (uint16_t i = 0; i < 100; i++) {
    digitalWrite(stepPins[axisIdx], HIGH);
    delayMicroseconds(4);
    digitalWrite(stepPins[axisIdx], LOW);
    delayMicroseconds(800);
  }
  io.write(CH::EN_123, HIGH);   // nonaktifkan lagi setelah selesai -- tidak perlu tetap ON antar-test
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

// --- Modul: Motor DC (AIN1/AIN2/BIN1/BIN2/STBY) -- ON/OFF sederhana, non-blocking.
// CATATAN: di SORTER, channel ini SAMA dgn CONV1_AIN/CONV1_BIN produksi (alias) -- kalau
// conveyor produksi sedang RUNNING, menu ini SEBAIKNYA tidak dipakai bersamaan (berpotensi
// tumpang tindih kontrol). Guard IDLE-only belum ditambahkan di versi ini -- operator perlu
// pastikan sendiri conveyor sedang STOP sebelum test modul motor DC.
uint8_t testModMotorAState = 0, testModMotorBState = 0;
void testModSetMotor(bool channelA, uint8_t dirState) {
  // DIPERBAIKI (bug ditemukan): channel A (AIN1/AIN2/LEDC_CH_MOTORA) SAMA PERSIS dgn actuator
  // palang otomatis -- test manual di sini bisa nabrak siklus reject sungguhan (setMotorA punya
  // guard yang sama, lihat komentarnya). Channel B (conveyor) TIDAK di-guard di sini -- operator
  // tetap perlu pastikan sendiri conveyor produksi sedang STOP sebelum test channel itu.
  if (channelA && palangState != PalangState::IDLE) {
    Serial.println("[TEST-MOTORDC] ChA ditolak -- palang otomatis sedang jalan, tunggu selesai");
    return;
  }
  // DIPERBAIKI: sebelumnya cuma set arah (AIN/BIN) + STBY, TANPA pernah ledcWrite ke PWMA/PWMB
  // -- TB6612FNG butuh sinyal PWM aktif di pin PWMA/PWMB spy motor benar-benar berputar, arah
  // doang tidak cukup. Pakai LEDC channel & speed yang SAMA dgn produksi (LEDC_CH_MOTORA/CONV1,
  // cfg.palangSpeed/conveyorSpeed) supaya hasil test representatif thd kecepatan aktual produksi.
  uint8_t ain1 = channelA ? CH::AIN1 : CH::BIN1, ain2 = channelA ? CH::AIN2 : CH::BIN2;
  uint8_t pwmCh = channelA ? LEDC_CH_MOTORA : LEDC_CH_CONV1;
  uint8_t speed = channelA ? cfg.palangSpeed : cfg.conveyorSpeed;
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

// --- Modul: Relay (RLY1/RLY2) -- toggle, rate-limit 300ms (beban induktif) ---
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

// --- Modul: Servo (PCA9685) -- cek deteksi I2C dulu, baru izinkan gerak. Pakai objek pwm
// GLOBAL yang sama dengan hopper produksi (SATU driver, bukan 2 cara berbeda lagi) ---
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
      case 2: menuState = MenuState::TEST_INPUT_CATEGORY; inputCatCursor = 0; drawInputCategorySelect(); break;
      case 3: menuState = MenuState::TEST_RS485; drawTestRs485(); break;
      case 4: menuState = MenuState::TEST_MODULE_SELECT; moduleTypeCursor = 0; drawModuleTypeSelect(); break;
    }
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

// --- LEVEL 1c: TEST COMMAND (simulasi command SEOLAH dari node lain via Modbus) ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 11;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"START",             Cmd::START,               0},
  {"STOP",               Cmd::STOP,                0},
  {"RESET_FAULT",        Cmd::RESET_FAULT,         0},
  {"SET_SPEED (test=200)",Cmd::SET_CONVEYOR_SPEED, 200},
  {"SET_DIR (test=0)",   Cmd::SET_CONVEYOR_DIR,    0},
  {"RESET_COUNTERS",     Cmd::RESET_COUNTERS,      0},
  {"MOTOR_A Maju",       Cmd::SET_MOTOR_A,         1},
  {"MOTOR_A Stop",       Cmd::SET_MOTOR_A,         0},
  {"Test Hopper M/M",    Cmd::TEST_HOPPER_CYCLE,   0},
  {"TEST_TRIGGER_PALANG",Cmd::TEST_TRIGGER_PALANG, 0},
  {"TEST_FAULT",         Cmd::TEST_FAULT,          0},
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
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

void handleTopMenuKeySorter(char key) {
  if (key == 'A') { topCursor = (topCursor == 0) ? TOP_COUNT - 1 : topCursor - 1; drawTopMenuSorter(); }
  else if (key == 'B') { topCursor = (topCursor + 1) % TOP_COUNT; drawTopMenuSorter(); }
  else if (key == 'C') {
    switch (topCursor) {
      case 0: menuState = MenuState::CAL_LIST; calCursor = 0; drawCalList(); break;
      case 1: menuState = MenuState::TEST_IO_CATEGORY; testIoCatCursor = 0; drawTestIoCategory(); break;
      case 2: menuState = MenuState::TEST_CMD_LIST; cmdTestCursor = 0; drawTestCmdList(); break;
    }
  } else if (key == 'D') {
    menuState = MenuState::NONE;
    lcd.clear();
    Serial.println("[CAL] Keluar mode kalibrasi");
  }
}

uint16_t onCmdWrite(TRegister* reg, uint16_t val) {
  if (menuState != MenuState::NONE) {
    Serial.println("[SORTER] Command Modbus diabaikan -- mode kalibrasi aktif (§12.6)");
    return val;
  }
  lastRs485Rx = millis(); modbusEverUsed = true;
  uint16_t opcode = val;
  uint16_t arg = mb.Hreg(Reg::CMD_ARG);
  uint16_t seq = mb.Hreg(Reg::CMD_SEQ);
  uint16_t lastAcked = mb.Hreg(Reg::CMD_ACK_SEQ);

  if (seq == lastAcked) {
    Serial.printf("[SORTER] CMD seq=%u sudah pernah diproses -- diabaikan\n", seq);
    return val;
  }

  Serial.printf("[SORTER] CMD (Modbus) opcode=%u arg=%u seq=%u\n", opcode, arg, seq);
  applyCommand(opcode, arg);
  mb.Hreg(Reg::CMD_ACK_SEQ, seq);
  return val;
}

ActivityCode activityCode();   // forward declaration -- didefinisikan di bawah, dipakai di handleSerialCommand()

void handleSerialCommand() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int spaceIdx = line.indexOf(' ');
  String cmd = (spaceIdx == -1) ? line : line.substring(0, spaceIdx);
  uint16_t arg = (spaceIdx == -1) ? 0 : line.substring(spaceIdx + 1).toInt();
  cmd.toUpperCase();

  if (cmd == "START")            applyCommand((uint16_t)Cmd::START, 0);
  else if (cmd == "STOP")        applyCommand((uint16_t)Cmd::STOP, 0);
  else if (cmd == "RESET_FAULT") applyCommand((uint16_t)Cmd::RESET_FAULT, 0);
  else if (cmd == "SPEED")       applyCommand((uint16_t)Cmd::SET_CONVEYOR_SPEED, arg);
  else if (cmd == "DIR")         applyCommand((uint16_t)Cmd::SET_CONVEYOR_DIR, arg);
  else if (cmd == "RESET_COUNT") applyCommand((uint16_t)Cmd::RESET_COUNTERS, 0);
  else if (cmd == "TEST_PALANG") applyCommand((uint16_t)Cmd::TEST_TRIGGER_PALANG, 0);
  else if (cmd == "TEST_FAULT")  applyCommand((uint16_t)Cmd::TEST_FAULT, 0);
  else if (cmd == "TESTPASS")    { enqueueClassification(false, millis()); Serial.println("[SERIAL] Simulasi klasifikasi PASS"); }
  else if (cmd == "TESTREJECT")  { enqueueClassification(true, millis());  Serial.println("[SERIAL] Simulasi klasifikasi REJECT"); }
  else if (cmd == "MOTORA") { uint8_t d = line.substring(spaceIdx + 1).toInt(); setMotorA(constrain(d, 0, 2)); Serial.printf("[MOTORA] state=%u\n", d); }
  else if (cmd == "MOTORASPEED") {
    cfg.palangSpeed = (uint8_t)constrain((int)line.substring(spaceIdx + 1).toInt(), 0, 255);
    saveConfigToNvs();
    if (motorAState != 0) ledcWrite(LEDC_CH_MOTORA, cfg.palangSpeed);   // update live kalau sedang jalan
    Serial.printf("[MOTORASPEED] %u\n", cfg.palangSpeed);
  }
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s fault=%u pass=%lu reject=%lu ESTOP=%d speed=%u dir=%d dist=%.1f mmps=%.1f\n",
                  stateText(currentState), faultCode, (unsigned long)passCount, (unsigned long)rejectCount,
                  io.read(CH::ESTOP), cfg.conveyorSpeed, cfg.conveyorDir, cfg.distMm, cfg.mmPerSecAtMaxPwm);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus motorA=%u palangState=%d\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000), motorAState, (int)palangState);
    Serial.printf("[STATUS] FW=%s build=%s freeHeap=%u\n", FW_VERSION, FW_BUILD, ESP.getFreeHeap());
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] Command tersedia:");
    Serial.println("  START | STOP | RESET_FAULT | RESET_COUNT");
    Serial.println("  SPEED <0-255> | DIR <0|1>");
    Serial.println("  TEST_PALANG | TEST_FAULT | TESTPASS | TESTREJECT | MOTORA <0-2> | MOTORASPEED <0-255> | STATUS");
  }
  else Serial.printf("[SERIAL] Command '%s' tidak dikenal -- ketik HELP untuk daftar\n", cmd.c_str());
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

// BARU: teks aktivitas SPESIFIK (bukan cuma state generik) -- terjemahkan sub-state internal
// jadi info yang jelas dibaca operator, mis. "Conveyor Jalan+Palang" bukan cuma "RUNNING"
String activityText() {
  if (currentState == NodeState::FAULT) return "FAULT!";
  if (currentState == NodeState::ESTOPPED) return "E-STOP!";
  bool conveyorRun = (currentState == NodeState::RUNNING_OR_MOVING);
  if (!conveyorRun && motorAState == 0) return "Diam";
  if (conveyorRun && motorAState == 0) return palangActive ? "Conv+Palang" : "Conveyor ON";
  if (!conveyorRun && motorAState != 0) return "Motor A ON";
  return palangActive ? "Conv+Plg+MtrA" : "Conv+MotorA";   // keduanya jalan bersamaan -- disingkat, muat 20 kolom LCD dgn prefix [AUTO]
}

// BARU: versi kode numerik dari activityText() -- dikirim ke register Modbus supaya OrangePi
// juga bisa identifikasi aktivitas spesifik, bukan cuma lewat tampilan LCD lokal
ActivityCode activityCode() {
  if (currentState == NodeState::FAULT) return ActivityCode::FAULT_AKTIF;
  if (currentState == NodeState::ESTOPPED) return ActivityCode::ESTOP_AKTIF;
  if (motorAState != 0) return ActivityCode::MOTOR_A_JALAN;
  if (currentState != NodeState::RUNNING_OR_MOVING) return ActivityCode::DIAM;
  return palangActive ? ActivityCode::CONVEYOR_JALAN_PALANG_AKTIF : ActivityCode::CONVEYOR_JALAN;
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

// --- BARU: scan I2C, tentukan lcdPresent/keypadPresent -- LCD/Keypad OPSIONAL, hanya utk kalibrasi ---
void scanI2CAndDetectOptional() {
  Serial.println("[I2C-SCAN] Memindai bus I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C-SCAN]   Terdeteksi di 0x%02X\n", addr);
      if (addr == I2CAddr::LCD)    lcdPresent = true;
      if (addr == I2CAddr::KEYPAD) keypadPresent = true;
    }
  }
  Serial.printf("[I2C-SCAN] LCD %s, Keypad %s -- keduanya OPSIONAL, produksi tetap jalan tanpanya\n",
                lcdPresent ? "ADA" : "TIDAK ADA (kalibrasi via Serial saja)",
                keypadPresent ? "ADA" : "TIDAK ADA (kalibrasi via Serial saja)");
}

// --- BARU: deteksi hot-plug LCD/Keypad DUA ARAH (colok DAN cabut) -- sebelumnya HANYA dicek
// sekali saat boot. Dipanggil berkala dari loop(), berlaku di state apa pun --
// cuma ping I2C ringan, tidak butuh sistem diam dulu. ---
void checkLcdKeypadHotplug() {
  static uint32_t lastHotplugCheck = 0;
  if (millis() - lastHotplugCheck < 3000) return;
  lastHotplugCheck = millis();

  Wire.beginTransmission(I2CAddr::LCD);
  bool lcdPing = (Wire.endTransmission() == 0);
  if (!lcdPresent && lcdPing) {
    lcdPresent = true;
    lcd.init(); lcd.backlight();
    Serial.println("[HOTPLUG] LCD baru terdeteksi -- diinisialisasi live, mulai dipakai sekarang");
  } else if (lcdPresent && !lcdPing) {
    lcdPresent = false;
    Serial.println("[HOTPLUG] !!! LCD TIDAK TERDETEKSI LAGI -- dihentikan (kalibrasi via Serial tetap jalan) !!!");
  }

  Wire.beginTransmission(I2CAddr::KEYPAD);
  bool kpPing = (Wire.endTransmission() == 0);
  if (!keypadPresent && kpPing) {
    keypadPresent = true;
    keypad.begin();
    Serial.println("[HOTPLUG] Keypad baru terdeteksi -- diinisialisasi live, mulai dipakai sekarang");
  } else if (keypadPresent && !kpPing) {
    keypadPresent = false;
    Serial.println("[HOTPLUG] !!! Keypad TIDAK TERDETEKSI LAGI !!!");
    // PENTING: kalau keypad hilang SAAT sedang di dalam menu kalibrasi, node bisa "terjebak" --
    // menu men-jeda command eksternal (Serial/Modbus), dan tanpa keypad tidak ada cara keluar manual.
    // Paksa keluar otomatis dari menu supaya command eksternal aktif lagi, cegah node macet permanen.
    if (menuState != MenuState::NONE) {
      menuState = MenuState::NONE;
      hopperIntervalTestMode = false;   // BARU -- cegah hopper terus bersiklus tanpa henti kalau keypad hilang saat test aktif
      Serial.println("[HOTPLUG] Keluar OTOMATIS dari mode kalibrasi -- keypad hilang, command eksternal diaktifkan lagi (cegah node terjebak)");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] SORTER mulai");

  Wire.begin(Pin::I2C_SDA, Pin::I2C_SCL, Pin::I2C_FREQ_HZ);
  scanI2CAndDetectOptional();

  if (lcdPresent) {
    lcd.init(); lcd.backlight();
    lcdPrint(0, 0, "SORTER");
    lcdPrint(0, 1, "Conv1+Palang+Prox");
    Serial.println("[BOOT] LCD OK");
  } else {
    Serial.println("[BOOT] LCD dilewati (tidak terdeteksi) -- kalibrasi tetap bisa via Serial (HELP)");
  }

  if (keypadPresent) {
    keypad.begin();
    Serial.println("[BOOT] Keypad OK");
  } else {
    Serial.println("[BOOT] Keypad dilewati (tidak terdeteksi)");
  }
  lcdBootProgress("I2C + LCD/Keypad");

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }

  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  io.pinMode(CH::CONV1_BIN1, OUTPUT); io.pinMode(CH::CONV1_BIN2, OUTPUT); io.pinMode(CH::CONV1_STBY, OUTPUT);
  // DIUBAH: RLY1 (dulu PALANG_RELAY) sekarang channel spare -- palang produksi pakai Motor A
  // (motor DC linear actuator). Tetap di-pinMode supaya bisa ditest manual raw (Test Output).
  io.pinMode(CH::RLY1, OUTPUT);
  // Motor A (AIN1/AIN2) -- SEKARANG dipakai produksi utk actuator palang, BUKAN lagi spare.
  io.pinMode(CH::CONV1_AIN1, OUTPUT); io.pinMode(CH::CONV1_AIN2, OUTPUT);
  io.pinMode(CH::PROX_PASS, INPUT_PULLUP);
  io.pinMode(CH::BTN_TEST_PASS, INPUT_PULLUP);     // BARU
  io.pinMode(CH::BTN_TEST_REJECT, INPUT_PULLUP);   // BARU
  // DIPERBAIKI (bug ditemukan): channel Test Modul berikut TIDAK PERNAH di-pinMode OUTPUT --
  // io.write() ke pin yang masih default INPUT MCP23017 TIDAK ADA efek fisik sama sekali.
  // Ini penyebab "Test Modul cuma RLY2 doang yang jalan" -- selebihnya diam bukan krn hardware.
  io.pinMode(CH::DIR_1_MCP, OUTPUT); io.pinMode(CH::DIR_2_MCP, OUTPUT); io.pinMode(CH::DIR_3_MCP, OUTPUT);
  io.pinMode(CH::EN_123, OUTPUT);
  io.pinMode(CH::RLY2, OUTPUT);
  // TBD HOPPER: io.pinMode(CH::LIMIT_HOPPER, INPUT_PULLUP); -- aktifkan lagi nanti
  Serial.printf("[BOOT] MCP23017: %s, semua pinMode selesai (hopper belum di-setup, TBD)\n", ioOk ? "OK" : "GAGAL");
  lcdBootProgress("I/O Expander MCP23017");

  // DIPERBAIKI (regresi ditemukan): pwm.begin() TIDAK BOLEH ditunda sampai setelah
  // loadConfigFromNvs() -- itu BUG BARU yang tidak sengaja saya perkenalkan saat
  // memperbaiki bug lain sebelumnya. PICKER (yang terbukti bekerja) memanggil pwm.begin()
  // SEGERA setelah io.pinMode() selesai, BUKAN ditunda sampai setelah akses NVS/flash.
  // pwm.begin() (inisialisasi CHIP) dipisah dari hopperSetUs() (PENERAPAN kalibrasi) --
  // yang PERTAMA harus SEDINI mungkin, yang KEDUA baru boleh setelah cfg dimuat dari NVS.
  pwm.begin(); pwm.setPWMFreq(50);
  // DITAMBAHKAN (akar masalah ditemukan!): pin Output-Enable PCA9685 (aktif-LOW) TIDAK PERNAH
  // ditarik LOW sebelumnya -- komunikasi I2C berhasil sempurna, PWM dihitung benar secara
  // internal, TAPI output pin secara FISIK tidak pernah aktif tanpa OE=LOW. Inilah penyebab
  // sebenarnya servo diam meski software terlihat benar, dan kenapa power-cycle sungguhan
  // (yang mereset MCP23017 channel OE ke default) selalu mematikan servo lagi.
  io.pinMode(CH::OE_PCA, OUTPUT);
  io.write(CH::OE_PCA, LOW);
  Serial.println("[BOOT] PCA9685 diinisialisasi (Adafruit_PWMServoDriver)");
  lcdBootProgress("Servo PCA9685");

  ledcSetup(LEDC_CH_CONV1, PWM_FREQ, PWM_RES);
  ledcAttachPin(GP::CONV1_PWM, LEDC_CH_CONV1);
  ledcSetup(LEDC_CH_MOTORA, PWM_FREQ, PWM_RES);       // BARU
  ledcAttachPin(GP::MOTOR_A_PWM, LEDC_CH_MOTORA);     // BARU
  // DIPERBAIKI (bug ditemukan): pin native STEP_1/2/3 dipakai testStepperPulse() (Test Modul
  // Stepper) via digitalWrite() langsung, TAPI TIDAK PERNAH di-pinMode(OUTPUT) -- default ESP32
  // setelah boot adalah INPUT, jadi pulsa STEP tidak pernah keluar secara elektrik. SORTER tidak
  // punya stepper produksi (DIR/EN lewat MCP sudah pinMode di tempat lain), jadi cuma pin ini
  // yang kelewat -- sama pola bug dgn PWM Motor DC yang sudah diperbaiki.
  pinMode(GP::STEP_1, OUTPUT); pinMode(GP::STEP_2, OUTPUT); pinMode(GP::STEP_3, OUTPUT);
  // BARU: Hopper servo -- pakai GP::STEP_1 (GPIO23), pin ini MENGANGGUR di board SORTER
  // (tidak ada stepper terpasang fisik untuk role SORTER)
  // DIHAPUS: ledcSetup/ledcAttachPin utk hopper -- servo sekarang lewat PCA9685 (I2C), bukan LEDC
  Serial.println("[BOOT] LEDC PWM conveyor1 + motor A OK");
  lcdBootProgress("PWM Conveyor/Motor");

  loadConfigFromNvs();   // BARU -- cfg sekarang persist antar reboot, sebelumnya selalu reset ke default
  Serial.println("[BOOT] SorterConfig dimuat dari NVS (kalau pernah disimpan)");

  hopperCurrentUs = cfg.hopperStartUs;   // BARU -- sinkronkan tracking posisi step-based dgn boot
  hopperSetUs(hopperCurrentUs);   // posisi awal servo saat boot, PAKAI nilai kalibrasi tersimpan
  Serial.println("[BOOT] Hopper diposisikan ke titik awal");
  lcdBootProgress("Kalibrasi NVS");

  Serial2.begin(Rs485Cfg::BAUD, SERIAL_8N1, Rs485Cfg::RX_PIN, Rs485Cfg::TX_PIN);
  mb.begin(&Serial2);
  mb.slave(Rs485Cfg::SLAVE_ID);
  mb.addHreg(Reg::STATE, (uint16_t)currentState);
  mb.addHreg(Reg::FAULT_CODE, faultCode);
  mb.addHreg(Reg::CMD, 0);
  mb.addHreg(Reg::CMD_ARG, 0);
  mb.addHreg(Reg::CMD_SEQ, 0);
  mb.addHreg(Reg::CMD_ACK_SEQ, 0);
  mb.addHreg(Reg::HEARTBEAT, 0);
  mb.addHreg(Reg::PASS_COUNT, 0);
  mb.addHreg(Reg::REJECT_COUNT, 0);
  mb.addHreg(Reg::CLASSIFY_IS_REJECT, 0);
  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- lihat penjelasan di registers.h
  mb.addHreg(Reg::ACTIVITY_CODE, 0);
  mb.addHreg(Reg::I2C_ERROR_COUNT, 0);
  mb.addHreg(Reg::LAST_FAULT_CODE, 0);
  mb.addHreg(Reg::UPTIME_SEC, 0);
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  mb.onSetHreg(Reg::CLASSIFY_IS_REJECT, onClassifyWrite);
  Serial.println("[BOOT] Modbus + register lengkap OK");
  lcdBootProgress("Modbus RS485");

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;

  setupOTA();
  lcdBootProgress("WiFi OTA");

  Serial.println("[BOOT] setup SELESAI");
  Serial.println("[BOOT] Ketik HELP di sini (serial monitor) untuk daftar command uji tanpa QModMaster");
}

void loop() {
  mb.task();
  updateOTA();

  // --- Menu kalibrasi (HANYA kalau keypad terdeteksi) ---
  if (keypadPresent) {
    char key = keypad.scan();
    if (key) {
      // DIUBAH: boleh masuk menu dari IDLE ATAU RUNNING sekarang -- logic fisik tetap jalan
      // selagi menu aktif (lihat di bawah), jadi tidak perlu lagi paksa STOP dulu sebelum kalibrasi.
      if (menuState == MenuState::NONE && key == '*') {
        menuState = MenuState::TOP_SELECT;
        if (lcdPresent) drawTopMenuSorter();
        Serial.println("[CAL] Masuk mode kalibrasi (command EKSTERNAL/Serial/Modbus dijeda, logic fisik TETAP jalan)");
      } else if (menuState == MenuState::TOP_SELECT) {
        handleTopMenuKeySorter(key);
      } else if (menuState == MenuState::CAL_LIST) {
        handleCalListKey(key);
      } else if (menuState == MenuState::JOG_PARAM) {
        handleParamKey(key);
      } else if (menuState == MenuState::TEST_IO_CATEGORY) {
        handleTestIoCategoryKey(key);
      } else if (menuState == MenuState::TEST_IO_I2CSCAN) {
        handleTestIoI2CScanKey(key);
      } else if (menuState == MenuState::TEST_OUTPUT_LIST) {
        handleTestOutputListKey(key);
      } else if (menuState == MenuState::TEST_OUTPUT_ITEM) {
        handleTestOutputItemKey(key);
      } else if (menuState == MenuState::TEST_INPUT_CATEGORY) {
        handleInputCategoryKey(key);
      } else if (menuState == MenuState::TEST_INPUT_LIST) {
        handleTestInputListKey(key);
      } else if (menuState == MenuState::TEST_RS485) {
        handleTestRs485Key(key);
      } else if (menuState == MenuState::TEST_MODULE_SELECT) {
        handleModuleTypeKey(key);
      } else if (menuState == MenuState::TEST_MOD_STEPPER) {
        handleTestModStepperKey(key);
      } else if (menuState == MenuState::TEST_MOD_MOTORDC) {
        handleTestModMotorDCKey(key);
      } else if (menuState == MenuState::TEST_MOD_RELAY) {
        handleTestModRelayKey(key);
      } else if (menuState == MenuState::TEST_MOD_SERVO) {
        handleTestModServoKey(key);
      } else if (menuState == MenuState::TEST_CMD_LIST) {
        handleTestCmdListKey(key);
      } else if (menuState == MenuState::CONFIRM_RESET) {
        handleConfirmResetKey(key);
      }
    }
  }

  mb.Hreg(Reg::STATE, (uint16_t)currentState);
  mb.Hreg(Reg::FAULT_CODE, faultCode);
  // BARU: update register Lapis 2 + Lapis 3 tiap loop -- OrangePi bisa poll kapan saja
  mb.Hreg(Reg::ACTIVITY_CODE, (uint16_t)activityCode());
  mb.Hreg(Reg::I2C_ERROR_COUNT, i2cErrorCount);
  mb.Hreg(Reg::LAST_FAULT_CODE, lastFaultCode);
  mb.Hreg(Reg::UPTIME_SEC, (uint16_t)(millis() / 1000));

  // --- Logic fisik (safety, conveyor, palang, sensor) SELALU jalan, baik menu aktif atau tidak ---
  // DIUBAH: sebelumnya bagian ini ikut di-skip saat menu aktif, sekarang tetap jalan supaya
  // RUN/STOP dari menu (§handleTopMenuKeySorter) benar-benar terlihat efeknya.
  // BARU: cek ulang kesehatan I2C MCP berkala (bukan cuma sekali di boot) -- degradasi koneksi
  // di tengah jalan (chip sempat OK saat begin() tapi longgar/gagal belakangan, mis. Error 263)
  // sekarang terdeteksi, tidak diam-diam terus jalan dengan data sensor basi
  static uint32_t lastMcpHealthCheck = 0;
  if (millis() - lastMcpHealthCheck > 2000) {
    lastMcpHealthCheck = millis();
    bool healthy = io.recheckHealth();
    if (!healthy) {
      i2cErrorCount++;   // BARU -- catat SETIAP kegagalan, terlepas dari state, utk diagnostik EMI
      if (currentState != NodeState::FAULT && currentState != NodeState::ESTOPPED) {
        faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING;
        currentState = NodeState::FAULT;
        Serial.println("[SAFETY] !!! MCP23017 berhenti merespons I2C -- FAULT dipicu (data sensor tidak bisa dipercaya) !!!");
      }
    }
  }

  checkLcdKeypadHotplug();   // BARU -- berlaku di state apa pun, tidak perlu IDLE dulu
  // DIPERBAIKI: jeda kontrol otomatis SELAMA sedang test channel auto-controlled di Test I/O --
  // supaya toggle manual operator tidak langsung ditimpa balik oleh logic otomatis
  bool pauseAutoIndicators = (menuState == MenuState::TEST_OUTPUT_ITEM && OUTPUT_TEST_ITEMS[outputTestCursor].autoControlled);
  if (!pauseAutoIndicators) updateUniversalIndicators();

  handleSafety();
  if (currentState != NodeState::ESTOPPED && currentState != NodeState::FAULT) {
    handleHopper();   // BARU -- servo hopper aktif otomatis selama RUNNING_OR_MOVING
    updateTestHopperCycle();   // BARU -- proses test sequence non-blocking (independen dari FSM hopper produksi)
    updateBuzzerBeep();   // BARU -- proses siklus ON-OFF buzzer non-blocking, TIDAK di-skip walau menu aktif
    handleConveyor();
    handlePalangQueue();
    handleSensors();
    handleTestButtons();   // BARU -- push button uji manual PASS/REJECT
  }

  // DIPERBAIKI: refresh live utk kategori Test Output (auto-controlled saja), Test Input
  // (semua input, selalu live), Test RS485, dan Test Modul
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

  if (menuState != MenuState::NONE) {
    return;   // HANYA command EKSTERNAL (Serial/Modbus) yang dijeda saat menu aktif (§12.6), bukan logic fisik
  }

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
    Serial.printf("[LOOP] state=%s pass=%lu reject=%lu ESTOP=%d\n",
                  stateText(currentState), (unsigned long)passCount, (unsigned long)rejectCount,
                  io.read(CH::ESTOP));
  }

  static bool lastRawProx = HIGH;
  bool rawProx = io.read(CH::PROX_PASS);
  if (rawProx != lastRawProx) {
    Serial.printf("[PROX-RAW] berubah: %d -> %d (%s)\n", lastRawProx, rawProx,
                  rawProx == LOW ? "LOW = idealnya ini saat objek TERDETEKSI" : "HIGH = idle/tidak terdeteksi");
    lastRawProx = rawProx;
  }

  if (lcdPresent) {
    static uint32_t lastLcdRefresh = 0;
    if (millis() - lastLcdRefresh > 500) {
      lastLcdRefresh = millis();
      lcdPrint(0, 0, "[AUTO] " + activityText());
      lcdPrint(0, 2, "State:" + String(stateText(currentState)));
      lcdPrint(0, 3, "P:" + String(passCount) + " R:" + String(rejectCount));
    }
  }
}