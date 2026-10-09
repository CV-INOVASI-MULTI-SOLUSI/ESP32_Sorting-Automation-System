// ============================================================
// SORTER — Conveyor1 + Palang + Proximity + Hopper + Menu Kalibrasi LCD/Keypad
// DIPERBAIKI (2026-09-20): baris ini dulu bilang "Hopper DINONAKTIFKAN sementara" -- sudah
// basi jauh sebelum ini. Hopper servo + rack-pinion AKTIF dan bersiklus otomatis selama
// RUNNING (lihat handleHopper()), laju umpannya diatur cfg.hopperCycleGapMs.
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
#include "kalibrasi_modbus.h"   // BARU 2026-10-09: backup/restore kalibrasi
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

// BARU (2026-09-29): teks kiri rata kiri, teks kanan rata kanan, dalam satu baris.
void lcdKiriKanan(uint8_t row, const String& kiri, const String& kanan) {
  int celah = (int)LcdCfg::COLS - (int)kiri.length() - (int)kanan.length();
  if (celah < 1) celah = 1;   // tetap dipisah; lcdPrint() memotong sisanya di kolom terakhir
  String baris = kiri;
  for (int i = 0; i < celah; i++) baris += ' ';
  lcdPrint(0, row, baris + kanan);
}

// BARU (2026-09-29): dua pasangan "label : nilai" -- kiri rata kiri, kanan rata kanan.
//
// Format lengkap "Pass : 12" tidak selalu muat. 20 kolom sudah habis begitu kedua angka
// mencapai 3 digit ("Pass : 123" + "Reject : 123" = 22 karakter). Karena itu dicoba
// bertingkat -- lengkap, lalu tanpa spasi di sekitar ':', lalu label singkat -- dan dipakai
// tingkat pertama yang muat. Yang dikorbankan selalu labelnya, bukan angkanya: sampai 7 digit
// per angka, angka tidak pernah terpotong.
void lcdPasangan(uint8_t row, const char* labelKiri, const char* singkatKiri, const String& nilaiKiri,
                 const char* labelKanan, const char* singkatKanan, const String& nilaiKanan) {
  String kiri, kanan;
  for (uint8_t tingkat = 0; tingkat < 3; tingkat++) {
    const char* pemisah = (tingkat == 0) ? " : " : ":";
    kiri  = String(tingkat < 2 ? labelKiri  : singkatKiri)  + pemisah + nilaiKiri;
    kanan = String(tingkat < 2 ? labelKanan : singkatKanan) + pemisah + nilaiKanan;
    if (kiri.length() + kanan.length() + 1 <= LcdCfg::COLS) break;
  }
  lcdKiriKanan(row, kiri, kanan);
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
// BARU: pemisah MAIN/TEST -- default FALSE (fail-safe, boot-IDLE). Cmd::START menyalakan ini
// (BUKAN cuma currentState), Cmd::STOP mematikannya lagi. Command TEST (SET_MOTOR_A/
// TEST_HOPPER_CYCLE/TEST_TRIGGER_PALANG) DITOLAK TOTAL selama mainModeActive.
bool mainModeActive = false;
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
  // BARU: hopper masih open-loop (gak ada sensor posisi fisik) -- "sampai" itu murni hitungan
  // software (hopperCurrentUs==target), BUKAN konfirmasi fisik beneran. Kalau kalibrasi Step/
  // Interval kebetulan lebih cepat dari kemampuan fisik servo, software bisa declare "sampai"
  // SEBELUM servo beneran nyampe -- gejala: "belum sampai ujung udah balik lagi". Field ini
  // kasih jeda tahan WAJIB di titik dorong sebelum retract, independen dari kalibrasi Step/
  // Interval, jadi ada jaminan waktu fisik walau kalibrasi kurang pas. DITARUH DI AKHIR struct.
  uint16_t hopperPushHoldMs = 300;
  // BARU (2026-09-20): jeda DIAM antar siklus hopper. Sebelumnya HopperState::AT_START
  // langsung lompat ke MOVING_TO_PUSH tanpa jeda sama sekali -- selama RUNNING hopper
  // dorong-tarik nonstop, tidak ada kendali laju umpan objek ke conveyor sama sekali.
  // 0 = perilaku lama (nonstop). DITARUH DI AKHIR struct supaya blob NVS lama tetap kebaca.
  uint16_t hopperCycleGapMs = 1000;
  // BARU (2026-09-21): jarak fisik PROX_1 (start) ke PROX_2 (finish), dipakai uji kecepatan
  // objek di conveyor. Ukur sendiri dengan penggaris lalu isikan di sini lewat menu
  // kalibrasi. DITARUH DI AKHIR struct supaya blob NVS lama tetap terbaca.
  uint16_t speedTestDistMm = 200;
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
// DIUBAH (2026-09-20): kedalaman 4 -> 8. Antrian penuh berarti hasil REJECT dibuang, dan 4
// slot itu tipis: satu siklus palang (push+retract) saja sudah ~600ms, sementara objek bisa
// datang beruntun lebih cepat dari itu.
constexpr uint8_t CLASS_QUEUE_LEN = 8;
PendingClass pendingQ[CLASS_QUEUE_LEN];
uint8_t qHead = 0, qTail = 0;
// BARU: berapa REJECT yang gagal jadi dorongan palang (antrian penuh ATAU giliran dorongnya
// sudah basi). Dikirim ke Reg::REJECT_MISSED_COUNT -- dulu hilang diam-diam tanpa jejak.
uint32_t rejectMissedCount = 0;

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
enum class TestHopperCycleStage { NONE, MOVING_TO_PUSH, PUSH_HOLD, MOVING_TO_START };
TestHopperCycleStage testHopperCycleStage = TestHopperCycleStage::NONE;
uint32_t testHopperPushHoldStartMs = 0;
// BARU (2026-09-28): jumlah siklus hopper yang SELESAI, dihitung dari kedua jalur -- siklus
// produksi/berulang di handleHopper() dan sekali-jalan di updateTestHopperCycle(). Dipakai
// layar Uji Hopper untuk membandingkan "berapa kali mendorong" dengan "berapa objek yang
// benar-benar lewat sensor". Selisih keduanya itulah yang dicari saat menguji hopper.
uint32_t hopperSiklusSelesai = 0;
// BARU (2026-10-05): umpan hopper dijeda (Cmd::SET_HOPPER_JEDA). Siklus yang SEDANG berjalan
// diselesaikan dulu -- hopper berhenti di posisi awal, tidak pernah menggantung di tengah dorongan.
bool hopperDijeda = false;
void setHopperJeda(bool jeda) {
  if (hopperDijeda != jeda) Serial.printf("[HOPPER] umpan %s\n", jeda ? "DIJEDA" : "dilanjutkan");
  hopperDijeda = jeda;
  mb.Hreg(Reg::HOPPER_DIJEDA, jeda ? 1 : 0);
}
// DIUBAH (2026-09-29): mengembalikan true kalau siklus benar-benar dimulai -- dipakai layar
// Test Command untuk menampilkan DITERIMA/DITOLAK di panel, bukan cuma di Serial.
bool startTestHopperCycle() {
  if (currentState != NodeState::IDLE) { Serial.println("[TEST] Hopper cycle ditolak -- node sedang tidak IDLE"); return false; }
  testHopperCycleStage = TestHopperCycleStage::MOVING_TO_PUSH;
  Serial.println("[TEST] Hopper cycle dimulai (pakai nilai kalibrasi Step/StepInterval/PushHold)");
  return true;
}
void updateTestHopperCycle() {
  switch (testHopperCycleStage) {
    case TestHopperCycleStage::MOVING_TO_PUSH:
      if (updateHopperTrajectory(cfg.hopperPushUs)) {
        testHopperCycleStage = TestHopperCycleStage::PUSH_HOLD;
        testHopperPushHoldStartMs = millis();
      }
      break;
    case TestHopperCycleStage::PUSH_HOLD:
      if (millis() - testHopperPushHoldStartMs >= cfg.hopperPushHoldMs) testHopperCycleStage = TestHopperCycleStage::MOVING_TO_START;
      break;
    case TestHopperCycleStage::MOVING_TO_START:
      if (updateHopperTrajectory(cfg.hopperStartUs)) {
        testHopperCycleStage = TestHopperCycleStage::NONE;
        hopperSiklusSelesai++;   // BARU -- dipakai layar Uji Hopper
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
  uint8_t next = (qTail + 1) % CLASS_QUEUE_LEN;
  if (next == qHead) {
    // DIPERBAIKI (2026-09-20): dulu cuma `return` -- antrian penuh = hasil klasifikasi HILANG
    // tanpa counter, tanpa log, tanpa fault. Kalau yang hilang itu REJECT, objeknya lolos ke
    // jalur pass dan tidak ada satu pun cara untuk tahu kejadiannya pernah terjadi.
    if (isReject) {
      rejectMissedCount++;
      mb.Hreg(Reg::REJECT_MISSED_COUNT, (uint16_t)rejectMissedCount);
      Serial.printf("[SORTER] !!! REJECT HILANG -- antrian klasifikasi PENUH (%u slot). Total terlewat=%lu !!!\n",
                    CLASS_QUEUE_LEN, (unsigned long)rejectMissedCount);
    } else {
      Serial.println("[SORTER] Klasifikasi pass dibuang -- antrian penuh (tidak berdampak, pass tidak butuh aksi)");
    }
    return;
  }
  pendingQ[qTail] = {true, scanTimeMs, isReject};
  qTail = next;
}

// BARU: kosongkan antrian + batalkan dorongan yang belum terjadi. Dipakai Cmd::STOP.
// Tanpa ini, klasifikasi lama tetap mengendap di antrian: begitu START dikirim lagi, semuanya
// langsung meletus sekaligus dengan waktu tempuh (TOF) yang sudah lama kedaluwarsa.
void clearClassificationQueue(const char* alasan) {
  uint8_t sisa = (qTail >= qHead) ? (qTail - qHead) : (uint8_t)(CLASS_QUEUE_LEN - qHead + qTail);
  qHead = qTail = 0;
  if (sisa > 0) Serial.printf("[SORTER] Antrian klasifikasi dikosongkan (%u entri) -- %s\n", sisa, alasan);
}

// DIUBAH TOTAL (celah #1): tombol fisik DULU simulasi klasifikasi pass/reject lewat
// enqueueClassification() PERSIS SAMA dgn jalur produksi asli (onClassifyWrite) -- masalahnya
// itu TIDAK di-gate mainModeActive sama sekali, jadi bisa nyelip masuk antrian klasifikasi
// asli kapan saja (termasuk pas produksi beneran jalan). Sekarang tombol jadi trigger TEST-only
// (persis pola Cmd::TEST_HOPPER_CYCLE / Cmd::TEST_TRIGGER_PALANG, ditolak kalau mainModeActive)
// -- konsisten sama tombol node lain (mis. Dispenser BTN_TEST_BOX_FULL).
void handleTestButtons() {
  static bool lastHopper = HIGH, lastPalang = HIGH;
  static uint32_t lastHopperEdge = 0, lastPalangEdge = 0;
  constexpr uint32_t DEBOUNCE_MS = 50;

  bool curHopper = io.read(CH::BTN_TEST_HOPPER);
  if (curHopper == LOW && lastHopper == HIGH && millis() - lastHopperEdge > DEBOUNCE_MS) {
    lastHopperEdge = millis();
    if (mainModeActive) {
      Serial.println("[TEST-BTN] Tombol Hopper ditolak -- MAIN aktif, STOP dulu");
    } else {
      startTestHopperCycle();
      Serial.println("[TEST-BTN] Tombol Hopper ditekan -- TEST_HOPPER_CYCLE");
    }
  }
  lastHopper = curHopper;

  bool curPalang = io.read(CH::BTN_TEST_PALANG);
  if (curPalang == LOW && lastPalang == HIGH && millis() - lastPalangEdge > DEBOUNCE_MS) {
    lastPalangEdge = millis();
    if (mainModeActive) {
      Serial.println("[TEST-BTN] Tombol Palang ditolak -- MAIN aktif, STOP dulu");
    } else {
      enqueueClassification(true, millis());
      Serial.println("[TEST-BTN] Tombol Palang ditekan -- TEST_TRIGGER_PALANG (simulasi reject)");
    }
  }
  lastPalang = curPalang;
}

bool menuIsActive();   // forward declaration -- MenuState baru didefinisikan jauh di bawah

uint16_t onClassifyWrite(TRegister* reg, uint16_t val) {
  // DIPERBAIKI (2026-09-20): jalur ini SATU-SATUNYA callback Modbus di keempat node yang
  // tidak pernah mengecek menu kalibrasi. onCmdWrite() menolak semua command saat operator
  // berada di menu (§12.6), tapi klasifikasi tetap menembus -- artinya operator yang sedang
  // mengkalibrasi, dengan tangan di area palang, tetap bisa membuat Motor A menghentak
  // karena Orange Pi/HuskyLens mengirim hasil klasifikasi. Sekarang ikut ditolak.
  if (menuIsActive()) {
    Serial.println("[SORTER] Klasifikasi diabaikan -- mode kalibrasi aktif (§12.6)");
    return val;
  }
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
enum class HopperState { AT_START, MOVING_TO_PUSH, PUSH_HOLD, MOVING_TO_START };
uint32_t hopperPushHoldStartMs = 0;
uint32_t hopperAtStartSinceMs = 0;   // BARU -- kapan hopper kembali diam di titik awal (utk jeda antar siklus)
HopperState hopperState = HopperState::AT_START;
bool hopperIntervalTestMode = false;   // di-set true/false dari handleCalListKey()/handleParamKey()

// ============================================================
// BARU (2026-09-28): UJI HOPPER -- dipakai layar kalibrasi "Uji Hopper".
//
// Sengaja TIDAK memakai hopperIntervalTestMode walau perilakunya mirip. Flag itu milik layar
// parameter Hopper Step/Step Interval, dan dimatikan oleh jalur keluar layar-layar tersebut.
// Kalau dipakai bersama, satu layar bisa mematikan mode milik layar lain tanpa ada yang tahu.
bool hopperUjiBerulang = false;

// Perintah tahan posisi. 0 = tidak ada. Gerakannya memakai updateHopperTrajectory() yang SAMA
// dengan produksi, bukan lompat langsung ke posisi -- lompatan 1000us sekaligus adalah hentakan
// keras pada servo dan rack-pinion, dan yang teramati jadi bukan gerakan yang sebenarnya dipakai.
uint16_t hopperManualTargetUs = 0;

// hopperSiklusSelesai dideklarasikan lebih ke atas -- updateTestHopperCycle() sudah memakainya.

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
  if (currentState != NodeState::RUNNING_OR_MOVING && !hopperIntervalTestMode && !hopperUjiBerulang) {
    if (hopperState != HopperState::AT_START) {
      hopperCurrentUs = cfg.hopperStartUs; hopperSetUs(hopperCurrentUs); hopperState = HopperState::AT_START;
    }
    hopperAtStartSinceMs = millis();   // BARU -- jeda dihitung dari saat RUNNING dimulai, bukan dari boot
    return;
  }
  switch (hopperState) {
    case HopperState::AT_START:
      // DIPERBAIKI (2026-09-20): dulu langsung lompat ke MOVING_TO_PUSH tanpa jeda -- hopper
      // dorong-tarik nonstop selama RUNNING. Sekarang diam dulu selama cfg.hopperCycleGapMs
      // (0 = perilaku lama). Ini satu-satunya kendali laju umpan yang dimiliki Sorter.
      if (cfg.hopperCycleGapMs > 0 && millis() - hopperAtStartSinceMs < cfg.hopperCycleGapMs) break;
      // BARU (2026-10-05): dijeda -- tetap di posisi awal. Jeda antar siklus dihitung ulang
      // dari saat dilanjutkan, jadi dorongan pertama sesudahnya tidak langsung menyusul.
      if (hopperDijeda && !hopperIntervalTestMode && !hopperUjiBerulang) { hopperAtStartSinceMs = millis(); break; }
      hopperState = HopperState::MOVING_TO_PUSH;
      break;
    case HopperState::MOVING_TO_PUSH:
      if (updateHopperTrajectory(cfg.hopperPushUs)) {
        hopperState = HopperState::PUSH_HOLD;
        hopperPushHoldStartMs = millis();
      }
      break;
    case HopperState::PUSH_HOLD:
      // BARU: jeda tahan WAJIB sebelum retract -- lihat komentar cfg.hopperPushHoldMs. Ini
      // jaminan waktu fisik independen dari kalibrasi Step/Interval, mencegah "belum sampai
      // ujung udah balik lagi" kalau kalibrasi kebetulan lebih cepat dari kemampuan servo asli.
      if (millis() - hopperPushHoldStartMs >= cfg.hopperPushHoldMs) hopperState = HopperState::MOVING_TO_START;
      break;
    case HopperState::MOVING_TO_START:
      if (updateHopperTrajectory(cfg.hopperStartUs)) {
        hopperState = HopperState::AT_START;
        hopperAtStartSinceMs = millis();   // BARU -- mulai hitung jeda antar siklus
        hopperSiklusSelesai++;             // BARU -- dipakai layar Uji Hopper
      }
      break;
  }
}

// ============================================================
// BARU (2026-09-28): perintah TAHAN POSISI untuk layar Uji Hopper.
//
// Ini satu-satunya cara memeriksa rack-pinion sebagai mekanisme. Satu siklus penuh hanya
// menahan titik dorong selama cfg.hopperPushHoldMs (bawaan 300ms) -- terlalu singkat untuk
// menilai apakah pendorong benar-benar mencapai ujung, apakah ada objek yang tersangkut, atau
// apakah Titik Awal sudah cukup mundur sehingga objek berikutnya bisa turun.
// ============================================================
bool hopperSedangBersiklus() {
  return testHopperCycleStage != TestHopperCycleStage::NONE
         || hopperUjiBerulang
         || currentState == NodeState::RUNNING_OR_MOVING;
}

void hopperManual(uint16_t targetUs) {
  if (hopperSedangBersiklus()) {
    Serial.println("[UJI-HOPPER] Tahan posisi ditolak -- siklus hopper sedang jalan");
    return;
  }
  hopperManualTargetUs = constrain(targetUs, HOPPER_MIN_US, HOPPER_MAX_US);
  Serial.printf("[UJI-HOPPER] Tahan di %uus\n", hopperManualTargetUs);
}

void updateHopperManual() {
  if (hopperManualTargetUs == 0) return;
  // Siklus apa pun menang atas perintah manual -- kalau tidak, dua pihak menulis satu servo
  // yang sama tiap tick dan posisinya jadi tarik-menarik.
  if (hopperSedangBersiklus()) { hopperManualTargetUs = 0; return; }
  if (updateHopperTrajectory(hopperManualTargetUs)) {
    // Sudah sampai. Target dilepas, tapi servo TETAP di posisi itu: handleHopper() tidak
    // menyentuhnya selama hopperState masih AT_START, dan tidak ada lagi yang menulis servo.
    hopperManualTargetUs = 0;
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
// BARU (2026-09-21): conveyor boleh dinyalakan dari layar kalibrasi "Uji Kecepatan"
// walaupun node tidak RUNNING. Pola yang sama dengan hopperIntervalTestMode, yang sudah
// lebih dulu memutar hopper selagi operator berada di layar kalibrasinya.
bool speedTestConveyorOn = false;

bool conveyorHarusJalan() {
  return (currentState == NodeState::RUNNING_OR_MOVING) || speedTestConveyorOn;
}

void updateConv1Stby() {
  bool conveyorRunning = conveyorHarusJalan();
  bool stbyNow = conveyorRunning || motorAState != 0;
  if (stbyNow != lastConv1StbyState) { io.write(CH::CONV1_STBY, stbyNow); lastConv1StbyState = stbyNow; }
}

bool lastConveyorDirWritten = false;
bool conveyorDirCacheValid = false;   // dirWritten belum pernah diisi -- paksa tulis pertama kali
void handleConveyor() {
  updateConv1Stby();
  if (!conveyorHarusJalan()) { ledcWrite(LEDC_CH_CONV1, 0); conveyorDirCacheValid = false; return; }
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
// BARU (2026-09-20): berapa lama dorongan palang masih dianggap masuk akal setelah waktu
// idealnya lewat. Di atas ini objeknya dianggap sudah kejauhan -- mendorong sekarang justru
// menyerempet objek yang salah, jadi lebih baik dibatalkan dan dicatat. Sengaja longgar:
// mendorong telat sedikit masih mengenai objek yang benar karena palangnya tidak setipis itu.
constexpr uint32_t PALANG_STALE_TOLERANCE_MS = 400;

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

// ============================================================
// BARU (2026-09-28): PALANG MANUAL -- dipakai layar kalibrasi "Uji Palang".
//
// Dorongan palang otomatis (handlePalangQueue) SENGAJA ditunda sejauh waktu tempuh objek
// (TOF, ~700ms pada kalibrasi bawaan) dan durasi push/retract-nya juga ditentukan kalibrasi.
// Itu benar untuk produksi, tapi membuat palang tidak bisa diperiksa sebagai mekanisme:
// apakah dia benar-benar menjulur penuh, apakah arahnya sudah benar, apakah dia macet di
// tengah jalan. Semua itu perlu ditahan pada satu posisi, bukan dilihat dalam 300ms.
//
// Arah mengikuti cfg.palangDir, BUKAN angka mentah setMotorA(). Kalau tidak, layar uji akan
// bertentangan dengan kalibrasi Palang Dir -- "ON" bisa berarti retract, dan operator akan
// mengkalibrasi ke arah yang salah tanpa tahu.
constexpr uint32_t PALANG_MANUAL_MAX_MS = 3000;   // batas aman satu perintah manual

// 0 = tidak ada perintah manual. Selain itu = millis() batas waktu motor dipadamkan sendiri.
uint32_t palangManualSampai = 0;
int8_t palangManualArah = 0;   // 0 = diam, +1 = PUSH (julur), -1 = RETRACT (tarik)

// Satu siklus otomatis pakai nilai kalibrasi, TANPA menunggu TOF. Jalur yang dipakai sama
// persis dengan produksi (handlePalangQueue), jadi push/retract/buzzer/rejectCount berperilaku
// identik -- yang dihilangkan hanya penundaannya.
void mulaiSiklusPalangSekarang() {
  palangTriggerAt = millis();
  palangPending = true;
}

void hentikanPalangManual(const char* alasan) {
  if (palangManualArah == 0 && palangManualSampai == 0) return;
  palangManualArah = 0;
  palangManualSampai = 0;
  setMotorA(0);
  Serial.printf("[UJI-PALANG] Motor dimatikan -- %s\n", alasan);
}

void palangManual(int8_t arah) {
  if (palangState != PalangState::IDLE) {
    Serial.println("[UJI-PALANG] Ditolak -- siklus palang otomatis sedang jalan");
    return;
  }
  if (arah == 0) { hentikanPalangManual("diminta STOP"); return; }
  palangManualArah = arah;
  palangManualSampai = millis() + PALANG_MANUAL_MAX_MS;
  // arah +1 (PUSH) dipetakan ke dirState sesuai kalibrasi Palang Dir
  bool maju = (arah > 0) ? cfg.palangDir : !cfg.palangDir;
  setMotorA(maju ? 1 : 2);
  Serial.printf("[UJI-PALANG] %s, mati sendiri dalam %lums\n",
                (arah > 0) ? "PUSH (julur)" : "RETRACT (tarik)",
                (unsigned long)PALANG_MANUAL_MAX_MS);
}

// Batas waktu ini BUKAN kenyamanan, tapi perlindungan: palang adalah linear actuator, dan
// menahannya terus setelah mentok berarti motor stall dengan arus penuh tanpa ada yang
// mematikannya. Kalau operator menekan PUSH lalu meninggalkan panel, tanpa ini motor
// tertahan mentok sampai daya dicabut.
// DIPANGGIL DI LUAR blok bersyarat loop(), bukan di dalamnya bersama handlePalangQueue().
// Alasannya menentukan: FAULT di node ini dipicu TANPA memadamkan motor (mis. MCP23017 lepas
// dari I2C), dan seluruh blok gerak ikut dilewati begitu FAULT aktif. Kalau batas waktu ini
// ada di dalam blok itu, perintah manual yang sedang ditahan tepat saat FAULT terjadi tidak
// akan pernah dipadamkan oleh siapa pun -- motor tertahan mentok sampai daya dicabut. Pola
// yang sama dengan updateBuzzerBeep() yang sudah lebih dulu dipindah keluar karena alasan
// sejenis (buzzer meraung terus kalau fault terjadi di tengah bunyi).
void updatePalangManual() {
  if (palangManualSampai == 0) return;
  if (currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
    hentikanPalangManual("FAULT/E-STOP aktif");
    return;
  }
  if ((int32_t)(millis() - palangManualSampai) >= 0) {
    hentikanPalangManual("batas waktu aman tercapai (mentok tidak boleh ditahan terus)");
  }
}

void handlePalangQueue() {
  uint32_t now = millis();
  if (qHead != qTail && !palangPending && palangState == PalangState::IDLE) {
    PendingClass &pc = pendingQ[qHead];
    if (pc.isReject) {
      uint32_t trigAt = pc.scanTimeMs + calculateTOF();
      // DIPERBAIKI (2026-09-20): antrian hanya diproses saat palang IDLE, padahal satu siklus
      // palang (push+retract) memakan ~600ms. REJECT berikutnya baru sempat diambil setelah
      // itu, dan waktu dorongnya bisa SUDAH LEWAT -- `now >= palangTriggerAt` langsung benar,
      // palang menghentak seketika padahal objeknya sudah jauh melewati palang. Yang kena
      // dorong malah objek di belakangnya (yang mungkin pass). Sekarang dorongan yang sudah
      // basi DIBATALKAN dan dicatat, bukan dipaksakan.
      if ((int32_t)(now - trigAt) > (int32_t)PALANG_STALE_TOLERANCE_MS) {
        rejectMissedCount++;
        mb.Hreg(Reg::REJECT_MISSED_COUNT, (uint16_t)rejectMissedCount);
        Serial.printf("[SORTER] !!! REJECT dilewati -- giliran dorong sudah lewat %ldms (antrian nunggu palang). Total terlewat=%lu !!!\n",
                      (long)(now - trigAt), (unsigned long)rejectMissedCount);
      } else {
        palangTriggerAt = trigAt;
        palangPending = true;
      }
    }
    qHead = (qHead + 1) % CLASS_QUEUE_LEN;
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

// ============================================================
// UJI KECEPATAN OBJEK (BARU 2026-09-21)
//
// PROX_1 (ch19, selama ini menganggur) = garis START.
// PROX_2 (ch18, sensor pass produksi)  = garis FINISH.
// Kecepatan = cfg.speedTestDistMm dibagi selisih waktu kedua tepi.
//
// Sepenuhnya PASIF terhadap logika lain: fungsi ini hanya MEMBACA sensor, tidak
// pernah menyentuh conveyor, palang, hopper, maupun antrian klasifikasi. Jalur
// produksi PROX_2 (handleSensors -> passCount) tidak diubah sama sekali dan tetap
// memakai deteksi tepinya sendiri. Jadi uji ini boleh menyala terus, termasuk
// selama produksi berjalan, tanpa mengganggu apa pun.
//
// Soal ketelitian: waktu yang dicatat adalah saat TEPI MENTAH terjadi, bukan saat
// debounce selesai. Jadi jeda debounce tidak ikut terhitung. Dan karena ambang
// kedua sensor sama persis, sisa kesalahan sistematis apa pun akan saling
// meniadakan di selisih waktunya.
// ============================================================
constexpr uint32_t SPEED_DEBOUNCE_MS = 15;      // cukup untuk menyaring pantulan, ringan untuk timing
constexpr uint32_t SPEED_TIMEOUT_MS  = 15000;   // objek dianggap tidak sampai finish

struct TepiSensor {
  uint8_t channel;
  bool rawLast;
  uint32_t rawChangedAt;   // kapan tepi MENTAH terjadi -- inilah stempel waktu yang dipakai
  bool stabil;
  TepiSensor(uint8_t ch) : channel(ch), rawLast(HIGH), rawChangedAt(0), stabil(HIGH) {}
};

// true TEPAT SEKALI saat sensor benar-benar tertutup (HIGH->LOW yang stabil).
// `saatTepi` diisi waktu tepi mentahnya, bukan waktu pemanggilan.
bool objekLewat(TepiSensor &s, uint32_t &saatTepi) {
  bool raw = io.read(s.channel);
  if (raw != s.rawLast) { s.rawLast = raw; s.rawChangedAt = millis(); }
  if (raw == s.stabil || millis() - s.rawChangedAt < SPEED_DEBOUNCE_MS) return false;
  bool sebelumnya = s.stabil;
  s.stabil = raw;
  if (sebelumnya == HIGH && s.stabil == LOW) { saatTepi = s.rawChangedAt; return true; }
  return false;
}

TepiSensor sensorStart(CH::PROX_1);
TepiSensor sensorFinish(CH::PROX_2);

bool speedSedangMengukur = false;
uint32_t speedMulaiMs = 0;
uint16_t speedLastMmS = 0, speedLastMs = 0, speedSampleCount = 0, speedMmSAtMaxPwm = 0;

void handleSpeedTest() {
  uint32_t saatTepi = 0;

  if (objekLewat(sensorStart, saatTepi)) {
    // Tepi START baru selalu menggantikan yang lama. Kalau objek sebelumnya tidak
    // pernah sampai finish, lebih baik mulai ulang dari objek terbaru daripada
    // menahan pengukuran yang sudah pasti gagal.
    speedSedangMengukur = true;
    speedMulaiMs = saatTepi;
    Serial.println("[KECEPATAN] START -- objek melewati PROX_1");
  }

  if (objekLewat(sensorFinish, saatTepi)) {
    if (speedSedangMengukur) {
      uint32_t selisih = saatTepi - speedMulaiMs;
      speedSedangMengukur = false;
      if (selisih == 0) {
        Serial.println("[KECEPATAN] Diabaikan -- selisih waktu 0 ms, dua sensor kemungkinan terpicu bersamaan");
      } else {
        uint32_t mmS = ((uint32_t)cfg.speedTestDistMm * 1000UL) / selisih;
        speedLastMs = (uint16_t)min(selisih, (uint32_t)65535);
        speedLastMmS = (uint16_t)min(mmS, (uint32_t)65535);
        speedSampleCount++;
        // Saran nilai kalibrasi TOF: mmPerSecAtMaxPwm adalah kecepatan pada PWM penuh,
        // sedangkan pengukuran ini dilakukan pada cfg.conveyorSpeed. Diskalakan balik.
        speedMmSAtMaxPwm = (cfg.conveyorSpeed > 0)
            ? (uint16_t)min((uint32_t)(mmS * 255UL / cfg.conveyorSpeed), (uint32_t)65535) : 0;
        mb.Hreg(Reg::SPEED_LAST_MM_S, speedLastMmS);
        mb.Hreg(Reg::SPEED_LAST_MS, speedLastMs);
        mb.Hreg(Reg::SPEED_SAMPLE_COUNT, speedSampleCount);
        mb.Hreg(Reg::SPEED_MM_S_AT_MAX_PWM, speedMmSAtMaxPwm);
        Serial.printf("[KECEPATAN] FINISH -- %u mm dalam %lu ms = %u mm/s "
                      "(pada PWM %u). Saran 'Mm/s Max' = %u\n",
                      cfg.speedTestDistMm, (unsigned long)selisih, speedLastMmS,
                      cfg.conveyorSpeed, speedMmSAtMaxPwm);
      }
    }
    // Kalau tidak sedang mengukur, tepi PROX_2 ini memang bukan bagian dari uji --
    // biarkan saja, jalur produksi (handleSensors) yang menanganinya.
  }

  if (speedSedangMengukur && millis() - speedMulaiMs > SPEED_TIMEOUT_MS) {
    speedSedangMengukur = false;
    Serial.printf("[KECEPATAN] BATAL -- objek tidak sampai PROX_2 dalam %lu detik\n",
                  (unsigned long)(SPEED_TIMEOUT_MS / 1000));
  }
}

void handleSafety() {
  if (io.read(CH::ESTOP) == LOW) {   // DIUBAH ke aktif-LOW sesuai instruksi terbaru
    currentState = NodeState::ESTOPPED;
    io.write(CH::CONV1_STBY, LOW);
    lastConv1StbyState = false;   // BARU -- lihat penjelasan di otaSafeStop()
    ledcWrite(LEDC_CH_CONV1, 0);
    // DIUBAH: palang sekarang motor DC (bukan relay) -- E-stop WAJIB potong daya motor langsung,
    // BUKAN coba retract dulu (itu masih butuh motor jalan, kontradiktif sama tujuan E-stop).
    // State palang dipaksa balik IDLE -- kalau lagi di tengah PUSH/RETRACT saat E-stop ditekan,
    // siklus itu DIBATALKAN, bukan dilanjut otomatis setelah E-stop dilepas.
    ledcWrite(LEDC_CH_MOTORA, 0); motorAState = 0;
    palangState = PalangState::IDLE; palangActive = false; palangPending = false;
    // BARU (2026-09-28): perintah palang manual ikut dibatalkan. Bendera dibersihkan langsung,
    // TIDAK lewat hentikanPalangManual() -- fungsi itu memanggil setMotorA() yang menulis I2C,
    // sedangkan di sini daya motor sudah dipotong langsung dan itu memang yang diinginkan.
    palangManualArah = 0; palangManualSampai = 0;
    // BARU (2026-09-28): uji hopper ikut dibatalkan. Tanpa ini, siklus berulang lanjut sendiri
    // begitu E-stop dilepas, padahal operator tidak memerintahkan apa pun.
    hopperUjiBerulang = false; hopperManualTargetUs = 0;
    // BARU: uji kecepatan ikut dibatalkan. Tanpa ini, sabuk akan berputar lagi
    // begitu E-stop dilepas, padahal operator tidak memerintahkan apa pun.
    speedTestConveyorOn = false;
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
// DIPERBAIKI (2026-09-20): CONV1_STBY di sini (dan di handleSafety()) ditulis LANGSUNG,
// melewati cache lastConv1StbyState milik updateConv1Stby(). Kalau saat itu cache berisi
// `true` (conveyor sedang jalan), cache jadi berbohong: pin fisik LOW tapi software yakin
// sudah HIGH, jadi tidak akan pernah ditulis ulang. Gejalanya: OTA gagal di tengah jalan
// tanpa reboot -> conveyor mati senyap (PWM tetap keluar, STBY-nya yang mati) sampai ada
// perubahan state yang kebetulan memaksa penulisan ulang. Cache disinkronkan di sini.
void otaSafeStop() {
  speedTestConveyorOn = false;   // BARU -- jangan sampai sabuk nyala lagi di tengah tulis flash
  ledcWrite(LEDC_CH_CONV1, 0);
  io.write(CH::CONV1_STBY, LOW);
  lastConv1StbyState = false;
  ledcWrite(LEDC_CH_MOTORA, 0); motorAState = 0;
  palangState = PalangState::IDLE; palangActive = false; palangPending = false;
  palangManualArah = 0; palangManualSampai = 0;   // BARU -- jangan sampai motor hidup lagi di tengah tulis flash
  hopperUjiBerulang = false; hopperManualTargetUs = 0;   // BARU -- idem utk servo hopper
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
// BARU (2026-10-07): IP & kekuatan sinyal WiFi ke Modbus, paling sering tiap 5 s (bukan tiap
// loop) -- satu WiFi.RSSI() puluhan mikrodetik, tidak terasa oleh loop utama.
void laporWifiKeModbus() {
  static uint32_t terakhir = 0;
  if (millis() - terakhir < 5000) return;
  terakhir = millis();
  bool tersambung = WiFi.status() == WL_CONNECTED;
  IPAddress ip = tersambung ? WiFi.localIP() : IPAddress(0, 0, 0, 0);
  mb.Hreg(Reg::WIFI_IP_HI, (uint16_t)((ip[0] << 8) | ip[1]));
  mb.Hreg(Reg::WIFI_IP_LO, (uint16_t)((ip[2] << 8) | ip[3]));
  mb.Hreg(Reg::WIFI_RSSI, tersambung ? (uint16_t)(int16_t)WiFi.RSSI() : 0);
}

void updateOTA() {
  laporWifiKeModbus();
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

// DIUBAH (2026-09-29): mengembalikan true = diterima & dijalankan, false = DITOLAK.
// Sebelumnya void -- penolakan hanya tercetak di Serial, sehingga layar Test Command menulis
// "Terkirim, amati aksi" persis sama untuk command yang dijalankan maupun yang ditolak.
// Operator di panel tidak punya cara membedakan keduanya. Pemanggil lama (Modbus, Serial)
// boleh mengabaikan nilai ini; perilakunya tidak berubah.
// ============================================================
// BARU (2026-10-09): BACKUP / RESTORE KALIBRASI lewat Modbus (include/kalibrasi_modbus.h).
// Orange Pi menyimpan kalibrasi node ini ke file dan bisa memuatnya ke ESP32 pengganti.
// CAL_FORMAT: NAIKKAN kalau isi CAL_SEG atau struct di dalamnya berubah -- Orange Pi menolak
// restore dari backup yang formatnya berbeda (layout byte tidak cocok lagi).
// ============================================================
KalibrasiModbus kal;
constexpr uint16_t CAL_FORMAT_NODE = 1;
const CalSeg CAL_SEG[] = { {&cfg, sizeof(cfg)} };
bool calBoleh() { return !mainModeActive && currentState != NodeState::RUNNING_OR_MOVING; }

bool applyCommand(uint16_t opcode, uint16_t arg) {
  switch ((Cmd)opcode) {
    case Cmd::START:
      if (faultCode != 0 || currentState == NodeState::FAULT || currentState == NodeState::ESTOPPED) {
        Serial.println("[CMD] START ditolak -- masih FAULT/ESTOPPED, RESET_FAULT dulu");
        return false;
      } else {
        currentState = NodeState::RUNNING_OR_MOVING;
        mainModeActive = true;   // BARU -- MAIN aktif, command TEST diblokir sampai STOP
        setHopperJeda(false);    // BARU (2026-10-05) -- jeda lama tidak boleh terbawa ke produksi baru
        Serial.println("[CMD] START -- MAIN aktif, command TEST diblokir sampai STOP");
      }
      break;
    case Cmd::STOP:
      currentState = NodeState::IDLE;
      mainModeActive = false;   // BARU -- balik ke TEST mode, command TEST boleh dipakai lagi
      // BARU (2026-09-20): STOP juga membatalkan klasifikasi yang belum sempat didorong.
      // Dulu antrian + palangPending dibiarkan utuh: conveyor sudah berhenti tapi palang
      // masih bisa menghentak sendiri setelahnya, dan sisa antrian meletus sekaligus saat
      // START berikutnya. Siklus palang yang SEDANG berjalan sengaja dibiarkan selesai --
      // menghentikannya di tengah meninggalkan palang menjulur di atas conveyor.
      clearClassificationQueue("STOP diterima");
      palangPending = false;
      setHopperJeda(false);    // BARU (2026-10-05)
      break;
    // BARU (2026-10-05): jeda/lanjut umpan hopper tanpa menghentikan conveyor -- lihat registers.h.
    case Cmd::SET_HOPPER_JEDA:
      setHopperJeda(arg != 0);
      break;
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
      passCount = 0; rejectCount = 0; rejectMissedCount = 0;   // BARU -- ikut direset
      mb.Hreg(Reg::PASS_COUNT, 0); mb.Hreg(Reg::REJECT_COUNT, 0); mb.Hreg(Reg::REJECT_MISSED_COUNT, 0);
      break;
    // BARU: command TEST/jog di bawah DITOLAK TOTAL selama mainModeActive -- Orange Pi wajib
    // STOP dulu. SET_PALANG_SPEED/SET_HOPPER_STEP DIKECUALIKAN (cuma tuning angka, gak gerakin
    // apa-apa sendiri, sengaja tetap boleh live selama produksi jalan).
    case Cmd::SET_MOTOR_A:
      if (mainModeActive) { Serial.println("[CMD] SET_MOTOR_A ditolak -- MAIN aktif, STOP dulu"); return false; }
      // BARU (ditemukan 2026-09-20, kelas bug sama dgn servoRefillStage Dispenser): Motor A
      // JUGA dipakai handlePalangQueue() (produksi asli, reject objek) -- itu jalan TANPA
      // gate mainModeActive karena CLASSIFY_IS_REJECT (input HuskyLens) sengaja ungated.
      // Kalau Orange Pi masih ngirim klasifikasi pas Sorter TEST mode, handlePalangQueue()
      // bisa nyerobot Motor A di tengah jog manual ini tanpa peringatan. Guard di sini.
      if (palangPending || palangState != PalangState::IDLE) {
        Serial.println("[CMD] SET_MOTOR_A ditolak -- palang (reject) lagi pakai Motor A");
        return false;
      }
      setMotorA((uint8_t)constrain(arg, 0, 2));
      break;
    // BARU -- biar Orange Pi bisa tuning kecepatan langsung. Sama pola dgn SET_CONVEYOR_SPEED/
    // SET_HOPPER_INTERVAL di atas (runtime-only, gak auto-save NVS -- simpan permanen tetap
    // lewat LCD '#' kalau mau bertahan setelah reboot).
    case Cmd::SET_PALANG_SPEED: cfg.palangSpeed = (uint8_t)constrain(arg, 0, 255); break;
    case Cmd::SET_HOPPER_STEP:  cfg.hopperStepUs = (uint16_t)constrain(arg, 1, 2500); break;
    case Cmd::TEST_HOPPER_CYCLE:
      if (mainModeActive) { Serial.println("[CMD] TEST_HOPPER_CYCLE ditolak -- MAIN aktif, STOP dulu"); return false; }
      return startTestHopperCycle();
    case Cmd::TEST_TRIGGER_PALANG:
      if (mainModeActive) { Serial.println("[CMD] TEST_TRIGGER_PALANG ditolak -- MAIN aktif, STOP dulu"); return false; }
      enqueueClassification(true, millis());
      Serial.println("[SORTER] TEST_TRIGGER_PALANG -- simulasi reject dikirim");
      break;
    // DIHAPUS (2026-09-20): Cmd::TEST_FAULT (opcode 99) -- memaksa node ke FAULT palsu.
    // Komentar aslinya di registers.h sudah menyuruh menghapusnya sebelum produksi.
    // BARU (2026-10-09): backup / restore kalibrasi -- hasil di Reg::CAL_HASIL
    case Cmd::CAL_BACA:     return kal.baca(arg, calBoleh());
    case Cmd::CAL_TULIS:    return kal.tulis(arg, calBoleh());
    case Cmd::CAL_TERAPKAN: return kal.terapkan(arg, calBoleh());
    default: Serial.printf("[CMD] opcode %u tidak dikenal\n", opcode); return false;
  }
  return true;
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

enum class MenuState { NONE, TOP_SELECT, CAL_GROUP, CAL_LIST, JOG_PARAM,
                        TEST_IO_CATEGORY, TEST_IO_I2CSCAN, TEST_OUTPUT_LIST, TEST_OUTPUT_ITEM,
                        TEST_INPUT_CATEGORY, TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
                        TEST_MOD_SERVO, TEST_CMD_LIST, CONFIRM_RESET,
                        TEST_KECEPATAN, TEST_PALANG, TEST_HOPPER, SYS_INFO};
MenuState menuState = MenuState::NONE;

// BARU (2026-09-22): layar Info Sistem didefinisikan tepat sebelum loop() -- di titik itu
// otaReady/i2cErrorCount/lastFaultCode sudah terdeklarasi. Menu atas memanggilnya lebih
// awal, jadi butuh dua baris pengenalan ini.
uint8_t sysInfoPage = 0;
void drawSysInfo();
void handleSysInfoKey(char key);

bool menuIsActive() { return menuState != MenuState::NONE; }   // BARU -- dipakai onClassifyWrite() di atas
uint8_t jogStepIdx = 0;
constexpr int16_t JOG_STEPS[4] = {5, 10, 20, 50};

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

// --- LEVEL 0: TOP MENU (3 kategori) ---
constexpr uint8_t TOP_COUNT = 4;   // DIUBAH 3 -> 4, tambah "Info Sistem"
const char* TOP_LABELS[TOP_COUNT] = { "Setting", "Test I/O", "Test Command", "Info Sistem" };
uint8_t topCursor = 0;

void drawTopMenuSorter() {
  String title = "[MANUAL] " + String(currentState == NodeState::RUNNING_OR_MOVING ? "[RUN]" : "[IDLE]");
  drawListMenu(title.c_str(), TOP_LABELS, TOP_COUNT, topCursor);
}

// --- LEVEL 1a: SETTING KALIBRASI ---
// DIUBAH TOTAL (2026-09-29): dua tingkat, dikelompokkan per PERANGKAT yang dioperasikan
// (permintaan operator). Dulu satu daftar datar 22 item yang mencampur conveyor, palang,
// hopper, buzzer dan reset -- operator harus menggulir melewati seluruh parameter hopper untuk
// sampai ke buzzer, dan tidak ada yang menunjukkan parameter mana milik perangkat mana.
//
// Sebagian label lama juga TERPOTONG di layar: baris daftar hanya menyisakan 15 kolom setelah
// "> a. ", sedangkan label seperti "Hopper Step Interval(ms)" 24 karakter. Dengan nama
// perangkat pindah ke judul layar, awalan itu tidak perlu lagi dan labelnya muat utuh.
//
//   Setting Kalibrasi
//     a. Conveyor  Speed, Dir, Mm/s Max, Jarak Uji, Uji Kecepatan
//     b. Palang    Speed, Dir, Push, Retract, Jarak TOF, Uji Palang
//     c. Hopper    Titik Awal, Titik Dorong, Step, Step Intvl, Push Hold, Gap Siklus, Uji Hopper
//     d. Buzzer    Bunyi On, Bunyi Off
//     e. Reset     Reset Fault, Reset Default
//
// Dalam tiap perangkat: parameter lebih dulu, layar uji paling akhir. "Mm/s Max" masuk
// Conveyor (itu kecepatan fisik sabuk, dan Uji Kecepatan di grup yang sama menghasilkan
// nilainya); "Jarak TOF" masuk Palang (itu letak palang dari titik scan).
//
// NOMOR PARAMETER (selParam) SENGAJA TIDAK DIUBAH. drawParamMenu()/handleParamKey() dan logika
// live-test (mis. hopperIntervalTestMode = selParam 11/12) bercabang pada nomor itu.
// Pengelompokan hanya memetakan urutan tampil ke nomor lama, jadi tidak satu pun layar
// parameter perlu disentuh. Nomor di CalId adalah ID tetap, BUKAN posisi di daftar.
namespace CalId {
  constexpr uint8_t RESET_FAULT = 0, CONV_SPEED = 1, CONV_DIR = 2, PALANG_SPEED = 3,
    PALANG_DIR = 4, PALANG_PUSH = 5, PALANG_RETRACT = 6, DIST_TOF = 7, MMS_MAX = 8,
    HOPPER_AWAL = 9, HOPPER_DORONG = 10, HOPPER_STEP = 11, HOPPER_STEP_INTERVAL = 12,
    HOPPER_PUSH_HOLD = 13, BUZZER_ON = 14, BUZZER_OFF = 15, HOPPER_GAP = 16, JARAK_UJI = 17,
    UJI_KECEPATAN = 18, UJI_PALANG = 19, UJI_HOPPER = 20, RESET_DEFAULT = 21;
}
constexpr uint8_t CAL_COUNT = 22;
// Label per ID, tanpa awalan nama perangkat. Maksimal 15 karakter.
const char* CAL_LABELS[CAL_COUNT] = {
  "Reset Fault",
  "Speed (PWM)", "Dir", "Speed (PWM)", "Dir",
  "Push (ms)", "Retract (ms)", "Jarak TOF (mm)", "Mm/s Max",
  "Titik Awal", "Titik Dorong", "Step (us)", "Step Intvl (ms)",
  "Push Hold (ms)",
  "Bunyi On (ms)", "Bunyi Off (ms)",
  "Gap Siklus (ms)", "Jarak Uji (mm)", "Uji Kecepatan",
  "Uji Palang", "Uji Hopper",
  "Reset Default" };

const uint8_t CAL_GRP_CONVEYOR[] = { CalId::CONV_SPEED, CalId::CONV_DIR, CalId::MMS_MAX,
                                     CalId::JARAK_UJI, CalId::UJI_KECEPATAN };
const uint8_t CAL_GRP_PALANG[]   = { CalId::PALANG_SPEED, CalId::PALANG_DIR, CalId::PALANG_PUSH,
                                     CalId::PALANG_RETRACT, CalId::DIST_TOF, CalId::UJI_PALANG };
const uint8_t CAL_GRP_HOPPER[]   = { CalId::HOPPER_AWAL, CalId::HOPPER_DORONG, CalId::HOPPER_STEP,
                                     CalId::HOPPER_STEP_INTERVAL, CalId::HOPPER_PUSH_HOLD,
                                     CalId::HOPPER_GAP, CalId::UJI_HOPPER };
const uint8_t CAL_GRP_BUZZER[]   = { CalId::BUZZER_ON, CalId::BUZZER_OFF };
const uint8_t CAL_GRP_RESET[]    = { CalId::RESET_FAULT, CalId::RESET_DEFAULT };

struct CalGroup { const char* judul; const uint8_t* id; uint8_t jumlah; };
constexpr uint8_t CAL_GROUP_COUNT = 5;
const CalGroup CAL_GROUPS[CAL_GROUP_COUNT] = {
  { "SETTING: CONVEYOR", CAL_GRP_CONVEYOR, sizeof(CAL_GRP_CONVEYOR) },
  { "SETTING: PALANG",   CAL_GRP_PALANG,   sizeof(CAL_GRP_PALANG) },
  { "SETTING: HOPPER",   CAL_GRP_HOPPER,   sizeof(CAL_GRP_HOPPER) },
  { "SETTING: BUZZER",   CAL_GRP_BUZZER,   sizeof(CAL_GRP_BUZZER) },
  { "SETTING: RESET",    CAL_GRP_RESET,    sizeof(CAL_GRP_RESET) },
};
const char* CAL_GROUP_LABELS[CAL_GROUP_COUNT] = { "Conveyor", "Palang", "Hopper", "Buzzer", "Reset" };

uint8_t calGroupCursor = 0;   // perangkat yang sedang dibuka
uint8_t calCursor = 0;        // posisi DI DALAM perangkat itu
uint8_t selParam = 0;         // ID parameter (CalId), bukan posisi

void drawCalGroup() { drawListMenu("SETTING KALIBRASI", CAL_GROUP_LABELS, CAL_GROUP_COUNT, calGroupCursor); }

// Nama dan state-nya SENGAJA dipertahankan (drawCalList / CAL_LIST). Semua layar parameter dan
// layar uji kembali lewat `menuState = CAL_LIST; drawCalList();` -- dengan begini mereka
// kembali ke daftar perangkat yang tadi dibuka, tanpa satu pun jalur keluar itu perlu diubah.
void drawCalList() {
  const CalGroup &g = CAL_GROUPS[calGroupCursor];
  static const char* labels[CAL_COUNT];
  for (uint8_t i = 0; i < g.jumlah; i++) labels[i] = CAL_LABELS[g.id[i]];
  drawListMenu(g.judul, labels, g.jumlah, calCursor);
}

// --- BARU: konfirmasi Reset ke Default -- aksi merusak (hapus NVS), wajib konfirmasi 2 langkah ---
// ============================================================
// LAYAR KALIBRASI: UJI KECEPATAN (BARU 2026-09-21)
//
// Conveyor dinyalakan langsung dari sini, tanpa perlu START/MAIN. Dengan begitu
// hopper tidak ikut bersiklus dan tidak ada objek yang dijatuhkan sendiri --
// operator cukup meletakkan objek di sabuk lalu mengamati hasilnya.
//
// Tombol:
//   A / B  kecepatan conveyor naik / turun (berlaku live, sabuk langsung berubah)
//   C      conveyor ON / OFF
//   D      kembali (conveyor DIMATIKAN otomatis)
//   #      simpan: kecepatan conveyor + terapkan saran 'Mm/s Max' ke kalibrasi TOF
// ============================================================
void drawTestKecepatan() {
  lcdPrint(0, 0, "UJI KEC. " + String(cfg.speedTestDistMm) + "mm");

  if (speedSampleCount > 0) {
    lcdPrint(0, 1, String(speedLastMmS) + "mm/s " + String(speedLastMs) + "ms #" + String(speedSampleCount));
  } else if (speedSedangMengukur) {
    lcdPrint(0, 1, "Jalan.. tunggu PROX_2");
  } else {
    lcdPrint(0, 1, "Lewatkan objek..");
  }

  lcdPrint(0, 2, "PWM" + String(cfg.conveyorSpeed) + " Max:" + String(speedMmSAtMaxPwm)
                 + (speedTestConveyorOn ? " ON" : " off"));
  lcdPrint(0, 3, "A+B-C:on/off #simpan");
}

void handleTestKecepatanKey(char key) {
  if (key == 'A') {
    cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed + 5, 0, 255);
  } else if (key == 'B') {
    cfg.conveyorSpeed = (uint8_t)constrain((int)cfg.conveyorSpeed - 5, 0, 255);
  } else if (key == 'C') {
    speedTestConveyorOn = !speedTestConveyorOn;
    Serial.printf("[KECEPATAN] Conveyor %s dari menu kalibrasi\n", speedTestConveyorOn ? "ON" : "OFF");
  } else if (key == '#') {
    // Saran 'Mm/s Max' hanya ada artinya kalau sudah pernah ada pengukuran berhasil.
    if (speedMmSAtMaxPwm > 0) {
      cfg.mmPerSecAtMaxPwm = (float)speedMmSAtMaxPwm;
      Serial.printf("[KECEPATAN] Mm/s Max diterapkan dari hasil ukur = %u\n", speedMmSAtMaxPwm);
    }
    saveConfigToNvs();
    lcdPrint(0, 3, "TERSIMPAN ke NVS!   ");
    return;   // jangan gambar ulang, biar pesan tersimpan sempat terbaca
  } else if (key == 'D') {
    // Meninggalkan layar ini WAJIB mematikan sabuk. Kalau tidak, conveyor akan
    // terus berputar tanpa ada layar yang menunjukkan kenapa.
    speedTestConveyorOn = false;
    menuState = MenuState::CAL_LIST;
    Serial.println("[KECEPATAN] Keluar -- conveyor dimatikan");
    drawCalList();
    return;
  }
  drawTestKecepatan();
}

// ============================================================
// LAYAR KALIBRASI: UJI PALANG (BARU 2026-09-28)
//
// Yang bisa diperiksa di sini dan TIDAK bisa diperiksa dari satu siklus otomatis:
//   - apakah palang menjulur PENUH (ditahan pada posisi julur, bukan lewat 300ms)
//   - apakah arahnya sudah benar terhadap kalibrasi Palang Dir
//   - apakah dia macet, berat, atau meleset dari jalur objek
//   - apakah dorongan benar-benar menyingkirkan objek SELAGI SABUK BERJALAN -- itu sebabnya
//     conveyor ikut bisa dinyalakan dari layar ini, bukan dari layar lain
//
// Conveyor memakai saklar yang sama dengan layar Uji Kecepatan (speedTestConveyorOn), jadi
// sabuk berjalan TANPA START/MAIN: hopper tidak ikut bersiklus dan tidak ada objek yang
// dijatuhkan sendiri. Operator meletakkan objek dengan tangan, lalu mendorongnya.
//
// Tombol:
//   A   palang ON  (PUSH / julur, ditahan)
//   B   palang OFF (RETRACT / tarik, ditahan)
//   0   motor STOP (netral -- tidak didorong ke arah mana pun)
//   #   1 siklus otomatis pakai nilai kalibrasi, tanpa penundaan TOF
//   C   conveyor ON / OFF
//   D   kembali (palang dan conveyor DIMATIKAN otomatis)
//
// Perintah manual A/B mati sendiri setelah PALANG_MANUAL_MAX_MS supaya motor tidak
// tertahan mentok tanpa batas. Lihat updatePalangManual().
// ============================================================
void drawTestPalang() {
  lcdPrint(0, 0, "UJI PALANG  Dir:" + String(cfg.palangDir ? "maju" : "bali"));

  // Keadaan palang OTOMATIS menang tampilan: kalau siklus produksi sedang jalan, perintah
  // manual memang sedang ditolak, dan itu harus terlihat -- bukan tampil sebagai "diam".
  if (palangState == PalangState::PUSHING) {
    lcdPrint(0, 1, "OTOMATIS: PUSH      ");
  } else if (palangState == PalangState::RETRACTING) {
    lcdPrint(0, 1, "OTOMATIS: RETRACT   ");
  } else if (palangPending) {
    lcdPrint(0, 1, "OTOMATIS: menunggu  ");
  } else if (palangManualArah > 0) {
    uint32_t sisa = (palangManualSampai > millis()) ? (palangManualSampai - millis()) : 0;
    lcdPrint(0, 1, "MANUAL: PUSH  " + String(sisa) + "ms ");
  } else if (palangManualArah < 0) {
    uint32_t sisa = (palangManualSampai > millis()) ? (palangManualSampai - millis()) : 0;
    lcdPrint(0, 1, "MANUAL: TARIK " + String(sisa) + "ms ");
  } else {
    lcdPrint(0, 1, "Palang DIAM         ");
  }

  lcdPrint(0, 2, "Plg" + String(cfg.palangSpeed) + " Conv" + String(cfg.conveyorSpeed)
                 + (speedTestConveyorOn ? " ON" : " off") + " R" + String(rejectCount));
  lcdPrint(0, 3, "A+ B- 0stop #siklus ");
}

void handleTestPalangKey(char key) {
  if (key == 'A') {
    palangManual(+1);
  } else if (key == 'B') {
    palangManual(-1);
  } else if (key == '0') {
    palangManual(0);
  } else if (key == '#') {
    if (palangState != PalangState::IDLE || palangPending) {
      Serial.println("[UJI-PALANG] Siklus ditolak -- masih ada siklus yang belum selesai");
    } else {
      // Perintah manual dihentikan lebih dulu. Kalau tidak, motor sedang didorong ke satu
      // arah sementara siklus otomatis mulai mendorongnya ke arah lain -- dua pihak menulis
      // satu output yang sama, persis jenis bentrok yang dicegah guard di setMotorA().
      hentikanPalangManual("digantikan siklus otomatis");
      mulaiSiklusPalangSekarang();
      Serial.println("[UJI-PALANG] 1 siklus otomatis dimulai (tanpa penundaan TOF)");
    }
  } else if (key == 'C') {
    speedTestConveyorOn = !speedTestConveyorOn;
    Serial.printf("[UJI-PALANG] Conveyor %s dari menu kalibrasi\n", speedTestConveyorOn ? "ON" : "OFF");
  } else if (key == 'D') {
    // Meninggalkan layar ini WAJIB memadamkan keduanya. Palang yang tertinggal menjulur
    // menghalangi sabuk, dan sabuk yang terus berputar tidak punya layar yang menjelaskan
    // kenapa. Siklus otomatis yang SEDANG berjalan dibiarkan selesai -- memotongnya di
    // tengah justru meninggalkan palang menjulur.
    hentikanPalangManual("keluar dari layar Uji Palang");
    speedTestConveyorOn = false;
    menuState = MenuState::CAL_LIST;
    Serial.println("[UJI-PALANG] Keluar -- palang & conveyor dimatikan");
    drawCalList();
    return;
  }
  drawTestPalang();
}

// ============================================================
// LAYAR KALIBRASI: UJI HOPPER (BARU 2026-09-28)
//
// Yang bisa diperiksa di sini dan TIDAK bisa diperiksa dari satu siklus otomatis:
//   - apakah pendorong benar-benar mencapai ujung (ditahan di Titik Dorong, bukan lewat 300ms)
//   - apakah Titik Awal sudah cukup mundur sehingga objek berikutnya bisa turun
//   - apakah ada objek yang tersangkut atau ikut terbawa balik
//   - dan yang paling menentukan: apakah SATU siklus benar-benar menjatuhkan SATU objek
//
// Pertanyaan terakhir itu tidak bisa dijawab pada sabuk diam -- objek menumpuk di titik yang
// sama dan jumlahnya tidak terbaca sensor. Karena itu conveyor ikut bisa dinyalakan dari layar
// ini, dan layar menampilkan dua angka berdampingan: berapa kali hopper mendorong, dan berapa
// objek yang benar-benar melewati PROX_2. Selisih keduanya adalah objek yang gagal jatuh atau
// jatuh dobel -- satu-satunya ukuran keandalan umpan yang dimiliki Sorter.
//
// Tombol:
//   A   tahan di TITIK AWAL (posisi mundur/diam)
//   B   tahan di TITIK DORONG (posisi julur penuh)
//   #   1 siklus penuh pakai nilai kalibrasi
//   *   siklus BERULANG on/off (termasuk jeda antar siklus cfg.hopperCycleGapMs)
//   C   conveyor ON / OFF
//   0   nol-kan penghitung siklus & objek
//   D   kembali (hopper dikembalikan ke Titik Awal, siklus & conveyor dimatikan)
// ============================================================
uint32_t hopperUjiSiklusBase = 0;
uint32_t hopperUjiObjekBase = 0;

void drawTestHopper() {
  lcdPrint(0, 0, "UJI HOPPER  " + String(hopperCurrentUs) + "us");

  if (hopperUjiBerulang) {
    lcdPrint(0, 1, "BERULANG gap" + String(cfg.hopperCycleGapMs) + "ms ");
  } else if (testHopperCycleStage != TestHopperCycleStage::NONE) {
    lcdPrint(0, 1, "1 SIKLUS jalan..    ");
  } else if (hopperManualTargetUs != 0) {
    lcdPrint(0, 1, "Menuju " + String(hopperManualTargetUs) + "us..   ");
  } else if (hopperCurrentUs == cfg.hopperPushUs) {
    lcdPrint(0, 1, "TAHAN di Titik Dorong");
  } else if (hopperCurrentUs == cfg.hopperStartUs) {
    lcdPrint(0, 1, "Diam di Titik Awal  ");
  } else {
    lcdPrint(0, 1, "Diam                ");
  }

  // Dua angka yang harus dibaca BERSAMA. Siklus tanpa objek = gagal jatuh; objek lebih banyak
  // dari siklus = jatuh dobel. Keduanya sama-sama merusak laju umpan, dan tidak satu pun
  // kelihatan kalau hanya salah satu angka yang ditampilkan.
  uint32_t siklus = hopperSiklusSelesai - hopperUjiSiklusBase;
  uint32_t objek = passCount - hopperUjiObjekBase;
  lcdPrint(0, 2, "Siklus:" + String(siklus) + " Objek:" + String(objek)
                 + (speedTestConveyorOn ? " C:ON" : " C:of"));
  lcdPrint(0, 3, "A/B tahan #1x *ulang");
}

void handleTestHopperKey(char key) {
  if (key == 'A') {
    hopperManual(cfg.hopperStartUs);
  } else if (key == 'B') {
    hopperManual(cfg.hopperPushUs);
  } else if (key == '#') {
    hopperManualTargetUs = 0;   // perintah manual dilepas -- satu servo, satu pengatur
    startTestHopperCycle();     // menolak sendiri kalau node tidak IDLE
  } else if (key == '*') {
    hopperUjiBerulang = !hopperUjiBerulang;
    hopperManualTargetUs = 0;
    if (!hopperUjiBerulang) {
      // Siklus dihentikan pada titik mana pun ia berada, lalu hopper dikembalikan ke Titik
      // Awal. Membiarkannya berhenti di tengah dorongan menahan objek berikutnya.
      hopperState = HopperState::AT_START;
      hopperManual(cfg.hopperStartUs);
    }
    Serial.printf("[UJI-HOPPER] Siklus berulang %s\n", hopperUjiBerulang ? "ON" : "OFF");
  } else if (key == 'C') {
    speedTestConveyorOn = !speedTestConveyorOn;
    Serial.printf("[UJI-HOPPER] Conveyor %s dari menu kalibrasi\n", speedTestConveyorOn ? "ON" : "OFF");
  } else if (key == '0') {
    hopperUjiSiklusBase = hopperSiklusSelesai;
    hopperUjiObjekBase = passCount;
    Serial.println("[UJI-HOPPER] Penghitung siklus & objek dinolkan");
  } else if (key == 'D') {
    hopperUjiBerulang = false;
    hopperState = HopperState::AT_START;
    hopperManual(cfg.hopperStartUs);   // dikembalikan, bukan ditinggal di posisi dorong
    speedTestConveyorOn = false;
    menuState = MenuState::CAL_LIST;
    Serial.println("[UJI-HOPPER] Keluar -- hopper kembali ke Titik Awal, conveyor dimatikan");
    drawCalList();
    return;
  }
  drawTestHopper();
}

void drawConfirmReset() {
  lcdClear();
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

void handleCalGroupKey(char key) {
  if (key == 'A') { calGroupCursor = (calGroupCursor == 0) ? CAL_GROUP_COUNT - 1 : calGroupCursor - 1; drawCalGroup(); }
  else if (key == 'B') { calGroupCursor = (calGroupCursor + 1) % CAL_GROUP_COUNT; drawCalGroup(); }
  else if (key == 'C') { menuState = MenuState::CAL_LIST; calCursor = 0; drawCalList(); }
  else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

void handleCalListKey(char key) {
  const CalGroup &g = CAL_GROUPS[calGroupCursor];
  if (key == 'A') { calCursor = (calCursor == 0) ? g.jumlah - 1 : calCursor - 1; drawCalList(); }
  else if (key == 'B') { calCursor = (calCursor + 1) % g.jumlah; drawCalList(); }
  else if (key == 'C') {
    // Bercabang pada ID, bukan posisi -- posisi berubah tiap kali daftar disusun ulang, ID
    // tidak. Dulu cabangnya `calCursor == 18/19/20/21`, dan setiap sisipan item baru memaksa
    // semua angka itu digeser satu per satu.
    uint8_t id = g.id[calCursor];
    if (id == CalId::RESET_FAULT) {   // langsung eksekusi (tidak destruktif, tanpa konfirmasi)
      applyCommand((uint16_t)Cmd::RESET_FAULT, 0);
      lcdPrint(0, 3, "Fault direset!      ");
      Serial.println("[CAL] Reset Fault dari menu LCD");
    } else if (id == CalId::UJI_KECEPATAN) {
      menuState = MenuState::TEST_KECEPATAN;
      lcdClear();
      drawTestKecepatan();
    } else if (id == CalId::UJI_PALANG) {
      menuState = MenuState::TEST_PALANG;
      lcdClear();
      drawTestPalang();
    } else if (id == CalId::UJI_HOPPER) {
      menuState = MenuState::TEST_HOPPER;
      hopperUjiSiklusBase = hopperSiklusSelesai;
      hopperUjiObjekBase = passCount;
      lcdClear();
      drawTestHopper();
    } else if (id == CalId::RESET_DEFAULT) {   // aksi merusak -- minta konfirmasi dulu
      menuState = MenuState::CONFIRM_RESET;
      drawConfirmReset();
    } else {
      selParam = id;
      // Test-live di KEDUA layar Hopper Step/Step Interval -- operator perlu melihat efeknya
      // langsung saat mengatur salah satu dari dua parameter ini.
      hopperIntervalTestMode = (selParam == CalId::HOPPER_STEP || selParam == CalId::HOPPER_STEP_INTERVAL);
      menuState = MenuState::JOG_PARAM;
      lcdClear(); drawParamMenu();
    }
  }
  else if (key == 'D') { menuState = MenuState::CAL_GROUP; drawCalGroup(); }
}

void drawParamMenu() {
  switch (selParam) {
    case 1: lcdPrint(0, 0, "CONVEYOR SPEED"); break;
    case 2: lcdPrint(0, 0, "CONVEYOR DIRECTION"); break;
    case 3: lcdPrint(0, 0, "PALANG SPEED"); break;
    case 4: lcdPrint(0, 0, "PALANG DIRECTION"); break;
    case 5: lcdPrint(0, 0, "PALANG PUSH (ms)"); break;
    case 6: lcdPrint(0, 0, "PALANG RETRACT(ms)"); break;
    case 7: lcdPrint(0, 0, "DIST_MM"); break;
    case 8: lcdPrint(0, 0, "MM_PER_SEC_MAX"); break;
    case 9: lcdPrint(0, 0, "HOPPER TITIK AWAL"); break;
    case 10: lcdPrint(0, 0, "HOPPER TITIK DORONG"); break;
    case 11: lcdPrint(0, 0, "HOPPER STEP (us)"); break;
    case 12: lcdPrint(0, 0, "HOPPER STEP INTV(ms)"); break;
    case 13: lcdPrint(0, 0, "HOPPER PUSH HOLD(ms)"); break;
    case 14: lcdPrint(0, 0, "BUZZER ON (ms)"); break;
    case 15: lcdPrint(0, 0, "BUZZER OFF (ms)"); break;
    case 16: lcdPrint(0, 0, "HOPPER GAP SIKLUS"); break;
    case 17: lcdPrint(0, 0, "JARAK UJI KEC.(mm)"); break;
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
    case 13: line1 = "ms:" + String(cfg.hopperPushHoldMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 14: line1 = "ms:" + String(cfg.buzzerOnMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 15: line1 = "ms:" + String(cfg.buzzerOffMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 16: line1 = "ms:" + String(cfg.hopperCycleGapMs) + (cfg.hopperCycleGapMs == 0 ? " (NONSTOP)" : "") + " S:" + String(JOG_STEPS[jogStepIdx]); break;
    case 17: line1 = "mm:" + String(cfg.speedTestDistMm) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
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
      // Kalibrasi Step/Interval agresif sekarang AMAN dari sisi "reverse sebelum sampai" --
      // ada Hopper Push Hold(ms) (param 13) yang jadi jaminan waktu fisik terpisah, independen
      // dari seberapa cepat kalibrasi Step/Interval ini di-set.
      if (key == 'A') cfg.hopperStepIntervalMs = (uint16_t)constrain((int)cfg.hopperStepIntervalMs + step, 0, 500);
      else if (key == 'B') cfg.hopperStepIntervalMs = (uint16_t)constrain((int)cfg.hopperStepIntervalMs - step, 0, 500);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 13:
      // BARU: jeda tahan WAJIB di titik dorong sebelum retract -- lihat komentar cfg.hopperPushHoldMs.
      if (key == 'A') cfg.hopperPushHoldMs = (uint16_t)constrain((int)cfg.hopperPushHoldMs + step * 10, 0, 5000);
      else if (key == 'B') cfg.hopperPushHoldMs = (uint16_t)constrain((int)cfg.hopperPushHoldMs - step * 10, 0, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 14:
      if (key == 'A') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs + step * 10, 50, 5000);
      else if (key == 'B') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs - step * 10, 50, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 15:
      if (key == 'A') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs + step * 10, 50, 5000);
      else if (key == 'B') cfg.buzzerOffMs = (uint16_t)constrain((int)cfg.buzzerOffMs - step * 10, 50, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 16:
      // BARU: jeda diam antar siklus hopper = kendali laju umpan. 0 = nonstop (perilaku lama).
      if (key == 'A') cfg.hopperCycleGapMs = (uint16_t)constrain((int)cfg.hopperCycleGapMs + step * 10, 0, 10000);
      else if (key == 'B') cfg.hopperCycleGapMs = (uint16_t)constrain((int)cfg.hopperCycleGapMs - step * 10, 0, 10000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 17:
      // Jarak PROX_1 ke PROX_2. Ukur fisiknya dengan penggaris; ketelitian uji
      // kecepatan bergantung langsung pada ketelitian angka ini.
      if (key == 'A') cfg.speedTestDistMm = (uint16_t)constrain((int)cfg.speedTestDistMm + step, 10, 5000);
      else if (key == 'B') cfg.speedTestDistMm = (uint16_t)constrain((int)cfg.speedTestDistMm - step, 10, 5000);
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

// --- Kategori: Test Output ---
// DIUBAH (2026-09-29, permintaan operator):
//   - Label memakai nama channel lengkap (LED_OPR, LED_RUN, ...) supaya sama dengan yang
//     tertulis di skema/wiring, bukan singkatan yang harus diterjemahkan dulu.
//   - RELAY_1 dan RELAY_2 dipindah ke SINI dari Test Modul -> Relay (layar itu dihapus).
//     Tidak ada yang memakai relay di Sorter sejak palang pindah ke motor DC, tapi
//     terminalnya tetap ada di board -- dan satu-satunya tempat mengujinya sempat ikut
//     terhapus. Toggle kena rate-limit 300ms di handleTestOutputItemKey() (beban induktif),
//     dan hanya diizinkan saat IDLE.
//   - CONV1_STBY DIBUANG. Pin ini dikendalikan updateConv1Stby() lewat cache
//     lastConv1StbyState. Men-toggle-nya dari sini membuat cache itu berbohong -- pin fisik
//     berubah, software yakin belum -- sehingga conveyor/palang bisa mati senyap sampai ada
//     perubahan state yang kebetulan memaksa penulisan ulang. Jenis bug yang sama persis
//     dengan yang sudah diperbaiki di otaSafeStop() (2026-09-20). Conveyor & palang diuji
//     lewat layar Uji Kecepatan / Uji Palang, yang melewati jalur kendali yang benar.
constexpr uint8_t OUTPUT_TEST_COUNT = 7;
IOTestItem OUTPUT_TEST_ITEMS[OUTPUT_TEST_COUNT] = {
  {"LED_OPR",    CH::LED_OPERATION, true},
  {"LED_RUN",    CH::LED_RUN,       true},
  {"LED_MANUAL", CH::LED_MANUAL,    true},
  {"LED_FAULT",  CH::LED_FAULT,     true},   // auto-controlled (lihat updateUniversalIndicators)
  {"BUZZER",     CH::BUZZER,        false},
  {"RELAY_1",    CH::RLY1,          false},
  {"RELAY_2",    CH::RLY2,          false},
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

// --- Kategori: Test Modul -- sub-menu pilihan JENIS modul, SAMA di semua 4 node (board
// universal -- channel/pin sudah didefinisikan sama persis terlepas modul itu benar2
// terpasang fisik atau tidak di board node ini).
bool testModFirstDraw = true;
String testModLine1 = "", testModLine2 = "";

// --- PCA9685 minimal raw driver (Wire langsung, TANPA library) -- utk Test Modul Servo.
// --- PCA9685 (driver sudah didefinisikan di atas, dipakai bersama produksi hopper & test modul) ---

// DIUBAH (2026-09-29): "Relay" dipindah dari sini ke Test I/O -> Test Output (RELAY_1/RELAY_2).
// Relay adalah output digital biasa, bukan modul yang butuh layar ujinya sendiri -- di Test
// Output ia diuji dengan cara yang sama persis dengan LED dan buzzer.
constexpr uint8_t MODULE_TYPE_COUNT = 3;
const char* MODULE_TYPE_LABELS[MODULE_TYPE_COUNT] = { "Stepper", "Motor DC", "Servo" };
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

// --- Modul: Servo (PCA9685) -- cek deteksi I2C dulu, baru izinkan gerak. Pakai objek pwm
// GLOBAL yang sama dengan hopper produksi (SATU driver, bukan 2 cara berbeda lagi) ---
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
      case 2: menuState = MenuState::TEST_MOD_SERVO; drawTestModServo(); break;
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
// DIPERIKSA ULANG (2026-09-29). Yang dibuang dan alasannya:
//
//   SET_SPEED (test=200) dan SET_DIR (test=0) -- keduanya MENIMPA kalibrasi. Speed jadi 200
//     dan arah conveyor jadi MUNDUR, dan tidak ada item untuk mengembalikannya. Lebih buruk
//     lagi: menekan '#' di layar parameter mana pun menyimpan SELURUH cfg ke NVS, jadi nilai
//     uji itu ikut tersimpan permanen tanpa operator sadar. Kecepatan & arah diatur di
//     Setting -> Conveyor, yang menampilkan nilainya.
//   MOTOR_A Maju / MOTOR_A Stop -- menggerakkan palang dengan arah MENTAH (mengabaikan
//     kalibrasi Palang Dir) dan TANPA batas waktu. Maju tanpa Stop menahan linear actuator
//     mentok dengan arus penuh sampai daya dicabut. Layar Setting -> Palang -> Uji Palang
//     melakukan hal yang sama dengan arah yang benar dan padam sendiri setelah 3 detik.
//     Keduanya tetap tersedia lewat Modbus (SET_MOTOR_A) untuk master.
//
// Label dipendekkan maksimal 15 karakter -- "TEST_TRIGGER_PALANG" (19) dulu terpotong jadi
// "TEST_TRIGGER_" di layar. Petunjuk "C=kirim D=kembali" di baris 4 juga dibuang: ia menimpa
// baris daftar ketiga, jadi item TERAKHIR tidak pernah terlihat saat kursor ada di situ.
// Tombolnya sama dengan semua layar daftar lain (A/B/C/D), petunjuk itu tidak diperlukan.
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
// URUTAN BAKU (2026-09-30) -- SAMA di keempat node dan di sorting_automation.py (file acuan):
//   a START_MAIN   b STOP_MAIN   c RESET_FAULT      <- huruf ini tetap di SEMUA node
//   lalu kelompok Produksi (hanya MAIN) -> Aksi (bebas mode) -> Uji (ditolak saat MAIN).
// Jangan menyisipkan item di tengah tanpa menyamakan OPCODES di sorting_automation.py.
constexpr uint8_t CMD_TEST_COUNT = 6;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  // Firmware Sorter menamai opcode 1/2 START/STOP -- perilakunya persis START_MAIN/STOP_MAIN
  // node lain (menyalakan / mematikan MAIN), jadi ditampilkan dengan nama yang sama.
  {"START_MAIN",     Cmd::START,               0},
  {"STOP_MAIN",      Cmd::STOP,                0},
  {"RESET_FAULT",    Cmd::RESET_FAULT,         0},
  {"RESET_COUNTERS", Cmd::RESET_COUNTERS,      0},
  {"HOPPER_CYCLE",   Cmd::TEST_HOPPER_CYCLE,   0},
  {"TRIGGER_PALANG", Cmd::TEST_TRIGGER_PALANG, 0},
};
const char* CMD_TEST_LABELS_ONLY[CMD_TEST_COUNT];
void buildCmdTestLabels() { for (uint8_t i = 0; i < CMD_TEST_COUNT; i++) CMD_TEST_LABELS_ONLY[i] = CMD_TEST_ITEMS[i].label; }
uint8_t cmdTestCursor = 0;

void drawTestCmdList() {
  buildCmdTestLabels();
  drawListMenu("TEST COMMAND", CMD_TEST_LABELS_ONLY, CMD_TEST_COUNT, cmdTestCursor);
}

void handleTestCmdListKey(char key) {
  if (key == 'A') { cmdTestCursor = (cmdTestCursor == 0) ? CMD_TEST_COUNT - 1 : cmdTestCursor - 1; drawTestCmdList(); }
  else if (key == 'B') { cmdTestCursor = (cmdTestCursor + 1) % CMD_TEST_COUNT; drawTestCmdList(); }
  else if (key == 'C') {
    CmdTestItem &item = CMD_TEST_ITEMS[cmdTestCursor];
    Serial.printf("[TEST-CMD] Simulasi command dari 'node lain': opcode=%u (%s) arg=%u\n",
                  (uint16_t)item.opcode, item.label, item.testArg);
    bool diterima = applyCommand((uint16_t)item.opcode, item.testArg);
    // Hasil ditulis di baris JUDUL, bukan baris daftar: item yang dipilih tetap terlihat
    // dengan kursornya, dan judul kembali normal begitu A/B ditekan. Alasan penolakan yang
    // paling sering disebut langsung -- keterangan lengkapnya tetap di Serial.
    String hasil;
    if (diterima)                                     hasil = "OK, amati aksinya";
    else if (currentState == NodeState::FAULT ||
             currentState == NodeState::ESTOPPED)     hasil = "TOLAK: FAULT/ESTOP";
    else if (mainModeActive)                          hasil = "TOLAK: MAIN aktif";
    else                                              hasil = "TOLAK: lihat Serial";
    lcdPrint(0, 0, hasil);
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

void handleTopMenuKeySorter(char key) {
  if (key == 'A') { topCursor = (topCursor == 0) ? TOP_COUNT - 1 : topCursor - 1; drawTopMenuSorter(); }
  else if (key == 'B') { topCursor = (topCursor + 1) % TOP_COUNT; drawTopMenuSorter(); }
  else if (key == 'C') {
    switch (topCursor) {
      case 0: menuState = MenuState::CAL_GROUP; calGroupCursor = 0; drawCalGroup(); break;
      case 1: menuState = MenuState::TEST_IO_CATEGORY; testIoCatCursor = 0; drawTestIoCategory(); break;
      case 2: menuState = MenuState::TEST_CMD_LIST; cmdTestCursor = 0; drawTestCmdList(); break;
      case 3: menuState = MenuState::SYS_INFO; sysInfoPage = 0; lcdClear(); drawSysInfo(); break;
    }
  } else if (key == 'D') {
    menuState = MenuState::NONE;
    lcdClear();
    Serial.println("[CAL] Keluar mode kalibrasi");
  }
}

// BARU (2026-09-30): watchdog komunikasi (COMM_TIMEOUT) diperbarui oleh SETIAP request Modbus
// yang sukses untuk node ini -- baca maupun tulis -- bukan hanya command.
//
// Dulu hanya onCmdWrite() yang memperbaruinya. Akibatnya node yang RUNNING lama tanpa menerima
// command jatuh ke FAULT COMM_TIMEOUT setelah 30 detik, walaupun master hidup dan terus
// membacanya. Di produksi itu PASTI terjadi: Sorter RUNNING sepanjang shift, sementara di
// antara dua batch master hanya MEMBACA PASS_COUNT dan MENULIS CLASSIFY_IS_REJECT -- tidak ada
// satu pun command. Arm Picker yang siklus MOVE_PACKAGE-nya lebih dari 30 detik, dan homing
// Stocker, kena masalah yang sama di tengah gerakan.
//
// Maksud watchdog ini adalah "master masih hidup", dan request apa pun membuktikannya.
// onRequestSuccess() dipanggil library SETELAH cek slave ID, jadi hanya request untuk node ini
// yang dihitung. modbusEverUsed SENGAJA tidak disetel di sini: watchdog tetap baru aktif
// setelah command pertama, sama seperti sebelumnya.
Modbus::ResultCode onModbusRequestSukses(Modbus::FunctionCode fc, const Modbus::RequestData data) {
  lastRs485Rx = millis();
  return Modbus::EX_SUCCESS;
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
  // DIHAPUS: command Serial "TEST_FAULT" -- opcode-nya sudah tidak ada lagi
  // BARU (celah #1): TESTPASS/TESTREJECT sekarang ikut di-gate mainModeActive juga -- gak boleh
  // simulasi klasifikasi manual selama produksi asli jalan, sama seperti tombol fisik.
  else if (cmd == "TESTPASS") {
    if (mainModeActive) Serial.println("[SERIAL] TESTPASS ditolak -- MAIN aktif, STOP dulu");
    else { enqueueClassification(false, millis()); Serial.println("[SERIAL] Simulasi klasifikasi PASS"); }
  }
  else if (cmd == "TESTREJECT") {
    if (mainModeActive) Serial.println("[SERIAL] TESTREJECT ditolak -- MAIN aktif, STOP dulu");
    else { enqueueClassification(true, millis()); Serial.println("[SERIAL] Simulasi klasifikasi REJECT"); }
  }
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
    Serial.printf("[STATUS] rejectMissed=%lu (antrian penuh / dorongan basi) menuAktif=%d\n",
                  (unsigned long)rejectMissedCount, menuIsActive() ? 1 : 0);
    Serial.printf("[STATUS] kecepatan: %u mm/s (%u mm dalam %u ms, sampel ke-%u) "
                  "saran Mm/s Max=%u, sedang mengukur=%d\n",
                  speedLastMmS, cfg.speedTestDistMm, speedLastMs, speedSampleCount,
                  speedMmSAtMaxPwm, speedSedangMengukur ? 1 : 0);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus motorA=%u palangState=%d\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000), motorAState, (int)palangState);
    Serial.printf("[STATUS] FW=%s build=%s freeHeap=%u\n", FW_VERSION, FW_BUILD, ESP.getFreeHeap());
  }
  else if (cmd == "HELP") {
    Serial.println("[HELP] Command tersedia:");
    Serial.println("  START | STOP | RESET_FAULT | RESET_COUNT");
    Serial.println("  SPEED <0-255> | DIR <0|1>");
    Serial.println("  TEST_PALANG | TESTPASS | TESTREJECT | MOTORA <0-2> | MOTORASPEED <0-255> | STATUS");
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

// DIHAPUS (2026-09-29): activityText() -- teks aktivitas panjang ("Conv+Plg+MtrA") yang dulu
// tampil di baris 1 layar utama. Digantikan activityPendek() di bawah; setelah itu tidak ada
// lagi yang memanggilnya.

// Kode aktivitas numerik untuk register Modbus ACTIVITY_CODE -- supaya Orange Pi bisa
// mengidentifikasi aktivitas spesifik, bukan cuma lewat tampilan LCD lokal.
ActivityCode activityCode() {
  if (currentState == NodeState::FAULT) return ActivityCode::FAULT_AKTIF;
  if (currentState == NodeState::ESTOPPED) return ActivityCode::ESTOP_AKTIF;
  // BARU: testHopperCycleStage dicek SEBELUM currentState==RUNNING_OR_MOVING -- cycle ini
  // sengaja gak ubah currentState (tetap IDLE), jadi kalau dicek belakangan gak akan
  // kesampaian sama sekali. Lihat komentar TEST_HOPPER_AKTIF di registers.h.
  if (testHopperCycleStage != TestHopperCycleStage::NONE) return ActivityCode::TEST_HOPPER_AKTIF;
  // DIPERBAIKI (2026-09-28): palangActive DULU dicek PALING BELAKANG, dan itu membuat
  // CONVEYOR_JALAN_PALANG_AKTIF TIDAK PERNAH BISA MUNCUL SAMA SEKALI. Siklus palang
  // (handlePalangQueue) ikut menyetel motorAState = 1 supaya updateConv1Stby() tahu motor
  // jalan, jadi baris motorAState di bawah selalu menang lebih dulu dan yang terbaca master
  // selalu MOTOR_A_JALAN -- kode yang artinya "jog manual", bukan "palang mendorong".
  // Akibatnya dua hal yang berbeda arti tidak bisa dibedakan dari Modbus, dan panduan uji
  // palang yang menyuruh menunggu kode 2 tidak akan pernah terpenuhi.
  // Sekarang kode 2 = palang sedang PUSH/RETRACT (apa pun keadaan conveyor), kode 3 = Motor A
  // jalan TANPA siklus palang, yaitu benar-benar jog manual.
  if (palangActive) return ActivityCode::CONVEYOR_JALAN_PALANG_AKTIF;
  if (motorAState != 0) return ActivityCode::MOTOR_A_JALAN;
  if (currentState != NodeState::RUNNING_OR_MOVING) return ActivityCode::DIAM;
  return ActivityCode::CONVEYOR_JALAN;
}

// BARU (2026-09-29): teks aktivitas untuk baris 1 layar utama, maksimal 6 karakter.
// "SORTER [AUTO]" sudah memakan 13 kolom, tersisa 6 setelah satu spasi -- teks lama seperti
// "Conv+Plg+MtrA" (13) terpotong di tengah kata. Keadaan sabuk tetap terbaca dari baris State
// (RUNNING = sabuk jalan), jadi
// di sini cukup aktuator yang paling perlu diperhatikan, dengan urutan prioritas yang sama
// persis dengan activityCode() supaya panel dan Modbus tidak pernah saling bertentangan.
const char* activityPendek() {
  if (currentState == NodeState::FAULT) return "FAULT!";
  if (currentState == NodeState::ESTOPPED) return "ESTOP!";
  if (testHopperCycleStage != TestHopperCycleStage::NONE) return "Hopper";
  if (palangActive) return "Palang";
  // "PlgMan" = palang digerakkan MANUAL (jog SET_MOTOR_A dari Modbus / Serial MOTORA).
  // Dulu tertulis "MotorA" -- nama channel driver TB6612FNG, bukan nama fungsinya, sehingga
  // operator tidak tahu bahwa yang bergerak sebenarnya palang. Motor fisiknya sama dengan
  // "Palang" di atas; bedanya yang ini arah mentah dan tidak berhenti sendiri.
  if (motorAState != 0) return "PlgMan";
  if (currentState == NodeState::RUNNING_OR_MOVING && hopperDijeda) return "HopJed";   // BARU 2026-10-05
  if (currentState == NodeState::RUNNING_OR_MOVING) return "Jalan";
  return "Diam";
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
    lcd.init(); lcd.backlight(); for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false;
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
      speedTestConveyorOn = false;      // BARU -- alasan sama: tanpa keypad tidak ada cara mematikan sabuk dari layar itu
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
    lcd.init(); lcd.backlight(); for (uint8_t i = 0; i < LcdCfg::ROWS; i++) lcdCacheValid[i] = false;
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
  // BARU (2026-09-21): PROX_1 sebelumnya TIDAK PERNAH di-pinMode di Sorter -- ia terdaftar
  // di menu Test Input tapi tidak pernah disiapkan, jadi pembacaannya tidak bermakna.
  // Sekarang dipakai sebagai garis START uji kecepatan objek.
  io.pinMode(CH::PROX_1, INPUT_PULLUP);
  io.pinMode(CH::BTN_TEST_HOPPER, INPUT_PULLUP);
  io.pinMode(CH::BTN_TEST_PALANG, INPUT_PULLUP);
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
  mb.addHreg(Reg::MAIN_MODE_ACTIVE, 0);   // BARU -- status live MAIN vs TEST mode
  mb.addHreg(Reg::MENU_ACTIVE, 0);           // BARU -- 1 = operator di menu kalibrasi, command Modbus diabaikan
  mb.addHreg(Reg::REJECT_MISSED_COUNT, 0);   // BARU -- REJECT yang gagal jadi dorongan palang
  mb.addHreg(Reg::SPEED_LAST_MM_S, 0);       // BARU -- uji kecepatan objek PROX_1 -> PROX_2
  mb.addHreg(Reg::SPEED_LAST_MS, 0);
  mb.addHreg(Reg::SPEED_SAMPLE_COUNT, 0);
  mb.addHreg(Reg::SPEED_MM_S_AT_MAX_PWM, 0);
  mb.addHreg(Reg::HOPPER_DIJEDA, 0);         // BARU 2026-10-05
  // BARU (2026-10-07): versi firmware = waktu BUILD (__DATE__/__TIME__ compiler), dibaca Orange Pi
  // di MORE > SYSTEM supaya ketahuan node mana yang sudah di-flash firmware terbaru.
  {
    const char* d = __DATE__;   // "Oct  7 2026"
    const char* t = __TIME__;   // "14:05:09"
    static const char BLN[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    uint16_t bulan = 0;
    for (uint8_t i = 0; i < 12; i++) if (strncmp(d, BLN + i * 3, 3) == 0) bulan = i + 1;
    uint16_t hari = (d[4] == ' ' ? 0 : (d[4] - '0') * 10) + (d[5] - '0');
    mb.addHreg(Reg::FW_TAHUN, (uint16_t)atoi(d + 7));
    mb.addHreg(Reg::FW_BULAN_HARI, (uint16_t)(bulan * 100 + hari));
    mb.addHreg(Reg::FW_JAM_MENIT, (uint16_t)(((t[0] - '0') * 10 + (t[1] - '0')) * 100 + (t[3] - '0') * 10 + (t[4] - '0')));
    mb.addHreg(Reg::FW_VERSI, FIRMWARE_VERSI);
    mb.addHreg(Reg::WIFI_IP_HI, 0); mb.addHreg(Reg::WIFI_IP_LO, 0); mb.addHreg(Reg::WIFI_RSSI, 0);
    Serial.printf("[BOOT] Firmware v%u.%02u, build %s %s\n", FIRMWARE_VERSI / 100, FIRMWARE_VERSI % 100, d, t);
  }
  kal.begin(mb, Reg::CAL_FORMAT, CAL_FORMAT_NODE, CAL_SEG, sizeof(CAL_SEG) / sizeof(CAL_SEG[0]),
            loadConfigFromNvs, saveConfigToNvs);
  mb.onSetHreg(Reg::CMD, onCmdWrite);
  mb.onRequestSuccess(onModbusRequestSukses);   // BARU -- watchdog dari request apa pun
  mb.onSetHreg(Reg::CLASSIFY_IS_REJECT, onClassifyWrite);
  Serial.println("[BOOT] Modbus + register lengkap OK");
  lcdBootProgress("Modbus RS485");

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;

  setupOTA();
  lcdBootProgress("WiFi OTA");

  Serial.println("[BOOT] setup SELESAI");
  Serial.println("[BOOT] Ketik HELP di sini (serial monitor) untuk daftar command uji tanpa QModMaster");
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
  else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); return; }
  drawSysInfo();
}

void loop() {
  kal.loop();   // BARU 2026-10-09: buffer backup kalibrasi & restart setelah restore
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
      } else if (menuState == MenuState::CAL_GROUP) {
        handleCalGroupKey(key);
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
      } else if (menuState == MenuState::TEST_MOD_SERVO) {
        handleTestModServoKey(key);
      } else if (menuState == MenuState::TEST_CMD_LIST) {
        handleTestCmdListKey(key);
      } else if (menuState == MenuState::CONFIRM_RESET) {
        handleConfirmResetKey(key);
      } else if (menuState == MenuState::TEST_KECEPATAN) {
        handleTestKecepatanKey(key);
      } else if (menuState == MenuState::TEST_PALANG) {
        handleTestPalangKey(key);
      } else if (menuState == MenuState::TEST_HOPPER) {
        handleTestHopperKey(key);
      } else if (menuState == MenuState::SYS_INFO) {
        handleSysInfoKey(key);
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
  mb.Hreg(Reg::MAIN_MODE_ACTIVE, mainModeActive ? 1 : 0);
  // BARU: master bisa bedain "node lagi dikalibrasi operator" vs "node mati/kabel putus" --
  // dua-duanya sama-sama TIDAK membalas CMD_ACK_SEQ, jadi sebelumnya tidak bisa dibedakan.
  mb.Hreg(Reg::MENU_ACTIVE, (menuState != MenuState::NONE) ? 1 : 0);

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
  // DIPERBAIKI (2026-09-20): buzzer dipindah KELUAR dari blok bersyarat di bawah. Komentarnya
  // sendiri bilang "TIDAK di-skip walau menu aktif" -- itu benar, tapi tetap ikut ter-skip saat
  // FAULT/ESTOPPED. Kalau fault terjadi tepat di tengah bunyi, tidak ada lagi yang mematikannya
  // dan buzzer meraung terus tanpa henti.
  updateBuzzerBeep();
  // BARU (2026-09-28): SENGAJA di luar blok di bawah -- lihat komentar di updatePalangManual().
  // Perintah palang manual harus bisa dipadamkan justru saat FAULT, bukan berhenti diawasi.
  updatePalangManual();

  if (currentState != NodeState::ESTOPPED && currentState != NodeState::FAULT) {
    handleHopper();   // BARU -- servo hopper aktif otomatis selama RUNNING_OR_MOVING
    updateHopperManual();   // BARU -- perintah tahan posisi dari layar Uji Hopper
    updateTestHopperCycle();   // BARU -- proses test sequence non-blocking (independen dari FSM hopper produksi)
    handleConveyor();
    handlePalangQueue();
    handleSensors();
    handleSpeedTest();     // BARU -- uji kecepatan objek, PASIF (cuma membaca sensor)
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
  // BARU: hasil uji kecepatan harus tampil LIVE -- objek lewat kapan saja, bukan
  // sebagai reaksi atas penekanan tombol.
  if (menuState == MenuState::TEST_KECEPATAN && lcdPresent) {
    static uint32_t lastKecRefresh = 0;
    if (millis() - lastKecRefresh > 200) { lastKecRefresh = millis(); drawTestKecepatan(); }
  }
  // Layar Uji Palang harus hidup sendiri: sisa batas waktu aman terus berkurang, dan siklus
  // otomatis berpindah PUSH -> RETRACT -> selesai tanpa ada tombol yang ditekan.
  if (menuState == MenuState::TEST_PALANG && lcdPresent) {
    static uint32_t lastPalangRefresh = 0;
    if (millis() - lastPalangRefresh > 150) { lastPalangRefresh = millis(); drawTestPalang(); }
  }
  // Layar Uji Hopper juga harus hidup sendiri: posisi servo bergerak per tick, dan penghitung
  // objek naik saat objek melewati sensor -- keduanya tanpa ada tombol yang ditekan.
  if (menuState == MenuState::TEST_HOPPER && lcdPresent) {
    static uint32_t lastHopperUjiRefresh = 0;
    if (millis() - lastHopperUjiRefresh > 150) { lastHopperUjiRefresh = millis(); drawTestHopper(); }
  }
  if (menuState == MenuState::TEST_MOD_MOTORDC && lcdPresent) {
    static uint32_t lastTestModRefresh = 0;
    if (millis() - lastTestModRefresh > 150) { lastTestModRefresh = millis(); drawTestModMotorDC(); }
  }
  // Uptime berjalan terus, jadi layar ini harus hidup sendiri tanpa menunggu tombol.
  // Aman dari masalah LCD-menahan-loop: lcdPrint() hanya menulis baris yang benar-benar
  // berubah, dan di halaman ini cuma baris detik yang bergerak.
  if (menuState == MenuState::SYS_INFO && lcdPresent) {
    static uint32_t lastSysRefresh = 0;
    if (millis() - lastSysRefresh > 500) { lastSysRefresh = millis(); drawSysInfo(); }
  }


  if (menuState != MenuState::NONE) {
    return;   // HANYA command EKSTERNAL (Serial/Modbus) yang dijeda saat menu aktif (§12.6), bukan logic fisik
  }

  handleSerialCommand();

  // Batas 30 detik tanpa request APA PUN dari master (baca atau tulis -- lihat
  // onModbusRequestSukses()). Dulu hanya command yang dihitung, sehingga node yang RUNNING
  // lama tanpa command jatuh FAULT walaupun master terus membacanya.
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
      // DIUBAH (2026-09-29), tata letak permintaan operator:
      //   baris 1  SORTER [AUTO]        Diam
      //   baris 2  Mode : TEST        Miss : 0
      //   baris 3  State : IDLE
      //   baris 4  Pass : 12        Reject : 3
      //
      // Baris 2 dipilih karena dua hal yang PALING sering membingungkan di Sorter tidak
      // terlihat di panel sama sekali sebelumnya:
      //   Mode -- MAIN/TEST tidak selalu sama dengan State. RESET_FAULT mengembalikan State ke
      //     IDLE tapi TIDAK mematikan MAIN, jadi node terlihat IDLE sementara semua command
      //     TEST ditolak dengan alasan "MAIN aktif". Tanpa baris ini, alasannya tidak kelihatan.
      //   Miss -- REJECT_MISSED_COUNT: objek reject yang LOLOS tidak didorong palang (antrian
      //     penuh, atau giliran dorongnya sudah basi). Kegagalan mutu yang sebelumnya hanya
      //     bisa dibaca lewat Modbus; harusnya 0, angka lain berarti ada yang salah.
      // Hasil uji kecepatan yang dulu menempati baris 2 tetap terbaca di layar
      // Setting -> Conveyor -> Uji Kecepatan dan di register 20-23.
      lcdKiriKanan(0, "SORTER [AUTO]", activityPendek());
      lcdPasangan(1, "Mode", "M", mainModeActive ? "MAIN" : "TEST",
                     "Miss", "Ms", String(rejectMissedCount));
      lcdPrint(0, 2, "State : " + String(stateText(currentState)));
      lcdPasangan(3, "Pass", "P", String(passCount), "Reject", "R", String(rejectCount));
    }
  }
}