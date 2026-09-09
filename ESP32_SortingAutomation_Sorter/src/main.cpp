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
  uint16_t palangPulseMs    = 300;
  float    distMm           = 150.0f;        // O2: ukur jarak fisik scan->palang
  float    mmPerSecAtMaxPwm = 300.0f;        // O2: ukur kecepatan conveyor aktual
  uint8_t  motorASpeed      = 180;           // BARU -- PWM kecepatan motor A (0-255), independen dari conveyor
  // BARU: Hopper servo + rack-pinion -- titik awal/dorong dalam microdetik pulsa (spt PICKER),
  // interval dalam ms (total waktu 1 siklus dorong+kembali, BUKAN cuma waktu tunggu)
  uint16_t hopperStartUs    = 1000;
  uint16_t hopperPushUs     = 2000;
  uint16_t hopperIntervalMs = 1000;
  // BARU: buzzer notifikasi -- durasi ON/OFF (ms) saat trigger event (palang aktif/reject)
  uint16_t buzzerOnMs  = 500;
  uint16_t buzzerOffMs = 500;
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
uint32_t palangTriggerAt = 0, palangOffAt = 0;
bool lastProxState = HIGH;
uint32_t lastProxEdge = 0;

struct PendingClass { bool valid; uint32_t scanTimeMs; bool isReject; };
PendingClass pendingQ[4];
uint8_t qHead = 0, qTail = 0;

constexpr int PWM_FREQ = 20000, PWM_RES = 8, LEDC_CH_CONV1 = 0, LEDC_CH_MOTORA = 1;

// BARU: driver PCA9685 raw (Wire langsung, tanpa library) -- dipakai UNTUK PRODUKSI hopper
// servo, KARENA board universal sudah jadi & tidak ada pin native yang benar-benar bebas
// dipakai ulang (semua pin sudah terikat desain PCB tetap). Servo hopper WAJIB lewat I2C.
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t HOPPER_PCA_CHANNEL = 0;   // channel PCA9685 yang dipakai utk servo hopper
bool pca9685Detected() { Wire.beginTransmission(PCA9685_ADDR); return (Wire.endTransmission() == 0); }
void pca9685Init() {
  Wire.beginTransmission(PCA9685_ADDR); Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x10); Wire.endTransmission();
  Wire.beginTransmission(PCA9685_ADDR); Wire.write((uint8_t)0xFE); Wire.write((uint8_t)121); Wire.endTransmission();
  Wire.beginTransmission(PCA9685_ADDR); Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x20); Wire.endTransmission();
  delay(5);
}
void pca9685SetServoUs(uint8_t channel, uint16_t us) {
  uint32_t ticks = (uint32_t)us * 4096UL / 20000UL;
  uint8_t reg = 0x06 + 4 * channel;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg); Wire.write((uint8_t)0); Wire.write((uint8_t)0);
  Wire.write((uint8_t)(ticks & 0xFF)); Wire.write((uint8_t)(ticks >> 8));
  Wire.endTransmission();
}

// DIUBAH: hopper servo sekarang lewat PCA9685 (I2C), BUKAN lagi LEDC/native GPIO
void hopperSetUs(uint16_t us) {
  pca9685SetServoUs(HOPPER_PCA_CHANNEL, us);
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
enum class HopperState { AT_START, PUSHING, RETURNING };
HopperState hopperState = HopperState::AT_START;
uint32_t hopperStateEnteredAt = 0;
constexpr uint32_t HOPPER_DWELL_MS = 200;   // waktu servo geser+settle per tahap gerak (fisik, bukan sensor)
bool hopperIntervalTestMode = false;   // di-set true/false dari handleCalListKey()/handleParamKey() -- lihat di bawah
void handleHopper() {
  // BARU: kalau operator SEDANG di layar kalibrasi "Hopper Interval(ms)", paksa FSM tetap
  // bersiklus LIVE walau currentState bukan RUNNING_OR_MOVING -- supaya waktu antar-dorong
  // bisa diamati/diverifikasi presisi langsung sambil nilai disesuaikan, tanpa perlu START
  // produksi penuh dulu. Berhenti otomatis begitu operator keluar layar ini.
  if (currentState != NodeState::RUNNING_OR_MOVING && !hopperIntervalTestMode) {
    if (hopperState != HopperState::AT_START) { hopperSetUs(cfg.hopperStartUs); hopperState = HopperState::AT_START; }
    hopperStateEnteredAt = millis();
    return;
  }
  uint32_t elapsed = millis() - hopperStateEnteredAt;
  switch (hopperState) {
    case HopperState::AT_START: {
      // sisa waktu tunggu = interval TOTAL dikurangi waktu gerak (push+return) -- supaya
      // total 1 siklus penuh persis sama dgn interval yg di-set, bukan interval+waktu gerak
      uint32_t waitMs = (cfg.hopperIntervalMs > 2 * HOPPER_DWELL_MS) ? (cfg.hopperIntervalMs - 2 * HOPPER_DWELL_MS) : 0;
      if (elapsed >= waitMs) {
        hopperSetUs(cfg.hopperPushUs);
        hopperState = HopperState::PUSHING;
        hopperStateEnteredAt = millis();
      }
      break;
    }
    case HopperState::PUSHING:
      if (elapsed >= HOPPER_DWELL_MS) {
        hopperSetUs(cfg.hopperStartUs);
        hopperState = HopperState::RETURNING;
        hopperStateEnteredAt = millis();
      }
      break;
    case HopperState::RETURNING:
      if (elapsed >= HOPPER_DWELL_MS) {
        hopperState = HopperState::AT_START;
        hopperStateEnteredAt = millis();   // mulai hitung interval BARU dari sini -- siklus berikutnya
      }
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

// BARU: motor A -- channel yang tadinya menganggur di chip TB6612FNG yang sama dgn conveyor.
// Kontrol manual (Test Command/Serial), TIDAK terintegrasi otomatis ke FSM produksi manapun --
// keputusan itu menunggu Anda tentukan mekanisme fisiknya untuk apa.
void setMotorA(uint8_t dirState) {
  motorAState = dirState;
  if (dirState == 0) { io.write(CH::CONV1_AIN1, LOW); io.write(CH::CONV1_AIN2, LOW); ledcWrite(LEDC_CH_MOTORA, 0); }
  else { motorWrite(CH::CONV1_AIN1, CH::CONV1_AIN2, dirState == 1); ledcWrite(LEDC_CH_MOTORA, cfg.motorASpeed); }
  updateConv1Stby();
}

void handlePalangQueue() {
  uint32_t now = millis();
  if (qHead != qTail && !palangPending) {
    PendingClass &pc = pendingQ[qHead];
    if (pc.isReject) { palangTriggerAt = pc.scanTimeMs + calculateTOF(); palangPending = true; }
    qHead = (qHead + 1) % 4;
  }
  if (palangPending && !palangActive && now >= palangTriggerAt) {
    io.write(CH::PALANG_RELAY, HIGH);   // solenoid push-pull 1 kumparan, HIGH=dorong
    triggerBuzzerBeep();   // BARU -- notifikasi audio saat objek REJECT ditolak
    palangActive = true; palangOffAt = now + cfg.palangPulseMs;
    rejectCount++;
    mb.Hreg(Reg::REJECT_COUNT, (uint16_t)rejectCount);
    Serial.printf("[SORTER] Palang TRIGGER! rejectCount=%lu\n", (unsigned long)rejectCount);
    palangPending = false;
  }
  if (palangActive && now >= palangOffAt) {
    io.write(CH::PALANG_RELAY, LOW);   // lepas, pegas balik sendiri
    palangActive = false;
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
    io.write(CH::CONV1_STBY, LOW); io.write(CH::PALANG_RELAY, LOW);
    ledcWrite(LEDC_CH_CONV1, 0); ledcWrite(LEDC_CH_MOTORA, 0); motorAState = 0;   // BARU -- motor A ikut mati
    return;
  }
  if (currentState == NodeState::ESTOPPED) currentState = NodeState::IDLE;   // diam, TIDAK auto-RUNNING (D11)
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
    case Cmd::SET_HOPPER_INTERVAL: cfg.hopperIntervalMs = (uint16_t)constrain(arg, 300, 10000); break;
    case Cmd::SET_CONVEYOR_SPEED:  cfg.conveyorSpeed = (uint8_t)constrain(arg, 0, 255); break;
    case Cmd::SET_CONVEYOR_DIR:    cfg.conveyorDir = (arg != 0); break;
    case Cmd::RESET_COUNTERS:
      passCount = 0; rejectCount = 0;
      mb.Hreg(Reg::PASS_COUNT, 0); mb.Hreg(Reg::REJECT_COUNT, 0);
      break;
    case Cmd::SET_MOTOR_A: setMotorA((uint8_t)constrain(arg, 0, 2)); break;
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
                        TEST_INPUT_LIST, TEST_RS485, TEST_MODULE_SELECT, TEST_MOD_STEPPER, TEST_MOD_MOTORDC,
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
constexpr uint8_t CAL_COUNT = 12;
const char* CAL_LABELS[CAL_COUNT] = { "Conveyor Speed", "Conveyor Dir", "Palang Pulse", "Dist (TOF mm)", "Mm/s Max",
                                        "Motor A Speed", "Hopper Titik Awal", "Hopper Titik Dorong", "Hopper Interval(ms)",
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
    if (calCursor == 11) {   // "Reset ke Default" -- minta konfirmasi dulu, bukan langsung eksekusi
      menuState = MenuState::CONFIRM_RESET;
      drawConfirmReset();
    } else {
      selParam = calCursor + 1;
      hopperIntervalTestMode = (selParam == 9);   // BARU -- aktifkan test-live cuma utk layar Hopper Interval
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
    case 3: lcdPrint(0, 0, "PALANG PULSE (ms)"); break;
    case 4: lcdPrint(0, 0, "DIST_MM"); break;
    case 5: lcdPrint(0, 0, "MM_PER_SEC_MAX"); break;
    case 6: lcdPrint(0, 0, "MOTOR A SPEED"); break;
    case 7: lcdPrint(0, 0, "HOPPER TITIK AWAL"); break;
    case 8: lcdPrint(0, 0, "HOPPER TITIK DORONG"); break;
    case 9: lcdPrint(0, 0, "HOPPER INTERVAL(ms)"); break;
    case 10: lcdPrint(0, 0, "BUZZER ON (ms)"); break;
    case 11: lcdPrint(0, 0, "BUZZER OFF (ms)"); break;
  }
  String line1;
  switch (selParam) {
    case 1: line1 = "Nilai:" + String(cfg.conveyorSpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 2: line1 = "Arah:" + String(cfg.conveyorDir ? "FORWARD" : "REVERSE"); break;
    case 3: line1 = "Nilai:" + String(cfg.palangPulseMs) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 4: line1 = "Nilai:" + String(cfg.distMm, 1) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 5: line1 = "Nilai:" + String(cfg.mmPerSecAtMaxPwm, 1) + " Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 6: line1 = "Nilai:" + String(cfg.motorASpeed) + "   Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 7: line1 = "us:" + String(cfg.hopperStartUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 8: line1 = "us:" + String(cfg.hopperPushUs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 9: line1 = "ms:" + String(cfg.hopperIntervalMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 10: line1 = "ms:" + String(cfg.buzzerOnMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
    case 11: line1 = "ms:" + String(cfg.buzzerOffMs) + "  Step:" + String(JOG_STEPS[jogStepIdx]); break;
  }
  lcdPrint(0, 1, line1);
  lcdPrint(0, 2, selParam == 2 ? "A=Forward B=Reverse" : "A+ B- C:step");
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
      if (key == 'A') cfg.palangPulseMs = (uint16_t)constrain((int)cfg.palangPulseMs + step, 50, 2000);
      else if (key == 'B') cfg.palangPulseMs = (uint16_t)constrain((int)cfg.palangPulseMs - step, 50, 2000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 4:
      if (key == 'A') cfg.distMm += step;
      else if (key == 'B') cfg.distMm = max(0.0f, cfg.distMm - step);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 5:
      if (key == 'A') cfg.mmPerSecAtMaxPwm += step;
      else if (key == 'B') cfg.mmPerSecAtMaxPwm = max(5.0f, cfg.mmPerSecAtMaxPwm - step);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 6:
      if (key == 'A') cfg.motorASpeed = (uint8_t)constrain((int)cfg.motorASpeed + step, 0, 255);
      else if (key == 'B') cfg.motorASpeed = (uint8_t)constrain((int)cfg.motorASpeed - step, 0, 255);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      if (motorAState != 0) ledcWrite(LEDC_CH_MOTORA, cfg.motorASpeed);   // update live kalau sedang jalan
      break;
    // BARU: Hopper titik awal/dorong -- PREVIEW LIVE (servo langsung gerak saat dijog), sama
    // pola dgn kalibrasi offset PICKER, supaya operator lihat langsung hasilnya di rack-pinion.
    case 7:
      if (key == 'A') cfg.hopperStartUs = (uint16_t)constrain((int)cfg.hopperStartUs + step, 500, 2500);
      else if (key == 'B') cfg.hopperStartUs = (uint16_t)constrain((int)cfg.hopperStartUs - step, 500, 2500);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      hopperSetUs(cfg.hopperStartUs);   // preview live
      break;
    case 8:
      if (key == 'A') cfg.hopperPushUs = (uint16_t)constrain((int)cfg.hopperPushUs + step, 500, 2500);
      else if (key == 'B') cfg.hopperPushUs = (uint16_t)constrain((int)cfg.hopperPushUs - step, 500, 2500);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      hopperSetUs(cfg.hopperPushUs);   // preview live
      break;
    case 9:
      if (key == 'A') cfg.hopperIntervalMs = (uint16_t)constrain((int)cfg.hopperIntervalMs + step * 10, 300, 10000);
      else if (key == 'B') cfg.hopperIntervalMs = (uint16_t)constrain((int)cfg.hopperIntervalMs - step * 10, 300, 10000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 10:
      if (key == 'A') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs + step * 10, 50, 5000);
      else if (key == 'B') cfg.buzzerOnMs = (uint16_t)constrain((int)cfg.buzzerOnMs - step * 10, 50, 5000);
      else if (key == 'C') jogStepIdx = (jogStepIdx + 1) % 4;
      break;
    case 11:
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
  {"PALANG", CH::PALANG_RELAY,  false},
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

// --- Kategori: Test Input (semua input, IN1..IN5, baca saja) ---
// --- Kategori: Test Input -- semua input tampil sekaligus (live, tanpa perlu masuk item) ---
constexpr uint8_t INPUT_TEST_COUNT = 5;
IOTestItem INPUT_TEST_ITEMS[INPUT_TEST_COUNT] = {
  {"IN1", CH::ESTOP,           false},
  {"IN2", CH::PROX_PASS,       false},
  {"IN3", CH::BTN_TEST_PASS,   false},
  {"IN4", CH::BTN_TEST_REJECT, false},
  {"IN5", CH::CONV1_BIN1,      false},
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
  else if (key == 'B') { if (inputTestScrollTop < (INPUT_TEST_COUNT > 4 ? INPUT_TEST_COUNT - 4 : 0)) { inputTestScrollTop++; drawTestInputList(); } }
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

// --- Modul: Servo (PCA9685) -- cek deteksi I2C dulu, baru izinkan gerak ---
uint8_t testModServoCh = 0;
bool testModServoDetected = false;
void drawTestModServo() {
  if (testModFirstDraw) {
    lcd.clear(); lcdPrint(0, 0, "MODUL: SERVO");
    testModServoDetected = pca9685Detected();
    if (testModServoDetected) pca9685Init();
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
    else if (key == 'A') { pca9685SetServoUs(testModServoCh, 1700); }
    else if (key == 'B') { pca9685SetServoUs(testModServoCh, 1300); }
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
  } else if (key == 'D') { menuState = MenuState::TOP_SELECT; drawTopMenuSorter(); }
}

// --- LEVEL 1c: TEST COMMAND (simulasi command SEOLAH dari node lain via Modbus) ---
struct CmdTestItem { const char* label; Cmd opcode; uint16_t testArg; };
constexpr uint8_t CMD_TEST_COUNT = 10;
CmdTestItem CMD_TEST_ITEMS[CMD_TEST_COUNT] = {
  {"START",             Cmd::START,               0},
  {"STOP",               Cmd::STOP,                0},
  {"RESET_FAULT",        Cmd::RESET_FAULT,         0},
  {"SET_SPEED (test=200)",Cmd::SET_CONVEYOR_SPEED, 200},
  {"SET_DIR (test=0)",   Cmd::SET_CONVEYOR_DIR,    0},
  {"RESET_COUNTERS",     Cmd::RESET_COUNTERS,      0},
  {"MOTOR_A Maju",       Cmd::SET_MOTOR_A,         1},
  {"MOTOR_A Stop",       Cmd::SET_MOTOR_A,         0},
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
    cfg.motorASpeed = (uint8_t)constrain((int)line.substring(spaceIdx + 1).toInt(), 0, 255);
    saveConfigToNvs();
    if (motorAState != 0) ledcWrite(LEDC_CH_MOTORA, cfg.motorASpeed);   // update live kalau sedang jalan
    Serial.printf("[MOTORASPEED] %u\n", cfg.motorASpeed);
  }
  else if (cmd == "STATUS") {
    Serial.printf("[STATUS] state=%s fault=%u pass=%lu reject=%lu ESTOP=%d speed=%u dir=%d dist=%.1f mmps=%.1f\n",
                  stateText(currentState), faultCode, (unsigned long)passCount, (unsigned long)rejectCount,
                  io.read(CH::ESTOP), cfg.conveyorSpeed, cfg.conveyorDir, cfg.distMm, cfg.mmPerSecAtMaxPwm);
    Serial.printf("[STATUS] activity=%u i2cErrCount=%u lastFault=%u uptime=%lus motorA=%u\n",
                  (uint16_t)activityCode(), i2cErrorCount, lastFaultCode, (unsigned long)(millis() / 1000), motorAState);
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

  bool ioOk = io.begin(I2CAddr::MCP1, I2CAddr::MCP2);
  if (!ioOk) { faultCode = (uint16_t)FaultCode::IO_EXPANDER_MISSING; currentState = NodeState::FAULT; }

  io.pinMode(CH::ESTOP, INPUT_PULLUP);
  io.pinMode(CH::LED_RUN, OUTPUT); io.pinMode(CH::LED_FAULT, OUTPUT); io.pinMode(CH::BUZZER, OUTPUT);
  io.pinMode(CH::LED_OPERATION, OUTPUT); io.pinMode(CH::LED_MANUAL, OUTPUT);   // BARU
  io.pinMode(CH::CONV1_BIN1, OUTPUT); io.pinMode(CH::CONV1_BIN2, OUTPUT); io.pinMode(CH::CONV1_STBY, OUTPUT);
  io.pinMode(CH::PALANG_RELAY, OUTPUT);
  // BARU: 2 motor DC opsional -- pinMode disiapkan meski belum tentu fisiknya terpasang
  io.pinMode(CH::CONV1_AIN1, OUTPUT); io.pinMode(CH::CONV1_AIN2, OUTPUT);   // BARU -- motor A (channel yg tadinya menganggur)
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

  ledcSetup(LEDC_CH_CONV1, PWM_FREQ, PWM_RES);
  ledcAttachPin(GP::CONV1_PWM, LEDC_CH_CONV1);
  ledcSetup(LEDC_CH_MOTORA, PWM_FREQ, PWM_RES);       // BARU
  ledcAttachPin(GP::MOTOR_A_PWM, LEDC_CH_MOTORA);     // BARU
  // BARU: Hopper servo -- pakai GP::STEP_1 (GPIO23), pin ini MENGANGGUR di board SORTER
  // (tidak ada stepper terpasang fisik untuk role SORTER)
  // DIHAPUS: ledcSetup/ledcAttachPin utk hopper -- servo sekarang lewat PCA9685 (I2C), bukan LEDC
  Serial.println("[BOOT] LEDC PWM conveyor1 + motor A OK");

  loadConfigFromNvs();   // BARU -- cfg sekarang persist antar reboot, sebelumnya selalu reset ke default
  Serial.println("[BOOT] SorterConfig dimuat dari NVS (kalau pernah disimpan)");

  // DIPERBAIKI (bug urutan): pca9685Init() + hopperSetUs() HARUS setelah loadConfigFromNvs() --
  // sebelumnya hopperSetUs() dipanggil pakai nilai DEFAULT compile-time (cfg.hopperStartUs belum
  // dimuat dari NVS), bukan nilai kalibrasi tersimpan yang sebenarnya.
  pca9685Init();
  hopperSetUs(cfg.hopperStartUs);   // posisi awal servo saat boot, PAKAI nilai kalibrasi tersimpan
  Serial.println("[BOOT] PCA9685 hopper servo diinisialisasi");

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

  if (currentState != NodeState::FAULT) currentState = NodeState::IDLE;
  Serial.println("[BOOT] setup SELESAI");
  Serial.println("[BOOT] Ketik HELP di sini (serial monitor) untuk daftar command uji tanpa QModMaster");
}

void loop() {
  mb.task();

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