#pragma once
#include <Arduino.h>

// ============================================================
// KONFIGURASI PIN & ALAMAT — FEEDER (Dispenser)
// BARU: 1 BOARD PCB UNIVERSAL untuk SEMUA 4 NODE (SORTER/PICKER/STOCKER/Dispenser).
// Channel & pin di file ini WAJIB SAMA PERSIS nomornya di keempat config.h --
// yang beda cuma MANA yang benar-benar dipakai main.cpp tiap node.
// Sumber: Sorting_Pinout.xlsx (dikoreksi) + skematik KiCad user.
// ============================================================

namespace Pin {
  constexpr uint8_t I2C_SDA = 21;
  constexpr uint8_t I2C_SCL = 22;
  constexpr uint32_t I2C_FREQ_HZ = 400000;
}

namespace I2CAddr {
  constexpr uint8_t LCD    = 0x21;
  constexpr uint8_t KEYPAD = 0x20;
  constexpr uint8_t MCP1   = 0x22;
  constexpr uint8_t MCP2   = 0x23;
  constexpr uint8_t PCA9685 = 0x40;   // BARU -- Test Modul Servo, konsisten dgn SORTER/PICKER/STOCKER
}

namespace LcdCfg {
  constexpr uint8_t COLS = 20;
  constexpr uint8_t ROWS = 4;
}

// --- Channel MCP23017 -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace CH {
  // Universal (CH0-5) -- sama di semua node
  constexpr uint8_t ESTOP = 0, LED_RUN = 1, LED_FAULT = 2, LED_OPERATION = 3, LED_MANUAL = 4, BUZZER = 5;

  // MCP1 (0x22) CH6-15
  constexpr uint8_t OE_PCA     = 6;    // PICKER
  constexpr uint8_t EN_123     = 7;    // STOCKER
  // DIKOREKSI: AIN1/AIN2 TERTUKAR dari wiring fisik nyata -- user konfirmasi AIN2 ada di
  // GPB0(ch8), AIN1 di GPB1(ch9). Nilai di bawah SUDAH disesuaikan supaya CH::AIN1 software
  // = pin AIN1 fisik TB6612 yang sebenarnya. Harus SAMA di keempat config.h (board universal).
  constexpr uint8_t AIN1       = 9;    // Dispenser push / SORTER motor-A
  constexpr uint8_t AIN2       = 8;
  constexpr uint8_t BIN1       = 10;   // Dispenser conveyor2 / SORTER conveyor
  constexpr uint8_t BIN2       = 11;
  constexpr uint8_t STBY       = 12;   // dipakai bersama AIN+BIN (1 chip TB6612FNG)
  constexpr uint8_t DIR_1_MCP  = 13;   // STOCKER
  constexpr uint8_t DIR_2_MCP  = 14;
  constexpr uint8_t DIR_3_MCP  = 15;

  // MCP2 (0x23) CH16-31
  constexpr uint8_t RLY1      = 17;   // SORTER
  constexpr uint8_t RLY2      = 16;
  // DIKOREKSI: PROX_1/PROX_2 TERTUKAR dari wiring fisik nyata -- user konfirmasi PROX_1 ada
  // di GPA3(ch19), PROX_2 di GPA2(ch18). Harus SAMA di keempat config.h (board universal).
  constexpr uint8_t PROX_1    = 19;   // SORTER
  constexpr uint8_t PROX_2    = 18;
  constexpr uint8_t BUTTON_2  = 20;   // SORTER
  constexpr uint8_t BUTTON_3  = 21;   // SORTER
  constexpr uint8_t LIM_1     = 23;   // Dispenser=LIM_STOCK_EMPTY / STOCKER=LIM_X
  constexpr uint8_t LIM_2     = 22;   // Dispenser=LIM_BOX_ARRIVED / STOCKER=LIM_Y
  constexpr uint8_t LIM_3     = 24;   // Dispenser=LIM_PUSH_HOME   / STOCKER=LIM_Z
  constexpr uint8_t LIM_4     = 25;   // Dispenser=LIM_PUSH_EXTENDED / STOCKER=RACK_LIM[0]
  constexpr uint8_t LIM_5     = 26;
  constexpr uint8_t LIM_6     = 27;
  constexpr uint8_t LIM_7     = 28;
  constexpr uint8_t LIM_8     = 29;
  constexpr uint8_t LIM_9     = 30;
  constexpr uint8_t LIM_10    = 31;   // spare

  // --- Alias khusus Dispenser (FEEDER) ---
  constexpr uint8_t DISP_AIN1 = AIN1, DISP_AIN2 = AIN2;
  constexpr uint8_t DISP_STBY = STBY;
  constexpr uint8_t CONV2_BIN1 = BIN1, CONV2_BIN2 = BIN2;
  constexpr uint8_t LIM_STOCK_EMPTY   = LIM_1;
  // DIKOREKSI: sensor fisik "box arrived" ternyata diwiring ke PROX_1(ch19), BUKAN LIM_2(ch22)
  // -- user konfirmasi. Sebelumnya FSM nungguin LIM_2 yang gak pernah nyala, conveyor selalu
  // timeout 8 detik (FAULT BOX_NOT_ARRIVED), servo gak pernah kepanggil sama sekali.
  constexpr uint8_t LIM_BOX_ARRIVED   = PROX_1;   // sensor UJUNG conveyor -- package FULL siap diambil arm
  // BARU: sensor TENGAH conveyor -- posisi package KOSONG yang diisi objek dari Sorter. Dulu
  // PROX_2 nganggur/spare di Dispenser, sekarang jadi bagian mekanisme 2-proximity.
  constexpr uint8_t PACKAGE_MIDDLE_SENSOR = PROX_2;
  constexpr uint8_t LIM_PUSH_HOME     = LIM_3;
  constexpr uint8_t LIM_PUSH_EXTENDED = LIM_4;
}

// --- Native GPIO -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace GP {
  // DIREVISI: STEP_1(X)=23, STEP_2(Y)=19, STEP_3(Z)=27 -- ditukar dari sebelumnya
  constexpr uint8_t STEP_1 = 23, STEP_2 = 19, STEP_3 = 27;   // tidak dipakai Dispenser
  // DIHAPUS: DIR_1_NTV/DIR_2_NTV/DIR_3_NTV -- tidak ada pin native lain tersedia di ESP32,
  // DIR kembali sepenuhnya ke MCP23017 (lihat CH::DIR_1_MCP/DIR_2_MCP/DIR_3_MCP)
  constexpr uint8_t PWMA_TB6612 = 26, PWMB_TB6612 = 25;   // DITUKAR dari sebelumnya

  // --- Alias khusus Dispenser ---
  constexpr uint8_t CONV2_PWM = PWMB_TB6612;   // conveyor2 (channel B TB6612)
}

// --- RS485 ---
namespace Rs485Cfg {
  constexpr uint8_t RX_PIN = 17, TX_PIN = 16;
  constexpr uint32_t BAUD = 19200;
  constexpr uint8_t SLAVE_ID = 3;   // Dispenser = slave 3
}