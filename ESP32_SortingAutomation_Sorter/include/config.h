#pragma once
#include <Arduino.h>

// ============================================================
// KONFIGURASI PIN & ALAMAT — SORTER
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
  constexpr uint8_t PCA9685 = 0x40;   // BARU -- servo hopper, SAMA alamat & konvensi dengan PICKER
}

// --- Channel MCP23017 -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace CH {
  // Universal (CH0-5) -- sama di semua node
  constexpr uint8_t ESTOP = 0, LED_RUN = 1, LED_FAULT = 2, LED_OPERATION = 3, LED_MANUAL = 4, BUZZER = 5;

  // MCP1 (0x22) CH6-15
  constexpr uint8_t OE_PCA     = 6;    // PICKER
  constexpr uint8_t EN_123     = 7;    // STOCKER
  constexpr uint8_t AIN1       = 8;    // SORTER motor-A / Dispenser dispenser-push
  constexpr uint8_t AIN2       = 9;
  constexpr uint8_t BIN1       = 10;   // SORTER conveyor / Dispenser conveyor2
  constexpr uint8_t BIN2       = 11;
  constexpr uint8_t STBY       = 12;   // dipakai bersama AIN+BIN (1 chip TB6612FNG)
  constexpr uint8_t DIR_1_MCP  = 13;   // STOCKER
  constexpr uint8_t DIR_2_MCP  = 14;   // STOCKER
  constexpr uint8_t DIR_3_MCP  = 15;   // STOCKER

  // MCP2 (0x23) CH16-31
  constexpr uint8_t RLY1      = 16;   // SORTER -- palang solenoid
  constexpr uint8_t RLY2      = 17;   // spare
  constexpr uint8_t PROX_1    = 18;   // SORTER -- proximity sorting
  constexpr uint8_t PROX_2    = 19;   // spare
  constexpr uint8_t BUTTON_2  = 20;   // SORTER -- test pass
  constexpr uint8_t BUTTON_3  = 21;   // SORTER -- test reject
  constexpr uint8_t LIM_1     = 23;   // STOCKER/Dispenser
  constexpr uint8_t LIM_2     = 22;
  constexpr uint8_t LIM_3     = 24;
  constexpr uint8_t LIM_4     = 25;
  constexpr uint8_t LIM_5     = 26;
  constexpr uint8_t LIM_6     = 27;
  constexpr uint8_t LIM_7     = 28;
  constexpr uint8_t LIM_8     = 29;
  constexpr uint8_t LIM_9     = 30;
  constexpr uint8_t LIM_10    = 31;   // spare

  // --- Alias khusus SORTER ---
  constexpr uint8_t CONV1_AIN1 = AIN1, CONV1_AIN2 = AIN2;   // motor A tambahan
  constexpr uint8_t CONV1_STBY = STBY;
  constexpr uint8_t CONV1_BIN1 = BIN1, CONV1_BIN2 = BIN2;   // conveyor existing
  constexpr uint8_t PALANG_RELAY  = RLY1;
  constexpr uint8_t PROX_PASS     = PROX_1;
  constexpr uint8_t BTN_TEST_PASS = BUTTON_2, BTN_TEST_REJECT = BUTTON_3;
}

// --- Native GPIO -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace GP {
  // DIREVISI: STEP_1(X)=23, STEP_2(Y)=19, STEP_3(Z)=27 -- ditukar dari sebelumnya
  constexpr uint8_t STEP_1 = 23, STEP_2 = 19, STEP_3 = 27;   // tidak dipakai SORTER
  // DIHAPUS: DIR_1_NTV/DIR_2_NTV/DIR_3_NTV -- tidak ada pin native lain tersedia di ESP32,
  // DIR kembali sepenuhnya ke MCP23017 (lihat CH::DIR_1_MCP/DIR_2_MCP/DIR_3_MCP)
  constexpr uint8_t PWMA_TB6612 = 26, PWMB_TB6612 = 25;   // DITUKAR dari sebelumnya

  // --- Alias khusus SORTER ---
  constexpr uint8_t MOTOR_A_PWM = PWMA_TB6612;   // motor A (channel A TB6612)
  constexpr uint8_t CONV1_PWM   = PWMB_TB6612;   // conveyor (channel B TB6612)
}

namespace LcdCfg {
  constexpr uint8_t COLS = 20;
  constexpr uint8_t ROWS = 4;
}

// --- RS485 ---
namespace Rs485Cfg {
  constexpr uint8_t RX_PIN = 17, TX_PIN = 16;   // GPIO16/17 (UART2 hardware)
  constexpr uint32_t BAUD = 19200;
  constexpr uint8_t SLAVE_ID = 1;   // SORTER = slave 1
}