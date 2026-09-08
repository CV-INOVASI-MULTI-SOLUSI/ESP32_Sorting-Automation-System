#pragma once
#include <Arduino.h>

// ============================================================
// KONFIGURASI PIN & ALAMAT — PICKER
// BARU: 1 BOARD PCB UNIVERSAL untuk SEMUA 4 NODE (SORTER/PICKER/STOCKER/Dispenser).
// Channel & pin di file ini WAJIB SAMA PERSIS nomornya di keempat config.h --
// yang beda cuma MANA yang benar-benar dipakai main.cpp tiap node.
// Sumber: Sorting_Pinout.xlsx (dikoreksi) + skematik KiCad user.
// ============================================================

namespace Pin {
  constexpr uint8_t I2C_SDA = 21;
  constexpr uint8_t I2C_SCL = 22;
  constexpr uint32_t I2C_FREQ_HZ = 100000;
}

namespace I2CAddr {
  constexpr uint8_t LCD    = 0x21;
  constexpr uint8_t KEYPAD = 0x20;
  constexpr uint8_t MCP1   = 0x22;
  constexpr uint8_t MCP2   = 0x23;
  constexpr uint8_t PCA9685 = 0x40;   // khusus PICKER
}

// --- Channel MCP23017 -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace CH {
  // Universal (CH0-5) -- sama di semua node
  constexpr uint8_t ESTOP = 0, LED_RUN = 1, LED_FAULT = 2, LED_OPERATION = 3, LED_MANUAL = 4, BUZZER = 5;

  // MCP1 (0x22) CH6-15
  constexpr uint8_t OE_PCA     = 6;    // PICKER -- PCA9685 output-enable
  constexpr uint8_t PCA_OE     = OE_PCA;   // alias -- nama yang dipakai main.cpp PICKER
  constexpr uint8_t EN_123     = 7;    // STOCKER
  constexpr uint8_t AIN1       = 8;    // SORTER/Dispenser
  constexpr uint8_t AIN2       = 9;
  constexpr uint8_t BIN1       = 10;
  constexpr uint8_t BIN2       = 11;
  constexpr uint8_t STBY       = 12;
  constexpr uint8_t DIR_1_MCP  = 13;   // STOCKER
  constexpr uint8_t DIR_2_MCP  = 14;
  constexpr uint8_t DIR_3_MCP  = 15;

  // MCP2 (0x23) CH16-31 -- tidak dipakai PICKER
  constexpr uint8_t RLY1      = 16;
  constexpr uint8_t RLY2      = 17;
  constexpr uint8_t PROX_1    = 18;
  constexpr uint8_t PROX_2    = 19;
  constexpr uint8_t BUTTON_2  = 20;
  constexpr uint8_t BUTTON_3  = 21;
  constexpr uint8_t LIM_1     = 23;
  constexpr uint8_t LIM_2     = 22;
  constexpr uint8_t LIM_3     = 24;
  constexpr uint8_t LIM_4     = 25;
  constexpr uint8_t LIM_5     = 26;
  constexpr uint8_t LIM_6     = 27;
  constexpr uint8_t LIM_7     = 28;
  constexpr uint8_t LIM_8     = 29;
  constexpr uint8_t LIM_9     = 30;
  constexpr uint8_t LIM_10    = 31;
}

// --- Native GPIO -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE (tidak dipakai PICKER) ---
namespace GP {
  // DIREVISI: STEP_1(X)=23, STEP_2(Y)=19, STEP_3(Z)=27 -- ditukar dari sebelumnya
  constexpr uint8_t STEP_1 = 23, STEP_2 = 19, STEP_3 = 27;
  // DIHAPUS: DIR_1_NTV/DIR_2_NTV/DIR_3_NTV -- tidak ada pin native lain tersedia di ESP32,
  // DIR kembali sepenuhnya ke MCP23017 (lihat CH::DIR_1_MCP/DIR_2_MCP/DIR_3_MCP)
  constexpr uint8_t PWMA_TB6612 = 26, PWMB_TB6612 = 25;   // DITUKAR dari sebelumnya
}

// --- Servo (5 joint lengan + 1 gripper = 6) ---
namespace ServoCfg {
  constexpr uint8_t NUM_JOINTS = 6;
  constexpr uint16_t MIN_US = 500, MAX_US = 2500;   // MG996R
  constexpr uint16_t FREQ_HZ = 50;
}

namespace LcdCfg {
  constexpr uint8_t COLS = 20;
  constexpr uint8_t ROWS = 4;
}

// --- RS485 ---
namespace Rs485Cfg {
  constexpr uint8_t RX_PIN = 17, TX_PIN = 16;
  constexpr uint32_t BAUD = 19200;
  constexpr uint8_t SLAVE_ID = 2;   // PICKER = slave 2
}