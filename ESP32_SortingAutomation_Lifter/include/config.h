#pragma once
#include <Arduino.h>

// ============================================================
// KONFIGURASI PIN & ALAMAT — STOCKER
// BARU: 1 BOARD PCB UNIVERSAL untuk SEMUA 4 NODE (SORTER/PICKER/STOCKER/Dispenser).
// Channel & pin di file ini WAJIB SAMA PERSIS nomornya di keempat config.h --
// yang beda cuma MANA yang benar-benar dipakai main.cpp tiap node.
// Sumber: Sorting_Pinout.xlsx (dikoreksi) + skematik KiCad user.
// ============================================================

namespace Pin {
  constexpr uint8_t I2C_SDA = 21;
  constexpr uint8_t I2C_SCL = 22;
  constexpr uint32_t I2C_FREQ_HZ = 400000;   // STOCKER dinaikkan dari 100000 -- 4x lebih cepat per transaksi I2C
}

namespace I2CAddr {
  constexpr uint8_t LCD    = 0x21;
  constexpr uint8_t KEYPAD = 0x20;
  constexpr uint8_t MCP1   = 0x22;
  constexpr uint8_t MCP2   = 0x23;
}

// --- Channel MCP23017 -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace CH {
  // Universal (CH0-5) -- sama di semua node
  constexpr uint8_t ESTOP = 0, LED_RUN = 1, LED_FAULT = 2, LED_OPERATION = 3, LED_MANUAL = 4, BUZZER = 5;

  // MCP1 (0x22) CH6-15 -- dipakai beda-beda tergantung node
  constexpr uint8_t OE_PCA     = 6;    // PICKER -- PCA9685 output-enable
  constexpr uint8_t EN_123     = 7;    // STOCKER -- EN gabungan 3x TMC2209 (1 sinyal, fan-out hardware)
  constexpr uint8_t AIN1       = 8;    // SORTER motor-A / Dispenser dispenser-push
  constexpr uint8_t AIN2       = 9;
  constexpr uint8_t BIN1       = 10;   // SORTER conveyor / Dispenser conveyor2
  constexpr uint8_t BIN2       = 11;
  constexpr uint8_t STBY       = 12;   // dipakai bersama AIN+BIN (1 chip TB6612FNG)
  constexpr uint8_t DIR_1_MCP  = 13;   // STOCKER -- opsi jumper DIR_X (alternatif dari native GPIO32)
  constexpr uint8_t DIR_2_MCP  = 14;   // STOCKER -- opsi jumper DIR_Y (alternatif dari native GPIO33)
  constexpr uint8_t DIR_3_MCP  = 15;   // STOCKER -- opsi jumper DIR_Z (alternatif dari native GPIO4)

  // MCP2 (0x23) CH16-31 -- dipakai beda-beda tergantung node
  constexpr uint8_t RLY1      = 16;   // SORTER -- palang solenoid
  constexpr uint8_t RLY2      = 17;   // spare
  constexpr uint8_t PROX_1    = 18;   // SORTER -- proximity sorting
  constexpr uint8_t PROX_2    = 19;   // spare
  constexpr uint8_t BUTTON_2  = 20;   // SORTER -- test pass
  constexpr uint8_t BUTTON_3  = 21;   // SORTER -- test reject
  constexpr uint8_t LIM_1     = 23;   // STOCKER=LIM_X, Dispenser=LIM_STOCK_EMPTY
  constexpr uint8_t LIM_2     = 22;   // STOCKER=LIM_Y, Dispenser=LIM_BOX_ARRIVED
  constexpr uint8_t LIM_3     = 24;   // STOCKER=LIM_Z, Dispenser=LIM_PUSH_HOME
  constexpr uint8_t LIM_4     = 25;   // STOCKER=RACK_LIM[0], Dispenser=LIM_PUSH_EXTENDED
  constexpr uint8_t LIM_5     = 26;   // STOCKER=RACK_LIM[1]
  constexpr uint8_t LIM_6     = 27;   // STOCKER=RACK_LIM[2]
  constexpr uint8_t LIM_7     = 28;   // STOCKER=RACK_LIM[3]
  constexpr uint8_t LIM_8     = 29;   // STOCKER=RACK_LIM[4]
  constexpr uint8_t LIM_9     = 30;   // STOCKER=RACK_LIM[5]
  constexpr uint8_t LIM_10    = 31;   // spare

  // --- Alias khusus STOCKER (dipetakan ke channel universal di atas, TIDAK dobel-alokasi) ---
  constexpr uint8_t EN_STEPPERS   = EN_123;
  constexpr uint8_t LIM_X         = LIM_1;
  constexpr uint8_t LIM_Y         = LIM_2;
  constexpr uint8_t LIM_Z         = LIM_3;
  constexpr uint8_t RACK_LIM[6]   = { LIM_4, LIM_5, LIM_6, LIM_7, LIM_8, LIM_9 };
  // DIHAPUS: RACK_LED -- fitur ini TIDAK dipakai lagi di board universal baru (konfirmasi user)
}

// --- Native GPIO -- UNIVERSAL, SAMA PERSIS DI SEMUA 4 NODE ---
namespace GP {
  // DIREVISI: STEP_1(X)=23, STEP_2(Y)=19, STEP_3(Z)=27 -- ditukar dari sebelumnya
  constexpr uint8_t STEP_1 = 23, STEP_2 = 19, STEP_3 = 27;   // STEP_1=X, STEP_2=Y, STEP_3=Z (dikonfirmasi user)
  // DIHAPUS: DIR_1_NTV/DIR_2_NTV/DIR_3_NTV -- tidak ada pin native lain tersedia di ESP32,
  // DIR kembali sepenuhnya ke MCP23017 (lihat CH::DIR_1_MCP/DIR_2_MCP/DIR_3_MCP)
  constexpr uint8_t PWMA_TB6612 = 26, PWMB_TB6612 = 25;   // DITUKAR dari sebelumnya

  // --- Alias khusus STOCKER ---
  constexpr uint8_t STEP_X = STEP_1, STEP_Y = STEP_2, STEP_Z = STEP_3;
}

// BARU: invert arah per-axis -- kalau motor berputar KEBALIK dari yang diperintahkan (misal
// saat AutoHome menuju arah yang salah), ubah flag axis yang bersangkutan jadi true di sini.
// TIDAK PERLU bongkar kabel motor/DIR fisik -- cukup ubah 1 baris ini + upload ulang.
// Berlaku OTOMATIS ke SEMUA gerakan (homing, jog, RUN_FULL_CYCLE) karena diterapkan di
// SATU titik (setAxisDirection()), bukan perlu diubah di banyak tempat.
namespace AxisInvert {
  constexpr bool X = true;   // ubah ke true kalau axis X (STEP_1) berputar kebalik
  constexpr bool Y = true;   // ubah ke true kalau axis Y (STEP_2) berputar kebalik
  constexpr bool Z = true;   // ubah ke true kalau axis Z (STEP_3) berputar kebalik
}

namespace LcdCfg {
  constexpr uint8_t COLS = 20;
  constexpr uint8_t ROWS = 4;
}

// --- RS485 ---
namespace Rs485Cfg {
  constexpr uint8_t RX_PIN = 17, TX_PIN = 16;   // GPIO16/17 (UART2 hardware) -- DIKOREKSI dari salah tulis "PHYS 21" di Excel awal
  constexpr uint32_t BAUD = 19200;
  constexpr uint8_t SLAVE_ID = 4;   // STOCKER = slave 4
}