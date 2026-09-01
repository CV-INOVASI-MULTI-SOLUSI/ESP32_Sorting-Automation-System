#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — SORTER (docs/07-PROTOCOL.md §7.1, §7.3)
// Base register (0-9) SAMA di semua node kalau nanti PICKER/FEEDER/STOCKER dibuat menyusul.
// Address 10+ spesifik SORTER.
// ============================================================
namespace Reg {
  // --- Base, universal ---
  constexpr uint16_t STATE       = 0;   // R
  constexpr uint16_t FAULT_CODE  = 1;   // R
  constexpr uint16_t CMD         = 2;   // W
  constexpr uint16_t CMD_ARG     = 3;   // W
  constexpr uint16_t CMD_SEQ     = 4;   // W
  constexpr uint16_t CMD_ACK_SEQ = 5;   // R
  constexpr uint16_t HEARTBEAT   = 6;   // W

  // --- SORTER spesifik ---
  constexpr uint16_t PASS_COUNT         = 10;  // R
  constexpr uint16_t REJECT_COUNT       = 11;  // R
  constexpr uint16_t CLASSIFY_IS_REJECT = 12;  // W, frekuensi tinggi (per objek)

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- supaya OrangePi bisa identifikasi status
  // ALAT secara rinci lewat Modbus, bukan cuma lewat LCD lokal (activityText())
  constexpr uint16_t ACTIVITY_CODE    = 13;  // R -- lihat enum ActivityCode di bawah
  constexpr uint16_t I2C_ERROR_COUNT  = 14;  // R -- naik tiap kali recheckHealth() gagal, indikasi EMI/gangguan fisik
  constexpr uint16_t LAST_FAULT_CODE  = 15;  // R -- fault TERAKHIR yang pernah terjadi (breadcrumb, walau sudah di-reset)
  constexpr uint16_t UPTIME_SEC       = 16;  // R -- detik sejak boot -- korelasikan dgn kapan fault/error terjadi
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik (bukan cuma IDLE/RUNNING generik) ---
enum class ActivityCode : uint16_t {
  DIAM = 0, CONVEYOR_JALAN = 1, CONVEYOR_JALAN_PALANG_AKTIF = 2, MOTOR_A_JALAN = 3,
  FAULT_AKTIF = 90, ESTOP_AKTIF = 91
};

// --- STATE enum ---
enum class NodeState : uint16_t {
  INIT = 0, IDLE = 1, RUNNING_OR_MOVING = 2, FAULT = 3, ESTOPPED = 4
};

// --- Fault code (docs/07-PROTOCOL.md §7.2-7.3) ---
enum class FaultCode : uint16_t {
  NONE = 0, COMM_TIMEOUT = 1, ESTOP_ACTIVE = 5, HOPPER_JAM = 10, IO_EXPANDER_MISSING = 20
};

// --- Opcode CMD (§7.3) ---
enum class Cmd : uint16_t {
  NONE = 0, START = 1, STOP = 2, RESET_FAULT = 3, SET_HOPPER_INTERVAL = 4,
  SET_CONVEYOR_SPEED = 5, SET_CONVEYOR_DIR = 6, RESET_COUNTERS = 7,
  SET_MOTOR_A = 8,   // BARU -- arg: 0=stop,1=maju,2=mundur (channel A chip TB6612FNG yg sama dgn conveyor)
  TEST_TRIGGER_PALANG = 98,   // O12 -- simulasi 1 hasil reject tanpa HuskyLens, utk kalibrasi TOF
  TEST_FAULT = 99             // test-only, HAPUS sebelum produksi riil
};