#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — FEEDER (docs/07-PROTOCOL.md §7.1, §7.5)
// ============================================================
namespace Reg {
  constexpr uint16_t STATE       = 0;
  constexpr uint16_t FAULT_CODE  = 1;
  constexpr uint16_t CMD         = 2;
  constexpr uint16_t CMD_ARG     = 3;
  constexpr uint16_t CMD_SEQ     = 4;
  constexpr uint16_t CMD_ACK_SEQ = 5;
  constexpr uint16_t HEARTBEAT   = 6;

  constexpr uint16_t STOCK_EMPTY_FLAG = 10;  // R

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- sinkron pola SORTER/PICKER/STOCKER
  constexpr uint16_t ACTIVITY_CODE   = 11;  // R
  constexpr uint16_t I2C_ERROR_COUNT = 12;  // R
  constexpr uint16_t LAST_FAULT_CODE = 13;  // R
  constexpr uint16_t UPTIME_SEC      = 14;  // R
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik FEEDER (refillState yg sudah ada) ---
enum class ActivityCode : uint16_t {
  DIAM = 0, CONVEYOR_JALAN = 1,
  // DIUBAH: MENDORONG_BOX/MENARIK_KEMBALI diganti 6 tahap servo1/servo2 (redesign mekanisme
  // dispenser dari DC-motor-push jadi 2-servo gerbang stack)
  SERVO1_BUKA = 2, SERVO1_TAHAN = 3, SERVO1_TUTUP = 4,
  SERVO2_BUKA = 5, SERVO2_TAHAN = 6, SERVO2_TUTUP = 7,
  SELESAI = 8,
  FAULT_AKTIF = 90, ESTOP_AKTIF = 91
};

enum class NodeState : uint16_t {
  INIT = 0, IDLE = 1, RUNNING_OR_MOVING = 2, FAULT = 3, ESTOPPED = 4
};

enum class FaultCode : uint16_t {
  NONE = 0, COMM_TIMEOUT = 1, ESTOP_ACTIVE = 5,
  STOCK_EMPTY = 10, BOX_NOT_ARRIVED = 11, PUSH_STUCK = 12,
  IO_EXPANDER_MISSING = 20
};

// --- Opcode CMD (§7.5) ---
enum class Cmd : uint16_t {
  NONE = 0,
  REQUEST_REFILL = 1,       // mulai FSM conveyor2+push
  RESET_FAULT = 2,
  SET_CONVEYOR_SPEED = 3,
  TEST_SERVO1_CYCLE = 97,   // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo1
  TEST_SERVO2_CYCLE = 98    // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo2
};