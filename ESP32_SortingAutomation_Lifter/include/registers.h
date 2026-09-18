#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — STOCKER (docs/07-PROTOCOL.md §7.1, §7.6)
// ============================================================
namespace Reg {
  constexpr uint16_t STATE       = 0;
  constexpr uint16_t FAULT_CODE  = 1;
  constexpr uint16_t CMD         = 2;
  constexpr uint16_t CMD_ARG     = 3;
  constexpr uint16_t CMD_SEQ     = 4;
  constexpr uint16_t CMD_ACK_SEQ = 5;
  constexpr uint16_t HEARTBEAT   = 6;

  constexpr uint16_t CURRENT_RACK_IDX      = 10;  // R, 0xFF = none/in-transit
  constexpr uint16_t ALL_HOMED_FLAG        = 11;  // R
  constexpr uint16_t RACK_OCCUPIED_BITMASK = 12;  // R, D21 -- bit0-5 = status fisik slot rack 0-5

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- sinkron pola SORTER
  constexpr uint16_t ACTIVITY_CODE   = 13;  // R
  constexpr uint16_t I2C_ERROR_COUNT = 14;  // R
  constexpr uint16_t LAST_FAULT_CODE = 15;  // R
  constexpr uint16_t UPTIME_SEC      = 16;  // R
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik STOCKER (cycleStage yg sudah ada) ---
enum class ActivityCode : uint16_t {
  DIAM = 0, HOMING = 1, MENUJU_RAK = 2, MENDORONG_BOX = 3, MENARIK_PUSHER = 4,
  KEMBALI_KE_HOME = 5, BERGERAK_MANUAL = 6, TEST_DORONG = 7, TEST_TARIK = 8,
  FAULT_AKTIF = 90, ESTOP_AKTIF = 91
};

enum class NodeState : uint16_t {
  INIT = 0, IDLE = 1, RUNNING_OR_MOVING = 2, FAULT = 3, ESTOPPED = 4
};

enum class FaultCode : uint16_t {
  NONE = 0, COMM_TIMEOUT = 1, ESTOP_ACTIVE = 5,
  HOMING_FAILED = 10, RACK_IDX_INVALID = 11, NOT_HOMED = 12, PUSH_STUCK = 13,
  IO_EXPANDER_MISSING = 20
};

// --- Opcode CMD (§7.6) ---
enum class Cmd : uint16_t {
  NONE = 0,
  HOME_ALL = 1,
  RUN_FULL_CYCLE = 2,     // arg: rack_idx 0-5 -- move->push->home, chaining penuh (O8)
  MOVE_TO_RACK = 3,       // arg: rack_idx 0-5 -- manual, tanpa auto-push
  PUSH_BOX = 4,           // manual
  RESET_FAULT = 5,
  GOTO_LOAD_POSITION = 6,  // BARU -- manual, menuju Load Position (titik standby terima package)
  // BARU -- biar kecepatan bisa di-tuning dari Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_STEP_INTERVAL = 7,         // arg: stepIntervalUs (kecepatan jelajah normal), 20-5000
  SET_HOMING_STEP_INTERVAL = 8   // arg: homingStepIntervalUs (kecepatan khusus homing), 20-5000
};