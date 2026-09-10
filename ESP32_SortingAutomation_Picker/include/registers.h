#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — PICKER (docs/07-PROTOCOL.md §7.1, §7.4)
// ============================================================
namespace Reg {
  constexpr uint16_t STATE       = 0;
  constexpr uint16_t FAULT_CODE  = 1;
  constexpr uint16_t CMD         = 2;
  constexpr uint16_t CMD_ARG     = 3;
  constexpr uint16_t CMD_SEQ     = 4;
  constexpr uint16_t CMD_ACK_SEQ = 5;
  constexpr uint16_t HEARTBEAT   = 6;

  constexpr uint16_t CURRENT_POSE = 10;   // R -- pose terakhir tercapai (0=home,1=pass,2=reject,3=lift)

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- sinkron pola SORTER
  constexpr uint16_t ACTIVITY_CODE   = 11;  // R
  constexpr uint16_t I2C_ERROR_COUNT = 12;  // R
  constexpr uint16_t LAST_FAULT_CODE = 13;  // R
  constexpr uint16_t UPTIME_SEC      = 14;  // R
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik PICKER ---
enum class ActivityCode : uint16_t {
  DIAM = 0, MENUJU_HOME = 1, MENUJU_PASS = 2, MENUJU_REJECT = 3, MENUJU_LIFT = 4,
  MENGAMBIL = 5, MELETAKKAN = 6, NAIK_CLEARANCE = 7, BERGERAK = 8, POST_PLACE_GERAK = 9,
  FAULT_AKTIF = 90, ESTOP_AKTIF = 91
};

enum class NodeState : uint16_t {
  INIT = 0, IDLE = 1, RUNNING_OR_MOVING = 2, FAULT = 3, ESTOPPED = 4
};

enum class FaultCode : uint16_t {
  NONE = 0, COMM_TIMEOUT = 1, ESTOP_ACTIVE = 5, IO_EXPANDER_MISSING = 20
};

// --- Opcode CMD (§7.4) ---
enum class Cmd : uint16_t {
  NONE = 0,
  RUN_SEQUENCE = 1,   // arg: 1=pass, 2=reject -- urutan penuh home->target->pick->lift->place->home
  GOTO_HOME = 2, GOTO_PASS = 3, GOTO_REJECT = 4,   // manual override
  PICK = 5, PLACE = 6,                              // manual override
  RESET_FAULT = 7,
  MOVE_PACKAGE = 8   // BARU -- urutan penuh home->PACKAGE_PICKUP(pose4)->pick->LIFT_LOAD(pose5)->place->home,
                      // dipicu Orange Pi saat SORTER.PASS_COUNT capai batch (mis. 20), TANPA arg
};