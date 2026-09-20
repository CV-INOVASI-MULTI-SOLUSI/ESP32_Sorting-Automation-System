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

  constexpr uint16_t CURRENT_POSE = 10;   // R -- pose terakhir tercapai (0=home,1=pass,3=lift,4=pickup,5=lift_load)

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- sinkron pola SORTER
  constexpr uint16_t ACTIVITY_CODE   = 11;  // R
  constexpr uint16_t I2C_ERROR_COUNT = 12;  // R
  constexpr uint16_t LAST_FAULT_CODE = 13;  // R
  constexpr uint16_t UPTIME_SEC      = 14;  // R
  // BARU: status live mainModeActive -- Orange Pi bisa cek node lagi MAIN (produksi) atau
  // TEST (manual, aman dipakai GOTO_HOME/GOTO_PASS/PICK/PLACE).
  constexpr uint16_t MAIN_MODE_ACTIVE = 15;  // R, 1 = MAIN aktif, 0 = TEST mode
  // BARU (2026-09-20): menu kalibrasi LCD meng-IGNORE semua command Modbus (§12.6) TANPA
  // kirim CMD_ACK_SEQ -- dari sisi master itu kelihatan identik dgn "node mati/kabel putus".
  // Register ini bikin master bisa bedain dua kondisi itu.
  constexpr uint16_t MENU_ACTIVE = 16;  // R, 1 = operator lagi di menu kalibrasi LCD
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik PICKER ---
enum class ActivityCode : uint16_t {
  // DIHAPUS: MENUJU_REJECT (value 3) -- Picker gak pernah reject, pose 2 gak pernah dituju lagi.
  DIAM = 0, MENUJU_HOME = 1, MENUJU_PASS = 2, MENUJU_LIFT = 4,
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
  RUN_SEQUENCE = 1,   // urutan penuh home->PASS->pick->lift->place->home (arg diabaikan, cuma pass)
  GOTO_HOME = 2, GOTO_PASS = 3,   // manual override
  // DIHAPUS: GOTO_REJECT (opcode 4) -- Picker fisik cuma ambil dari PASS, gak pernah reject
  // (reject ditangani hopper SORTER sebelum objek sampai ke Picker). Opcode 4 SENGAJA
  // TIDAK dipakai ulang -- PICK/PLACE dkk tetap nomor eksplisit di bawah biar gak geser.
  PICK = 5, PLACE = 6,                              // manual override
  RESET_FAULT = 7,
  MOVE_PACKAGE = 8,  // BARU -- urutan penuh home->PACKAGE_PICKUP(pose4)->pick->LIFT_LOAD(pose5)->place->home,
                      // dipicu Orange Pi saat SORTER.PASS_COUNT capai batch (mis. 20), TANPA arg
  // BARU -- biar kecepatan trajectory (berlaku SAMA ke semua 6 joint) bisa di-tuning dari
  // Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_TRAJ_STEP = 9,           // arg: trajStepUs, 1-500
  SET_TRAJ_STEP_INTERVAL = 10, // arg: trajStepIntervalMs, 5-200
  // BARU -- pemisah MAIN/TEST eksplisit (sama konsep dgn SORTER/DISPENSER/STOCKER). Default
  // boot = mainModeActive FALSE. Selama MAIN aktif, command "manual override" (GOTO_HOME/
  // GOTO_PASS/PICK/PLACE) DITOLAK TOTAL -- cuma RUN_SEQUENCE/MOVE_PACKAGE (produksi
  // asli) yang jalan. Sebaliknya, RUN_SEQUENCE/MOVE_PACKAGE ditolak selama masih TEST mode.
  START_MAIN = 11,
  STOP_MAIN = 12
};