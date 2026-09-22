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

  // DIPERBARUI (2026-09-22): slot pose dinomori ulang rapat jadi 0, 1, 2 -- lubang bekas
  // jalur objek satuan (pass/reject/lift) dibuang seluruhnya.
  constexpr uint16_t CURRENT_POSE = 10;   // R -- pose terakhir tercapai (0=home, 1=package_pickup, 2=lift_load)

  // BARU: Lapis 2 (SubState) + Lapis 3 (diagnostik) -- sinkron pola SORTER
  constexpr uint16_t ACTIVITY_CODE   = 11;  // R
  constexpr uint16_t I2C_ERROR_COUNT = 12;  // R
  constexpr uint16_t LAST_FAULT_CODE = 13;  // R
  constexpr uint16_t UPTIME_SEC      = 14;  // R
  // BARU: status live mainModeActive -- Orange Pi bisa cek node lagi MAIN (produksi) atau
  // TEST (manual, aman dipakai GOTO_HOME/PICK/PLACE).
  constexpr uint16_t MAIN_MODE_ACTIVE = 15;  // R, 1 = MAIN aktif, 0 = TEST mode
  // BARU (2026-09-20): menu kalibrasi LCD meng-IGNORE semua command Modbus (§12.6) TANPA
  // kirim CMD_ACK_SEQ -- dari sisi master itu kelihatan identik dgn "node mati/kabel putus".
  // Register ini bikin master bisa bedain dua kondisi itu.
  constexpr uint16_t MENU_ACTIVE = 16;  // R, 1 = operator lagi di menu kalibrasi LCD
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik PICKER ---
enum class ActivityCode : uint16_t {
  // DIHAPUS (2026-09-22): MENUJU_PASS (2), MENUJU_REJECT (3) dan MENUJU_LIFT (4) -- ketiganya
  // milik jalur objek satuan yang sudah dibuang seluruhnya.
  //
  // Dua tujuan produksi yang tersisa (pose 1 = PACKAGE_PICKUP, pose 2 = LIFT_LOAD) kini punya
  // kode sendiri. Sebelumnya keduanya jatuh ke BERGERAK yang generik, sehingga operator tidak
  // bisa membedakan "menuju package" dari "menuju lift" lewat register.
  //
  // Nomor 10 dan 11 dipakai, BUKAN mendaur ulang 2/3/4 yang baru dikosongkan -- master atau
  // catatan lama yang masih memegang arti lama tidak boleh salah membaca.
  DIAM = 0, MENUJU_HOME = 1, MENUJU_PACKAGE_PICKUP = 10, MENUJU_LIFT_LOAD = 11,
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
  // DIHAPUS (2026-09-22): RUN_SEQUENCE (opcode 1) dan GOTO_PASS (opcode 3).
  // Arm robot HANYA memindahkan package penuh dari ujung Dispenser -- tidak pernah
  // menangani objek satuan. Seluruh jalur "ambil satu objek dari jalur PASS" karena itu
  // tidak punya pemakai: orchestrator produksi pun hanya mengirim MOVE_PACKAGE.
  // Opcode 1 dan 3 SENGAJA dibiarkan kosong, jangan dipakai ulang untuk hal lain, supaya
  // master/script lama yang masih mengirimnya tidak memicu perintah yang berbeda arti.
  GOTO_HOME = 2,   // manual override
  // DIHAPUS: GOTO_REJECT (opcode 4) -- Picker fisik cuma ambil dari PASS, gak pernah reject
  // (reject ditangani hopper SORTER sebelum objek sampai ke Picker). Opcode 4 SENGAJA
  // TIDAK dipakai ulang -- PICK/PLACE dkk tetap nomor eksplisit di bawah biar gak geser.
  PICK = 5, PLACE = 6,                              // manual override
  RESET_FAULT = 7,
  MOVE_PACKAGE = 8,  // urutan penuh home->PACKAGE_PICKUP(pose1)->pick->LIFT_LOAD(pose2)->place->home,
                      // dipicu Orange Pi saat SORTER.PASS_COUNT capai batch (mis. 20), TANPA arg
  // BARU -- biar kecepatan trajectory (berlaku SAMA ke semua 6 joint) bisa di-tuning dari
  // Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_TRAJ_STEP = 9,           // arg: trajStepUs, 1-500
  SET_TRAJ_STEP_INTERVAL = 10, // arg: trajStepIntervalMs, 5-200
  // BARU -- pemisah MAIN/TEST eksplisit (sama konsep dgn SORTER/DISPENSER/STOCKER). Default
  // boot = mainModeActive FALSE. Selama MAIN aktif, command "manual override" (GOTO_HOME/
  // PICK/PLACE) DITOLAK TOTAL -- cuma MOVE_PACKAGE (produksi
  // asli) yang jalan. Sebaliknya, MOVE_PACKAGE ditolak selama masih TEST mode.
  START_MAIN = 11,
  STOP_MAIN = 12
};