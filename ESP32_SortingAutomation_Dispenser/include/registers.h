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

  // BARU: mekanisme 2-proximity (UJUNG+TENGAH) -- package FULL sampai di UJUNG, siap diambil
  // arm robot. Flag ini WAJIB di-clear eksplisit oleh Orange Pi (Cmd::ACK_PACKAGE_TAKEN) SETELAH
  // arm benar-benar ambil -- TIDAK auto-clear dari sensor, supaya Orange Pi yang pegang kendali
  // kapan siklus berikutnya (REQUEST_REFILL) boleh mulai.
  constexpr uint16_t PACKAGE_READY_FLAG = 15;  // R, 1 = package di UJUNG siap diambil arm
  // BARU: status live PACKAGE_MIDDLE_SENSOR (PROX_2) -- Orange Pi butuh ini buat cek startup
  // ("apa udah ada package siap diisi di tengah?") tanpa nunggu edge trigger produksi.
  constexpr uint16_t MIDDLE_PACKAGE_PRESENT = 16;  // R, 1 = ada package terdeteksi di tengah
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik FEEDER (refillState yg sudah ada) ---
enum class ActivityCode : uint16_t {
  DIAM = 0, CONVEYOR_JALAN = 1,
  // DIUBAH: MENDORONG_BOX/MENARIK_KEMBALI diganti 6 tahap servo1/servo2 (redesign mekanisme
  // dispenser dari DC-motor-push jadi 2-servo gerbang stack)
  SERVO1_BUKA = 2, SERVO1_TAHAN = 3, SERVO1_TUTUP = 4,
  SERVO2_BUKA = 5, SERVO2_TAHAN = 6, SERVO2_TUTUP = 7,
  SELESAI = 8,
  TUNGGU_KONFIRM_TENGAH = 9,   // BARU -- servo selesai jatuhin, nunggu PACKAGE_MIDDLE_SENSOR konfirmasi
  FAULT_AKTIF = 90, ESTOP_AKTIF = 91
};

enum class NodeState : uint16_t {
  INIT = 0, IDLE = 1, RUNNING_OR_MOVING = 2, FAULT = 3, ESTOPPED = 4
};

enum class FaultCode : uint16_t {
  NONE = 0, COMM_TIMEOUT = 1, ESTOP_ACTIVE = 5,
  STOCK_EMPTY = 10, BOX_NOT_ARRIVED = 11, PUSH_STUCK = 12,
  MIDDLE_PACKAGE_MISSING = 13,   // BARU -- servo sudah jatuhin package baru, tapi PACKAGE_MIDDLE_SENSOR
                                  // gak pernah konfirmasi LOW dalam batas waktu (macet/kehabisan stok)
  IO_EXPANDER_MISSING = 20
};

// --- Opcode CMD (§7.5) ---
enum class Cmd : uint16_t {
  NONE = 0,
  REQUEST_REFILL = 1,       // DIUBAH MAKNA: sekarang "maju karena package di TENGAH sudah FULL" --
                              // conveyor jalan sampai package itu sampai UJUNG (PACKAGE_READY_FLAG=1)
  RESET_FAULT = 2,
  SET_CONVEYOR_SPEED = 3,
  SET_CONVEYOR_DIR = 4,     // BARU -- arg: 0=reverse, 1=forward
  ACK_PACKAGE_TAKEN = 5,    // BARU -- Orange Pi konfirmasi arm SUDAH ambil package di UJUNG,
                              // clear PACKAGE_READY_FLAG kembali ke 0
  // BARU -- utk startup/recovery: paksa isi TENGAH sekarang (conveyor MAJU + servo jatuhin
  // package baru, PARALEL) sampai PACKAGE_MIDDLE_SENSOR=LOW -- BUKAN nunggu edge produksi biasa.
  // Dipakai kalau boot pertama kali tengah masih kosong dari awal (gak ada edge yg bisa dideteksi).
  FORCE_MIDDLE_REFILL = 6,
  // BARU -- biar kecepatan servo1/servo2 bisa di-tuning dari Orange Pi langsung.
  SET_SERVO1_STEP = 7,           // arg: servo1StepUs, 1-2500
  SET_SERVO1_STEP_INTERVAL = 8,  // arg: servo1StepIntervalMs, 0-500
  SET_SERVO2_STEP = 9,           // arg: servo2StepUs, 1-2500
  SET_SERVO2_STEP_INTERVAL = 10, // arg: servo2StepIntervalMs, 0-500
  TEST_SERVO1_CYCLE = 97,   // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo1
  TEST_SERVO2_CYCLE = 98    // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo2
};