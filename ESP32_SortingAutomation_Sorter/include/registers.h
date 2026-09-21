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
  // BARU: status live mainModeActive -- Orange Pi bisa cek node lagi MAIN (produksi, START
  // sudah dikirim) atau TEST (manual, aman dipakai TEST_HOPPER_CYCLE/SET_MOTOR_A dkk).
  constexpr uint16_t MAIN_MODE_ACTIVE = 17;  // R, 1 = MAIN aktif, 0 = TEST mode
  // BARU (2026-09-20): menu kalibrasi LCD meng-IGNORE semua command Modbus (§12.6) TANPA
  // kirim CMD_ACK_SEQ -- dari sisi master itu kelihatan identik dgn "node mati/kabel putus".
  // Register ini bikin master bisa bedain dua kondisi itu.
  constexpr uint16_t MENU_ACTIVE = 18;  // R, 1 = operator lagi di menu kalibrasi LCD
  // BARU (2026-09-20): berapa hasil klasifikasi REJECT yang TIDAK sempat jadi dorongan palang.
  // Dua sebabnya: (a) antrian penuh saat klasifikasi baru datang, (b) giliran dorongnya sudah
  // lewat (basi) pas antrian akhirnya sempat diproses. Sebelumnya dua-duanya hilang diam-diam
  // tanpa jejak apa pun -- objek reject lolos ke jalur pass dan tidak ada yang tahu.
  constexpr uint16_t REJECT_MISSED_COUNT = 19;  // R
  // BARU (2026-09-21): uji kecepatan objek di conveyor. PROX_1 = garis start,
  // PROX_2 = garis finish, jaraknya diatur lewat menu kalibrasi "Jarak Uji Kec.(mm)".
  // Murni pengukuran pasif -- tidak menggerakkan apa pun dan tidak mengganggu jalur
  // produksi PROX_2 (passCount) yang tetap berjalan seperti biasa.
  constexpr uint16_t SPEED_LAST_MM_S        = 20;  // R, kecepatan hasil ukur terakhir (mm/detik)
  constexpr uint16_t SPEED_LAST_MS          = 21;  // R, waktu tempuh terakhir (ms)
  constexpr uint16_t SPEED_SAMPLE_COUNT     = 22;  // R, berapa kali pengukuran berhasil
  // Kecepatan yang disetarakan ke PWM penuh (255). Ini SARAN nilai untuk parameter
  // kalibrasi "Mm/s Max", yang dipakai menghitung waktu tempuh objek scan->palang.
  constexpr uint16_t SPEED_MM_S_AT_MAX_PWM  = 23;  // R
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik (bukan cuma IDLE/RUNNING generik) ---
enum class ActivityCode : uint16_t {
  DIAM = 0, CONVEYOR_JALAN = 1, CONVEYOR_JALAN_PALANG_AKTIF = 2, MOTOR_A_JALAN = 3,
  // BARU: TEST_HOPPER_CYCLE (opcode 97, ATAU tombol fisik BTN_TEST_HOPPER) TIDAK pernah
  // ubah currentState (sengaja, biar guard-nya cuma butuh currentState==IDLE) -- tanpa
  // kode ini, ACTIVITY_CODE tetap DIAM(0) walau hopper BENERAN lagi gerak (bug sama persis
  // yang ketemu di Dispenser TEST_SERVO1/2_CYCLE, lihat catatan analisa 2026-09-19).
  TEST_HOPPER_AKTIF = 4,
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
  // BARU -- biar kecepatan bisa di-tuning dari Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_PALANG_SPEED = 9,   // arg: palangSpeed PWM 0-255
  SET_HOPPER_STEP  = 10,  // arg: hopperStepUs (pasangan SET_HOPPER_INTERVAL yg sudah ada), 1-2500
  TEST_HOPPER_CYCLE = 97,      // test-only -- 1x siklus maju-mundur hopper pakai nilai KALIBRASI
  TEST_TRIGGER_PALANG = 98    // O12 -- simulasi 1 hasil reject tanpa HuskyLens, utk kalibrasi TOF
  // DIHAPUS (2026-09-20): TEST_FAULT = 99 -- komentarnya sendiri sudah menyuruh hapus sebelum
  // produksi. Opcode 99 SENGAJA dibiarkan kosong, jangan dipakai ulang untuk hal lain supaya
  // script/master lama yang masih mengirimnya tidak memicu perintah yang berbeda arti.
};