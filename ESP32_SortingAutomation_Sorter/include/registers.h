#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — SORTER (docs/07-PROTOCOL.md §7.1, §7.3)
// Base register (0-9) SAMA di semua node kalau nanti PICKER/FEEDER/STOCKER dibuat menyusul.
// Address 10+ spesifik SORTER.
// ============================================================
// NOMOR RILIS FIRMWARE -- NAIKKAN setiap kali firmware baru dikirim ke mesin (200 = v2.00,
// 201 = v2.01, 210 = v2.10). Terbaca Orange Pi di MORE > SYSTEM > NODE, bersama waktu build.
constexpr uint16_t FIRMWARE_VERSI = 201;   // 201: backup/restore kalibrasi (2026-10-09)

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
  // BARU (2026-10-05): 1 = hopper dijeda lewat Cmd::SET_HOPPER_JEDA (conveyor tetap jalan).
  constexpr uint16_t HOPPER_DIJEDA          = 24;  // R
  // BARU (2026-10-07): waktu BUILD firmware, diisi otomatis saat compile (__DATE__/__TIME__).
  // Orange Pi membacanya untuk melacak node mana yang sudah di-flash firmware terbaru.
  // Firmware lama tidak punya register ini (master menerima ILLEGAL DATA ADDRESS).
  constexpr uint16_t FW_TAHUN      = 25;  // R, mis. 2026
  constexpr uint16_t FW_BULAN_HARI = 26;  // R, bulan*100 + hari, mis. 1007
  constexpr uint16_t FW_JAM_MENIT  = 27;  // R, jam*100 + menit, mis. 1405
  constexpr uint16_t FW_VERSI      = 28;  // R, nomor rilis FIRMWARE_VERSI, mis. 200 = v2.00
  // BARU (2026-10-07): alamat IP & kekuatan sinyal WiFi, diperbarui tiap 5 s. Orange Pi memakai
  // IP ini untuk update firmware OTA tanpa mengetik IP (IP dari DHCP boleh berubah).
  constexpr uint16_t WIFI_IP_HI    = 29;  // R, oktet 1 << 8 | oktet 2 (0 = WiFi tidak tersambung)
  constexpr uint16_t WIFI_IP_LO    = 30;  // R, oktet 3 << 8 | oktet 4
  constexpr uint16_t WIFI_RSSI     = 31;  // R, dBm sebagai int16 (mis. -61), 0 = tidak tersambung
  // BARU (2026-10-09): backup / restore kalibrasi lewat Modbus -- protokol lengkap di
  // include/kalibrasi_modbus.h (sama di keempat node). Firmware < v2.01 tidak punya register ini.
  constexpr uint16_t CAL_FORMAT = 32;  // R, nomor format gambar kalibrasi
  constexpr uint16_t CAL_UKURAN = 33;  // R, panjang gambar (byte)
  constexpr uint16_t CAL_CRC    = 34;  // R, CRC16-CCITT gambar (diisi CAL_BACA offset 0)
  constexpr uint16_t CAL_OFFSET = 35;  // R, offset jendela terakhir
  constexpr uint16_t CAL_HASIL  = 36;  // R, kode hasil perintah CAL terakhir (CalHasil)
  constexpr uint16_t CAL_DATA0  = 37;  // RW, 32 register = jendela 64 byte (sampai 68)
}

// --- BARU: ActivityCode -- Lapis 2, aktivitas spesifik (bukan cuma IDLE/RUNNING generik) ---
enum class ActivityCode : uint16_t {
  // CONVEYOR_JALAN_PALANG_AKTIF (2) = palang sedang PUSH/RETRACT, apa pun keadaan conveyor.
  // DIPERBAIKI 2026-09-28: dulu kode ini tidak pernah bisa muncul karena siklus palang ikut
  // menyetel motorAState=1 dan MOTOR_A_JALAN (3) dicek lebih dulu. Sekarang 3 berarti khusus
  // jog manual Motor A (SET_MOTOR_A / Serial MOTORA) TANPA siklus palang.
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
  // BARU (2026-10-09): backup / restore kalibrasi (include/kalibrasi_modbus.h), ack langsung.
  // Opcode SAMA di keempat node. Ditolak selama MAIN aktif / node bergerak.
  CAL_BACA = 60, CAL_TULIS = 61, CAL_TERAPKAN = 62,
  NONE = 0, START = 1, STOP = 2, RESET_FAULT = 3, SET_HOPPER_INTERVAL = 4,
  SET_CONVEYOR_SPEED = 5, SET_CONVEYOR_DIR = 6, RESET_COUNTERS = 7,
  SET_MOTOR_A = 8,   // BARU -- arg: 0=stop,1=maju,2=mundur (channel A chip TB6612FNG yg sama dgn conveyor)
  // BARU -- biar kecepatan bisa di-tuning dari Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_PALANG_SPEED = 9,   // arg: palangSpeed PWM 0-255
  SET_HOPPER_STEP  = 10,  // arg: hopperStepUs (pasangan SET_HOPPER_INTERVAL yg sudah ada), 1-2500
  // BARU (2026-10-05): arg 1 = jeda umpan hopper, 0 = lanjut. Conveyor & palang TETAP jalan,
  // supaya objek yang sudah di belt tetap terklasifikasi dan didorong tepat waktu. Dipakai
  // orchestrator saat package diganti. START/STOP selalu membatalkan jeda.
  SET_HOPPER_JEDA  = 11,
  TEST_HOPPER_CYCLE = 97,      // test-only -- 1x siklus maju-mundur hopper pakai nilai KALIBRASI
  TEST_TRIGGER_PALANG = 98    // O12 -- simulasi 1 hasil reject tanpa HuskyLens, utk kalibrasi TOF
  // DIHAPUS (2026-09-20): TEST_FAULT = 99 -- komentarnya sendiri sudah menyuruh hapus sebelum
  // produksi. Opcode 99 SENGAJA dibiarkan kosong, jangan dipakai ulang untuk hal lain supaya
  // script/master lama yang masih mengirimnya tidak memicu perintah yang berbeda arti.
};