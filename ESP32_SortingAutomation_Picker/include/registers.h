#pragma once
#include <Arduino.h>

// ============================================================
// REGISTER MAP MODBUS — PICKER (docs/07-PROTOCOL.md §7.1, §7.4)
// ============================================================
// NOMOR RILIS FIRMWARE -- NAIKKAN setiap kali firmware baru dikirim ke mesin (200 = v2.00,
// 201 = v2.01, 210 = v2.10). Terbaca Orange Pi di MORE > SYSTEM > NODE, bersama waktu build.
constexpr uint16_t FIRMWARE_VERSI = 201;   // 201: backup/restore kalibrasi (2026-10-09)

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
  constexpr uint16_t CURRENT_POSE = 10;   // R -- pose terakhir tercapai (0=home, 1=package_pickup, 2=lift_load, 3=ready)

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
  // BARU (2026-09-30): gerakan yang sedang dijalankan (sejak 2026-10-01: 0=Home>Ready
  // 1=Ready>Pick 2=Pick>Home 3=Home>Place 4=Place>Home, 255 = tidak ada) dan slot adegan yang sedang dikerjakan (1-20, 0 = belum
  // mulai). Kalau lengan berhenti di tengah gerakan karena FAULT/E-stop, dua angka ini
  // menunjukkan tepat di adegan mana -- tanpa itu posisinya harus ditebak dari nilai servo.
  constexpr uint16_t GERAKAN_AKTIF = 17;  // R
  constexpr uint16_t ADEGAN_KE     = 18;  // R
  // BARU (2026-10-07): waktu BUILD firmware, diisi otomatis saat compile (__DATE__/__TIME__).
  // Orange Pi membacanya untuk melacak node mana yang sudah di-flash firmware terbaru.
  // Firmware lama tidak punya register ini (master menerima ILLEGAL DATA ADDRESS).
  constexpr uint16_t FW_TAHUN      = 19;  // R, mis. 2026
  constexpr uint16_t FW_BULAN_HARI = 20;  // R, bulan*100 + hari, mis. 1007
  constexpr uint16_t FW_JAM_MENIT  = 21;  // R, jam*100 + menit, mis. 1405
  constexpr uint16_t FW_VERSI      = 22;  // R, nomor rilis FIRMWARE_VERSI, mis. 200 = v2.00
  // BARU (2026-10-07): alamat IP & kekuatan sinyal WiFi, diperbarui tiap 5 s. Orange Pi memakai
  // IP ini untuk update firmware OTA tanpa mengetik IP (IP dari DHCP boleh berubah).
  constexpr uint16_t WIFI_IP_HI    = 23;  // R, oktet 1 << 8 | oktet 2 (0 = WiFi tidak tersambung)
  constexpr uint16_t WIFI_IP_LO    = 24;  // R, oktet 3 << 8 | oktet 4
  constexpr uint16_t WIFI_RSSI     = 25;  // R, dBm sebagai int16 (mis. -61), 0 = tidak tersambung
  // BARU (2026-10-09): backup / restore kalibrasi lewat Modbus -- protokol lengkap di
  // include/kalibrasi_modbus.h (sama di keempat node). Firmware < v2.01 tidak punya register ini.
  constexpr uint16_t CAL_FORMAT = 26;  // R, nomor format gambar kalibrasi
  constexpr uint16_t CAL_UKURAN = 27;  // R, panjang gambar (byte)
  constexpr uint16_t CAL_CRC    = 28;  // R, CRC16-CCITT gambar (diisi CAL_BACA offset 0)
  constexpr uint16_t CAL_OFFSET = 29;  // R, offset jendela terakhir
  constexpr uint16_t CAL_HASIL  = 30;  // R, kode hasil perintah CAL terakhir (CalHasil)
  constexpr uint16_t CAL_DATA0  = 31;  // RW, 32 register = jendela 64 byte (sampai 62)
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
  // BARU (2026-09-30): sedang menjalankan salah satu adegan sebuah gerakan. Gerakan mana dan
  // adegan ke berapa ada di register GERAKAN_AKTIF / ADEGAN_KE.
  ADEGAN_GERAK = 12,
  MENUJU_READY = 13,   // BARU (2026-10-01) -- menuju pose 3, titik siaga dekat package
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
  // BARU (2026-10-09): backup / restore kalibrasi (include/kalibrasi_modbus.h), ack langsung.
  // Opcode SAMA di keempat node. Ditolak selama MAIN aktif / node bergerak.
  CAL_BACA = 60, CAL_TULIS = 61, CAL_TERAPKAN = 62,
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
  MOVE_PACKAGE = 8,  // DIUBAH 2026-10-01: Ready -> Pick -> Home -> Place -> Home -> Ready lewat
                      // lima GERAKAN (masing-masing 20 slot adegan, lihat main.cpp). TANPA arg.
  // BARU -- biar kecepatan trajectory (berlaku SAMA ke semua 6 joint) bisa di-tuning dari
  // Orange Pi langsung, gak wajib lewat LCD/Serial lokal.
  SET_TRAJ_STEP = 9,           // arg: trajStepUs, 1-500
  SET_TRAJ_STEP_INTERVAL = 10, // arg: trajStepIntervalMs, 5-200
  // BARU -- pemisah MAIN/TEST eksplisit (sama konsep dgn SORTER/DISPENSER/STOCKER). Default
  // boot = mainModeActive FALSE. Selama MAIN aktif, command "manual override" (GOTO_HOME/
  // PICK/PLACE) DITOLAK TOTAL -- cuma MOVE_PACKAGE (produksi
  // asli) yang jalan. Sebaliknya, MOVE_PACKAGE ditolak selama masih TEST mode.
  START_MAIN = 11,
  STOP_MAIN = 12,
  // BARU (2026-09-22): menuju pose mana pun, arg = nomor slot (0=HOME, 1=PACKAGE_PICKUP,
  // 2=LIFT_LOAD). Sebelumnya hanya GOTO_HOME yang punya opcode, sehingga pose 1 dan 2 --
  // dua-duanya tujuan produksi -- tidak bisa didatangi dari Orange Pi sama sekali. Untuk
  // memverifikasi kalibrasi dari jarak jauh, keduanya justru yang paling perlu dilihat.
  // Manual override, jadi DITOLAK selama MAIN aktif seperti GOTO_HOME/PICK/PLACE.
  GOTO_POSE_N = 13,
  // BARU (2026-09-30): jalankan SATU gerakan, arg 0-4 (nomornya lihat GERAKAN_AKTIF).
  // Lengan lebih dulu dibawa ke pose ASAL gerakan itu, karena adegan-adegannya
  // dirancang dari sana. Manual override -- ditolak selama MAIN aktif.
  RUN_GERAKAN = 14,
  // BARU (2026-10-01): ke pose 3 READY lewat HOME + gerakan Home>Ready -- titik siaga lengan
  // menunggu MOVE_PACKAGE. Manual override, ditolak selama MAIN aktif.
  GOTO_READY = 15
};