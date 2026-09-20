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
  // BARU: status live PROX_BOX_ARRIVED (PROX_1, UJUNG) -- simetris dgn MIDDLE_PACKAGE_PRESENT,
  // buat Orange Pi/skrip test poll status UJUNG langsung tanpa lewat PACKAGE_READY_FLAG (yang
  // cuma ke-set otomatis lewat alur REQUEST_REFILL, gak reflect sensor mentah pas mode manual).
  constexpr uint16_t UJUNG_PACKAGE_PRESENT = 17;  // R, 1 = ada package terdeteksi di ujung
  // BARU: status live mainModeActive -- Orange Pi bisa cek node lagi MAIN (produksi otomatis)
  // atau TEST (manual) tanpa perlu nebak dari efek command lain.
  constexpr uint16_t MAIN_MODE_ACTIVE = 18;  // R, 1 = MAIN aktif, 0 = TEST mode
  // BARU: counter LATCH -- naik +1 SETIAP kali PACKAGE_MIDDLE_SENSOR (PROX_2) kedeteksi
  // package baru dateng (edge HIGH->LOW, live == MIDDLE_PACKAGE_PRESENT 0->1), TERLEPAS
  // kapan Orange Pi sempat baca. Beda dari MIDDLE_PACKAGE_PRESENT (live/instant, bisa
  // kelewatan kalau Orange Pi lagi sibuk poll command lain, mis. servo round-trip ~7 detik) --
  // counter ini gak pernah kelewatan event, script tinggal bandingin nilai SEBELUM vs SESUDAH
  // nunggu. Wrap dari 65535 balik ke 0 itu WAJAR (uint16_t Modbus), tetap valid dibandingkan
  // != asal gak lebih dari 65535 event kelewatan di satu jendela tunggu (mustahil praktiknya).
  constexpr uint16_t MIDDLE_ARRIVAL_COUNT = 19;  // R
  // BARU (2026-09-20): menu kalibrasi LCD meng-IGNORE semua command Modbus (§12.6) TANPA
  // kirim CMD_ACK_SEQ -- dari sisi master itu kelihatan identik dgn "node mati/kabel putus".
  // Register ini bikin master bisa bedain dua kondisi itu.
  constexpr uint16_t MENU_ACTIVE = 20;  // R, 1 = operator lagi di menu kalibrasi LCD
  // BARU (2026-09-20): counter LATCH dua tombol fisik yang selama ini menganggur, dipakai
  // sebagai KONFIRMASI NYATA menggantikan delay/timing tebakan di script test:
  //   BUTTON_2 -> "package sudah PENUH"        (dulu placeholder timing)
  //   BUTTON_3 -> "package sudah DIAMBIL robot" (dulu tidak ada sama sekali)
  // Naik +1 tiap penekanan (debounce 50 ms). Sama pola dgn MIDDLE_ARRIVAL_COUNT: master cukup
  // membandingkan nilai SEBELUM vs SESUDAH menunggu, jadi tidak pernah kehilangan kejadian
  // walau polling-nya sempat tertunda. Wrap 65535 -> 0 wajar dan tetap valid dibandingkan !=.
  constexpr uint16_t BTN_PACKAGE_FULL_COUNT  = 21;  // R
  constexpr uint16_t BTN_PACKAGE_TAKEN_COUNT = 22;  // R
  // BARU (2026-09-20): berapa kali FIRMWARE menghentikan conveyor sendiri begitu PROX_2
  // mendeteksi package datang -- tanpa menunggu perintah dari master. Master tidak selalu
  // sedang memperhatikan (saat mengirim rangkaian perintah servo, misalnya), dan package
  // keburu lewat kalau penghentiannya menunggu giliran polling. Counter ini murni untuk
  // memastikan penghentian itu benar-benar terjadi dan menghitung berapa kali.
  constexpr uint16_t CONVEYOR_AUTOSTOP_COUNT = 23;  // R
  // BARU (2026-09-20): kedatangan SAH di PROX_1 (UJUNG). Hanya naik kalau sebelumnya memang
  // ada package yang tercatat meninggalkan PROX_2 -- satu-satunya jalan menuju UJUNG adalah
  // lewat TENGAH. PROX_1 yang menyala tanpa itu (tangan operator, benda tersenggol, pantulan
  // sensor) DIABAIKAN, tidak menaikkan counter ini dan tidak menghentikan conveyor.
  // Register live UJUNG_PACKAGE_PRESENT (17) tetap melaporkan keadaan sensor apa adanya.
  constexpr uint16_t UJUNG_ARRIVAL_COUNT = 24;  // R
  // BARU (2026-09-20): Dispenser menyatakan dirinya SIAP. 1 = package sudah duduk di TENGAH
  // dengan gerbang terbuka, jadi benar-benar siap menampung objek baru. Orange Pi memakai ini
  // sebagai syarat sebelum mulai menghitung batch -- bukan sekadar "node hidup".
  constexpr uint16_t DISPENSER_READY = 25;  // R
  // Tahap pipeline produksi, untuk diagnosa jarak jauh (lihat PipelineStage di main.cpp).
  constexpr uint16_t PIPELINE_STAGE  = 26;  // R
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
  // BARU -- biar Orange Pi/skrip jarak jauh BISA TAU TEST REFILL LOOP lagi aktif lewat Modbus
  // (sebelumnya blind spot total -- pesan "ditolak" cuma tercetak ke Serial USB device, gak
  // ada jejak apapun di register, operator remote gak ada cara tau kenapa command lain ditolak).
  TEST_LOOP_AKTIF = 10,
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
  // BARU -- jog manual conveyor ON/OFF (sama konsep dgn SORTER::SET_MOTOR_A), TIDAK terikat
  // ke sensor apapun -- ganti nyala/mati bebas. Ditolak kalau ada siklus otomatis lain (refill/
  // force-refill/test-loop) yang lagi jalan, biar gak rebutan output fisik yang sama.
  SET_CONVEYOR_ON_OFF = 11,      // arg: 0=mati, 1=nyala (arah ikut cfg.conveyorDir)
  // BARU -- jog manual 1 arah (BUKAN round-trip kayak TEST_SERVOx_CYCLE), buat kebutuhan
  // masukin package pertama kali manual (operator buka gerbang, taruh package, tutup lagi
  // pakai command terpisah -- gak otomatis balik sendiri kayak test cycle).
  MOVE_SERVO1_TO = 12,   // arg: 0=titik awal (servo1StartUs), 1=titik akhir (servo1EndUs)
  MOVE_SERVO2_TO = 13,   // arg: 0=titik awal (servo2StartUs), 1=titik akhir (servo2EndUs)
  // BARU -- pemisah MAIN/TEST eksplisit: Orange Pi (master) WAJIB kirim START_MAIN dulu
  // sebelum produksi otomatis (REQUEST_REFILL/FORCE_MIDDLE_REFILL/ACK_PACKAGE_TAKEN + trigger
  // otomatis handleMiddleSensor()) mau jalan. Default boot = mainModeActive FALSE (fail-safe,
  // sama prinsip "boot-IDLE" yang dipakai di semua node) -- SEBELUM ini, node cuma bisa
  // dipakai testing manual (SET_CONVEYOR_ON_OFF/MOVE_SERVOx_TO/TEST_SERVOx_CYCLE/TESTLOOP).
  // Command TEST DITOLAK TOTAL selama MAIN aktif (dan sebaliknya) -- gak ada celah bentrok lagi.
  START_MAIN = 14,
  STOP_MAIN = 15,
  TEST_SERVO1_CYCLE = 97,   // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo1
  TEST_SERVO2_CYCLE = 98    // test-only -- 1x siklus maju-mundur pakai nilai KALIBRASI servo2
};