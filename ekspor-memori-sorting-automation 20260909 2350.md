# Ekspor Memori — Sorting Automation System

Dokumen ini berisi SEMUA fakta yang benar-benar tersimpan di sistem memori Claude
terkait proyek ini (bukan seluruh percakapan). Disusun dari beberapa file memori
terpisah yang digabung di sini. Setiap poin ditandai `[stated]` — artinya berasal
dari sesuatu yang pernah Anda nyatakan langsung.

**Catatan penting sebelum impor:** ada 2 kelompok sumber di bawah, dari FASE proyek
yang BERBEDA:
- **Kelompok A** — arsitektur SAAT INI (4 node terpisah: SORTER/PICKER/Dispenser/STOCKER)
- **Kelompok B** — proyek AWAL sebelum dipecah jadi 4 node (1 firmware gabungan DC motor
  + stepper). Sebagian info di sini KEMUNGKINAN SUDAH TIDAK RELEVAN dengan arsitektur
  sekarang, tapi saya sertakan apa adanya karena memang tersimpan.

---

## KELOMPOK A — Arsitektur Saat Ini (dari /areas/sorting-automation-system.md)

- Main controller: Orange Pi PC Plus — menangani orkestrasi sistem dan analisis kamera
  HuskyLens (klasifikasi pass/reject)
- ESP32 Node 1: hopper + conveyor1 + palang reject, pakai driver motor DC TB6612FNG
- ESP32 Node 2: lengan robot 6DOF pakai driver servo PCA9685, dengan posisi arm_home,
  arm_pass, arm_reject, arm_lift, arm_pick, arm_place
- ESP32 Node 3: box dispenser dengan conveyor2 dan mekanisme **2 gerbang servo** (gerbang
  bawah + gerbang atas pada stack package) — **diredesign** dari mekanisme push TB6612FNG
  sebelumnya
- Subsistem Lift: axis XYZ pakai motor stepper dengan driver TMC2209, plus pendorong
  box (TB6612FNG); memindahkan box ke rak 2x3 (posisi 1-2 dari bawah = reject, 3-6 = pass)
- Box menampung 20 objek sebelum dipindahkan oleh lengan robot
- Lift butuh info rack-occupancy dari database controller utama dan lapor balik setelah
  meletakkan box
- Terkait: mount kamera HuskyLens (lihat Kelompok C di bawah) — untuk conveyor1 yang sama
- **Konvensi penamaan node:** Node1=SORTER (hopper+conveyor1+palang), Node2=PICKER (lengan),
  Node3=Dispenser (sebelumnya disebut FEEDER), Node4=Lifter (sebelumnya disebut STOCKER,
  lift+rak)
- **Hardware universal per node:** MCP23017 untuk I/O digital, LCD PCF8574 (0x20) + Keypad
  PCF8574 (0x21) untuk debug/kalibrasi lokal
- **Kebijakan operasional:** fault di 1 node menghentikan seluruh line; rak penuh
  menghentikan line dulu; motor yang dipakai kecil (skala simulasi/prototipe, bukan
  industri sesungguhnya)
- **Firmware direstrukturisasi jadi 4 project terpisah per node** (SORTER, STOCKER,
  FEEDER, PICKER) menggantikan pendekatan 1 binary universal — dimulai dari SORTER dulu
- LCD yang dipakai adalah 20x4 (bukan 16x2)
- **Keempat node standar pakai library resmi `Adafruit_PWMServoDriver`** untuk kontrol
  servo PCA9685 (menggantikan driver I2C mentah tulisan sendiri di beberapa node)
- **Sedang berpindah workspace pengembangan utama proyek ini ke Claude Code**

---

## KELOMPOK B — Proyek Awal / Firmware Motor Control (Sebelum Dipecah 4 Node)

### Prinsip & Pembelajaran Teknis
- Motor stepper lebih disukai dibanding motor DC biasa untuk kontrol posisi open-loop;
  motor DC akan butuh encoder atau limit switch ganda untuk presisi setara
- Wiring limit switch NC (normally closed) memberi perilaku fail-safe terhadap kondisi
  kabel putus
- Masalah motor tidak berputar bisa disebabkan pasangan koil yang salah — diselesaikan
  lewat uji kontinuitas multimeter
- Mirror-reversal langsung untuk return-to-home lebih efisien daripada menjalankan ulang
  seluruh urutan homing

### Bug yang Ditemukan & Diselesaikan
- Arah homing hardcoded di logic keselamatan endstop
- Penanganan tanda (sign) yang salah di arah PUSH
- Return-to-home yang boros memanggil `startHoming()` penuh, bukan mirror-reversal langsung
- PUSH return akhir pakai mirror-reversal langsung (`-forwardStepsUsed`), dengan endstop
  safety sebagai fallback yang otomatis mengoreksi `positionSteps` ke 0 kalau switch
  tersentuh lebih awal

### Konfigurasi Hardware (Terkonfirmasi, Fase Awal)
- NEMA17 di 1.2A/phase; Vref ~0.85V (80% arus rating, Rsense 0.11Ω)
- Limit switch wired NC di GPIO4 ke GND dengan `INPUT_PULLUP` (desain fail-safe)
- Wiring koil: M1A→Pin1, M1B→Pin3, M2A→Pin4, M2B→Pin6
- `HOMING_DIR_TOWARD_SWITCH` = `true` (switch di sisi maju)

### Arsitektur Kode (Fase Awal)
- `pinout_config.h` — semua definisi pin GPIO dan parameter hardware
- `push_config.h` — konfigurasi kalibrasi untuk command PUSH (`PUSH_SPEED_SPS`,
  `PUSH_TARGET_STEPS`)
- `main.cpp` — semua logic program
- `platformio.ini` — konfigurasi build

### Tools
- Platform: ESP32 dengan PlatformIO (VSCode)
- ESP32 Arduino Core default: 3.x — API LEDC berbasis pin (bukan
  `ledcSetup`/`ledcAttachPin`/`ledcWrite` berbasis channel); berlaku untuk semua proyek
  ESP32/PlatformIO kecuali dinyatakan lain secara eksplisit
- Driver motor: TB6612FNG (motor DC), TMC2209 (stepper)

### Cara Kerja yang Disepakati (Working Agreements)
- File output dikirim sebagai file `.txt` dengan timestamp di header — TIDAK PERNAH
  sebagai arsip zip
- Cuma file yang benar-benar berubah isinya yang ditampilkan tiap revisi; file yang
  tidak berubah tidak pernah disertakan
- Pengembangan iteratif, bertahap: minta perubahan spesifik ke kode yang sudah ada,
  bukan penulisan ulang penuh
- Pertanyaan konseptual (misal trade-off jenis motor) diterima berdampingan dengan
  kerja implementasi

---

## KELOMPOK C — Mount Kamera HuskyLens (Terkait Conveyor1)

- Butuh holder 3D-print untuk modul kamera AI HuskyLens K210
- Kamera harus menghadap ke bawah ke arah conveyor; layar menghadap ke atas/ke operator
- Lebar conveyor belt: 99.5mm
- Dirancang holder OpenSCAD parametrik; tiga iterasi: jembatan/gantry → bracket yang
  disempurnakan kosmetiknya → dudukan miring satu sisi dengan shell enclosure
- Prinsip kunci: interface lewat bracket aluminium OEM dengan lubang slot (toleransi
  ±5mm), bukan pocket bentuk-pas ke badan kamera
- PETG direkomendasikan; dikirim STL + source SCAD dengan `stand_lib.scad` bersama

---

## ⚠️ Yang PENTING Anda Tahu — Ini BUKAN Rangkuman Percakapan Ini

Sesi kerja **sangat panjang** yang baru saja kita lalui (channel mapping universal
board, register Modbus, opcode tiap node, bug-bug yang ditemukan dan diperbaiki —
ESTOP polarity, watchdog timeout, pinMode Test Modul, servo PICKER, hopper SORTER,
buzzer, dsb) **TIDAK sepenuhnya tercatat** di sistem memori terstruktur ini. Sistem
memori hanya menyimpan fakta-fakta **stabil dan berulang** yang diproses otomatis
setelah tiap giliran percakapan — bukan transkrip teknis mendetail dari sesi kerja
individual.

**Kalau Anda butuh detail teknis LENGKAP dari sesi ini** (channel map persis, register
address, opcode per node, riwayat bug) — itu ada di **file kode sungguhan** yang sudah
saya kirim sepanjang percakapan ini (`main.cpp`/`config.h`/`registers.h` tiap node) dan
**dokumen "Alur Kerja Command & Fungsi"** yang saya buat sebelumnya — bukan di sini.
Kalau perlu, saya bisa bantu susun **ringkasan teknis terpisah** dari isi kode-kode
tersebut, khusus untuk keperluan impor/referensi ke akun lain.