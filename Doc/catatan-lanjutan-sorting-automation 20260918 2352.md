# Catatan Lanjutan — Sorting Automation System (v2)

> **SUPERSEDED (2026-09-20).** Dokumen ini berhenti di 2026-09-18 dan TIDAK mencakup
> kerja dua hari setelahnya (MAIN/TEST mode, celah #1-#21, redesign protokol Dispenser,
> audit node). Untuk status TERKINI, baca `cara-kerja-sistem-dan-analisa-celah 20260919.md`.
> Dokumen ini disimpan hanya sebagai arsip.

Dibuat 2026-09-18 karena context window sesi hampir penuh, sesi ini SANGAT panjang
(lanjutan dari sesi sebelumnya, ada `catatan-lanjutan-sorting-automation 20260911 0033.md`
di repo yang sekarang SEBAGIAN BESAR sudah basi/di-supersede oleh catatan ini).

**Baca ini duluan** sebelum lanjut kerja di repo ini. Catatan lama (0911) masih ada beberapa
info hardware/fisik yang relevan (power supply, wiring E18-D80NK, dll) — tapi SEMUA soal
status kerjaan firmware sudah digantikan catatan ini.

---

## Status Sekarang (PALING PENTING, baca duluan)

- **Branch**: `claude/sorting-pipeline-fixes-and-2-proximity`, PR masih sama:
  https://github.com/CV-INOVASI-MULTI-SOLUSI/ESP32_Sorting-Automation-System/pull/1
  **Belum di-merge.**
- **Commit terakhir sesi ini**: `ddc0226` (sudah di-push). Semua source code firmware +
  4 skrip test Python baru SUDAH ke-commit & ke-push. Git bersih (kecuali file yang memang
  sengaja gak di-track, lihat di bawah).
- **File yang SENGAJA gak di-commit**: `ESP32_SortingAutomation.code-workspace` (referensi
  folder lokal user, bukan project-wide), `Panduan Penggunaan Sorting Automation
  20260911.docx` (dokumen manual, sudah dikirim ke user via file delivery, murni referensi).
- **Firmware yang UDAH di-flash ke hardware asli** (dikonfirmasi user selama sesi):
  - **Dispenser** — sudah di-OTA beberapa kali sepanjang sesi, versi TERAKHIR yang di-flash
    itu SEBELUM fix "MAIN/TEST mode separation" (commit `ddc0226`). Jadi **firmware fisik
    Dispenser MASIH VERSI LAMA** (sebelum START_MAIN/STOP_MAIN ada) — WAJIB di-flash ulang
    sebelum bisa pakai fitur MAIN/TEST yang baru.
  - **Sorter, Picker, Stocker** — setau catatan ini, **BELUM PERNAH di-flash sama sekali**
    di sesi ini (semua kerjaan baru sebatas compile-check `pio run`). Firmware fisik masih
    versi SEBELUM sesi ini dimulai.
- **⚠️ CRITICAL — orchestrator BELUM di-update**: `orangepi-orchestrator-batch.py` (script
  produksi asli buat Orange Pi) **BELUM mengirim `START_MAIN`/`START`** ke node manapun di
  `startup_sequence()`-nya. Setelah fitur "MAIN/TEST mode separation" ini di-flash ke
  hardware, **PRODUKSI OTOMATIS TIDAK AKAN JALAN** (REQUEST_REFILL/RUN_SEQUENCE/MOVE_PACKAGE/
  RUN_FULL_CYCLE semua DITOLAK) sampai orchestrator diperbaiki buat kirim START_MAIN (atau
  START utk Sorter) ke tiap node dulu sebelum masuk `run_batch_sequence()`. **INI TUGAS
  PERTAMA yang harus dikerjain di sesi berikutnya kalau mau lanjut ke arah produksi.**
- Alamat WiFi node berubah-ubah selama sesi (dari `Raspberry R7` ke SSID `DD1010`, password
  WiFi & OTA tetap `wlanebc417`). IP terakhir yang dipakai buat OTA Dispenser: `10.68.240.118`
  (BISA BERUBAH kalau DHCP re-lease — cek Serial Monitor boot log node buat IP terbaru).
  Orange Pi PC Plus sendiri diakses di `192.168.3.61` (SSH, `root`), TAPI koneksi SSH dari
  laptop sempat gagal/timeout di awal sesi — belakangan user beralih testing LANGSUNG dari
  terminal Orange Pi sendiri (`root@orangepipcplus:~/sorting#`), bukan lewat SSH dari laptop.

---

## FITUR BESAR BARU SESI INI: Pemisah MAIN/TEST Mode (SEMUA 4 NODE)

Ini perubahan arsitektur PALING BESAR sesi ini, latar belakangnya: ditemukan berkali-kali
konflik antara logic OTOMATIS yang jalan di background (terutama `handleMiddleSensor()`
Dispenser yang mantau PROX_2 terus-menerus TANPA peduli command apa yang lagi dikirim) vs
command MANUAL/TEST yang dikirim buat testing — bikin command manual "ke-terima" tapi
sebenernya rebutan hardware sama logic otomatis, atau sebaliknya command otomatis nyelonong
pas lagi testing manual.

**Solusi**: setiap node sekarang punya `bool mainModeActive` (default **FALSE** = fail-safe,
boot-IDLE). Command dikelompokkan 2 kubu yang **SALING EKSKLUSIF**:
- **Kubu MAIN** (produksi asli, dipicu Orange Pi) — DITOLAK kalau `mainModeActive==false`.
- **Kubu TEST** (manual/jog/debug) — DITOLAK kalau `mainModeActive==true`.
- **Kubu NETRAL** (tuning angka doang, SET_*_STEP/SET_*_SPEED/SET_*_INTERVAL) — SENGAJA
  DIKECUALIKAN dari pemisahan ini, tetap boleh dipanggil kapan saja (Orange Pi perlu bisa
  tuning kecepatan live pas produksi jalan, itu fitur yang udah dibangun duluan sesi ini).

### Tabel lengkap per node

| Node | Opcode START/STOP MAIN | Kubu MAIN (butuh MAIN aktif) | Kubu TEST (butuh TEST mode) | Kubu NETRAL (kapan saja) |
|---|---|---|---|---|
| **Sorter** | `START`(1) / `STOP`(2) — **REUSE opcode lama**, bukan opcode baru | *(otomatis lewat currentState==RUNNING_OR_MOVING setelah START)* | `SET_MOTOR_A`(8), `TEST_HOPPER_CYCLE`(97), `TEST_TRIGGER_PALANG`(98) | `SET_PALANG_SPEED`(9), `SET_HOPPER_STEP`(10), `SET_HOPPER_INTERVAL`(4), `SET_CONVEYOR_SPEED`(5), `SET_CONVEYOR_DIR`(6) |
| **Dispenser** | `START_MAIN`(14) / `STOP_MAIN`(15) — opcode BARU | `REQUEST_REFILL`(1), `FORCE_MIDDLE_REFILL`(6), `ACK_PACKAGE_TAKEN`(5) | `SET_CONVEYOR_ON_OFF`(11), `MOVE_SERVO1_TO`(12), `MOVE_SERVO2_TO`(13), `TEST_SERVO1_CYCLE`(97), `TEST_SERVO2_CYCLE`(98), `TESTLOOP` (Serial/LCD, bukan opcode Modbus) | `SET_SERVO1_STEP`(7), `SET_SERVO1_STEP_INTERVAL`(8), `SET_SERVO2_STEP`(9), `SET_SERVO2_STEP_INTERVAL`(10) |
| **Picker** | `START_MAIN`(11) / `STOP_MAIN`(12) — opcode BARU | `RUN_SEQUENCE`(1), `MOVE_PACKAGE`(8) | `GOTO_HOME`(2), `GOTO_PASS`(3), `GOTO_REJECT`(4), `PICK`(5), `PLACE`(6) | `SET_TRAJ_STEP`(9), `SET_TRAJ_STEP_INTERVAL`(10) |
| **Stocker** | `START_MAIN`(9) / `STOP_MAIN`(10) — opcode BARU | `RUN_FULL_CYCLE`(2) | `MOVE_TO_RACK`(3), `PUSH_BOX`(4) | `SET_STEP_INTERVAL`(7), `SET_HOMING_STEP_INTERVAL`(8), **`HOME_ALL`(1) & `GOTO_LOAD_POSITION`(6) SENGAJA TIDAK di-gate** (dipakai bareng produksi & Test Rak, Stocker gak punya auto-trigger background yang bisa bentrok) |

`RESET_FAULT` di SEMUA node **selalu jalan kapanpun**, gak pernah diblokir (safety escape hatch).

Register baru **`MAIN_MODE_ACTIVE`** ditambah di semua node (Sorter=17, Dispenser=18,
Picker=15, Stocker=17) — Orange Pi bisa poll status MAIN/TEST kapan saja.

### Konsekuensi PENTING
1. **Orchestrator WAJIB update** (lihat "CRITICAL" di atas) — tanpa `START_MAIN`/`START`,
   node akan MENOLAK semua command produksi setelah firmware baru di-flash.
2. Skrip test yang sudah dibuat (`test-dispenser-*`, `test-stocker-rack-sequence`) — SEMUA
   sudah disesuaikan/dicek terhadap perubahan ini:
   - `test-stocker-rack-sequence` — SUDAH diperbaiki, kirim `START_MAIN` sebelum loop rak,
     `STOP_MAIN` di akhir.
   - `test-dispenser-full-protocol` / `-loop-cycle` / `-manual-sequence` — TIDAK perlu diubah,
     semua cuma pakai command kubu TEST yang default jalan (mainModeActive=false saat boot).

---

## Kejadian Penting Lain Sesi Ini (kronologis, ringkas)

### 1. OTA (WiFi firmware update) — sudah ada dari SEBELUM sesi ini dimulai
Semua 4 node punya ArduinoOTA aktif. Kredensial WiFi di `include/wifi_credentials.h`
(gitignored, template di `.h.example`). Command upload:
```powershell
$env:Path += ";$env:USERPROFILE\.platformio\penv\Scripts"
$env:OTA_PASS = "wlanebc417"
pio.exe run -e esp32dev_ota -t upload --upload-port <IP_NODE>
```
(pakai `-d <folder_node>` kalau CWD bukan di folder project node-nya). **SSID WiFi terakhir
`DD1010`** (berubah dari `Raspberry R7` di tengah sesi, user yang ubah router).

### 2. Dispenser — hapus total limit switch, redesign jadi murni 2-proximity
- `LIM_STOCK_EMPTY`/`LIM_PUSH_HOME`/`LIM_PUSH_EXTENDED` DIHAPUS total dari firmware (device
  fisik Dispenser gak ada limit switch, sesuai konfirmasi user).
- `LIM_BOX_ARRIVED` di-rename jadi `PROX_BOX_ARRIVED` (nama jujur, itu proximity bukan LIM).
- **TEST REFILL LOOP** — fitur test lokal baru (tombol fisik `BUTTON_2` + toggle LCD/Serial
  `TESTLOOP`), retry TANPA BATAS ngisi TENGAH (conveyor+servo jalan bareng sampai PROX_2
  konfirmasi), lalu tombol fisik simulasi "box penuh" jalanin conveyor ke UJUNG. Terpisah
  total dari state machine produksi.
- Command baru: `SET_CONVEYOR_ON_OFF`(11, jog manual conveyor), `MOVE_SERVO1_TO`/
  `MOVE_SERVO2_TO`(12/13, jog manual 1 arah servo, buat masukin package pertama kali manual).
- Register baru: `UJUNG_PACKAGE_PRESENT`(17, live PROX_1), `MAIN_MODE_ACTIVE`(18).
- **Bug ditemukan & diperbaiki**: `RESET_FAULT` gak clear `testLoopStage` yang kesisa aktif
  dari testing sebelumnya (invisible dari Orange Pi, cuma keliatan di Serial USB device) —
  sekarang `RESET_FAULT` juga paksa keluar TEST REFILL LOOP.
- **Bug ditemukan & diperbaiki**: `MOVE_SERVOx_TO`/`TEST_SERVOx_CYCLE` cuma cek
  `currentState==IDLE`, TIDAK cek `servoRefillStage` (refill otomatis PROX_2 gak pernah ubah
  `currentState`) — bisa rebutan servo. Sekarang ada guard `isServoRefillAutoActive()`.

### 3. Sorter — Hopper Push Hold (fix "reverse sebelum sampai")
Root cause "hopper mundur sebelum sampai ujung": sistem step-rate itu OPEN-LOOP (gak ada
sensor posisi fisik hopper), software declare "sampai" murni dari hitungan step, BUKAN
konfirmasi fisik. Kalau kalibrasi Step/Interval kebetulan lebih cepat dari kemampuan fisik
servo, software udah declare "sampai" padahal fisiknya belum. **Fix**: parameter baru
`Hopper Push Hold(ms)` — jeda tahan WAJIB di titik dorong sebelum retract, independen dari
akurasi kalibrasi Step/Interval. Diterapkan ke produksi (`HopperState::PUSH_HOLD`) DAN Test
Hopper Cycle (`TestHopperCycleStage::PUSH_HOLD`).

### 4. Stepper Stocker — drama microstep TMC2209 vs DRV8825 (PENTING buat dibaca!)
- User coba percepat motor stepper Stocker (17HS8401 di X/Z, 36BYG1204 di Y) dengan ganti
  jumper MS1/MS2 TMC2209.
- **Tabel microstep TMC2209 di kode AWALNYA SALAH** (nyampur sama pola driver lain) — sudah
  diperbaiki ke tabel resmi (sumber: Watterott SilentStepStick docs):
  `MS1=GND,MS2=GND→1/8 (PALING CEPAT/default floating)`, `MS1=VIO,MS2=GND→1/32`,
  `MS1=GND,MS2=VIO→1/64`, `MS1=VIO,MS2=VIO→1/16`. **TIDAK ADA cara lebih cepat dari 1/8 lewat
  jumper TMC2209** — itu udah batas maksimal via pin.
- User sempat coba GANTI MODUL ke DRV8825 (yang punya full-step via jumper M0/M1/M2) — saya
  udah TERAPKAN firmware-nya (tabel microstep 1/2/4/8/16/32, dll), TAPI **USER MEMBATALKAN**
  ("roll back tidak jadi menggunakan DRV") — firmware SUDAH DI-ROLLBACK PENUH ke TMC2209.
  **Fisik Stocker TETAP PAKAI TMC2209**, jangan bingung kalau lihat jejak diskusi DRV8825 di
  riwayat chat, itu sudah dibatalkan.
- `microstepMode` di firmware **CUMA LABEL STATUS**, gak ada kontrol elektris apapun ke chip
  (microstep fisik ditentukan MURNI jumper manual) — cuma dipakai buat kalkulasi RPM/estimasi
  di percakapan, gak dipakai kalkulasi real internal firmware.
- **VREF (arus motor)**: user belum dapat datasheet resmi 17HS8401/36BYG1204 (web search
  sempat down). Nilai KONSERVATIF yang disepakati: 17HS8401 (X/Z) VREF≈0,90V (~1,26A),
  36BYG1204 (Y) VREF≈0,40V (~0,56A) — pakai rumus TMC2209 `VREF=Arus÷1,41`. **BELUM ada
  konfirmasi final apakah nilai ini udah dites fisik / dipasang di potensiometer modul.**

### 5. Stocker — redesign "Test ke Rak" + instrumentasi timing
- Dulu: Home → Rak (berhenti, gak ada push/tarik). Sekarang: **Load Position → Rak → Push →
  Tarik → Load Position lagi**, reuse `cycleStage` produksi yang sama persis dgn
  `RUN_FULL_CYCLE` (bukan logic duplikat). Butuh sudah AutoHome dulu (gak lagi auto-home
  paksa tiap test).
- Tambah pengukuran & tampilan **durasi 1 siklus penuh** (LCD + Serial), plus instrumentasi
  `[CYCLE-TIMING]` per-tahap (MOVING_XZ/PUSHING_Y/RETRACT_Y/RETURNING_XZ) buat debug performa
  — **investigasi gap timing (test 31,64 detik vs kalkulasi 23,38 detik) BELUM TUNTAS**,
  terakhir ketemu kontributor parsial (keypad scan blocking ~1-2 detik) tapi masih ada ~4-6
  detik yang belum terjelaskan. User belum sempat jalanin ulang test dgn instrumentasi timing
  baru ini buat lihat breakdown pastinya.
- `stepIntervalUs` (Kecepatan > Jelajah) sekarang boleh **0** (dulu minimum 20) — 0 = tanpa
  jeda sama sekali, kecepatan dibatasi murni oleh seberapa cepat `loop()` jalan.
- `I2C_FREQ_HZ` (400000 sekarang) sempat dibahas naikin ke 1000000 buat kurangin overhead I2C
  (kontributor lain ke gap timing) — **BELUM DITERAPKAN**, baru sebatas diskusi/saran.

### 6. Command Modbus "set kecepatan" ditambah ke SEMUA 4 node
Sebelumnya cuma bisa lewat LCD/Serial lokal. Sekarang Orange Pi bisa tuning langsung:
- Sorter: `SET_PALANG_SPEED`(9), `SET_HOPPER_STEP`(10)
- Dispenser: `SET_SERVO1_STEP`(7), `SET_SERVO1_STEP_INTERVAL`(8), `SET_SERVO2_STEP`(9),
  `SET_SERVO2_STEP_INTERVAL`(10)
- Picker: `SET_TRAJ_STEP`(9), `SET_TRAJ_STEP_INTERVAL`(10)
- Stocker: `SET_STEP_INTERVAL`(7), `SET_HOMING_STEP_INTERVAL`(8)

Semua runtime-only (gak auto-save NVS, biar gak boros write-cycle flash) — permanen tetap
lewat LCD `#` atau Serial di board-nya langsung.

### 7. HuskyLens — arsitektur SUDAH ketauan dari kode, BELUM diimplementasi
Register `Reg::CLASSIFY_IS_REJECT` (Sorter, addr 12) memang disiapkan buat "jalur Modbus asli
dari OrangePi/HuskyLens" (komentar asli di kode). Orange Pi tinggal `write_register(12, 0/1)`
tiap ada objek lewat sensor vision — firmware OTOMATIS hitung TOF (`calculateTOF()`, pakai
`distMm` & `mmPerSecAtMaxPwm` yang WAJIB dikalibrasi akurat) dan trigger palang di waktu yang
tepat, Orange Pi gak perlu itung timing sendiri. **Belum ada kerjaan lanjutan soal HuskyLens
integrasi fisik/kabel/kode Orange Pi** — ini murni exploratory discussion, belum actionable.

### 8. Dokumen Word "Panduan Penggunaan" — sudah dibuat & dikirim
`Panduan Penggunaan Sorting Automation 20260911.docx` — panduan lengkap fitur, parameter
kalibrasi tiap node, contoh kalkulasi kecepatan/RPM/interval. **SUDAH BASI SEBAGIAN** (dibuat
sebelum banyak perubahan sesi ini: MAIN/TEST mode, command baru, dll) — kalau user minta
dokumen serupa lagi, PERLU DIPERBARUI dulu, jangan asumsikan isinya masih akurat 100%.

---

## Skrip Python Baru Sesi Ini (semua di root repo, `DRY_RUN=True` default)

| File | Fungsi |
|---|---|
| `test-stocker-rack-sequence 20260918.py` | AutoHome → Load Pos → (START_MAIN) → RUN_FULL_CYCLE rak 1,2,3,4 berurutan (push+tarik+balik built-in), delay 2s cuma pas di Load Position |
| `test-dispenser-manual-sequence 20260918.py` | Urutan awal: conveyor mati→servo1 cycle→conveyor nyala→servo2 cycle→tunggu PROX_2→conveyor nyala→delay 2s→conveyor nyala lagi→tunggu PROX_1→conveyor mati (versi AWAL, lebih simpel) |
| `test-dispenser-loop-cycle 20260918.py` | Versi LOOPING (Ctrl+C stop): setup sekali (servo1 only) lalu loop PROX_2→servo2 cycle→PROX_1 trigger→PROX_1 clear→ulang |
| `test-dispenser-full-protocol 20260918.py` | Versi PALING LENGKAP 17 langkah user (servo1+servo2 round-trip di setup DAN tiap loop, placeholder timing utk "konfirmasi jumlah objek"/"ambil package", fungsi `manual_load_servo()` buat isi package pertama kali pakai MOVE_SERVOx_TO) |

**Index rak di `test-stocker-rack-sequence`**: dipakai `[1,2,3,4]` apa adanya sesuai kata user
— BELUM dikonfirmasi apakah maksud user itu rack_idx 1-4 (0-indexed, lewatin slot 0) atau 4
rak pertama (harusnya `[0,1,2,3]`). **Perlu ditanya ulang kalau user lanjut testing ini.**

---

## TODO Prioritas Sesi Berikutnya

1. **🔴 WAJIB PALING PERTAMA**: Update `orangepi-orchestrator-batch.py` — tambah
   `START_MAIN`(Dispenser/Picker/Stocker) dan `START`(Sorter) di `startup_sequence()` sebelum
   masuk loop produksi. Tanpa ini, produksi otomatis TOTAL GAK JALAN setelah firmware baru
   di-flash.
2. **Flash ulang SEMUA 4 node** dengan firmware commit `ddc0226` (baru Dispenser yang pernah
   di-OTA, itupun versi LAMA sebelum MAIN/TEST — Sorter/Picker/Stocker belum pernah disentuh
   fisik sama sekali sepanjang sesi ini).
3. **Lanjutin investigasi gap timing Stocker** (31,64s vs 23,38s teori) — pakai instrumentasi
   `[CYCLE-TIMING]` yang udah ditambah, breakdown per-tahap buat isolasi penyebab pasti.
   Pertimbangkan juga naikin `I2C_FREQ_HZ` ke 1000000 (belum diterapkan, baru wacana).
4. **Konfirmasi & pasang VREF** driver TMC2209 Stocker (nilai konservatif sudah dihitung,
   belum ada kabar dipasang/dites fisik).
5. **Test `test-dispenser-full-protocol.py`** ke hardware asli (`DRY_RUN=False`) — belum
   pernah dites real sepanjang sesi, cuma dibuat & dicompile-check.
6. Klarifikasi index rak di `test-stocker-rack-sequence` (poin di atas).
7. HuskyLens — masih exploratory, belum ada kerjaan konkret dimulai.
8. Update `Panduan Penggunaan...docx` kalau user butuh dokumentasi terbaru (banyak yang basi).
9. Pertimbangkan PR #1 mau di-merge kapan — makin lama makin menggunung perubahannya.

---

## Referensi Silang

- Catatan sesi SEBELUMNYA (sebagian basi soal status kerjaan, tapi info hardware/wiring/power
  masih relevan): `catatan-lanjutan-sorting-automation 20260911 0033.md`
- `orangepi-orchestrator-batch.py` — masih `DRY_RUN=True`, dan sekarang JUGA belum kirim
  START_MAIN (lihat TODO #1) — dua alasan kenapa belum bisa dipakai produksi riil.
- `contoh-kirim-command 20260909 2350.py` — contoh pola dasar Modbus 1 node, masih relevan.
