# Catatan Lanjutan — Sorting Automation System

Dibuat 2026-09-11 karena context window sesi sebelumnya hampir penuh. Tujuan dokumen
ini: supaya sesi berikutnya (atau saya sendiri kalau history ke-summarize) bisa
lanjut kerja tanpa perlu re-derive semua keputusan desain dari awal, dan supaya
gak ada yang kelupaan/keulang.

**Baca ini duluan** sebelum lanjut kerja di repo ini.

---

## Status Sekarang (paling penting)

- **PR #1 sudah dibuat**: https://github.com/CV-INOVASI-MULTI-SOLUSI/ESP32_Sorting-Automation-System/pull/1
  Branch: `claude/sorting-pipeline-fixes-and-2-proximity`, base `main`. **Belum di-merge.**
- **BELUM ADA satupun firmware yang di-upload/di-flash ke board fisik** sepanjang sesi
  kemarin. Semua kerjaan baru sebatas: tulis kode → `pio run` (compile-check doang) →
  lanjut ke topik berikutnya. Board masih jalanin firmware LAMA (sebelum semua fix ini).
- `orangepi-orchestrator-batch.py` masih `DRY_RUN = True`, belum pernah nyentuh bus
  Modbus asli sama sekali.
- Kalau lanjut kerja: cek dulu `git status`/`git log` beneran, karena timestamp
  "sekarang" bisa aja user udah upload sendiri di antara sesi tanpa bilang.

---

## Ringkasan Perubahan per Node (semua sudah di-commit ke PR #1)

### SORTER (`ESP32_SortingAutomation_Sorter`)
- Fix: Test Modul Motor DC gak pernah `ledcWrite()` PWM → motor gak muter walau arah bener.
- Fix: Test Modul Stepper nulis native GPIO tanpa `pinMode(OUTPUT)` dulu → pulsa gak keluar.
- Fix: Test Modul Motor DC direction cuma toggle stop↔maju, sekarang cycle stop→maju→mundur.
- Fix: `AIN1`/`AIN2` kebalik di `config.h` (dikonfirmasi vs skematik user) — sekarang `AIN1=9,
  AIN2=8`.
- Fix: `PROX_1`/`PROX_2` kebalik juga — sekarang `PROX_1=19(A3), PROX_2=18(A2)`.
- **Redesign besar — Palang**: dulu relay/solenoid (1 pulsa, pegas balik sendiri), SEKARANG
  linear actuator DC motor di channel **Motor A** (`CH::AIN1/AIN2`, `LEDC_CH_MOTORA`) — channel
  yang tadinya "nganggur, tunggu keputusan dipakai buat apa" sekarang jawabannya: buat palang.
  - Field kalibrasi baru: `palangSpeed` (rename dari `motorASpeed`), `palangDir` (arah PUSH),
    `palangPushMs`, `palangRetractMs` (durasi push & retract TERPISAH).
  - State machine: `PalangState { IDLE, PUSHING, RETRACTING }` — murni time-based, TIDAK ada
    sensor limit switch di actuator ini (beda dari servo yang udah step-rate/position-based).
  - E-stop motong daya motor LANGSUNG, gak coba retract dulu (state force IDLE).
  - RLY1 (relay lama) jadi channel spare, masih testable manual, gak dipakai produksi lagi.
  - **Bug ditemukan & DIPERBAIKI**: channel Motor A dipakai BARENG manual jog (`SET_MOTOR_A`/
    serial `MOTORA`/Test Command) DAN siklus otomatis palang, TANPA saling kunci — sekarang
    `setMotorA()` dan `testModSetMotor()` channel A DITOLAK kalau `palangState != IDLE`.
- **Redesign — Hopper**: dulu duration-based (`hopperIntervalMs` = target waktu tetap, bisa
  "mundur sebelum sampai" kalau di-set kekecilan dari kemampuan fisik servo). SEKARANG
  step-rate based (`hopperStepUs` + `hopperStepIntervalMs`, sama pola Picker) — selesai
  ditentukan POSISI (`hopperCurrentUs==target`), bukan waktu abis. Batas `stepUs` 1-2500,
  `stepIntervalMs` 0-500 (0 = tanpa jeda/secepat mungkin).
- Boot loading animation di LCD (kalau ada LCD; kalau gak ada LCD, delay tetap jalan biar
  waktu settle hardware konsisten).

### DISPENSER (`ESP32_SortingAutomation_Dispenser`)
- Fix yang sama kayak Sorter: Motor DC PWM, Stepper pinMode, direction cycle, AIN swap,
  PROX swap.
- Fix: `LIM_BOX_ARRIVED` ternyata harus alias ke `PROX_1` (channel 19), BUKAN `LIM_2`(22) —
  sebelumnya FSM refill selalu timeout 8 detik sebelum sempat ke tahap servo.
- Fix: layar kalibrasi "Servo1/2 Interval" gak pernah gerak live (beda dari Hopper Sorter)
  — sekarang ada live-test-loop juga.
- **Redesign besar — mekanisme 2-proximity**: `PROX_1`=UJUNG (package FULL siap diambil arm),
  `PROX_2`=`PACKAGE_MIDDLE_SENSOR`=TENGAH (posisi isi objek).
  - `REQUEST_REFILL` (opcode 1) makna baru: conveyor MAJU bawa package dari TENGAH ke UJUNG.
  - Begitu `PROX_1`=LOW: set register `PACKAGE_READY_FLAG` (addr 15) = 1.
  - `PACKAGE_READY_FLAG` HARUS di-clear eksplisit lewat `ACK_PACKAGE_TAKEN` (opcode 5) —
    `REQUEST_REFILL` berikutnya DITOLAK selama flag masih 1.
  - Servo refill (isi ulang TENGAH) SEKARANG trigger dari edge `PROX_2` (LOW→HIGH = package
    ninggalin tengah), BUKAN delay tebakan lagi (`servoStartDelayMs` pensiun dari alur normal).
  - Setelah servo1+servo2 selesai jatuhin, TUNGGU `PROX_2` konfirmasi LOW lagi
    (`WAIT_MIDDLE_CONFIRM`) — timeout → `FaultCode::MIDDLE_PACKAGE_MISSING`.
  - Register baru: `PACKAGE_READY_FLAG`(15), `MIDDLE_PACKAGE_PRESENT`(16, live status PROX_2
    buat Orange Pi cek startup).
  - Opcode baru: `ACK_PACKAGE_TAKEN`(5), `FORCE_MIDDLE_REFILL`(6, khusus startup/recovery
    kalau tengah kosong dari awal — conveyor jalan duluan, servo nyusul setelah
    `servoStartDelayMs` [field ini DIPAKAI LAGI tapi KHUSUS buat command ini]).
  - Servo1/Servo2 trajectory JUGA diubah ke step-rate (`servo1StepUs/StepIntervalMs`, dst) —
    sama alasan kayak Hopper Sorter, gantiin `servo1IntervalMs` lama.
  - **Bug ditemukan & DIPERBAIKI**: `REQUEST_REFILL` dulu set `currentState=RUNNING_OR_MOVING`
    DI `applyCommand()` SEBELUM guard `PACKAGE_READY_FLAG`/`LIM_STOCK_EMPTY` dicek (guard ada
    di dalam `handleRefillFSM()`). Kalau ditolak, `refillState` tetap IDLE tanpa
    fault/sinyal apapun, terus `updateRefillJoin()` langsung balikin `currentState` ke IDLE
    lagi SEKETIKA — Orange Pi poll STATE abis kirim command cuma lihat IDLE lagi, kesannya
    command selesai/diabaikan padahal `refillRequested` nyangkut `true` diam-diam di
    background. **FIX**: semua guard dipindah ke `applyCommand()`, reject SELALU
    instan & eksplisit sekarang.
  - Menu kalibrasi total 18 entry sekarang (`Force Servo Delay(ms)` ditambahin di akhir,
    sebelum Reset).
- **CATATAN FISIK BELUM TERVERIFIKASI**: `FORCE_MIDDLE_REFILL` jalanin conveyor DAN servo
  BARENGAN (delay-based, conveyor duluan baru servo nyusul) menuju sensor yang SAMA (PROX_2)
  — kalau timing keduanya bikin 2 package "ketemu" di titik fisik yang sama, bisa nabrak.
  **Belum ada cara verifikasi ini tanpa hardware asli** — test pelan-pelan dulu (conveyorSpeed
  rendah) kalau mau coba command ini.

### PICKER (`ESP32_SortingAutomation_Picker`)
- Fix yang sama: Motor DC PWM (channel A/AIN doang, no chip TB6612 sebenernya di Picker —
  fix ini tetap valid buat kalau chip fisik ada), Stepper pinMode (juga gak ada stepper asli
  di Picker, sama alasan), direction cycle, AIN swap, PROX swap (channel-channel ini emang
  universal tapi gak dipakai produksi Picker).
- **Fitur baru — `MOVE_PACKAGE`** (opcode 8): urutan penuh
  `home → PACKAGE_PICKUP(pose4) → pick → LIFT_LOAD(pose5) → place → home`.
  - `POSES[4]` jadi `POSES[6]` — 2 pose baru: slot 4=`PACKAGE_PICKUP` (ambil package dari
    UJUNG Dispenser), slot 5=`LIFT_LOAD` (taruh di Load Position STOCKER).
  - **BELUM DIKALIBRASI** — default masih sama posisi home (aman, gak akan gerak ekstrem,
    tapi juga berarti kalau dipanggil sebelum kalibrasi, secara fisik gak ngapa-ngapain
    berarti, cuma home→home→home).
  - Cara kalibrasi: menu LCD (Setting Kalibrasi > Pose > pilih joint > JOG > `#` > slot 4/5)
    atau serial (`JOG <joint> <us>` lalu `SAVEPOSE 4` / `SAVEPOSE 5`).
  - Serial shortcut baru: `MOVEPKG`. Menu kalibrasi slot sekarang 0-5 (dulu 0-3).

### LIFTER / STOCKER (`ESP32_SortingAutomation_Lifter`)
- Fix yang sama: Motor DC PWM (no chip asli, tapi channel-nya tetap di-fix buat konsistensi),
  Stepper pinMode (STOCKER PUNYA stepper asli — sudah benar dari awal, gak ada bug di sini),
  direction cycle, AIN swap, PROX swap.
- Fix: `platformio.ini` kurang dependency `Adafruit PWM Servo Driver Library` — Test Modul
  Servo gagal compile sebelumnya (ini gak ketauan sampai saya coba build beneran).
- **TIDAK ADA perubahan logic inti** (homing, `RUN_FULL_CYCLE`, `GOTO_LOAD_POSITION`, dual-motor
  Z sync) — semua itu sudah benar dari sebelumnya, gak disentuh sesi ini.
- Command yang relevan buat orchestrator: `HOME_ALL`(1), `RUN_FULL_CYCLE`(2, arg=rack 0-5),
  `GOTO_LOAD_POSITION`(6). Register: `ALL_HOMED_FLAG`(11), `RACK_OCCUPIED_BITMASK`(12).
- **Penting**: setelah `RUN_FULL_CYCLE` selesai, X/Z OTOMATIS balik ke `loadPos` (BUKAN ke
  Home) — jadi orchestrator gak perlu kirim `GOTO_LOAD_POSITION` tiap siklus, cuma perlu
  sekali di awal (startup) setelah homing.

### File baru: `orangepi-orchestrator-batch.py`
- Master control script buat Orange Pi (Python + `minimalmodbus` + `pyserial`).
- `startup_sequence()`: cek `MIDDLE_PACKAGE_PRESENT` Dispenser (force-refill kalau kosong) →
  Picker home → Stocker home+load-position → Sorter start.
- `run_batch_sequence()`: `REQUEST_REFILL` → tunggu `PACKAGE_READY_FLAG` → `GOTO_LOAD_POSITION`
  → `MOVE_PACKAGE` → `ACK_PACKAGE_TAKEN` → `RUN_FULL_CYCLE` → `RESET_COUNTERS`.
- **Masih `DRY_RUN=True`**, belum pernah nyentuh hardware asli. Register/opcode di script ini
  HARUS disinkronkan manual kalau ada perubahan lagi di firmware — gak ada mekanisme
  otomatis buat jaga sinkron.

---

## Yang Masih Terbuka / TODO (urutan prioritas)

1. **Upload semua firmware yang berubah ke board fisik.** Belum satupun yang di-flash.
2. **Tambah proteksi overcurrent per-channel TB6612** (polyfuse/PTC) sebelum test motor lagi
   — udah pernah ada 1 chip TB6612 gosong (overcurrent motor stall), fuse 8A yang ada
   sekarang cuma proteksi utama (short besar), BUKAN proteksi chip (rating cuma 1.2A
   kontinu/3.2A peak per channel). Belum ada info user udah nambah polyfuse atau belum.
3. **Kalibrasi fisik** (semua ini masih nilai default, belum diukur ke unit fisik):
   - Sorter: `palangSpeed/Dir/PushMs/RetractMs`, `hopperStepUs/StepIntervalMs`.
   - Dispenser: `servo1/2 StepUs/StepIntervalMs/HoldMs/StartUs/EndUs`, `conveyorSpeed`,
     `middleConfirmTimeoutMs`.
   - Picker: pose 4 (`PACKAGE_PICKUP`) & pose 5 (`LIFT_LOAD`) — WAJIB sebelum `MOVE_PACKAGE`
     dipakai produksi.
   - Stocker: `loadPos` (X/Z) harus align FISIK sama pose `LIFT_LOAD` Picker, `RACK[0-5]`,
     `pushExtendSteps`.
4. **Test `orangepi-orchestrator-batch.py` dengan `DRY_RUN=False`** terhadap hardware asli,
   step-by-step manual dulu sebelum lepas otomatis penuh.
5. **Verifikasi fisik `FORCE_MIDDLE_REFILL`** (resiko tabrakan conveyor+servo di PROX_2,
   lihat catatan di bagian Dispenser di atas).
6. Cek apakah user mau PR #1 di-merge, atau masih mau nambah/ubah sesuatu dulu.

---

## Konteks Fisik/Hardware yang Perlu Diinget

- **Power**: tiap node PSU 12V/40A, buck XL4015A→5V (servo + logic 1 rail sama — resiko
  brownout ESP32/MCP23017 kalau servo narik arus gede pas logic lagi kerja, sudah dikasih
  cap 2200uF buat mitigasi), AMS1117→3.3V (logic only). TB6612 VM langsung dari 12V
  (voltase aman, ~13.5V max rating chip), fuse 8A di jalur masuk 12V (1 fuse utama per
  node, BUKAN per motor).
- **Sensor proximity**: E18-D80NK, NPN open-collector aktif-LOW, wiring pull-up ke 3.3V
  (BUKAN ke 5V sensor) — jangan pasang resistor seri tambahan, itu bikin voltage divider
  yang bikin level LOW jadi ambigu (udah pernah dibahas & diperbaiki di awal sesi).
- **Arsitektur Modbus**: Orange Pi = MASTER, 4 node = SLAVE (Sorter=1, Picker=2, Dispenser=3,
  Stocker=4). RS485, 19200 baud, 8N1. Node gak pernah bisa saling ngomong langsung atau
  inisiasi komunikasi ke Orange Pi — semua lewat polling.

---

## File Referensi Lain di Repo

- `README.md` — overview arsitektur (SEBAGIAN SUDAH BASI, terutama soal Dispenser yang
  masih cerita mekanisme DC-motor-push lama, padahal sekarang servo 2-gerbang).
- `Readme_<Node>.md` di tiap folder node — sama, sebagian basi soal Dispenser & Sorter
  (palang) setelah redesign sesi ini. **Belum di-update** — kalau ada waktu, worth
  disinkronkan supaya operator baru gak kebingungan baca dokumentasi lama.
- `contoh-kirim-command 20260909 2350.py` — contoh minimal Modbus 1 node (Sorter doang),
  masih relevan sebagai referensi pola dasar (bukan orchestrator penuh).
- `ekspor-memori-sorting-automation 20260909 2350.md` — export memori sesi SEBELUM sesi ini,
  campur arsitektur lama (1 firmware gabungan) & baru — hati-hati bagian "Kelompok B" udah
  gak relevan.
