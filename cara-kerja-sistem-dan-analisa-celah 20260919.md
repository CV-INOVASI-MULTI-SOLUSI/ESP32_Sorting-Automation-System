# Cara Kerja Sistem Sekarang & Analisa Celah/Dobel

Tanggal: 2026-09-19 (update: semua 7 temuan sudah diputuskan & 6 di antaranya sudah diperbaiki)
Cakupan: hasil audit menyeluruh terhadap firmware 4 node (Sorter, Dispenser, Picker, Stocker/Lifter) dan seluruh script Python di Orange Pi, setelah arsitektur MAIN/TEST mode ditambahkan (commit `ddc0226`) dan orchestrator produksi diperbaiki hari ini (belum di-commit).

**Status ringkas semua temuan (lihat detail di §6):**

| # | Temuan | Status |
|---|---|---|
| 1 | Tombol fisik Sorter nyelip masuk klasifikasi produksi | ✅ Diperbaiki -- tombol diubah fungsi jadi trigger test-cycle (TEST-gated), konsisten di Sorter/Picker/Dispenser |
| 2 | Orchestrator gak kirim STOP_MAIN pas Ctrl+C | ✅ Diperbaiki -- `shutdown_sequence()` baru |
| 3 | 3 script Dispenser tumpang tindih | ✅ Diperbaiki -- 2 script lama dihapus, sisa `full-protocol` |
| 4 | Komentar basi `orangepi-test-tool.py` | ✅ Diperbaiki -- direvisi nunjuk file yang beneran ada |
| 5 | Firmware belum di-flash ke hardware | ⏳ Ditunda -- user flash manual sendiri |
| 6 | Ambiguitas index rak | ✅ Diperbaiki -- dikonfirmasi cuma 4 rak fisik (Rak 1-4), firmware+orchestrator dibatasi |
| 7 | LCD belum tampilkan progress | ℹ️ TERNYATA SUDAH ADA -- fitur `[AUTO] activityText()` sudah lama jalan di ke-4 node, lihat §6 |
| 8 | Reset Fault gak gampang diakses lewat LCD | ✅ Diperbaiki -- jadi item pertama di `Setting Kalibrasi` ke-4 node |
| 9 | Picker punya fungsi reject yang gak kepake fisik | ✅ Diperbaiki -- `GOTO_REJECT`/`MENUJU_REJECT`/tombol BTN3 dihapus total dari Picker |
| 10 | Dispenser `ACTIVITY_CODE` buta terhadap `TEST_SERVO1/2_CYCLE` | ✅ Diperbaiki & dikonfirmasi fisik -- servo1 SEHAT dari awal, cuma register-nya yang gak lapor |
| 11 | Sorter `ACTIVITY_CODE` buta terhadap `TEST_HOPPER_CYCLE` (bug sama kayak #10) | ✅ Diperbaiki -- tambah `TEST_HOPPER_AKTIF` |
| 12 | Guard PROX_2 cuma jalan di poin 14/16, poin 8-13 gak dipantau | ✅ Diperbaiki -- guard reaktif diperluas ke servo watch + timing placeholder |
| 13 | Poin 14/16 "berhenti total" vs "lanjut" | ✅ Diputuskan -- lanjut, bukan berhenti (keputusan sempat dibalik) |
| 14 | Debounce cuma di counter, register live masih mentah | ✅ Diperbaiki -- `MIDDLE_PACKAGE_PRESENT` + `UJUNG_PACKAGE_PRESENT` ikut didebounce 2 detik di firmware |
| 15 | ESP32 Dispenser hang, OTA gagal 3x ("No response from the ESP") | ⚠️ BELUM SELESAI -- butuh power-cycle fisik, firmware device ketinggalan dari repo |
| 16 | Picker: komentar `POSES[6]` basi soal slot 2 "reject" | ✅ Diperbaiki -- slot 2 ditandai sisa/gak kepake |
| 17 | `[13]` cek PROX_2 sekali lalu nyerah, conveyor nyangkut mati selamanya | ✅ Diperbaiki -- self-healing |
| 18 | Guard universal spam command tiap 0.2 detik | ✅ Diperbaiki -- edge-triggered, sekali per episode |
| 19 | `[14]` punya celah sama kayak #17, plus konflik requirement di `[16]` | ✅ Diperbaiki -- `[14]` self-healing, `[16]` dikembalikan tanpa guard |
| 20 | Kondisi resume pasca-guard salah (nunggu sensor clear, harusnya "package full") | ✅ Diperbaiki -- `wait_package_full_placeholder()` |
| 21 | Sorter: `SET_MOTOR_A` rebutan Motor A dengan `handlePalangQueue()` | ✅ Diperbaiki -- guard `palangPending`/`palangState`; Picker & Stocker diaudit BERSIH |
| 22 | **Stocker: penomoran bit `RACK_OCCUPIED_BITMASK` geser 1 dari yang dibaca Orange Pi** | ✅ Diperbaiki -- konvensi baru bit N = Rak N |
| 23 | Stocker: FAULT tidak menghentikan gerakan (hanya ESTOPPED yang dijaga) | ✅ Diperbaiki -- `raiseFault()`/`haltMotion()` + guard loop |
| 24 | Stocker: E-stop tidak mereset `tgtPos`, dorongan Y tertinggal dilanjutkan nanti | ✅ Diperbaiki -- target disamakan ke posisi sekarang |
| 25 | Stocker: `faultCode` di-set tanpa `currentState`/`state` ikut FAULT | ✅ Diperbaiki -- semua jalur lewat `raiseFault()` |
| 26 | Stocker: `ackPending` menggantung saat fault, menimpa ack `RESET_FAULT` | ✅ Diperbaiki -- ack dituntaskan di `raiseFault()` |
| 27 | Stocker: `STOP_MAIN` tidak membatalkan siklus yang sedang jalan | ✅ Diperbaiki -- `abortCycle()`, retract Y dulu baru berhenti |
| 28 | Sorter: `onClassifyWrite()` menembus gate menu kalibrasi (§12.6) | ✅ Diperbaiki -- ikut ditolak saat menu aktif |
| 29 | Sorter: dorongan palang basi tetap dieksekusi, objek salah yang kena | ✅ Diperbaiki -- toleransi 400 ms, lewat itu dibatalkan & dicatat |
| 30 | Sorter: antrian klasifikasi penuh membuang REJECT diam-diam | ✅ Diperbaiki -- kedalaman 4→8 + `REJECT_MISSED_COUNT` |
| 31 | Sorter: antrian/`palangPending` tidak dibersihkan saat `STOP` | ✅ Diperbaiki -- `clearClassificationQueue()` |
| 32 | Sorter: hopper bersiklus nonstop selama RUNNING, tanpa kendali laju umpan | ✅ Diperbaiki -- `cfg.hopperCycleGapMs` + item kalibrasi baru |
| 33 | Sorter: cache `CONV1_STBY` desync setelah penulisan langsung (E-stop/OTA) | ✅ Diperbaiki -- cache ikut disinkronkan |
| 34 | Semua node: master tidak bisa bedakan "node di menu kalibrasi" vs "node mati" | ✅ Diperbaiki -- register `MENU_ACTIVE` baru di 4 node |
| 35 | Sorter: `Cmd::TEST_FAULT` (opcode 99) masih ada padahal harus dibuang | ✅ Diperbaiki -- dihapus dari enum, menu, Serial, dan Readme |
| 36 | Semua node: `LED_FAULT` tidak pernah dinyalakan otomatis | ✅ Diperbaiki -- dikelola `updateUniversalIndicators()` |
| 37 | Sorter/Picker/Stocker: buzzer bisa nyangkut nyala kalau fault/E-stop di tengah bunyi | ✅ Diperbaiki -- `updateBuzzerBeep()` dipindah keluar blok bersyarat |

---

## 1. Arsitektur Ringkas

```
                         Orange Pi PC Plus (MASTER)
                         orangepi-orchestrator-batch.py
                                    |
                      Modbus RTU (RS485, 19200 8N1)
        +--------------+--------------+--------------+--------------+
        |              |              |              |              |
   SORTER (id 1)   PICKER (id 2)  DISPENSER (id 3) STOCKER/LIFTER (id 4)
```

Semua node SLAVE, pasif nunggu command dari Orange Pi lewat 3 register standar:

1. Orange Pi tulis `CMD_ARG` (alamat 3) — argumen command (kalau ada).
2. Orange Pi tulis `CMD_SEQ` (alamat 4) — nomor urut naik terus, anti double-execute.
3. Orange Pi tulis `CMD` (alamat 2) — opcode, ini yang MEMICU eksekusi.
4. Node balas dengan nulis `CMD_ACK_SEQ` (alamat 5) = nilai `CMD_SEQ` yang barusan diproses.

Node WAJIB dikirim urutan ARG → SEQ → CMD (bukan sebaliknya), dan Orange Pi WAJIB nunggu `CMD_ACK_SEQ` match sebelum kirim command berikutnya.

---

## 2. Konsep MAIN/TEST Mode (kenapa dibikin)

**Masalah awal**: kalau program background node sendiri (auto-trigger, misal Dispenser yang otomatis ngisi ulang pas sensor tengah kosong) jalan BARENGAN dengan command manual/test yang dikirim Orange Pi, keduanya bisa rebutan output fisik yang sama (servo/motor) di waktu yang sama → gerakan fisik jadi kacau/nyangkut.

**Solusi**: setiap node sekarang punya flag `mainModeActive` (boolean, default **FALSE** = fail-safe saat boot/reset). Semua command dibagi 2 kubu:

- **Kubu MAIN** (produksi asli, dikirim Orange Pi pas jalan otomatis) — **DITOLAK** kalau `mainModeActive == false`.
- **Kubu TEST** (manual/jog/kalibrasi/debug) — **DITOLAK TOTAL** kalau `mainModeActive == true`.
- **Kubu NETRAL** (setting kecepatan/tuning parameter) — TIDAK di-gate sama sekali, boleh dipakai kapan saja (termasuk sambil produksi jalan, buat tuning live).

Command `START_MAIN`/`STOP_MAIN` (atau `START`/`STOP` khusus Sorter) adalah SATU-SATUNYA saklar yang mengubah `mainModeActive`. Kalau node ditolak eksekusi command, pesan penolakan HANYA muncul di Serial USB lokal (`[CMD] ... ditolak ...`) — **TIDAK ADA sinyal fault/error via Modbus**, cuma `CMD_ACK_SEQ` tetap ke-update normal (command dianggap "diterima" walau isinya ditolak diam-diam). Ini penting: Orange Pi **TIDAK BISA** tahu command-nya ditolak cuma dari ack — harus cek register `MAIN_MODE_ACTIVE` kalau curiga.

**Lapis tambahan (di luar MAIN/TEST)**: kalau LCD lokal salah satu node lagi buka menu kalibrasi (`menuState != NONE`), **SEMUA** command Modbus dari luar (apapun kubu-nya) ditolak diam-diam juga, sampai menu ditutup (tombol `D`) atau ke-timeout otomatis (hotplug-safety). Ini konsisten di ke-4 node.

---

## 3. Tabel Lengkap Gating Command per Node

### SORTER (slave id 1)

| Opcode | Command | Butuh mode | Catatan |
|---|---|---|---|
| 1 | START | (ungated, cuma cek fault) | **Efek**: set `mainModeActive = true` |
| 2 | STOP | ungated | **Efek**: set `mainModeActive = false` |
| 3 | RESET_FAULT | ungated | |
| 4 | SET_HOPPER_INTERVAL | **NETRAL** | tuning, boleh kapan saja |
| 5 | SET_CONVEYOR_SPEED | **NETRAL** | |
| 6 | SET_CONVEYOR_DIR | **NETRAL** | |
| 7 | RESET_COUNTERS | ungated | |
| 8 | SET_MOTOR_A | **TEST only** | ditolak kalau MAIN aktif, DAN ditolak kalau palang lagi punya job (lihat Temuan #21) |
| 9 | SET_PALANG_SPEED | **NETRAL** | |
| 10 | SET_HOPPER_STEP | **NETRAL** | |
| 97 | TEST_HOPPER_CYCLE | **TEST only** | |
| 98 | TEST_TRIGGER_PALANG | **TEST only** | |
| 99 | TEST_FAULT | ungated | test-only, komentar kode sendiri bilang "HAPUS sebelum produksi riil" — belum dihapus |
| reg 12 | **CLASSIFY_IS_REJECT** (tulis langsung, bukan lewat CMD) | ungated (SENGAJA -- ini jalur produksi asli HuskyLens/OrangePi, bukan kubu TEST) | aman, bukan celah |
| — | Tombol fisik BTN_TEST_HOPPER (BTN2) | **TEST only** ✅ | DIUBAH -- dulu simulasi "pass", sekarang trigger `TEST_HOPPER_CYCLE` |
| — | Tombol fisik BTN_TEST_PALANG (BTN3) | **TEST only** ✅ | DIUBAH -- dulu simulasi "reject", sekarang trigger `TEST_TRIGGER_PALANG` |
| — | Serial `TESTPASS` / `TESTREJECT` | **TEST only** ✅ | DIUBAH -- ditambah guard `mainModeActive` |

### DISPENSER / FEEDER (slave id 3)

| Opcode | Command | Butuh mode | Catatan |
|---|---|---|---|
| 1 | REQUEST_REFILL | **MAIN only** | |
| 2 | RESET_FAULT | ungated | juga paksa keluar TEST_LOOP kalau nyangkut |
| 3 | SET_CONVEYOR_SPEED | **NETRAL** | |
| 4 | SET_CONVEYOR_DIR | **NETRAL** | |
| 5 | ACK_PACKAGE_TAKEN | **MAIN only** | |
| 6 | FORCE_MIDDLE_REFILL | **MAIN only** | |
| 7-10 | SET_SERVO1/2_STEP[_INTERVAL] | **NETRAL** | |
| 11 | SET_CONVEYOR_ON_OFF | **TEST only** | |
| 12 | MOVE_SERVO1_TO | **TEST only** | |
| 13 | MOVE_SERVO2_TO | **TEST only** | |
| 14 | START_MAIN | ungated (cek fault + test-loop dulu) | |
| 15 | STOP_MAIN | ungated | |
| 97 | TEST_SERVO1_CYCLE | **TEST only** | |
| 98 | TEST_SERVO2_CYCLE | **TEST only** | |
| — | Tombol fisik BTN_TEST_BOX_FULL (Test Refill Loop) | **TEST only** ✅ sudah di-gate | aman |
| — | Auto-trigger `handleMiddleSensor()` (servo isi ulang otomatis) | **MAIN only** ✅ sudah di-gate | aman |

### PICKER (slave id 2)

| Opcode | Command | Butuh mode | Catatan |
|---|---|---|---|
| 1 | RUN_SEQUENCE | **MAIN only** | DIUBAH -- arg diabaikan, SELALU ke pose PASS (dulu arg 1=pass/2=reject) |
| 2 | GOTO_HOME | **TEST only** | |
| 3 | GOTO_PASS | **TEST only** | |
| ~~4~~ | ~~GOTO_REJECT~~ | DIHAPUS | Picker fisik cuma ambil dari PASS -- opcode 4 sengaja gak dipakai ulang (biar nomor di bawah gak geser) |
| 5 | PICK | **TEST only** | |
| 6 | PLACE | **TEST only** | |
| 7 | RESET_FAULT | ungated | |
| 8 | MOVE_PACKAGE | **MAIN only** | dipakai `run_batch_sequence()` |
| 9-10 | SET_TRAJ_STEP[_INTERVAL] | **NETRAL** | |
| 11 | START_MAIN | ungated | |
| 12 | STOP_MAIN | ungated | |
| — | Tombol fisik BTN_TEST_GOTO_PASS (BTN2) | **TEST only** ✅ | trigger `GOTO_PASS` |
| — | Tombol BTN3 | nganggur | DIHAPUS -- dulu trigger GOTO_REJECT, sekarang gak dipetakan (gak ada fungsi reject) |

### STOCKER / LIFTER (slave id 4)

| Opcode | Command | Butuh mode | Catatan |
|---|---|---|---|
| 1 | HOME_ALL | **ungated (sengaja)** | boleh MAIN atau TEST |
| 2 | RUN_FULL_CYCLE | **MAIN only** | arg rack_idx **1-4** (DIUBAH -- dulu 0-5, fisik cuma ada 4 rak) |
| 3 | MOVE_TO_RACK | **TEST only** | arg rack_idx **1-4** (sama, lihat di atas) |
| 4 | PUSH_BOX | **TEST only** | |
| 5 | RESET_FAULT | ungated | |
| 6 | GOTO_LOAD_POSITION | **ungated (sengaja)** | boleh MAIN atau TEST |
| 7-8 | SET_STEP_INTERVAL / SET_HOMING_STEP_INTERVAL | **NETRAL** | |
| 9 | START_MAIN | ungated | |
| 10 | STOP_MAIN | ungated | |

---

## 4. Alur Kerja Produksi Sekarang (`orangepi-orchestrator-batch.py`, versi hari ini — BELUM di-commit)

### 4a. `startup_sequence()` — jalan SEKALI di awal program

```
[0/5] PICKER   -> GOTO_HOME            (masih TEST mode, WAJIB sebelum START_MAIN Picker)
[1/5] DISPENSER-> START_MAIN
      DISPENSER-> cek MIDDLE_PACKAGE_PRESENT
                  kalau kosong -> FORCE_MIDDLE_REFILL -> tunggu MIDDLE_PACKAGE_PRESENT=1
[2/5] PICKER   -> START_MAIN
[3/5] STOCKER  -> cek ALL_HOMED, kalau belum -> HOME_ALL
      STOCKER  -> GOTO_LOAD_POSITION
      STOCKER  -> START_MAIN
[4/5] SORTER   -> START               (otomatis set mainModeActive=true di firmware Sorter)
=== semua 4 node MAIN mode aktif ===
```

Urutan Picker sengaja dibalik (GOTO_HOME dulu baru START_MAIN) karena `GOTO_HOME` itu command TEST, kalau START_MAIN Picker dikirim duluan maka GOTO_HOME-nya sendiri bakal ditolak.

### 4b. `run_batch_sequence()` — dipanggil berulang tiap `SORTER.PASS_COUNT` capai `BATCH_SIZE` (default 20)

```
[1/6] cari rack kosong lewat STOCKER.RACK_OCCUPIED_BITMASK -- kalau semua penuh, batch DITAHAN
[2/6] DISPENSER -> REQUEST_REFILL           (conveyor bawa package TENGAH -> UJUNG)
[3/6] tunggu DISPENSER.PACKAGE_READY_FLAG == 1
[4/6] STOCKER   -> GOTO_LOAD_POSITION       (idempotent)
[5/6] PICKER    -> MOVE_PACKAGE             (ambil dari UJUNG Dispenser, taruh Load Position)
[6/6] DISPENSER -> ACK_PACKAGE_TAKEN        (WAJIB, buka jalan refill berikutnya)
      STOCKER   -> RUN_FULL_CYCLE(rack_idx) (simpan ke rak)
      SORTER    -> RESET_COUNTERS           (batch berikutnya mulai 0)
```

Paralel di background (independen, tidak dipicu Orange Pi): begitu sensor TENGAH Dispenser kosong (`PROX_2` clear) DAN `mainModeActive == true`, servo1+servo2 otomatis jatuhin package baru ke TENGAH sendiri.

---

## 5. Daftar Semua Script Python & Fungsinya

| File | Fungsi | Status |
|---|---|---|
| `orangepi-orchestrator-batch.py` | **Produksi asli** — dijalankan terus-menerus, loop monitoring PASS_COUNT + trigger batch | Aktif, baru diperbaiki hari ini (MAIN_MODE), belum di-commit |
| `contoh-kirim-command 20260909 2350.py` | Contoh belajar dasar (START/STOP Sorter + tulis CLASSIFY_IS_REJECT) | Referensi lama, lihat Temuan #4 |
| `test-stocker-rack-sequence 20260918.py` | Test manual: AutoHome → Load Position → RUN_FULL_CYCLE rak 1,2,3,4 berurutan (pakai START_MAIN/STOP_MAIN sendiri) | Aktif, index rak 1-4 SUDAH dikonfirmasi benar |
| `test-dispenser-full-protocol 20260919.py` | Versi PALING detail (17 langkah persis permintaan), termasuk command servo titik awal/akhir buat loading manual pertama kali + cek MAIN/TEST mode otomatis di awal | Aktif (satu-satunya script Dispenser sekarang), **belum pernah dites ke hardware asli** |

~~`test-dispenser-manual-sequence 20260918.py`~~ dan ~~`test-dispenser-loop-cycle 20260918.py`~~ sudah **DIHAPUS** (tumpang tindih, kalah lengkap dari `full-protocol`).

---

## 6. TEMUAN CELAH & POTENSI DOBEL (status setelah diputuskan & diperbaiki)

**#1 — ✅ DIPERBAIKI — Tombol fisik Sorter nyelip masuk klasifikasi produksi**
Dulu `BTN_TEST_PASS`/`BTN_TEST_REJECT` (+ Serial `TESTPASS`/`TESTREJECT`) simulasi klasifikasi PERSIS SAMA dengan jalur produksi asli, TANPA gating `mainModeActive` -- bisa nyelip masuk antrian klasifikasi asli kapan saja, termasuk pas produksi jalan.
Keputusan: tombol diseragamkan jadi **trigger test-cycle mekanisme node sendiri, TEST-mode gated** (pola sama di semua node, seperti tombol Dispenser yang sudah benar sejak awal). Perubahan:
- **Sorter**: BTN2 (`BTN_TEST_HOPPER`) -> `TEST_HOPPER_CYCLE`, BTN3 (`BTN_TEST_PALANG`) -> `TEST_TRIGGER_PALANG` (simulasi reject). Keduanya ditolak kalau `mainModeActive` aktif. Serial `TESTPASS`/`TESTREJECT` juga ditambah guard yang sama.
- **Picker**: channel BTN2 dulu nganggur, sekarang trigger `GOTO_PASS` (`BTN_TEST_GOTO_PASS`), TEST-only. BTN3 awalnya juga dipetakan ke `GOTO_REJECT`, TAPI dicabut lagi (lihat Temuan #9) -- Picker fisik gak pernah reject.
- **Dispenser**: TIDAK diubah -- BTN2 (`BTN_TEST_BOX_FULL`) sudah TEST-gated sejak awal, sudah benar.
- **Stocker**: TIDAK diubah -- sudah punya Test Rak via LCD+keypad, channel BTN2/3 dibiarkan nganggur.
- `CLASSIFY_IS_REJECT` (register produksi asli dari HuskyLens/Orange Pi) SENGAJA TETAP ungated -- itu bukan kubu TEST, itu jalur produksi utama.
Firmware Sorter/Picker sudah `pio run` SUCCESS. **Belum di-flash ke hardware.**

**#2 — ✅ DIPERBAIKI — Orchestrator tidak kirim STOP_MAIN saat berhenti**
Ditambah `shutdown_sequence()` baru di `orangepi-orchestrator-batch.py`, dipanggil di blok `except KeyboardInterrupt` sebelum `sys.exit(0)` -- kirim `STOP` ke Sorter + `STOP_MAIN` ke Dispenser/Picker/Stocker (best-effort, fire-and-forget, gak nunggu ack biar shutdown gak nyangkut). Sekarang abis Ctrl+C, semua node balik ke TEST mode otomatis, script test manual bisa langsung dipakai lagi.

**#3 — ✅ DIPERBAIKI — 3 script Dispenser tumpang tindih**
`test-dispenser-manual-sequence 20260918.py` dan `test-dispenser-loop-cycle 20260918.py` **DIHAPUS** (kurang fungsional/lengkap dibanding versi terbaru). Sisa `test-dispenser-full-protocol 20260919.py` (ditulis ulang 2026-09-19, gantikan versi `20260918` -- isi protokol sama, ditambah cek/auto-fix MAIN/TEST mode di awal) sebagai satu-satunya script test Dispenser.

**#4 — ✅ DIPERBAIKI — Komentar basi di `contoh-kirim-command 20260909 2350.py`**
Referensi ke `orangepi-test-tool.py` (file yang gak ada) direvisi -- sekarang nunjuk ke `orangepi-orchestrator-batch.py` (produksi) dan `test-stocker-rack-sequence*.py`/`test-dispenser-full-protocol*.py` (test manual), yang beneran ada di repo.

**#5 — ⏳ DITUNDA (sesuai keputusan Anda) — Firmware baru belum di-flash ke hardware**
Sorter, Picker, Lifter (dengan SEMUA perubahan hari ini: MAIN/TEST mode + fix tombol + fix rak 1-4) sudah lolos `pio run` SUCCESS, tapi **belum di-flash ke fisik**. Dispenser fisik juga masih firmware LAMA (pre-MAIN/TEST). Anda akan flash sendiri secara manual.

**#6 — ✅ DIPERBAIKI — Ambiguitas index rak**
Dikonfirmasi: fisik cuma ada **4 rak** (Rak 1-4). Perbaikan:
- Firmware Stocker: `moveToRackXZ()` sekarang cuma terima rackIdx 1-4 (dulu 0-5) -- rackIdx 0/5 langsung `FaultCode::RACK_IDX_INVALID`.
- LCD "Test ke Rak": selector sekarang cuma tampilkan "Rak 1".."Rak 4" (dulu "Rak 0".."Rak 5").
- LCD "Simpan ke Slot Rak": keypad cuma terima digit 1-4 (dulu 0-5).
- Orchestrator `find_free_rack()`: loop dibatasi rak 1-4 (dulu 0-5 -- bug laten, bisa balikin "rak 0" yang gak ada fisiknya karena bit-nya emang selalu kosong).
- `test-stocker-rack-sequence 20260918.py`: komentar ambigu dihapus, `RACK_SEQUENCE=[1,2,3,4]` dikonfirmasi BENAR, tidak perlu ubah.

**#7 — ℹ️ TERNYATA SUDAH ADA — LCD progress produksi**
Dicek ke kode: fitur ini **sudah lama ada** di ke-4 node (bukan bagian perubahan hari ini) -- tiap node print `[AUTO] activityText()` + `State:...` (+ info tambahan spesifik node, mis. Pass/Reject count di Sorter, Refill state di Dispenser) ke LCD, refresh tiap 500ms, HANYA aktif saat `menuState == NONE` (otomatis berhenti update saat menu kalibrasi dibuka, jadi gak tabrakan sama layar kalibrasi). Jadi kalau LCD node nyala normal (gak lagi di menu kalibrasi), progress produksi SUDAH kelihatan di sana. Tidak ada perubahan kode diperlukan.

Catatan menu kalibrasi (bukan celah, cuma pengingat): kalau LCD salah satu node kepencet nyasar masuk mode kalibrasi pas produksi jalan, layar `[AUTO]` di atas otomatis berhenti (ketimpa layar kalibrasi) DAN semua command Modbus dari luar ditolak sampai menu ditutup (`D`) atau ke-timeout hotplug-safety. Kalau tiba-tiba 1 node "gak respon"/LCD-nya diam gak nampilin progress pas produksi, cek LCD fisiknya dulu.

**#8 — ✅ DIPERBAIKI — Reset Fault gak gampang diakses lewat LCD (ditemukan setelah dokumen ini pertama ditulis)**
Dicek: `RESET_FAULT` sebenarnya SUDAH bisa dipicu dari LCD di ke-4 node, tapi kepetak jauh -- cuma ada sebagai item ke-6/6 di menu `Test Command` (list generic "simulasi command node lain", `*` → `Test Command` → scroll → `C`), BUKAN di menu `Setting Kalibrasi` yang lebih sering dibuka. Konsisten di ke-4 node (bukan Stocker doang yang begitu).
Keputusan: `Reset Fault` dipindah jadi **item PERTAMA** di `Setting Kalibrasi` semua 4 node -- langsung eksekusi (`applyCommand(RESET_FAULT)`), gak perlu konfirmasi (non-destruktif). Alur baru: `*` → `Setting Kalibrasi` → langsung `C` (cursor default di situ). Semua nomor item lain di tiap node cuma geser +1, isinya gak berubah. Firmware ke-4 node sudah `pio run` SUCCESS.

**#9 — ✅ DIPERBAIKI — Picker gak pernah reject, fungsi GOTO_REJECT dihapus total**
Dikonfirmasi: Picker fisik cuma ambil objek dari jalur PASS (reject sudah ditangani hopper SORTER sebelum objek sampai ke Picker). Semua jejak reject dihapus:
- `Cmd::GOTO_REJECT` (opcode 4) dihapus dari `applyCommand()` -- opcode 4 SENGAJA gak dipakai ulang/renumber, biar `PICK`(5)/`PLACE`(6)/`MOVE_PACKAGE`(8)/`START_MAIN`(11)/`STOP_MAIN`(12) dst TETAP nomor yang sama (gak pecah kompatibilitas Modbus/orchestrator).
- `ActivityCode::MENUJU_REJECT` (value 3) dihapus, pose index 2 gak pernah dituju lagi.
- `Cmd::RUN_SEQUENCE` DIUBAH -- dulu arg 1=pass/2=reject nentuin tujuan, sekarang SELALU ke pose PASS apapun arg-nya (arg dibiarkan di signature demi kompatibilitas).
- Tombol fisik BTN3 (`BTN_TEST_GOTO_REJECT`, ditambah di Temuan #1) DICABUT lagi -- BTN3 balik nganggur.
- Item `"RUN_SEQUENCE(reject)"` dan `"GOTO_REJECT"` dihapus dari menu LCD `Test Command` (`CMD_TEST_COUNT` 9→7).
Firmware Picker sudah `pio run` SUCCESS.

**#10 — ✅ DIPERBAIKI — Dispenser `ACTIVITY_CODE` buta terhadap `TEST_SERVO1_CYCLE`/`TEST_SERVO2_CYCLE`**
Ditemukan sambil debug laporan "servo1 gak gerak pas ditest lewat script Python" (`test-dispenser-full-protocol`). Dibuatkan `diagnose-dispenser-servo1 20260919.py` buat intip `ACTIVITY_CODE` real-time -- hasilnya `STATE=IDLE`, `FAULT_CODE=0`, `MAIN_MODE_ACTIVE=0` (semua bersih), TAPI `ACTIVITY_CODE` tetap 0/DIAM terus walau command ke-ack.
Akar masalah BUKAN servo/hardware/guard produksi (`servoRefillStage`) -- itu semua kebukti bersih dari data yang sama. Akar masalahnya: fungsi `activityCode()` di firmware Dispenser **cuma pernah baca `testLoopStage`, `servoRefillStage`, dan `refillState`** -- `testServoCycleStage` (state machine internal buat `Cmd::TEST_SERVO1_CYCLE`/`TEST_SERVO2_CYCLE`, DAN buat live-preview di layar kalibrasi Servo Step/Interval) **gak pernah dicek sama sekali**. Efeknya: servo bisa aja beneran gerak normal, tapi `ACTIVITY_CODE` tetap nunjuk DIAM(0) -- diagnosa jarak jauh (tanpa Serial USB) jadi nyasar nyimpulin "gak jalan" padahal cuma registernya yang buta.
Fix: `activityCode()` sekarang juga cek `testServoCycleStage` (reuse kode `SERVO1/2_BUKA/TAHAN/TUTUP` yang sama, karena aksi fisiknya identik dgn `servoRefillStage` -- cuma beda pemicu, dan keduanya saling eksklusif). Firmware Dispenser sudah di-OTA ke `10.68.240.118` (2026-09-19).
**DIKONFIRMASI FISIK**: re-test `diagnose-dispenser-servo1` setelah OTA -- `ACTIVITY_CODE` ngikut SERVO1_BUKA(2)→SERVO1_TAHAN(3)→SERVO1_TUTUP(4)→DIAM(0) PERSIS bareng servo1 beneran gerak fisik (dikonfirmasi user). Firmware & servo1 SEHAT dari awal -- akar masalah "servo1 kelihatan gak gerak" murni bug visibility register ini, BUKAN bug fisik/kalibrasi.

---

## 7. SESI 2026-09-20 -- Debugging Dispenser Real-Hardware & Perbaikan Tambahan

Lanjutan sesi 2026-09-19, hasil test `test-dispenser-full-protocol` langsung ke hardware asli (Orange Pi + ESP32 Dispenser via RS485). Semua temuan di bawah **SUDAH di-commit** (`93805ec`, branch `claude/sorting-pipeline-fixes-and-2-proximity`, PR #1 ter-update).

**Temuan #11 — ✅ DIPERBAIKI — Sorter punya bug `ACTIVITY_CODE` yang SAMA kayak Temuan #10**
`startTestHopperCycle()` (opcode 97, TEST_HOPPER_CYCLE) sengaja gak pernah ubah `currentState` (tetap IDLE selama guard-nya butuh IDLE), tapi `activityCode()` Sorter return `DIAM` kalau `currentState != RUNNING_OR_MOVING` -- persis blind spot yang sama kayak Dispenser. Fix: tambah `ActivityCode::TEST_HOPPER_AKTIF` (value 4), dicek di `activityCode()` SEBELUM cek `currentState`. Sudah dicek juga Picker (aman, `startMoveAbs()` beneran ubah currentState) dan Stocker/Lifter (aman, Test Rak reuse state machine produksi) -- cuma Sorter yang bolong. `pio run` SUCCESS, **belum di-flash** (Sorter belum pernah di-OTA sesi ini).

**Temuan #12 — ✅ DIPERBAIKI — Guard PROX_2 diperluas ke SELURUH loop, bukan cuma poin 14/16**
User laporan real-test: pas loop lagi di poin 10-13 (servo round-trip), `PROX_2` kepencet lagi (package berikutnya dateng) tapi conveyor TETAP jalan -- guard sebelumnya cuma dipasang di poin 14/16. Fix di `test-dispenser-full-protocol`: `stop_conveyor_if_middle_triggered()` sekarang dipanggil BERULANG selama servo watch (`[10]`/`[12]`) dan tiap potongan `timing_placeholder` (`[8]`/`[9]`/`[15]`) -- reaktif, gak nunggu poin 14. `[9b]` (nyalain conveyor sebelum servo) SENGAJA tetap unconditional (kalau di-gate PROX_2 malah deadlock, package yang baru kedeteksi di `[6]` sendiri masih nempatin sensor). `[13]` pakai `conveyor_on_if_safe()` -- gak nge-undo kalau guard barusan matiin conveyor.

**Temuan #13 — ✅ DIPERBAIKI — Keputusan poin 14/16 dibalik: lanjut, bukan berhenti total**
Awalnya diputuskan "PROX_2 kepencet lagi -> conveyor stop -> loop berhenti total" (opsi break). Setelah dites, user balik keputusan: harus **lanjut** nunggu PROX_1 (opsi awal yang sempat ditawarkan). `wait_prox_with_middle_guard()` diubah -- conveyor tetap di-stop SEKALI (gak spam), tapi fungsi TETAP lanjut poll PROX_1 sampai trigger/timeout, gak `return False` langsung.

**Temuan #14 — ✅ DIPERBAIKI — Debounce 2 detik: awalnya cuma counter, diperluas ke live register**
Permintaan user: kalau `PROX_2`/`PROX_1` trigger lagi dalam 2 detik dari trigger terakhir (package permukaan gak rata, bisa kebaca sensor berkali-kali), diabaikan. Awalnya cuma `MIDDLE_ARRIVAL_COUNT` (counter arrival poin 6) yang didebounce. Diperluas ke:
- Register live `MIDDLE_PACKAGE_PRESENT` (PROX_2) -- filter debounce SAMA (2 detik) di titik commit-nya, bukan cuma di counter.
- Register live `UJUNG_PACKAGE_PRESENT` (PROX_1) -- ditambah debounce SIMETRIS lewat `updateUjungPresentDebounced()`, dulu ditulis mentah tiap loop tanpa filter sama sekali.
Semua guard di script Python (yang baca live register, bukan counter) otomatis ikut kebal noise TANPA perlu ubah kode Python -- filternya di sumber (firmware). `pio run` SUCCESS, **sudah di-flash** ke Dispenser TAPI lihat Temuan #15 di bawah (flash TERAKHIR gagal, firmware fisik sekarang mungkin cuma sampai Temuan #10/versi 15:04, belum tentu termasuk #12-#14).

**Temuan #15 — ⚠️ BELUM SELESAI — ESP32 Dispenser hang, OTA gagal 3x berturut ("No response from the ESP")**
Setelah beberapa siklus test, OTA ke `10.68.240.118` gagal terus meski `ping` ICMP 0% packet loss (network hidup, tapi service ArduinoOTA di ESP32-nya gak respon -- beda level). Kemungkinan besar ESP32 hang/nyangkut dari sesi test sebelumnya. **Belum di-power-cycle** -- ini alasan kenapa Dispenser di-hold, lanjut besok. Firmware TERAKHIR yang KEBUKTI ke-flash sukses: OTA jam 15:04 (debounce dasar utk MIDDLE_ARRIVAL_COUNT + MIDDLE_PACKAGE_PRESENT, TANPA extended-debounce PROX_1 & TANPA fix "lanjut bukan berhenti total"). Jadi kode LOKAL sudah lebih maju dari firmware FISIK Dispenser sekarang.

**Temuan #16 — ✅ DIPERBAIKI — Picker: komentar `POSES[6]` basi soal slot 2 "reject"**
Array kalibrasi pose masih ada komentar "2=reject" walau `Cmd::GOTO_REJECT` udah dihapus total (Temuan #9). Dibersihkan -- slot 2 sekarang eksplisit ditandai SISA/GAK KEPAKE, array TETAP 6 slot (gak di-reorder, resiko lebih tinggi drpd manfaatnya karena slot 4/5 udah dipakai produksi).

**Perubahan non-kode (infrastruktur/konfigurasi):**
- `wifi_credentials.h` (gitignored, 4 node) -- SSID diubah dari `DD1010` ke `Raspberry R71`, password TETAP. **Compile-time** -- baru berlaku setelah masing-masing node di-flash ulang. **Efek samping**: IP tiap ESP32 bakal BEDA setelah pindah jaringan (`10.68.240.118` dkk gak berlaku lagi begitu Dispenser reconnect ke jaringan baru).
- Orange Pi (`192.168.3.61`) sempat unreachable total (SSH timeout) beberapa kali sesi ini -- kemungkinan terkait migrasi jaringan di atas atau masalah WiFi sementara. Kalau masih gak bisa diakses besok, cek fisik Orange Pi-nya duluan.
- Remote Control diaktifkan buat sesi Claude Code ini (`connect_new_sessions_to_remote_control` scope sesi, bukan account-wide) -- bisa dilanjut/dipantau dari claude.ai/code atau app mobile.
- Script `test-picker-manual-sequence 20260919.py` dibuat baru (GOTO_HOME→GOTO_PASS→PICK→GOTO_HOME→PLACE, `ensure_test_mode()`, `print_status()` tiap command) -- **belum pernah dites ke hardware**, Picker belum jadi fokus (rencana besok/setelah Dispenser kelar).
- Ditemukan (belum diputuskan): `diagnose-dispenser-servo1 20260919.py` sekarang borderline redundan -- fungsinya subset dari instrumentasi bawaan `test-dispenser-full-protocol` (print_status + watch_activity_s). Belum dihapus, nunggu keputusan.

---

## 8. Status Saat Ini & Rencana Lanjutan (per 2026-09-20)

**Dispenser -- DI-HOLD, lanjut besok:**
1. Power-cycle fisik ESP32 Dispenser (matiin-nyalain) -- WAJIB sebelum OTA bisa jalan lagi.
2. OTA firmware terbaru (sudah ada Temuan #11-#14, commit `93805ec`) -- pastikan IP masih sama SEBELUM migrasi WiFi SSID, atau cari IP baru kalau ternyata udah reconnect ke `Raspberry R71`.
3. Test ulang `test-dispenser-full-protocol 20260919.py` end-to-end, fokus verifikasi: guard PROX_2 di poin 10-13 (Temuan #12), lanjut-bukan-berhenti di poin 14/16 (Temuan #13), debounce PROX_1 (Temuan #14).

**Picker -- rencana test berikutnya:**
- Jalankan `test-picker-manual-sequence 20260919.py` (belum pernah dites ke hardware), user berencana test satu-satu step.
- Firmware Picker (fix GOTO_REJECT + tombol + Reset Fault + komentar pose) sudah `pio run` SUCCESS, **belum pernah di-flash sama sekali** sesi ini -- perlu OTA/flash dulu sebelum test.

**Sorter -- pending:**
- Firmware (fix tombol celah #1, ActivityCode test-hopper Temuan #11, Reset Fault) `pio run` SUCCESS, **belum pernah di-flash** sesi ini.

**Stocker/Lifter -- pending:**
- Firmware (rack 1-4, Reset Fault) `pio run` SUCCESS, **belum pernah di-flash** sesi ini.
- Investigasi timing gap Test Rak (31.64s vs kalkulasi 23.38s) -- instrumentasi `[CYCLE-TIMING]` masih nunggu hasil test baru, belum tersentuh sesi ini.

**Git:** semua perubahan kode+script sesi 2026-09-19 & 2026-09-20 SUDAH di-commit (`93805ec`, `f62093d`) & di-push ke `claude/sorting-pipeline-fixes-and-2-proximity` (PR #1). Dokumen `.md` dan `.docx` TETAP lokal (gak di-commit, sesuai kebiasaan sesi).

---

## 9. Temuan #17-19 -- Analisa Ulang Lewat Flowchart (2026-09-20, lanjutan)

Diminta bikin flowchart alur `test-dispenser-full-protocol` buat cek ulang ada celah race-condition atau enggak. Nelusurin tiap titik guard PROX_2 satu-satu ketemu 3 celah, semua udah diperbaiki & di-commit (`f62093d`):

**Temuan #17 — ✅ DIPERBAIKI — `[13]` cek PROX_2 sekali lalu nyerah, conveyor nyangkut mati selamanya**
`conveyor_on_if_safe()` dulu: kalau `PROX_2` masih trigger pas mau nyalain conveyor lagi, cuma DITAHAN sekali -- gak pernah dicoba nyalain lagi setelahnya. Akibatnya package 1 gak pernah kebawa maju, `[14]` nunggu `PROX_1` yang GAK AKAN PERNAH trigger (conveyor mati permanen) sampai timeout 120 detik baru SELURUH SCRIPT ke-abort. Fix: sekarang nunggu `PROX_2` clear dulu (pakai `wait_prox`, timeout sama dgn `PROX_WAIT_TIMEOUT_S`), baru nyalain conveyor -- self-healing, gak butuh restart manual buat kasus dua package numpuk deket-deketan.

**Temuan #18 — ✅ DIPERBAIKI — Guard universal spam command tiap 0.2 detik**
`stop_conveyor_if_middle_triggered()` dulu level-triggered murni -- kirim `SET_CONVEYOR_ON_OFF(0)` SETIAP kali dipanggil selama `PROX_2` masih terbaca 1, bukan cuma sekali. Dipanggil tiap 0.2 detik dari servo watch `[10]`/`[12]` dan tiap potongan `timing_placeholder`, jadi selama `PROX_2` trigger berkepanjangan bisa spam puluhan command Modbus beruntun. Fix: ditambah flag edge-triggered (`_middle_guard_already_stopped`, reset begitu `PROX_2` balik clear) -- sekarang cuma kirim sekali per episode trigger.

**Temuan #19 — ✅ DIPERBAIKI — `[14]` punya celah SAMA kayak Temuan #17, plus konflik requirement di `[16]`**
`wait_prox_with_middle_guard()` (dipakai `[14]` DAN `[16]`) dulu: begitu `PROX_2` trigger, conveyor di-stop sekali (via flag `middle_guard_fired`), lalu CUMA lanjut nunggu kondisi utama tanpa pernah nyalain conveyor lagi -- persis kelas masalah Temuan #17, cuma bedanya versi ini "lanjut nunggu" alih-alih langsung `break` (sesuai Temuan #13 kemarin), tapi ujung-ujungnya tetap timeout 120 detik juga karena conveyor gak pernah jalan lagi.
Sekaligus ketauan **konflik requirement**: instruksi PALING AWAL soal poin `[16]` eksplisit bilang "prox_1 sudah 0 tapi prox_2 belum 0 -> conveyor tetap maju" -- tapi Temuan #12 kemarin keliru nerapin guard universal ke `[14]` DAN `[16]` sekaligus lewat fungsi yang sama, padahal `[16]` dari awal punya pengecualian sendiri.
Keputusan user: `[14]` dibikin self-healing (stop -> tunggu `PROX_2` clear -> nyalain lagi -> lanjut nunggu `PROX_1`), `[16]` DIKEMBALIKAN ke `wait_prox` polos TANPA guard PROX_2 sama sekali (conveyor tetap nyala apapun status `PROX_2`, sesuai instruksi asli).

**Batasan yang TETAP ada (desain, bukan bug):** dua package perlu jarak fisik minimal ~2 detik (window debounce firmware) biar kehitung sebagai kedatangan terpisah.

**Temuan #20 — ✅ DIPERBAIKI (commit `62cac97`) — Kondisi resume conveyor pasca-guard salah: nunggu sensor clear, harusnya nunggu placeholder "package full"**
Klarifikasi user: begitu `PROX_2` kedeteksi (debounced, beneran baru) di tengah cycle MANAPUN, aturannya "apapun yang terjadi, tunggu command package full untuk menjalankan lagi" -- BUKAN nunggu sensor `PROX_2` balik clear kayak yang saya implementasikan di Temuan #17/#19. Dua hal ini beda: sensor bisa clear duluan sebelum "package full" beneran terkonfirmasi (placeholder timing [8]/[9]).
Fix: `conveyor_on_if_safe()` (poin [13]) dan `wait_prox_with_middle_guard()` (poin [14]) sekarang manggil `wait_package_full_placeholder()` (helper baru, gabungan 2 timing_placeholder yang SAMA kayak poin [8]/[9]) sebagai syarat resume -- bukan `wait_prox(..., clear)` lagi. Akibatnya `conveyor_on_if_safe()` gak punya jalur "menyerah/timeout" lagi -- placeholder pasti selesai dalam waktu tetap, jadi selalu lanjut nyalain conveyor. Poin [8]-[9] alur utama direfactor pakai helper yang sama (satu sumber kebenaran).
Debounce PROX_2/PROX_1 dikonfirmasi TETAP 2 detik (sempat ditanya apa mau diubah ke 3 detik, dijawab tetap 2).

Kalau `PROX_2` beneran macet gak pernah clear (indikasi hardware), itu gak lagi relevan buat `[13]`/`[14]` -- keduanya sekarang cuma nunggu placeholder WAKTU TETAP (bukan status sensor), jadi gak ada lagi jalur timeout/abort di titik ini akibat sensor macet. Deteksi sensor macet tetap ada di tempat lain (`[6]`'s `wait_middle_arrival` dan `[16]`'s `wait_prox` polos, keduanya masih ada timeout 120 detik).

**Temuan #21 — ✅ DIPERBAIKI — Sorter: `SET_MOTOR_A` vs `handlePalangQueue()` rebutan Motor A fisik**
Diminta audit node lain (Picker, Stocker) buat celah serupa Dispenser (mekanisme background otomatis vs command manual rebutan output fisik, tanpa saling cek). Ketemu SATU di Sorter, kelas bug SAMA PERSIS dgn `servoRefillStage` vs `MOVE_SERVOx_TO` Dispenser (dari sesi sebelumnya):
`Cmd::SET_MOTOR_A` (jog manual Motor A, TEST-only) dan `handlePalangQueue()` (mekanisme produksi asli -- dorong reject via Motor A) SAMA-SAMA nulis ke output fisik Motor A yang sama (`CONV1_AIN1`/`CONV1_AIN2`/`LEDC_CH_MOTORA`), TANPA saling cek. `handlePalangQueue()` jalan TANPA gate `mainModeActive` sama sekali (dipanggil tiap loop tanpa syarat) karena jalur masuknya (`CLASSIFY_IS_REJECT`, register ditulis HuskyLens/Orange Pi) sengaja ungated sejak Temuan #1 (itu input produksi asli, bukan kubu TEST). Akibatnya: kalau Orange Pi kebetulan masih ngirim data klasifikasi pas Sorter lagi TEST mode, `handlePalangQueue()` bisa nyerobot Motor A di tengah operator lagi jog manual via `SET_MOTOR_A`, tanpa peringatan apapun.
Fix: `SET_MOTOR_A` sekarang nolak kalau `palangPending` true ATAU `palangState != PalangState::IDLE` (palang lagi ada job/jalan) -- pola guard SAMA kayak `isServoRefillAutoActive()` di Dispenser (lindungi manual command dari mekanisme produksi otomatis, bukan sebaliknya -- produksi tetap boleh jalan kapan saja). `pio run` SUCCESS, **belum di-flash**.
Audit Picker dan Stocker/Lifter: BERSIH, gak ada mekanisme background otomatis serupa -- Picker semua gerakan (manual & produksi) lewat 1 queue+trajectory engine yang sama; Stocker/Lifter Test Rak reuse `CycleStage` produksi yang sama persis. Gak ada jalur paralel yang bisa rebutan output fisik di dua node ini.

---

## 10. Audit Sorter & Stocker (2026-09-20) — Temuan #22-#37

Konteks: user sedang menguji Dispenser dan Picker, lalu meminta audit prediktif terhadap dua node yang belum disentuh (Sorter dan Stocker/Lifter). Semua temuan di bawah ditemukan lewat pembacaan kode, bukan dari gejala di lapangan — jadi belum satu pun terkonfirmasi fisik. Semuanya sudah diperbaiki dan `pio run` SUCCESS untuk keempat node, **belum ada yang di-flash**.

### 10.1 STOCKER / LIFTER

**Temuan #22 — Penomoran bit `RACK_OCCUPIED_BITMASK` geser 1 dari yang dibaca Orange Pi. INI YANG PALING PARAH.**
Firmware menulis `mask |= (1 << i)` dengan `i` = index array `RACK_LIM[]`, sedangkan array `RACK[]` sendiri sudah 1-based (`RACK[0]` tidak dipakai sejak Temuan #6). Artinya `RACK_LIM[0]` adalah limit switch fisik **Rak 1**, tapi statusnya ditaruh di **bit 0**. Orange Pi `find_free_rack()` membaca bit 1-4.
Akibatnya bit Rak 1 tidak pernah dibaca sama sekali: Rak 1 SELALU terlihat kosong, `find_free_rack()` selalu mengembalikan 1, dan seluruh box ditumpuk ke Rak 1 terus-menerus sejak batch pertama. Bit 4 yang justru dibaca itu `LIM_8`, channel yang tidak ada switch fisiknya (floating `INPUT_PULLUP`, selalu terbaca HIGH = "kosong").
Fix: konvensi disatukan jadi **bit N = Rak N** (bit1..bit4 = Rak 1..4, bit0 selalu 0) di `readRackOccupiedBitmask()` DAN `updateRackStatusTask()` (dua fungsi ini dulu sama-sama salah), scan dibatasi 4 rak fisik lewat `RACK_PHYSICAL_COUNT`. Sisi Orange Pi tidak berubah — kodenya memang sudah sesuai konvensi baru ini, jadi yang wajib di-flash adalah Stocker.

**Temuan #23 — FAULT tidak menghentikan gerakan apa pun.**
`loop()` hanya menjaga `currentState != ESTOPPED`, dan cabang FAULT di `handleCycle()` mengecek `state` (`LiftState`) bukan `currentState`. Sorter sudah lama menjaga keduanya (`!= ESTOPPED && != FAULT`), Stocker tidak.
Akibatnya COMM_TIMEOUT atau MCP23017 mati di tengah `RUN_FULL_CYCLE`: stepper terus melangkah dan siklus tetap selesai sampai habis, sambil register melaporkan FAULT ke master. Kasus terburuknya MCP mati saat homing — limit switch dibaca lewat MCP yang sama, jadi datanya tidak bisa dipercaya tapi axis tetap didorong sampai timeout 30 detik.
Fix: fungsi terpusat `haltMotion()` + `raiseFault()`, semua jalur pemicu fault (homing timeout, Y-retract gagal, rack invalid, belum homing, MCP mati, comm timeout) sekarang lewat `raiseFault()`, dan `loop()` menjaga FAULT juga.
**Catatan keselamatan yang disengaja:** `haltMotion()` TIDAK mematikan driver stepper. Axis Z vertikal — mematikan driver berarti melepas torsi tahan dan beban bisa jatuh. Jadi saat FAULT langkah dihentikan (target = posisi sekarang) tapi driver tetap energized. Mematikan driver tetap hanya dilakukan E-stop, yang memang tujuannya memutus daya. Ini mengubah perilaku lama pada homing-timeout, yang dulu memanggil `setStepperEnabled(false)`.

**Temuan #24 — E-stop tidak mereset `tgtPos`.**
`handleSafety()` hanya membersihkan `cycleStage`/`yRetracting`. Kalau E-stop ditekan saat `PUSHING_Y`, `tgtPos[1]` tetap berisi `pushExtendSteps`. Begitu E-stop dilepas dan command gerak berikutnya menyetel `state = MOVING`, `updateSteppers()` ikut melanjutkan dorongan Y yang tertinggal itu — padahal tidak ada yang memintanya.
Fix: target disamakan ke posisi sekarang untuk ketiga axis, di `handleSafety()` dan `otaSafeStop()`.

**Temuan #25 — `faultCode` di-set tanpa `currentState`/`state` ikut FAULT.**
Terjadi di `moveToRackXZ()` (rack invalid / belum homing) dan `updateYRetract()` (PUSH_STUCK). Register STATE melaporkan IDLE atau RUNNING padahal FAULT_CODE bukan 0, `activityCode()` tidak pernah mengembalikan `FAULT_AKTIF`, tapi `blockIfFaulted()` tetap menolak semua command. Dari sisi master kelihatan "node IDLE tapi semua perintah ditolak tanpa alasan". Fix: lewat `raiseFault()`.
Ikut diperbaiki: cabang gagal pada rantai Test Rak di `loop()` dulu memaksa `currentState = IDLE` setelah `moveToRackXZ()` gagal — itu menghapus jejak fault yang baru saja dipasang.

**Temuan #26 — `ackPending` menggantung saat fault pada gerak non-cycle.**
`MOVE_TO_RACK`/`GOTO_LOAD_POSITION` menyetel `ackPending` tapi `cycleStage` tetap `NONE`, sehingga cabang pembersih di `handleCycle()` tidak pernah kena. Ack baru terkirim belakangan saat `RESET_FAULT` mengembalikan state ke IDLE — dan saat itu `CMD_ACK_SEQ` ditimpa dengan seq LAMA, sehingga master mengira `RESET_FAULT` tidak di-ack lalu mengulanginya. Jalur homing-timeout lebih parah lagi: `ackPending` hanya di-CLEAR tanpa pernah dikirim, master menunggu ack yang tidak akan pernah datang.
Fix: `raiseFault()` selalu menuntaskan ack yang menggantung. Konvensi protokolnya memang "ack = command diterima & selesai diproses", bukan "command berhasil" — master membedakan berhasil/gagal lewat FAULT_CODE.

**Temuan #27 — `STOP_MAIN` tidak membatalkan siklus yang sedang berjalan.**
Dulu hanya mematikan flag `mainModeActive`; `RUN_FULL_CYCLE` yang sedang jalan terus sampai selesai. Padahal `shutdown_sequence()` Orange Pi mengirim `STOP_MAIN` justru untuk menghentikan node, dan selain E-stop tidak ada satu pun cara menghentikan lift.
Fix: `abortCycle()`. Tidak langsung berhenti total — kalau dibatalkan saat pusher Y sedang menjulur, Y akan tertinggal di luar dan lift lalu bergerak di X/Z dengan pusher menjulur (bisa menabrak rak). Jadi X/Z dihentikan di tempat, lalu Y ditarik balik sampai limit, baru node dinyatakan IDLE. Tidak butuh opcode baru, orchestrator tidak berubah.

### 10.2 SORTER

**Temuan #28 — `onClassifyWrite()` menembus gate menu kalibrasi.**
`onCmdWrite()` menolak semua command saat `menuState != NONE` (§12.6), tapi callback `CLASSIFY_IS_REJECT` tidak punya pengecekan itu sama sekali — satu-satunya callback Modbus di keempat node yang tembus. Operator yang sedang mengkalibrasi, dengan tangan di area palang, tetap bisa membuat Motor A menghentak karena Orange Pi/HuskyLens mengirim hasil klasifikasi. Fix: helper `menuIsActive()` (forward-declared, karena `MenuState` didefinisikan jauh di bawah) dipakai untuk menolak juga.

**Temuan #29 — Dorongan palang yang sudah basi tetap dieksekusi.**
Antrian hanya diproses saat palang IDLE, padahal satu siklus palang (push + retract) memakan ~600 ms. REJECT berikutnya baru sempat diambil setelah itu, dan `palangTriggerAt` (= waktu scan + TOF) bisa SUDAH LEWAT — `now >= palangTriggerAt` langsung benar, palang menghentak seketika padahal objeknya sudah jauh melewati palang. Yang kena dorong justru objek di belakangnya, yang mungkin pass.
Fix: toleransi `PALANG_STALE_TOLERANCE_MS = 400`. Lewat dari itu dorongan dibatalkan dan dicatat, bukan dipaksakan. Sengaja longgar karena telat sedikit masih mengenai objek yang benar (palangnya tidak setipis itu).

**Temuan #30 — Antrian klasifikasi penuh membuang hasil REJECT diam-diam.**
`enqueueClassification()` dulu hanya `return` saat antrian penuh: tanpa counter, tanpa log, tanpa fault. Kalau yang hilang itu REJECT, objeknya lolos ke jalur pass dan tidak ada cara apa pun untuk tahu kejadiannya pernah terjadi.
Fix: kedalaman antrian 4 → 8, plus register baru `REJECT_MISSED_COUNT` (19) yang menghitung dua sebab sekaligus (antrian penuh + dorongan basi dari #29), ikut direset oleh `RESET_COUNTERS` dan ikut tampil di `STATUS` Serial.

**Temuan #31 — Antrian & `palangPending` tidak dibersihkan saat `STOP`.**
Conveyor sudah berhenti tapi palang masih bisa menghentak sendiri setelahnya, dan sisa antrian meletus sekaligus saat `START` berikutnya dengan TOF yang sudah lama kedaluwarsa. Fix: `clearClassificationQueue()` + `palangPending = false` di `Cmd::STOP`. Siklus palang yang SEDANG berjalan sengaja dibiarkan selesai — menghentikannya di tengah meninggalkan palang menjulur di atas conveyor.

**Temuan #32 — Hopper bersiklus nonstop selama RUNNING.**
`HopperState::AT_START` langsung lompat ke `MOVING_TO_PUSH` tanpa jeda sama sekali, jadi hopper dorong-tarik terus-menerus tanpa kendali laju umpan objek ke conveyor. Fix: `cfg.hopperCycleGapMs` (default 1000 ms, 0 = perilaku lama nonstop), ditaruh di akhir struct supaya blob NVS lama tetap terbaca, plus item kalibrasi baru "Hopper Gap Siklus(ms)" di index 16 — disisipkan SEBELUM "Reset ke Default" supaya index 0-15 tidak bergeser (`selParam` memakai angka yang sama).
Sekalian: komentar header file yang masih menulis "Hopper DINONAKTIFKAN sementara" dibuang, sudah basi jauh sebelum ini.

**Temuan #33 — Cache `CONV1_STBY` desync setelah penulisan langsung.**
`handleSafety()` (E-stop) dan `otaSafeStop()` menulis `CONV1_STBY` LOW langsung, melewati cache `lastConv1StbyState` milik `updateConv1Stby()`. Kalau saat itu cache berisi `true`, cache jadi berbohong: pin fisik LOW tapi software yakin sudah HIGH, jadi tidak akan pernah ditulis ulang. Gejalanya: OTA gagal di tengah jalan tanpa reboot → conveyor mati senyap (PWM tetap keluar, STBY-nya yang mati) sampai ada perubahan state yang kebetulan memaksa penulisan ulang. Fix: cache ikut disinkronkan di kedua tempat.

**Temuan #35 — `Cmd::TEST_FAULT` (opcode 99) dihapus.**
Komentarnya sendiri sudah menyuruh membuangnya sebelum produksi. Dihapus dari enum, `applyCommand()`, tabel Test Command LCD (`CMD_TEST_COUNT` 11 → 10), command Serial, teks HELP, dan tabel di `Readme_Sorter.md`. Opcode 99 sengaja dibiarkan kosong, jangan dipakai ulang untuk hal lain supaya script/master lama yang masih mengirimnya tidak memicu perintah yang berbeda arti.

### 10.3 Berlaku di semua node

**Temuan #34 — Master tidak bisa membedakan "node di menu kalibrasi" vs "node mati".**
Keempat node meng-ignore command Modbus saat menu kalibrasi aktif TANPA mengirim `CMD_ACK_SEQ`. Dari sisi master itu identik dengan kabel putus atau ESP32 hang. Fix: register `MENU_ACTIVE` baru di keempat node (Sorter 18, Picker 16, Dispenser 20, Stocker 18).

**Temuan #36 — `LED_FAULT` tidak pernah dinyalakan otomatis.**
Di keempat node channel ini di-`pinMode` OUTPUT di `setup()` dan terdaftar di menu Test Output, tapi tidak ada satu pun `io.write(CH::LED_FAULT, ...)` otomatis — `updateUniversalIndicators()` hanya mengurus OPERATION/RUN/MANUAL. Lampu fault praktis mati permanen selama produksi. Fix: dikelola di `updateUniversalIndicators()` (nyala saat FAULT atau ESTOPPED), dan entri tabel Test Output-nya diubah jadi `autoControlled: true` supaya toggle manual operator tidak langsung ditimpa balik.

**Temuan #37 — Buzzer bisa nyangkut nyala.**
`updateBuzzerBeep()` berada di dalam blok bersyarat `currentState != ESTOPPED` (Picker, Stocker) atau `!= ESTOPPED && != FAULT` (Sorter). Kalau fault/E-stop terjadi tepat di tengah bunyi, tidak ada lagi yang mematikannya dan buzzer meraung terus tanpa henti. Di Sorter komentarnya bahkan sudah menulis "TIDAK di-skip walau menu aktif" — benar untuk menu, tapi tetap ter-skip untuk fault. Fix: dipindah keluar blok di ketiga node (Dispenser sudah benar sejak awal).

### 10.4 Yang TIDAK disentuh

Logika alur Dispenser (`handleRefillFSM`, guard PROX_2, debounce, placeholder package-full) sengaja tidak diubah sama sekali — node itu sedang dalam pengujian aktif. Dispenser hanya menerima dua tambahan yang murni aditif dan tidak mengubah perilaku apa pun: register `MENU_ACTIVE` dan penulisan `LED_FAULT`.
