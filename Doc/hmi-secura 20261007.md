# HMI SECURA — panel LCD 320×240 (2026-10-07)

Lapisan tampilan baru di atas mesin produksi yang sama. Semua kode di `applib.py`, bagian 5 (LAYAR).

## 1. Arsitektur

```
sorting-automation.py ── jalankan_panel()
        │
        ├─ LayarILI9341      SPI, update hanya kotak yang berubah (tidak berubah)
        ├─ SentuhXPT2046     + Kalibrasi (touch_kalibrasi.json, format sama)
        ├─ AsetGambar        assets/*.jpg dibaca & dirapikan SEKALI saat start
        ├─ Bus               SATU-SATUNYA pemilik /dev/ttyS3
        │    ├─ Pemantau     thread: baca blok register tiap node 1x/detik (untuk layar)
        │    └─ jalankan_produksi()   thread produksi, TIDAK diubah
        └─ Panel             state machine boot + layar; tidak pernah membaca Modbus
                             langsung (command lewat thread kecil, hasil diamati)
```

Yang tidak berubah: Bus, periksa(), startup(), batch(), pantau(), shutdown(), HuskyLens,
rak, alarm_riwayat.json, kalibrasi sentuh, display.config `[lcd]`, mode `--uji/--cek/--tanpa-lcd`.

Satu tambahan kecil di mesin produksi: `STATUS['kamera']` (OK/GAGAL) diisi thread kamera
untuk indikator CAM.

## 2. Peta layar

| Menu | Isi |
|---|---|
| Boot | SPLASH → LOADING (4 langkah, %) → SYSTEM CHECK (10 item) → SYSTEM READY → menu |
| HOME | status mesin, alur mesin berupa IKON dengan status hidup (HOPPER › CONVEYOR › CAMERA › PUSHER, turun ke DISPENSER ‹ PICKER ‹ STOCKER), TOTAL/OK/REJECT/YIELD, throughput, cycle time, siklus, batch, rak, aksi berikutnya |
| AUTO | START / PAUSE / STOP / RESET, Cycle X/Y, Batch X/Y, rak, aktivitas tiap node, NEXT |
| MANUAL | kontrol manual per node (SORTER / DISPENSER / PICKER / STOCKER), STOP ALL, layar sensor live; CAMERA: LIVE, KLASIFIKASI + LIVE (terus, dicatat ke `kamera_hasil.csv`), tabel HASIL, DIAGNOSA PORT |
| ALARM | ACTIVE (dari keadaan node sekarang) / HISTORY (alarm_riwayat.json) → DETAIL (kode, node, waktu, deskripsi, tindakan, RESET FAULT) |
| SEGMENT | pilih segment (dulu RECIPE), 7 parameter, LOAD / SAVE / NEW / DELETE |
| MORE | NODE STATUS, HISTORY, SETTING, SYSTEM |
| NODE STATUS | 4 kartu → detail (field per node) → RAW register / COMMAND / RESET FAULT |
| HISTORY | DAILY (hari ini + 5 hari) / BATCH (riwayat batch) |
| SETTING | Production, Calibration, Rack, Display (baca saja), System, Engineering |
| SYSTEM | info Orange Pi, LOG, kalibrasi sentuh |

Tab bar: 6 menu, 4 terlihat; geser kiri/kanan atau ketuk panah di tepi.

## 3. Aset

| File | Dipakai di | Yang ditimpa gambar hidup |
|---|---|---|
| 01_splash.jpg | SPLASH; logo kecil header dipotong dari sini | tulisan versi "v1.0.0" |
| 02_loading.jpg | LOADING | progress bar, % dan ikon status 4 langkah |
| 03_system_check.jpg | SYSTEM CHECK, SYSTEM READY | kolom status 10 item, progress bawah |
| 04_bg_main.jpg | latar menu utama | header tercetak dibuang (dipotong) |
| 05_machine_diagram.jpg | TIDAK dipakai lagi -- HOME memakai ikon yang digambar program | - |
| 06_bg_panel.jpg | latar layar MORE | header tercetak dibuang |

Catatan aset: keenam JPEG adalah potongan lembar mockup — tepi atas/kiri membawa pita putih
berisi nama file lembar itu, dan sebagian berisi status/angka/jam yang tercetak mati. Pita putih
dihapus otomatis saat start (`AsetGambar._rapikan`), bagian dinamis ditutup dan digambar ulang.
Hasil terbaik: ekspor ulang aset bersih (tanpa pita, tanpa angka/status), nama file sama.
Tanpa folder `assets/` layar tetap jalan dengan latar polos.

## 4. Perubahan

- `applib.py`: AsetGambar, FAULT_INFO (dari enum FaultCode firmware), Resep, RiwayatProduksi,
  Panel baru, loop `jalankan()` tanpa `sleep()` (menunggu event dengan batas waktu terjadwal).
- `display.config`: `judul = SECURA`.
- `sorting-automation.py`: `--simulasi` memakai `produksi_riwayat_simulasi.json`.
- `assets/`: 6 JPEG. `.gitignore`: resep.json, produksi_riwayat*.json.

Keputusan yang perlu diketahui:
- **PAUSE** tampil tapi nonaktif: mesin produksi belum punya pause; menambahkannya mengubah
  perilaku produksi. Diketuk = penjelasan.
- **MANUAL** (menggantikan MAINTENANCE & tab NODE) hanya memakai command yang ada di OPCODES;
  urutan uji = beberapa command berurutan, tiap langkah diverifikasi dari bacaan node yang BARU.
  Harus diaktifkan dulu (konfirmasi area aman) tiap kali tab dibuka; terkunci selama produksi.
  - Sorter: conveyor hanya jalan di mode MAIN (START), jadi RUN = START + jeda hopper. PUSHER =
    Motor A (`SET_MOTOR_A`, arah mentah, panel mematikannya setelah durasi 0,1-3 s); `1x` =
    TRIGGER_PALANG, terverifikasi REJECT_COUNT naik. HOPPER 1 CYCLE (TEST), FEED/PAUSE (MAIN).
    CONV+PUSH, AUTO CYCLE (1 objek sampai PASS/REJECT naik), SENSOR.
  - Picker: pose HOME/READY/PICKUP/LIFT, 5 gerakan, PICK/PLACE, FULL CYCLE.
  - Dispenser: conveyor ON/OFF + speed, servo 1/2 AWAL/AKHIR/CYCLE, PACKAGE KE UJUNG, SENSOR.
    REFILL tidak ada di manual (firmware hanya menerimanya saat MAIN).
  - Stocker: HOME ALL, READY, RAK 1-4, PUSH BOX, RACK TEST 1-4. Ack Stocker datang setelah
    gerakan selesai, jadi batas tunggunya T_STOCKER_HOMING / T_FULL_CYCLE.
  - Speed conveyor tidak bisa dibaca balik dari firmware: yang tampil = nilai terakhir dikirim.
  - STOP ALL: batalkan urutan + Motor A Sorter 0, conveyor Dispenser OFF, STOP_MAIN semua node.
    Bukan pengganti E-STOP fisik.
  - Koreksi: versi sebelumnya memberi label "CONV FWD/REV" pada SET_MOTOR_A -- itu motor
    palang/pusher, bukan conveyor.
- **Verifikasi**: ack tidak dianggap selesai. Sesudah ack, layar mengamati STATE/ACTIVITY/tahap
  node 3 s; RESET FAULT membaca ulang FAULT_CODE.
- **Resep** hanya berisi nilai sorting.config yang sudah ada + kecepatan conveyor Sorter
  (SET_CONVEYOR_SPEED saat LOAD, kosong = tidak diubah). Nama resep baru otomatis "RESEP n";
  ganti nama di resep.json. Selama produksi semua tombol resep terkunci.
- **Riwayat produksi** dihitung panel dari counter Sorter yang sudah dibaca (tanpa bacaan bus
  tambahan); hanya tercatat selama program panel berjalan.

## 5. Kode

`applib.py` (bagian 5) dan file yang disebut di atas — lihat commit.

## 6. Pengujian

Tanpa perangkat keras (Windows/PC):
1. Render semua layar ke PNG dengan node tiruan (dipakai saat pengembangan).
2. Headless melawan `simulasi-node.py` (port virtual): boot + system check, ketuk tab bar,
   MANUAL: conveyor RUN/STOP, hopper, AUTO CYCLE, Picker FULL CYCLE, Stocker HOME/RAK, STOP ALL
   di tengah urutan; START → 1 siklus SELESAI,
   maintenance terkunci saat jalan, fault Dispenser → ALARM aktif → RESET FAULT terverifikasi.

Di Orange Pi:
1. Salin `applib.py`, `sorting-automation.py`, `display.config`, folder `assets/`.
2. `sudo systemctl stop sorting-automation` lalu `sudo python3 sorting-automation.py`.
3. Lihat urutan boot; SYSTEM CHECK harus menunjukkan 4 node OK bila semua menyala.
4. HOME: angka OK/REJECT naik saat objek lewat; titik status ikut berubah.
5. Tab bar: geser kiri/kanan, ketuk panah.
6. MANUAL saat berhenti: aktifkan, coba tiap node; lalu START dan pastikan MANUAL terkunci.
7. Cabut satu node: muncul di ALARM ACTIVE dan lonceng header.
8. `--kalibrasi` tetap membuka kalibrasi sentuh langsung.

## 7. Rollback

Semua perubahan ada di satu commit. Kembali ke versi sebelumnya:

```bash
git revert <commit-hmi-secura>
```

Atau di Orange Pi cukup salin kembali `applib.py` versi lama — file data (resep.json,
produksi_riwayat.json, folder assets/) diabaikan versi lama dan boleh dibiarkan.

## Lampiran: LED, buzzer & SW1 (INDIKATOR)

Satu thread untuk seluruh umur program (bukan hanya saat produksi). Pengaturan di
`sorting.config` `[pin]` dan `[indikator]`, juga di layar SETTING > System.

| Komponen | Pin (wPi / fisik) | Fungsi |
|---|---|---|
| LED OPR | 26 / 38 (BLUE) | heartbeat: kedip 0,1 s tiap 1 s selama loop program berdetak; macet > 3 s atau program berhenti = padam |
| LED RUN | 27 / 40 (GREEN) | nyala = produksi berjalan, kedip = pemeriksaan awal / startup |
| LED ALARM | 25 / 37 (RED) | nyala = fault / E-STOP / node tidak menjawab / ditahan / gagal; tetap menyala setelah program berhenti bila ada masalah |
| BUZZER | 20 / 31 (BUZ) | VALID 1x pendek, INVALID 2x pendek, alarm 2 s bunyi / 1 s diam berulang, boot 2x pendek = normal / 3x panjang = error |
| SW1 | `sw1 = none` (isi nomor wPi) | tekan singkat = diamkan buzzer alarm; tahan 3 s = STOP produksi (shutdown aman); ditahan saat program mulai = kalibrasi sentuh |

Buzzer alarm diam saat SW1 ditekan atau layar ALARM dibuka, dan berbunyi lagi bila muncul alarm BARU.
Heartbeat di mode panel berasal dari loop layar; di mode terminal (`--tanpa-lcd`) dari aktivitas bus.
Batas: kalau seluruh Orange Pi macet (kernel), GPIO berhenti di keadaan terakhirnya -- kedip yang
pendek (0,1 s) membuat LED OPR hampir selalu tertinggal padam.

## Lampiran: versi firmware & update OTA dari Orange Pi

Firmware v2.00 (keempat node) punya 7 register baca-saja berurutan mulai alamat
Sorter 25, Picker 19, Dispenser 27, Stocker 19:
FW_TAHUN, FW_BULAN_HARI, FW_JAM_MENIT (waktu build, otomatis dari compiler), FW_VERSI
(`FIRMWARE_VERSI` di `include/registers.h`, naikkan tiap rilis: 200 = v2.00), WIFI_IP_HI,
WIFI_IP_LO, WIFI_RSSI (diperbarui tiap 5 s). Beban: flash +~1 KB, RAM statis +0, tidak ada
kode baru di loop selain pembaruan IP/RSSI tiap 5 s.

Update OTA: MORE > SYSTEM > NODE > UPDATE FW. Orange Pi membaca IP node lewat RS485, mengirim
firmware dengan protokol ArduinoOTA (tanpa espota.py), lalu membaca ulang versi untuk memastikan
node benar-benar memakai firmware baru. Dikunci selama produksi; alarm "tidak menjawab" node
yang sedang di-update diredam.

1. PC: `pio run` di tiap folder node, lalu `python kumpulkan-firmware.py` -> folder `firmware/`.
2. Salin `firmware/` ke Orange Pi (folder program).
3. Orange Pi: buat `ota_password.txt` berisi password OTA (sama dengan OTA_PASSWORD di
   `wifi_credentials.h`). File ini & `firmware/` tidak ikut git.
4. Syarat jaringan: LAN Orange Pi dan WiFi ESP32 satu router/segmen.

Pertama kali (node masih firmware lama, belum melaporkan IP): flash v2.00 dari PC
(USB atau `pio run -e esp32dev_ota -t upload`). Setelah itu update bisa dari Orange Pi.

## Lampiran: uji kamera & kamera_hasil.csv

MANUAL > CAMERA > **KLASIFIKASI + LIVE** berjalan terus sampai STOP KLASIFIKASI, memakai fungsi
PRODUKSI yang sama (`HuskyLens.ambil_sampel` + `putuskan_reject` + `tunggu_kosong`). Tiap objek:
tunggu objek dikenali -> ambil `jumlah_sampel` sampel -> putuskan dengan `min_dominan` -> (palang) ->
buzzer/LED -> catat -> tunggu bidang pandang kosong `min_kosong_s` (objek yang sama tidak dihitung dua kali).
Bacaan LIVE (ID, jawaban/s) tetap tampil selama klasifikasi. "INVALID ke palang" boleh diubah saat berjalan.

Tombol **CONVEYOR** menjalankan conveyor Sorter (mode MAIN, hopper dijeda, sama dengan RUN di tab SORTER):
taruh objek di conveyor, objek lewat kamera dengan kecepatan produksi. **CONV OFF** / tab SORTER / STOP ALL
menghentikannya. Conveyor tetap jalan kalau pindah tab.

**Perbaikan sampling produksi (2026-10-08):** jendela `batas_ambil_s` dulu dimulai saat fungsi dipanggil,
ketika kamera masih kosong. Objek yang lewat di akhir jendela hanya mendapat 1-2 sampel, dan dengan
`min_dominan = 3` objek VALID ikut di-REJECT (simulasi objek 0,5 s di depan kamera: 3 dari 8 VALID
ter-REJECT). Sekarang jendela dihitung sejak objek PERTAMA terlihat dan bacaan pertama ikut dihitung:
8 dari 8 dapat 5 sampel.

Setiap hasil ditambahkan ke `kamera_hasil.csv` (folder program, titik koma, bisa dibuka Excel):

    No;Timestamp;Sampel;Hasil
    1;2026-10-08 14:30:05;1,1,1,1,1;VALID
    2;2026-10-08 14:30:09;1,1,0,0;INVALID

Sampel: 1 = ID VALID, 0 = ID INVALID; bisa kurang dari `jumlah_sampel` kalau objek cepat hilang.
Nomor berlanjut dari baris terakhir file. Tombol **HASIL** menampilkan 200 hasil terakhir (8 per halaman,
terbaru di atas). Hanya uji kamera yang dicatat, bukan produksi. File di-gitignore; hapus file untuk mulai dari No 1.

## Lampiran: backup & restore kalibrasi node (firmware v2.01, 2026-10-09)

Kalibrasi tiap node (isi NVS ESP32) bisa disimpan ke Orange Pi dan dimuat ke ESP32 pengganti.

- Panel: MORE > SYSTEM > NODE > **KALIBRASI**. Halaman langsung menjalankan CEK: kolom "vs NODE"
  membandingkan isi node sekarang dengan backup terakhir (SAMA / BERUBAH / belum backup / FW < v2.01).
- **BACKUP SEMUA**: kalibrasi keempat node -> `kalibrasi/<NODE>_<YYYYMMDD-HHMMSS>.json` (maks 30 per node).
- **RESTORE**: pilih baris node, pilih backup dengan `<` `>`, konfirmasi. Node menyimpan ke NVS, restart
  sendiri, lalu Orange Pi membaca ulang dan membandingkan CRC (terverifikasi).
- Ditolak selama produksi / MAIN aktif / node bergerak. Restore hanya diterima kalau format & ukuran
  gambar sama (firmware sama dengan saat backup). Node yang FAULT tetap bisa di-backup/restore.

Isi gambar kalibrasi (dari NVS, bukan nilai RAM sementara):

| Node | Isi | Ukuran |
|---|---|---|
| Sorter | SorterConfig (`sorter_cal/cfg`) | sizeof(cfg) |
| Dispenser | FeederConfig (`feeder_cal/cfg`) | sizeof(cfg) |
| Picker | gerakan 5x20 adegan, 4 pose, offset pick/place/clear/postplace, kecepatan, ramp, buzzer | ~508 B |
| Stocker | posisi rak, load position, push extend, kecepatan, homing, ramp, microstep, buzzer | ~73 B |

Protokol Modbus: `include/kalibrasi_modbus.h` (sama di 4 proyek). Register mulai Sorter 32,
Picker 26, Dispenser 34, Stocker 26 (FORMAT, UKURAN, CRC, OFFSET, HASIL, 32 register DATA);
command CAL_BACA 60, CAL_TULIS 61, CAL_TERAPKAN 62. Tidak ada beban saat produksi.

Ganti ESP32: flash firmware v2.01 lewat USB (ESP32 baru belum punya OTA) -> pasang -> MORE > SYSTEM >
KALIBRASI -> pilih node -> RESTORE.
