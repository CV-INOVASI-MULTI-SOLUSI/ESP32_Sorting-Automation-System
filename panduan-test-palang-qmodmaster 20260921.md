# Panduan Uji PALANG Sorter dengan QModMaster

Dokumen ini untuk menguji palang (actuator penolak objek REJECT) di node SORTER
secara langsung dari PC, tanpa menjalankan script Python di Orange Pi.

Palang secara fisik adalah linear actuator (motor DC) di channel Motor A — bukan
solenoid. Ia harus di-drive aktif dua arah: maju (push) lalu mundur (retract).
Tidak ada pegas yang menariknya balik.

---

## 1. Sebelum mulai — satu bus, satu master

**RS485 hanya boleh punya SATU master pada satu waktu.** Kalau Orange Pi masih
menjalankan orchestrator atau script uji sementara QModMaster juga tersambung,
keduanya akan saling merusak: balasan tertukar, CRC gagal, dan hasil bacaan jadi
ngawur. Gejalanya menipu — kelihatan seperti node rusak, padahal cuma tabrakan.

Hentikan dulu apa pun yang jalan di Orange Pi:

```bash
ssh root@192.168.3.61
ps -eo pid,cmd | grep python3 | grep -v grep
kill <PID>
fuser -v /dev/ttyS3        # pastikan tidak ada yang memegang port
```

Yang dibutuhkan di sisi PC:

- Konverter USB–RS485 (CH340/FTDI/CP2102)
- Kabel A/B dari konverter ke bus yang sama dengan Sorter
- QModMaster (Windows/Linux, gratis)

Sambungkan A ke A dan B ke B. Kalau tidak ada balasan sama sekali, coba tukar —
penandaan A/B antar merek tidak selalu konsisten.

---

## 2. Setting koneksi

Di QModMaster: **Options → Modbus RTU** (atau ikon port serial).

| Isian | Nilai |
|---|---|
| Serial Port | COM sesuai konverter (lihat Device Manager) |
| Baud | **19200** |
| Data Bits | **8** |
| Stop Bits | **1** |
| Parity | **None** |
| Response Timeout | 1000 ms |

Lalu di jendela utama:

| Isian | Nilai |
|---|---|
| Slave Addr | **1** (Sorter) |
| Scan Rate | 500 ms kalau memakai mode Scan |

Tekan tombol **Connect**.

Slave ID node lain, kalau perlu: Picker 2, Dispenser 3, Stocker 4.

---

## 3. JEBAKAN PALING SERING — Base Address

QModMaster punya pengaturan **Base Address: 0 atau 1**. Semua alamat di dokumen
ini adalah alamat protokol **berbasis 0**, sama persis dengan yang ada di
`registers.h`.

- Base Address **0** → masukkan angka apa adanya. `STATE` = 0, `CMD` = 2.
- Base Address **1** → **tambahkan 1** ke setiap alamat. `STATE` = 1, `CMD` = 3.

Kalau semua bacaan terasa "bergeser satu" — misalnya alamat 0 memberi angka yang
seharusnya milik alamat 1 — inilah penyebabnya. Cek dulu ke sini sebelum
mencurigai firmware.

Semua register di sini adalah **Holding Register**. Fungsi yang dipakai:

- **FC3** — Read Holding Registers
- **FC6** — Write Single Register

---

## 4. Peta register Sorter

### Dibaca (FC3)

| Addr | Nama | Arti |
|---|---|---|
| 0 | `STATE` | 0=INIT 1=IDLE 2=RUNNING 3=FAULT 4=ESTOPPED |
| 1 | `FAULT_CODE` | 0 = sehat |
| 5 | `CMD_ACK_SEQ` | balasan nomor urut command |
| 10 | `PASS_COUNT` | objek lolos |
| 11 | `REJECT_COUNT` | **naik tiap palang mendorong** |
| 13 | `ACTIVITY_CODE` | 0=DIAM 1=CONVEYOR 2=**PALANG AKTIF** 3=MOTOR_A 4=TEST_HOPPER 90=FAULT 91=ESTOP |
| 17 | `MAIN_MODE_ACTIVE` | 1 = MAIN (produksi), 0 = TEST |
| 18 | `MENU_ACTIVE` | 1 = operator sedang di menu LCD |
| 19 | `REJECT_MISSED_COUNT` | reject yang gagal jadi dorongan |

> Register 18 dan 19 baru ada setelah Sorter di-flash dengan perbaikan
> 2026-09-20. Sebelum itu, membacanya menghasilkan error *illegal data address*.
> Itu normal, bukan kerusakan.

### Ditulis (FC6)

| Addr | Nama | Keterangan |
|---|---|---|
| 2 | `CMD` | opcode — **ditulis paling akhir** |
| 3 | `CMD_ARG` | argumen |
| 4 | `CMD_SEQ` | nomor urut, wajib naik terus |
| 12 | `CLASSIFY_IS_REJECT` | 1 = reject, 0 = pass |

### Opcode palang

| Opcode | Nama | Arg | Syarat |
|---|---|---|---|
| 98 | `TEST_TRIGGER_PALANG` | 0 | ditolak kalau MAIN aktif |
| 8 | `SET_MOTOR_A` | 0=stop 1=maju 2=mundur | ditolak kalau MAIN aktif **atau** palang sedang dipakai |
| 9 | `SET_PALANG_SPEED` | 0–255 | bebas, kapan saja |
| 2 | `STOP` | 0 | mematikan MAIN |
| 3 | `RESET_FAULT` | 0 | |

---

## 5. Langkah 1 — pastikan sambungan hidup

Baca dulu sebelum menulis apa pun.

1. FC = **Read Holding Registers (3)**
2. Start Address = **0**, Number of Registers = **2**
3. Tekan **Read**

Yang diharapkan: dua angka muncul. `STATE` biasanya 1 (IDLE), `FAULT_CODE` 0.

Kalau tidak ada balasan sama sekali: cek baud 19200, slave 1, polaritas A/B, dan
pastikan tidak ada master lain di bus.

---

## 6. Langkah 2 — cek tiga prasyarat

Ketiganya diam-diam membuat command tidak berpengaruh, jadi periksa dulu.

**a. Menu kalibrasi LCD** — baca addr **18**. Kalau `1`, operator sedang di menu
dan **semua command Modbus diabaikan tanpa dibalas**. Dari sisi master ini
kelihatan persis seperti node mati. Tekan `D` di keypad sampai kembali ke layar
utama.

**b. Fault** — baca addr **1**. Kalau bukan 0, command gerak akan ditolak.
Kirim `RESET_FAULT` (lihat langkah 4), atau lewat LCD: *Setting Kalibrasi →
Reset Fault*.

**c. Mode MAIN/TEST** — baca addr **17**. Kalau `1`, command uji akan ditolak.
Kirim `STOP` dulu.

---

## 7. Langkah 3 — dorong palang lewat jalur PRODUKSI

Ini jalur yang sesungguhnya dipakai saat produksi, dan yang paling sederhana:
cukup **satu** penulisan, tanpa urutan apa pun.

1. FC = **Write Single Register (6)**
2. Start Address = **12**
3. Value = **1**
4. Tekan **Write**

Register ini berdiri sendiri karena ditulis sangat sering — sekali per objek —
jadi sengaja dibuat semurah mungkin. Konsekuensinya **tidak ada ack**.

Jalur ini tidak dibatasi MAIN/TEST, karena memang input produksi dari
HuskyLens/Orange Pi.

**Verifikasinya** ada di langkah 6 — palang tidak mendorong seketika.

---

## 8. Langkah 4 — dorong palang lewat jalur TEST

Command ber-opcode wajib **tiga penulisan berurutan**. Firmware bereaksi saat
`CMD` ditulis, jadi argumen dan nomor urut harus sudah siap lebih dulu.

Semua memakai FC = **Write Single Register (6)**:

| Urutan | Address | Value | Keterangan |
|---|---|---|---|
| 1 | 3 | `0` | argumen |
| 2 | 4 | `7` | nomor urut — angka bebas, asal belum pernah dipakai |
| 3 | 2 | `98` | opcode, **paling terakhir** |

Lalu baca **addr 5** (FC3). Kalau berisi `7`, command sudah diproses.

Untuk command berikutnya, naikkan nomor urut jadi 8, 9, 10, dan seterusnya.
**Nomor urut yang diulang akan diabaikan** — firmware menganggapnya retry dari
command yang sama, supaya satu perintah tidak dieksekusi dua kali kalau ada
gangguan komunikasi.

> **Ack bukan berarti berhasil.** Firmware menulis `CMD_ACK_SEQ` juga untuk
> command yang baru saja ditolaknya. Ack hanya berarti "sudah sampai dan sudah
> diproses". Berhasil atau tidaknya dinilai dari efek fisik dan dari
> `REJECT_COUNT`.

---

## 9. Langkah 5 — jog motor palang langsung

Menggerakkan motornya mentah, tanpa siklus push/retract otomatis. Berguna untuk
memastikan arah kabel dan mekaniknya benar.

Maju:

| Urutan | Address | Value |
|---|---|---|
| 1 | 3 | `1` |
| 2 | 4 | `8` |
| 3 | 2 | `8` |

Mundur: ulangi dengan addr 3 = `2`, nomor urut `9`.

Berhenti: ulangi dengan addr 3 = `0`, nomor urut `10`.

**Motor tidak berhenti sendiri.** Selalu akhiri dengan arg `0`.

Command ini ditolak kalau palang otomatis sedang memakai Motor A — sedang
push/retract, atau ada reject yang menunggu giliran. Keduanya menulis ke output
fisik yang sama, jadi jog manual yang mengalah: produksi menang atas diagnostik.

---

## 10. Langkah 6 — verifikasi, dan kenapa harus diamati

**Palang tidak mendorong seketika setelah command dikirim.** Firmware
menjadwalkannya sejauh waktu tempuh objek — jarak scan ke palang dibagi
kecepatan conveyor. Jadi jedanya tergantung kalibrasi, bukan tetap.

Cara mengamatinya di QModMaster: pakai mode **Scan** (polling berkala) pada
addr **13** (`ACTIVITY_CODE`), scan rate 200–500 ms, sambil menjalankan langkah
3 atau 4.

Dua tanda palang benar-benar bekerja:

1. `ACTIVITY_CODE` (addr 13) sempat bernilai **2** = `CONVEYOR_JALAN_PALANG_AKTIF`
2. `REJECT_COUNT` (addr 11) **naik 1**

Kalau keduanya tidak terlihat, baca addr **19** (`REJECT_MISSED_COUNT`). Kalau
angkanya naik, berarti reject-nya dibuang — antrian penuh, atau gilirannya sudah
kedaluwarsa karena palang masih sibuk dengan dorongan sebelumnya.

Cara paling pasti tetap: colok USB ke Sorter, buka serial monitor 115200, dan
baca pesannya langsung.

---

## 11. Langkah 7 — atur kecepatan palang

| Urutan | Address | Value |
|---|---|---|
| 1 | 3 | `200` (PWM 0–255) |
| 2 | 4 | `11` |
| 3 | 2 | `9` |

Command ini **netral** — tidak dibatasi MAIN/TEST dan boleh dikirim kapan saja,
termasuk selagi produksi berjalan, karena ia hanya mengubah angka dan tidak
menggerakkan apa pun.

Tapi perubahannya **hanya di RAM**. Setelah reboot, nilainya kembali ke yang
tersimpan. Untuk permanen, simpan dari menu LCD (lihat bawah).

---

## 12. Yang tidak bisa diatur lewat Modbus

Dari semua parameter palang, **hanya kecepatan** yang punya opcode. Sisanya cuma
bisa lewat menu LCD di panel:

| Parameter | Item menu | Pengaruh |
|---|---|---|
| Palang Speed | 3 | sama dengan opcode 9 |
| Palang Dir | 4 | arah dorong |
| Palang Push (ms) | 5 | lama motor maju |
| Palang Retract (ms) | 6 | lama motor mundur |
| Dist TOF (mm) | 7 | jarak scan ke palang |
| Mm/s Max | 8 | kecepatan conveyor sebenarnya |

**Item 7 dan 8 yang menentukan jeda dorong.** Kalau palang terasa telat atau
terlalu cepat sehingga mengenai objek yang salah, dua angka inilah yang disetel
— bukan kecepatan palangnya.

Cara masuk menu di panel:

```
tekan *  →  Setting Kalibrasi  →  A/B pilih item  →  C masuk
A/B ubah nilai   C ganti besar langkah   # SIMPAN   D keluar
```

**Tanpa menekan `#`, nilainya hilang saat reboot.**

Ingat: selama berada di menu ini, Sorter mengabaikan semua command Modbus. Keluar
dulu sebelum melanjutkan uji dari QModMaster.

---

## 13. Kalau bermasalah

| Gejala | Kemungkinan | Tindakan |
|---|---|---|
| Tidak ada balasan sama sekali | dua master di satu bus | hentikan script di Orange Pi |
| | baud/slave/polaritas salah | cek 19200, slave 1, tukar A/B |
| | operator di menu LCD | baca addr 18; tekan D di keypad |
| Semua bacaan bergeser satu alamat | Base Address QModMaster = 1 | ubah ke 0, atau tambahkan 1 ke semua alamat |
| Ack masuk tapi palang diam | command ditolak | cek addr 17 (MAIN), lihat serial `[CMD] ... ditolak` |
| | masih dalam jeda waktu tempuh | tunggu, amati addr 13 |
| Illegal data address di addr 18/19 | firmware lama | flash Sorter dengan versi terbaru |
| Palang mendorong objek yang salah | kalibrasi TOF meleset | setel item menu 7 dan 8 |
| `REJECT_MISSED_COUNT` naik | antrian penuh / giliran basi | objek datang terlalu rapat untuk satu siklus palang |

---

## Ringkasan cepat

```
Koneksi : /dev/COMx  19200 8N1  slave 1  Base Addr 0

Dorong palang (produksi) :  FC6 addr 12 = 1

Dorong palang (test)     :  FC6 addr 3 = 0
                            FC6 addr 4 = <seq naik terus>
                            FC6 addr 2 = 98
                            FC3 addr 5 → harus sama dengan seq

Jog maju / mundur / stop :  addr 3 = 1 / 2 / 0, addr 2 = 8

Atur kecepatan           :  addr 3 = 0..255, addr 2 = 9

Verifikasi               :  addr 13 sempat 2, addr 11 naik 1
```
