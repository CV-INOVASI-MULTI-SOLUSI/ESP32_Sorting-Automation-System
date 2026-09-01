# SORTER — Firmware Terpisah (D30)

Node SORTER: Conveyor1 + Palang (solenoid relay) + Proximity. **Hopper belum aktif** (mekanisme fisik belum ditentukan).

## Struktur Project

```
firmware-sorter-step1/
├── platformio.ini
├── include/
│   ├── config.h       ← semua pin fisik & alamat I2C
│   ├── registers.h    ← alamat register Modbus & opcode command
│   ├── keypad4x4.h
│   └── io_expander.h
├── src/main.cpp
└── README.md
```

## Hardware & Alamat (Final, Terkoreksi dari Hardware Nyata)

| Item | Nilai |
|---|---|
| I2C SDA/SCL | GPIO21/22 |
| LCD (PCF8574) | `0x21` |
| Keypad (PCF8574) | `0x20` |
| MCP23017 #1 / #2 | `0x22` / `0x23` |
| RS485 RX/TX | GPIO17/16, 19200 baud, Slave ID **1** |
| Conveyor1 PWM | GPIO23 (native LEDC) |
| E-Stop | Aktif-**HIGH** (wiring NC + pull-up internal MCP) |

### Channel MCP23017

| Channel | Fungsi |
|---|---|
| CH0-3 | ESTOP, LED_RUN, LED_FAULT, BUZZER (universal) |
| CH4/5/6 | Conveyor1: BIN1/BIN2/STBY |
| CH7 | Palang: relay solenoid (1 kumparan, ON/OFF via ULN2803) |
| CH17 | Proximity pass sensor (NPN aktif-LOW) |
| ~~CH16~~ | ~~Limit switch hopper~~ — TBD, belum aktif |

**LCD & Keypad OPSIONAL** — kalau tidak terdeteksi saat boot (`[I2C-SCAN]`), produksi tetap jalan normal; kalibrasi tetap bisa lewat Serial.

## Register Modbus

| Alamat | Nama | R/W | Isi |
|---|---|---|---|
| 0 | STATE | R | 1=IDLE, 2=RUNNING, 3=FAULT, 4=ESTOPPED |
| 1 | FAULT_CODE | R | 0=normal, 5=ESTOP, 10=hopper jam(TBD), 20=MCP hilang, 99=test |
| 2 | CMD | W | opcode (lihat `registers.h`) |
| 3 | CMD_ARG | W | argumen command |
| 4 | CMD_SEQ | W | nomor urut command — naikkan tiap command baru |
| 5 | CMD_ACK_SEQ | R | balasan — cocokkan dgn CMD_SEQ utk tahu command selesai |
| 6 | HEARTBEAT | R | naik tiap detik |
| 10 | PASS_COUNT | R | — |
| 11 | REJECT_COUNT | R | — |
| 12 | CLASSIFY_IS_REJECT | W | 1=reject/0=pass → jadwalkan TOF trigger palang |

Urutan wajib tulis command: `CMD_ARG`(3) → `CMD_SEQ`(4) → `CMD`(2) TERAKHIR.

## Command — Modbus, Serial, ATAU Menu LCD (Semua Setara)

| Command | CMD (Modbus) | Serial | Menu LCD |
|---|---|---|---|
| Mulai conveyor | `1` | `START` | `6` (dari top menu) |
| Berhenti | `2` | `STOP` | `7` |
| Clear fault | `3` | `RESET_FAULT` | — |
| Set speed conveyor | `5` | `SPEED <0-255>` | `1` (jog) |
| Set arah conveyor | `6` | `DIR <0\|1>` | `2` (A=Forward/B=Reverse) |
| Reset counter | `7` | `RESET_COUNT` | — |
| Simulasi trigger palang (uji TOF) | `98` | `TEST_PALANG` | — |
| Simulasi fault | `99` | `TEST_FAULT` | — |
| Kalibrasi durasi palang | — | — | `3` (jog) |
| Kalibrasi jarak scan→palang | — | — | `4` (jog) |
| Kalibrasi kecepatan conveyor aktual | — | — | `5` (jog) |
| Status lengkap | — | `STATUS` | (baris 2-4 LCD, live) |
| Daftar command | — | `HELP` | — |

**Penting:** logic fisik (conveyor/palang/sensor/safety) **selalu jalan**, termasuk saat menu kalibrasi aktif — yang dijeda cuma command dari luar (Serial/Modbus). Jadi bisa `6`=RUN dari menu lalu kalibrasi speed sambil conveyor jalan.

## Menu Kalibrasi LCD

```
* (dari IDLE) → "1=Speed 2=Dir 3=Palang 4=Dist 5=Mm/s 6=RUN 7=STOP"
  → pilih 1/3/4/5 → A+/B- jog (C=step) → # SIMPAN ke NVS → * kembali
  → pilih 2 → A=Forward B=Reverse
  → 6/7 → langsung RUN/STOP (ditolak kalau masih FAULT/ESTOPPED)
```

Semua nilai kalibrasi (`conveyorSpeed`, `conveyorDir`, `palangPulseMs`, `distMm`, `mmPerSecAtMaxPwm`) **persist ke NVS** — bertahan setelah reboot.

## Cara Uji Cepat

1. `pio run -t upload`, buka serial monitor (115200)
2. `HELP` → `STATUS` → pastikan `state=IDLE`
3. `START` → `STATUS` → conveyor berputar fisik
4. Lambaikan tangan di proximity → `STATUS` → `pass` bertambah
5. `TEST_PALANG` → tunggu delay TOF → solenoid dorong-lepas otomatis, `reject` bertambah
6. Tekan E-Stop fisik → `STATUS` → `state=ESTOPPED`

## Troubleshooting (Riwayat Masalah yang Sudah Ditemukan)

| Gejala | Penyebab | Solusi |
|---|---|---|
| LED tidak nyala saat command dikirim, tapi baca-ulang MCP MATCH | Wiring LED (polaritas/pin), bukan firmware | Cek fisik LED & jalur ULN2803 |
| Proximity tidak berubah, selalu 3.3V | Sensor PNP disangka NPN | Ukur sensor pakai multimeter dulu |
| `Chip1`/`Chip2 TIDAK TERDETEKSI` | Alamat jumper A0-A2 salah / bentrok LCD-Keypad | Cek ulang alamat fisik vs `config.h` |
| E-Stop terbalik logic-nya | Wiring NC bukan NO | Sudah dikoreksi ke aktif-HIGH (D33), `INPUT_PULLUP` tetap dipakai |

## Yang Masih Terbuka

1. **Mekanisme hopper** — kode ditandai `TBD HOPPER`, aktifkan lagi setelah mekanisme fisik ditentukan.
2. **Kalibrasi TOF nyata** — nilai default masih perlu diukur ke unit fisik (§16.1a docs proyek).