# PICKER — Firmware Terpisah (D30)

Node PICKER: robot arm **5 joint + 1 gripper** (6 servo via PCA9685) — bukan 6+1 seperti rencana awal.

## Struktur Project

```
firmware-picker-step1/
├── platformio.ini
├── include/
│   ├── config.h       ← pin fisik, alamat I2C, ServoCfg
│   ├── registers.h    ← alamat register Modbus & opcode command
│   ├── keypad4x4.h
│   └── io_expander.h
├── src/main.cpp
└── README.md
```

## Hardware & Alamat

| Item | Nilai |
|---|---|
| I2C SDA/SCL | GPIO21/22 |
| LCD (PCF8574) | `0x21` |
| Keypad (PCF8574) | `0x20` |
| MCP23017 #1 / #2 | `0x22` / `0x23` |
| PCA9685 (servo) | `0x40` |
| RS485 RX/TX | GPIO17/16, 19200 baud, Slave ID **2** |
| E-Stop | Aktif-**HIGH** (sama koreksi dgn SORTER, D33) |
| Servo | MG996R, pulsa 500-2500us |

### Channel MCP23017

| Channel | Fungsi |
|---|---|
| CH0-3 | ESTOP, LED_RUN, LED_FAULT, BUZZER (universal) |
| CH4 | PCA_OE (PCA9685 output-enable, active LOW) |

### Channel PCA9685 (Servo)

| Channel | Fungsi |
|---|---|
| 0-4 | Joint lengan 1-5 |
| 5 | Gripper |

`JOG <n>` = channel `n` langsung (tanpa indirection) — cek label silkscreen di board PCA9685 fisik.

**LCD & Keypad OPSIONAL** — sama seperti SORTER, produksi jalan tanpanya.

## Register Modbus

| Alamat | Nama | R/W | Isi |
|---|---|---|---|
| 0-6 | STATE/FAULT_CODE/CMD/CMD_ARG/CMD_SEQ/CMD_ACK_SEQ/HEARTBEAT | — | sama pola base seperti SORTER |
| 10 | CURRENT_POSE | R | pose terakhir tercapai |

## Command — Modbus, Serial, ATAU Menu LCD

| Command | CMD (Modbus) | Serial |
|---|---|---|
| Jalankan urutan penuh (home→target→pick→lift→**place**→home) | `1`, arg 1=pass/2=reject | `SEQ <1\|2>` |
| Goto pose manual | `2`-`4` | `GOTO <0-3>` |
| Pick / Place manual | `5` / `6` | — |
| Reset fault | `7` | `RESET_FAULT` |
| Jog 1 joint langsung | — | `JOG <0-5> <500-2500>` |
| Simpan pose sekarang | — | `SAVEPOSE <0-3>` |
| Atur kehalusan gerak | — | `STEP <1-500>` / `INTERVAL <5-200>` |
| Status lengkap | — | `STATUS` |

**Gerakan servo pakai step tetap per interval** (bukan interpolasi waktu) — default `STEP=20us`, `INTERVAL=20ms`. Joint dgn jarak lebih jauh otomatis butuh waktu lebih lama (tidak lagi selesai bersamaan persis).

**`RUN_SEQUENCE` (opcode 1 / `SEQ`)** — urutan lengkap: home → **naik sedikit (clearance)** → pass/reject → pick → lift → **place** (sebelumnya sempat hilang, sudah diperbaiki) → home. Ack ke Modbus **ditunda** sampai seluruh urutan benar-benar tuntas (bukan langsung setelah command diterima).

## Menu Kalibrasi LCD

```
* (dari IDLE) → "1=Pose 2=Pick 3=Place 4=Clear 5=Speed"
  → 1: pilih joint (0-5) → A+/B- jog → D=kunci → #+slot(0-3)=simpan
  → 2/3/4: jog NILAI DELTA (relatif thd home) → #=simpan
  → 5: pilih 1=Step/2=Interval → A+/B- jog → #=simpan
```

Semua nilai (`POSES[4]`, `PICK_OFFSET`, `PLACE_OFFSET`, `CLEARANCE_OFFSET`, `trajStepUs`, `trajStepIntervalMs`) **persist ke NVS**.

## Cara Uji Cepat

1. `pio run -t upload`, buka serial monitor
2. `HELP` → `STATUS`
3. `JOG 0 1500` → cek fisik joint0 bergerak
4. Kalibrasi 4 pose (home/pass/reject/lift) via `JOG`+`SAVEPOSE`, atau via menu LCD
5. `SEQ 1` → amati urutan penuh berjalan (termasuk clearance naik & place)
6. Tekan E-Stop → `STATUS` → `state=ESTOPPED`, semua servo langsung disable (`PCA_OE`)

## Troubleshooting (Riwayat Masalah yang Sudah Ditemukan)

| Gejala | Penyebab | Solusi |
|---|---|---|
| `undefined reference Keypad4x4::KEYMAP` | `static constexpr` array butuh definisi out-of-class (C++11/14) | Sudah ada `constexpr char Keypad4x4::KEYMAP[4][4];` di `main.cpp` |
| LCD blank total dari boot pertama, serial tampil `ESP32 RS485 Auto Direction Started` | **Firmware yang ter-upload BUKAN punya kita** (sketch lain) | `pio run -t clean` lalu upload ulang, pastikan project yang benar terbuka |
| Bus I2C macet total (`i2cWriteReadNonStop Error -1`) | Short/tertukar wiring SDA-SCL ke PCA9685 baru dipasang | Cabut kabel PCA9685, cek `[I2C-SCAN]` normal lagi → isolasi ke situ |

## Yang Masih Terbuka

Tidak ada open item besar tersisa — PICKER sudah lengkap (servo, menu, NVS, step-control).