# Sorting Automation System

Sistem otomasi *sorting* & *palletizing* berbasis 4 node ESP32 yang berkomunikasi dengan Orange Pi (main controller) via **Modbus RTU/RS485**. Objek discan kamera HuskyLens, diklasifikasi (pass/reject), dipisahkan secara fisik, diambil lengan robot, lalu disimpan ke rak penyimpanan otomatis.

## Arsitektur

```
                         ┌─────────────┐
                         │  Orange Pi  │  ← HuskyLens (klasifikasi objek)
                         │ (RS485 Master) │
                         └──────┬──────┘
                                │ Modbus RTU (RS485, 19200 baud)
        ┌───────────┬──────────┼──────────┬───────────┐
        │           │          │          │           │
   ┌────▼────┐ ┌────▼────┐┌────▼────┐┌────▼─────┐
   │ SORTER  │ │ PICKER  ││ STOCKER ││Dispenser │
   │ Slave 1 │ │ Slave 2 ││ Slave 4 ││ Slave 3  │
   └─────────┘ └─────────┘└─────────┘└──────────┘
```

Keempat node **tidak** berkomunikasi langsung satu sama lain — semua koordinasi lewat Orange Pi sebagai Modbus Master, berdasarkan status yang di-poll berkala dari tiap node.

## Node Overview

| Node | Peran | Slave ID | Folder |
|---|---|---|---|
| **SORTER** | Conveyor + palang reject (klasifikasi pass/reject via Time-of-Flight) | 1 | [`firmware-sorter-step1/`](firmware-sorter-step1) |
| **PICKER** | Lengan robot 5-joint + gripper (pick & place, servo PCA9685) | 2 | [`firmware-picker-step1/`](firmware-picker-step1) |
| **Dispenser** | Conveyor2 + mekanisme push box (motor DC) | 3 | [`firmware-feeder-step1/`](firmware-feeder-step1) |
| **STOCKER** | Lift 3-axis (X/Y/Z stepper TMC2209) + rak penyimpanan 6 slot | 4 | [`firmware-stocker-step1/`](firmware-stocker-step1) |

Detail lengkap tiap node ada di README masing-masing folder.

## Hardware Umum (Semua Node)

- **MCU:** ESP32-WROOM-32 (DevKit V1)
- **I/O Expander:** 2× MCP23017 (`0x22`, `0x23`) — 32 channel digital I/O
- **RS485 Transceiver:** MAX13487E (auto-direction)
- **LCD:** 20×4 I2C (`0x21`) — opsional, hot-plug
- **Keypad:** 4×4 I2C (`0x20`) — opsional, hot-plug
- **1 PCB Universal** — desain board yang sama dipakai untuk keempat node, cuma beda komponen mana yang dipasang (populasi) dan wiring lapangan

## Channel Universal (Sama di Semua 4 Node)

| Channel | Fungsi |
|---|---|
| CH0 | `ESTOP` (input, aktif-HIGH, wiring NC fail-safe) |
| CH1 | `LED_RUN` |
| CH2 | `LED_FAULT` |
| CH3 | `LED_OPERATION` (heartbeat, berkedip terus selama firmware hidup) |
| CH4 | `LED_MANUAL` (nyala otomatis selama mode kalibrasi aktif) |
| CH5 | `BUZZER` |

Channel CH6-31 dipakai berbeda-beda tergantung node — lihat README masing-masing folder.

## Menu Kalibrasi (Sama Pola di Semua Node)

Diakses via tombol `*` di keypad:

```
* → a. Setting Kalibrasi   b. Test I/O   c. Test Command
A=naik  B=turun  C=pilih  D=kembali
```

| Kategori | Isi |
|---|---|
| **Setting Kalibrasi** | Parameter tersimpan NVS + Reset ke Default (konfirmasi 2 langkah) |
| **Test I/O** | Pengujian channel langsung (baca/tulis manual) |
| **Test Command** | Simulasi command Modbus untuk verifikasi opcode → aksi fisik |

## Build & Flash

Proyek pakai **PlatformIO**. Tiap node adalah proyek PlatformIO independen (folder terpisah).

```bash
cd firmware-<nama-node>-step1
pio run -t clean
pio run -t upload
pio device monitor -b 115200
```

## Skema Versi Firmware

```
v.MAJOR.MINOR.DDMMYYYY.HH.MM
```

Contoh: `v.01.00.25082026.21.17` — MAJOR/MINOR dinaikkan manual tiap rilis signifikan, timestamp diambil dari waktu build (WIB/GMT+7).

## Protokol Komunikasi

- **Modbus RTU** via RS485, 19200 baud, 8N1
- Command memakai pola `CMD_ARG` → `CMD_SEQ` → `CMD` (opcode ditulis terakhir) untuk mencegah eksekusi dobel akibat retry komunikasi
- PICKER dan STOCKER menerima **command 1-langkah** yang memicu seluruh urutan gerak otomatis — Orange Pi tidak perlu kirim command bertahap

## Struktur Repository

```
.
├── firmware-sorter-step1/
│   ├── include/    (config.h, registers.h, io_expander.h, keypad4x4.h)
│   ├── src/main.cpp
│   └── README.md
├── firmware-picker-step1/     (struktur sama)
├── firmware-stocker-step1/    (struktur sama)
├── firmware-feeder-step1/     (struktur sama, Dispenser)
└── README.md                   (dokumen ini)
```

## Prinsip Desain

- **Non-blocking** — seluruh logic gerak berbasis `millis()`/`micros()`, bukan `delay()`
- **Fail-safe by default** — boot dalam kondisi diam (IDLE), tidak ada gerakan otomatis tanpa command eksplisit
- **LCD & Keypad opsional**, hot-plug 2 arah — produksi tetap berjalan normal tanpa keduanya
- Setiap perubahan performa/bug diverifikasi 2 lapis (syntax-check compiler + audit urutan deklarasi) sebelum dirilis
