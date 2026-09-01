# FEEDER — Firmware Terpisah (D30)

Node FEEDER: Dispenser box (motor DC push via TB6612FNG) + Conveyor2. Push **sensor-confirmed 2 sisi** (D22) — bukan lagi berbasis timer.

## Struktur Project

```
firmware-feeder-step1/
├── platformio.ini
├── include/
│   ├── config.h       ← pin fisik & alamat I2C
│   ├── registers.h    ← alamat register Modbus & opcode command
│   ├── keypad4x4.h
│   └── io_expander.h
├── src/main.cpp
└── README.md
```

## Hardware & Alamat (Dikonfirmasi Sama dgn SORTER/PICKER/STOCKER)

| Item | Nilai |
|---|---|
| I2C SDA/SCL | GPIO21/22 |
| LCD (PCF8574) | `0x21` |
| Keypad (PCF8574) | `0x20` |
| MCP23017 #1 / #2 | `0x22` / `0x23` |
| RS485 RX/TX | GPIO17/16, 19200 baud, Slave ID **3** |
| Conveyor2 PWM | GPIO23 (native LEDC) |
| E-Stop | Aktif-**HIGH** (D33) |

### Channel MCP23017

| Channel | Fungsi |
|---|---|
| CH0-3 | ESTOP, LED_RUN, LED_FAULT, BUZZER (universal) |
| CH4/5/6 | Dispenser push: AIN1/AIN2/STBY |
| CH7/8 | Conveyor2: BIN1/BIN2 (STBY dipakai bersama dgn dispenser — 1 IC fisik sama) |
| CH16 | LIM_STOCK_EMPTY |
| CH17 | LIM_BOX_ARRIVED |
| CH18 | LIM_PUSH_HOME |
| CH19 | LIM_PUSH_EXTENDED (D22 — baru, dulu tidak ada) |

**LCD & Keypad OPSIONAL** — sama seperti node lain.

## Alur Refill (FSM)

```
IDLE (nunggu REQUEST_REFILL)
  → CONVEYOR_RUN (conveyor2 jalan sampai LIM_BOX_ARRIVED)
  → PUSHING (dorong sampai LIM_PUSH_EXTENDED -- sensor-confirmed, D22)
  → RETRACT (mundur sampai LIM_PUSH_HOME -- sensor-confirmed)
  → DONE → kembali IDLE
```

`pushTimeoutMs` sekarang cuma **fallback fault-detection** (kalau limit switch tidak tersentuh dalam batas waktu = macet), bukan lagi penentu durasi dorong.

## Register Modbus

| Alamat | Nama | R/W | Isi |
|---|---|---|---|
| 0-6 | STATE/FAULT_CODE/CMD/CMD_ARG/CMD_SEQ/CMD_ACK_SEQ/HEARTBEAT | — | base pattern sama semua node |
| 10 | STOCK_EMPTY_FLAG | R | 1 = stok box kosong |

## Command — Modbus, Serial, ATAU Menu LCD

| Command | CMD (Modbus) | Serial | Menu LCD |
|---|---|---|---|
| Mulai refill | `1` | `REFILL` | `3` (top menu) |
| Reset fault | `2` | `RESET_FAULT` | — |
| Set speed conveyor2 | `3` | `SPEED <0-255>` | `1` (jog) |
| Set push timeout | — | `TIMEOUT <100-5000>` | `2` (jog) |
| Status lengkap | — | `STATUS` | (baris 2-3 LCD, live) |

**Logic fisik selalu jalan**, termasuk saat menu aktif — sama pola dengan SORTER.

## Menu Kalibrasi LCD

```
* (kapan saja) → "1=Speed 2=PushTimeout 3=Test REFILL"
  → 1/2: A+/B- jog (C=step) → # SIMPAN ke NVS
  → 3: langsung trigger REQUEST_REFILL dari menu
```

`conveyorSpeed`/`pushTimeoutMs` **persist ke NVS**.

## Cara Uji Cepat (Besok)

1. `pio run -t upload`, buka serial monitor (115200)
2. Amati `[I2C-SCAN]` → pastikan LCD(`0x21`)/Keypad(`0x20`)/MCP1(`0x22`)/MCP2(`0x23`) semua terdeteksi
3. `HELP` → `STATUS` → pastikan `state=IDLE`
4. **Tekan tiap limit switch fisik manual** dulu (LIM_STOCK_EMPTY, LIM_BOX_ARRIVED, LIM_PUSH_HOME, LIM_PUSH_EXTENDED), amati lewat `STATUS`/serial print — pastikan wiring benar SEBELUM coba gerak motor
5. `REFILL` → amati FSM jalan lewat serial (`[LOOP] refillState=...`) → harus berakhir `DONE` → kembali `IDLE`
6. Tekan E-Stop fisik → `STATUS` → `state=ESTOPPED`

## Troubleshooting (Warisan dari Node Lain — Kemungkinan Sama)

| Gejala | Kemungkinan Penyebab |
|---|---|
| LCD blank | Alamat I2C salah utk unit ini — cek `[I2C-SCAN]` |
| Bus I2C macet total | Wiring salah/short di salah satu device baru — cabut satu-satu utk isolasi |
| `REFILL` macet di `CONVEYOR_RUN` | `LIM_BOX_ARRIVED` tidak tersentuh — cek wiring/posisi switch |
| `REFILL` macet di `PUSHING`/`RETRACT` | `LIM_PUSH_EXTENDED`/`LIM_PUSH_HOME` tidak tersentuh, atau `pushTimeoutMs` terlalu pendek |

## Yang Masih Perlu Dikalibrasi

`conveyorSpeed` dan `pushTimeoutMs` masih nilai default generik — sesuaikan setelah lihat perilaku fisik mekanisme dispenser Anda.