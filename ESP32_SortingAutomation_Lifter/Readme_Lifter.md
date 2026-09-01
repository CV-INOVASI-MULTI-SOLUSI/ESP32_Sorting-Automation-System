# STOCKER — Firmware Terpisah (D30)

Node STOCKER: Lift XYZ. **Y-axis ITU SENDIRI mekanisme pusher** (bukan motor DC terpisah seperti rencana awal). Z-axis 2 motor stepper sinkron via wiring paralel STEP/DIR (hardware sync).

## Struktur Project

```
firmware-stocker-step1/
├── platformio.ini
├── include/
│   ├── config.h       ← pin fisik, alamat I2C
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
| RS485 RX/TX | GPIO17/16, 19200 baud, Slave ID **4** |
| STEP_X/Y/Z | GPIO23/25/33 (native, WAJIB — presisi mikrodetik) |
| E-Stop | Aktif-**HIGH** (D33) |

**PENTING — wiring Z dual-motor:** `STEP_Z`/`DIR_Z` (dan `EN_STEPPERS`) di-fanout **paralel** ke 2 driver stepper fisik terpisah (bukan 1 driver ke 2 motor). Ini sinkron otomatis di level hardware — firmware memperlakukan Z sebagai 1 axis logis.

### Channel MCP23017

| Channel | Fungsi |
|---|---|
| CH0-3 | ESTOP, LED_RUN, LED_FAULT, BUZZER (universal) |
| CH4/5/6 | DIR_X / DIR_Y(pusher) / DIR_Z |
| CH7 | EN_STEPPERS (shared, semua stepper) |
| CH16/17/18 | LIM_X (kiri) / LIM_Y (posisi awal pusher) / LIM_Z (bawah) |
| CH19-24 | RACK_LIM_0 .. RACK_LIM_5 (6 limit switch rack) |

**LCD & Keypad OPSIONAL** — sama seperti SORTER/PICKER.

## Register Modbus

| Alamat | Nama | R/W | Isi |
|---|---|---|---|
| 0-6 | STATE/FAULT_CODE/CMD/CMD_ARG/CMD_SEQ/CMD_ACK_SEQ/HEARTBEAT | — | base pattern sama semua node |
| 10 | CURRENT_RACK_IDX | R | 0-5, atau `0xFF`=none/in-transit |
| 11 | ALL_HOMED_FLAG | R | 1 = X,Y,Z semua sudah homing |
| 12 | RACK_OCCUPIED_BITMASK | R | bit0-5 = status fisik 6 rack (dari limit switch, bukan asumsi software) |

## Alur `RUN_FULL_CYCLE` (Chaining Penuh)

```
1. Cek Y sudah di home (WAJIB, cegah tabrakan rak) -> kalau belum, ditolak (FAULT NOT_HOMED)
2. X + Z gerak SERENTAK ke posisi rak tujuan (Y tetap diam di home)
3. Y maju (extend) sejauh pushExtendSteps -- dorong box masuk rak
4. Y mundur (retract) SENSOR-CONFIRMED sampai LIM_Y tersentuh lagi (bukan hitung step)
5. X + Z kembali ke (0,0)
```

## Command — Modbus, Serial, ATAU Menu LCD

| Command | CMD (Modbus) | Serial | Menu LCD |
|---|---|---|---|
| Homing X,Y,Z | `1` | `HOME` | `1` |
| Siklus penuh (move→push→home) | `2`, arg=rack 0-5 | `FULLCYCLE <0-5>` | — |
| Pindah ke rak (manual, tanpa push) | `3`, arg=rack 0-5 | `MOVE <0-5>` | `4` (test) |
| Push manual (uji lokal, X/Z TIDAK ikut gerak) | `4` | `PUSH` | — |
| Reset fault | `5` | — | — |
| Jog 1 axis manual | — | `JOG <0=X,1=Y,2=Z> <steps>` | `2` (Move Axis) |
| Simpan posisi rak (X,Z saja) | — | `SETRACK <0-5>` | `3` |
| Kalibrasi jarak dorong Y | — | `SETPUSH` | `5` |
| Atur kecepatan stepper | — | `STEPSPEED <100-5000>` | — |
| Status lengkap | — | `STATUS` | (baris 2-3 LCD, live) |

**Penting:** `PUSH_BOX`/`PUSH` sengaja **tidak** memicu gerak X/Z — cuma uji Y lokal (extend+retract) di posisi manapun lengan sedang berada.

## Menu Kalibrasi LCD

```
* (dari IDLE) → "1=Home 2=MoveAxis 3=SetRack 4=Test 5=SetPush"
  → 1: AUTO HOME (X→Y→Z berurutan)
  → 2: WAJIB sudah homing dulu -- pilih axis (1=X,2=Y,3=Z) → A+/B- jog → C=step
  → 3: simpan posisi X,Z sekarang ke slot 0-5
  → 4: uji MOVE_TO_RACK ke slot 0-5
  → 5: simpan Y sekarang sebagai jarak dorong (Y harus >0 dulu, jog maju via opsi 2)
```

Semua nilai (`RACK[6]`, `pushExtendSteps`, `stepIntervalUs`) **persist ke NVS**.

## Cara Uji Cepat (Urutan Disarankan)

1. `pio run -t upload`, buka serial monitor
2. `HELP` → `STATUS`
3. `HOME` → tunggu → `STATUS` → cek `homed=1,1,1`
4. `JOG 1 500` → uji Y (pusher) manual dulu, paling sederhana, amati posisi
5. `PUSH` → uji extend+retract otomatis (Y saja, X/Z diam)
6. `SETPUSH` (setelah jog Y ke jarak dorong yang diinginkan)
7. `JOG 0 <steps>` / `JOG 2 <steps>` → posisikan ke rak fisik pertama → `SETRACK 0`
8. Ulangi langkah 7 untuk 5 slot rak sisanya
9. `MOVE 0` → verifikasi posisi tepat
10. `FULLCYCLE 0` → uji siklus penuh
11. Tekan E-Stop → `STATUS` → `state=ESTOPPED`

## Troubleshooting

| Gejala | Penyebab | Solusi |
|---|---|---|
| `homed` tidak pernah jadi 1 utk 1 axis tertentu | Limit switch axis itu tidak tersentuh / polaritas salah | Cek wiring & `INPUT_PULLUP`, uji manual dgn multimeter |
| `FULLCYCLE` ditolak, fault `NOT_HOMED` walau sudah `HOME` sekali | Y tidak persis di posisi 0 (drift/macet sebelumnya) | `JOG 1` manual ke `LIM_Y`, atau `HOME` ulang |
| X/Z ikut bergerak saat cuma mau uji `PUSH` | (Sudah diperbaiki) — pastikan pakai firmware versi terbaru | — |

## Yang Masih Terbuka

1. **Kalibrasi jarak fisik** — `RACK[]`, `pushExtendSteps`, `stepIntervalUs` masih nilai default/0, wajib diukur ke mekanik unit ini.
2. **Verifikasi wiring Z dual-motor paralel** — pastikan kedua driver Z benar-benar menerima sinyal identik sebelum uji beban penuh.