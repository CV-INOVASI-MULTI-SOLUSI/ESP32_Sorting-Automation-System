#!/usr/bin/env python3
"""
============================================================
 DIAGNOSA -- kenapa Servo1 Dispenser gak gerak
============================================================
Skrip SEKALI PAKAI buat nyari akar masalah "servo1 tidak bergerak" TANPA
akses Serial USB -- cuma lewat RS485/Modbus dari Orange Pi.

Cara kerja: baca STATE/FAULT_CODE/MAIN_MODE_ACTIVE SEBELUM kirim command,
kirim TEST_SERVO1_CYCLE (opcode 97), lalu INTIP register ACTIVITY_CODE
setiap 200ms selama beberapa detik -- kalau firmware BENERAN mulai gerakin
servo, ACTIVITY_CODE bakal berubah dari DIAM(0) ke SERVO1_BUKA(2) lalu
SERVO1_TAHAN(3) lalu SERVO1_TUTUP(4) lalu balik DIAM/SELESAI. Kalau
ACTIVITY_CODE TETAP 0 (DIAM) dari awal sampai akhir, artinya firmware
GAK PERNAH MULAI eksekusi cycle-nya sama sekali (soal software/gating),
BUKAN soal servo/wiring/power fisik.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial
"""

import minimalmodbus
import serial
import time

SERIAL_PORT = '/dev/ttyS3'     # GANTI sesuai port RS485 Orange Pi Anda
BAUDRATE = 19200
DISPENSER_SLAVE_ID = 3

POLL_DURATION_S = 8.0
POLL_INTERVAL_S = 0.2

# --- Register (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
REG_STATE = 0
REG_FAULT_CODE = 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_ACTIVITY_CODE = 11
REG_MAIN_MODE_ACTIVE = 18   # cuma ada kalau firmware SUDAH versi MAIN/TEST mode

CMD_TEST_SERVO1_CYCLE = 97

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING_OR_MOVING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM", 1: "CONVEYOR_JALAN", 2: "SERVO1_BUKA", 3: "SERVO1_TAHAN", 4: "SERVO1_TUTUP",
    5: "SERVO2_BUKA", 6: "SERVO2_TAHAN", 7: "SERVO2_TUTUP", 8: "SELESAI",
    9: "TUNGGU_KONFIRM_TENGAH", 10: "TEST_LOOP_AKTIF", 90: "FAULT_AKTIF", 91: "ESTOP_AKTIF",
}


def connect(slave_id):
    instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
    instr.serial.baudrate = BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout = 0.5
    instr.mode = minimalmodbus.MODE_RTU
    return instr


def read_reg(instr, addr, label):
    try:
        return instr.read_register(addr, functioncode=3)
    except Exception as e:
        print(f"  !! Gagal baca {label} (reg {addr}): {e}")
        return None


def main():
    print(f"[BOOT] Diagnosa Servo1 Dispenser -- port={SERIAL_PORT} baud={BAUDRATE}")
    dispenser = connect(DISPENSER_SLAVE_ID)

    print("\n=== [1] Baca status SEBELUM kirim command ===")
    state = read_reg(dispenser, REG_STATE, "STATE")
    fault = read_reg(dispenser, REG_FAULT_CODE, "FAULT_CODE")
    main_mode = read_reg(dispenser, REG_MAIN_MODE_ACTIVE, "MAIN_MODE_ACTIVE")
    print(f"  STATE       = {state} ({STATE_NAMES.get(state, '?')})")
    print(f"  FAULT_CODE  = {fault} {'-- ADA FAULT, ini kemungkinan besar penyebabnya! RESET_FAULT dulu.' if fault else '(bersih)'}")
    if main_mode is None:
        print("  MAIN_MODE_ACTIVE = (gagal baca -- kemungkinan firmware fisik BELUM punya register ini,")
        print("                      berarti masih versi LAMA pre-MAIN/TEST-mode. Ini INFO, bukan error.)")
    else:
        print(f"  MAIN_MODE_ACTIVE = {main_mode} {'-- MAIN aktif, TEST_SERVO1_CYCLE PASTI ditolak! Kirim STOP_MAIN dulu.' if main_mode == 1 else '(TEST mode, aman)'}")

    if fault:
        print("\n!! FAULT_CODE tidak nol -- kemungkinan besar INI penyebabnya, command ditolak firmware.")
        print("!! Reset dulu (RESET_FAULT via LCD Dispenser: * -> Setting Kalibrasi -> Reset Fault -> C,")
        print("!! atau kirim opcode 2 lewat script/QModMaster), baru coba skrip ini lagi.")

    print("\n=== [2] Kirim TEST_SERVO1_CYCLE (opcode 97) ===")
    global_seq = int(time.time()) % 60000
    dispenser.write_register(REG_CMD_ARG, 0, functioncode=6)
    dispenser.write_register(REG_CMD_SEQ, global_seq, functioncode=6)
    dispenser.write_register(REG_CMD, CMD_TEST_SERVO1_CYCLE, functioncode=6)
    print(f"  -> terkirim (seq={global_seq})")

    print(f"\n=== [3] Intip ACTIVITY_CODE tiap {POLL_INTERVAL_S}s selama {POLL_DURATION_S}s ===")
    print("     (kalau angkanya TETAP 0/DIAM terus, firmware GAK PERNAH mulai gerakin servo)")
    seen_nonzero = False
    last_val = None
    deadline = time.monotonic() + POLL_DURATION_S
    while time.monotonic() < deadline:
        val = read_reg(dispenser, REG_ACTIVITY_CODE, "ACTIVITY_CODE")
        if val is not None and val != last_val:
            name = ACTIVITY_NAMES.get(val, f"?({val})")
            print(f"  t={POLL_DURATION_S - (deadline - time.monotonic()):.1f}s  ACTIVITY_CODE = {val} ({name})")
            last_val = val
            if val != 0:
                seen_nonzero = True
        time.sleep(POLL_INTERVAL_S)

    fault_after = read_reg(dispenser, REG_FAULT_CODE, "FAULT_CODE")
    ack = read_reg(dispenser, REG_CMD_ACK_SEQ, "CMD_ACK_SEQ")

    print("\n=== KESIMPULAN ===")
    print(f"  CMD_ACK_SEQ setelah kirim = {ack} (kirim seq={global_seq}) "
          f"{'-- MATCH, command diterima' if ack == global_seq else '-- BEDA/gagal baca, cek koneksi RS485/slave ID'}")
    print(f"  FAULT_CODE setelah        = {fault_after}")
    if seen_nonzero:
        print("  ACTIVITY_CODE SEMPAT berubah dari DIAM -- firmware BENERAN mulai eksekusi cycle servo1.")
        print("  Kalau servo tetap keliatan gak gerak fisik, ini kemungkinan besar soal HARDWARE:")
        print("  power PCA9685/servo, wiring channel servo1, atau nilai kalibrasi servo1StartUs/")
        print("  servo1EndUs kebetulan sama (range gerak = 0, cycle 'jalan' tapi gak ada gerakan nyata).")
    else:
        print("  ACTIVITY_CODE TETAP 0/DIAM dari awal sampai akhir -- firmware GAK PERNAH mulai")
        print("  eksekusi cycle-nya sama sekali. Ack tetap 'sukses' (itu cuma nandain command DITERIMA,")
        print("  BUKAN berhasil dieksekusi -- CMD_ACK_SEQ selalu ditulis unconditional di onCmdWrite()).")
        print("  Kemungkinan penyebab (bukan hardware): FAULT_CODE aktif, MAIN mode aktif (kalau firmware")
        print("  sudah versi baru), atau ada siklus otomatis lain (refill/test-loop) yang lagi jalan dan")
        print("  bikin guard di firmware nolak diam-diam.")


if __name__ == '__main__':
    main()
