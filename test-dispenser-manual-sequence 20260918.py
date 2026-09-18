#!/usr/bin/env python3
"""
============================================================
 TEST DISPENSER -- urutan manual conveyor + servo1/servo2 + prox1/prox2
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node
DISPENSER (slave 3), ngikutin urutan yang diminta persis:

  1. Conveyor MATI (pastikan)
  2. Servo1 lepas -> tarik lagi (1x siklus)
  3. Conveyor MENYALA
  4. Servo2 lepas -> tarik lagi (1x siklus)
  5. TUNGGU PROX_2 (TENGAH) trigger -> begitu trigger, pastikan conveyor menyala
  6. Delay 2 detik
  7. Conveyor menyala lagi
  8. TUNGGU PROX_1 (UJUNG) trigger
  9. Conveyor MATI

Langkah #5 dan #8 BENERAN nunggu sensor fisik ke-trigger (poll register live),
BUKAN cuma delay buta -- situ perlu taruh objek di depan PROX_1/PROX_2 manual
pas skrip nyampe langkah itu (ada print "TUNGGU ..." di layar).

Command yang dipakai BARU ditambahkan ke firmware Dispenser sesi ini:
  - Cmd::SET_CONVEYOR_ON_OFF (opcode 11) -- jog manual conveyor ON/OFF, gak
    terikat sensor (beda dari REQUEST_REFILL/FORCE_MIDDLE_REFILL yang auto-stop
    sendiri). DITOLAK firmware kalau ada siklus otomatis lain (refill/test-loop)
    lagi jalan.
  - Reg::UJUNG_PACKAGE_PRESENT (register 17) -- status live PROX_1, simetris
    dengan Reg::MIDDLE_PACKAGE_PRESENT (16) yang sudah ada duluan buat PROX_2.
WAJIB flash ulang firmware Dispenser dulu sebelum skrip ini jalan ke hardware asli.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial
"""

import minimalmodbus
import serial
import time
import sys

# ============================================================
# KONFIGURASI -- SESUAIKAN DENGAN HARDWARE ANDA
# ============================================================
SERIAL_PORT = '/dev/ttyS3'     # GANTI sesuai port RS485 Orange Pi Anda
BAUDRATE = 19200
DISPENSER_SLAVE_ID = 3

ACK_TIMEOUT_S = 15.0
ACK_POLL_INTERVAL_S = 0.2
PROX_WAIT_TIMEOUT_S = 60.0     # nunggu operator taruh objek manual -- kasih waktu longgar
PROX_POLL_INTERVAL_S = 0.2
STEP_DELAY_S = 2.0             # sesuai permintaan -- delay 2 detik di langkah #6

DRY_RUN = True   # WAJIB False dulu manual sebelum jalan ke hardware asli --
                  # saat True, skrip cuma PRINT rencana aksi, TIDAK kirim apapun ke bus.

# --- Register base (SAMA di semua node) ---
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_MIDDLE_PACKAGE_PRESENT = 16   # PROX_2 (TENGAH), live
REG_UJUNG_PACKAGE_PRESENT = 17    # PROX_1 (UJUNG), live -- BARU

# --- Opcode DISPENSER (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
CMD_SET_CONVEYOR_ON_OFF = 11   # BARU -- arg: 0=mati, 1=nyala
CMD_TEST_SERVO1_CYCLE = 97
CMD_TEST_SERVO2_CYCLE = 98


def connect(slave_id):
    instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
    instr.serial.baudrate = BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout = 0.5
    instr.mode = minimalmodbus.MODE_RTU
    return instr


_seq_counter = 0

def send_command(instr, label, opcode, arg=0):
    global _seq_counter
    _seq_counter += 1
    seq = _seq_counter
    print(f"  -> DISPENSER: {label} (opcode={opcode} arg={arg} seq={seq})")
    if DRY_RUN:
        return seq
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
    instr.write_register(REG_CMD, opcode, functioncode=6)
    return seq


def wait_ack(instr, label, seq, timeout=ACK_TIMEOUT_S):
    if DRY_RUN:
        print(f"  .. (DRY_RUN) anggap {label} tuntas (ack seq={seq})")
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            ack = instr.read_register(REG_CMD_ACK_SEQ, functioncode=3)
            if ack == seq:
                return True
        except Exception as e:
            print(f"  !! DISPENSER: gagal baca CMD_ACK_SEQ ({e}), retry...")
        time.sleep(ACK_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak ack seq={seq} dalam {timeout}s")
    return False


def do_command(instr, label, opcode, arg=0, timeout=ACK_TIMEOUT_S):
    seq = send_command(instr, label, opcode, arg)
    return wait_ack(instr, label, seq, timeout=timeout)


def wait_prox(instr, label, reg_addr, timeout=PROX_WAIT_TIMEOUT_S):
    """Poll register live PROX sampai bernilai 1 (trigger) -- BUKAN delay buta.
    Taruh objek di depan sensor fisik yang sesuai pas skrip nyampe sini."""
    print(f"  .. TUNGGU {label} trigger -- taruh objek di depan sensor SEKARANG")
    if DRY_RUN:
        print(f"  .. (DRY_RUN) anggap {label} sudah trigger")
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            val = instr.read_register(reg_addr, functioncode=3)
            if val == 1:
                print(f"  .. {label} TERDETEKSI trigger")
                return True
        except Exception as e:
            print(f"  !! DISPENSER: gagal baca {label} ({e}), retry...")
        time.sleep(PROX_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak trigger dalam {timeout}s")
    return False


def main():
    print(f"[BOOT] Test Dispenser Manual Sequence -- port={SERIAL_PORT} baud={BAUDRATE} DRY_RUN={DRY_RUN}")
    dispenser = connect(DISPENSER_SLAVE_ID)

    print("\n=== [1] Conveyor MATI (pastikan) ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
        sys.exit(1)

    print("\n=== [2] Servo1 lepas -> tarik lagi ===")
    if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE):
        sys.exit(1)

    print("\n=== [3] Conveyor MENYALA ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    print("\n=== [4] Servo2 lepas -> tarik lagi ===")
    if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE):
        sys.exit(1)

    print("\n=== [5] Tunggu PROX_2 (TENGAH) trigger -> pastikan conveyor menyala ===")
    if not wait_prox(dispenser, "PROX_2 (TENGAH)", REG_MIDDLE_PACKAGE_PRESENT):
        sys.exit(1)
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    print(f"\n=== [6] Delay {STEP_DELAY_S}s ===")
    time.sleep(STEP_DELAY_S)

    print("\n=== [7] Conveyor menyala lagi ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    print("\n=== [8] Tunggu PROX_1 (UJUNG) trigger ===")
    if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT):
        sys.exit(1)

    print("\n=== [9] Conveyor MATI ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
        sys.exit(1)

    print("\n=== SELESAI ===")


if __name__ == '__main__':
    main()
