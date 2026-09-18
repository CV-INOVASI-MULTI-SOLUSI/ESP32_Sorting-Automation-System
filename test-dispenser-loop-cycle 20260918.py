#!/usr/bin/env python3
"""
============================================================
 TEST DISPENSER -- siklus looping conveyor + servo1/servo2 + prox1/prox2
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node
DISPENSER (slave 3). BEDA dari "test-dispenser-manual-sequence" (one-shot) --
skrip ini LOOPING TERUS sampai di-stop manual (Ctrl+C), sesuai alur:

SETUP (1x doang, sebelum masuk loop):
  1. Conveyor MATI
  2. Servo1: dorong -> tahan -> tarik (1x siklus)
  3. Conveyor MENYALA

LOOP (mulai poin 4, balik ke poin 4 lagi di poin 12 -- terus-menerus):
  4. TUNGGU package melewati PROX_2 (TENGAH)
  5. Conveyor MATI
  6. Servo2: dorong -> tahan -> tarik (1x siklus)
  7. Conveyor MENYALA
  8. TUNGGU package (yang tadi di PROX_2) maju melewati PROX_1 (UJUNG)
  9. Conveyor MATI
  10. TUNGGU PROX_1 CLEAR (package udah gak ke-deteksi lagi -- disimulasikan arm ambil)
  11. Conveyor MENYALA
  12. -> balik ke poin 4

CATATAN ASUMSI: poin 6 di permintaan aslinya tertulis "servo 2 dorong tahan SERVO 1
tarik" -- dianggap TYPO, diimplementasikan sebagai SERVO2 penuh (dorong->tahan->tarik),
BUKAN campur servo1. Servo1 (poin 2) SENGAJA cuma sekali di SETUP, TIDAK ikut diulang
tiap putaran loop -- sesuai instruksi asli (loop cuma mulai dari poin 4).

Command/register yang dipakai (BARU ditambahkan ke firmware Dispenser sesi ini --
WAJIB firmware Dispenser sudah di-flash versi terbaru sebelum skrip ini jalan):
  - Cmd::SET_CONVEYOR_ON_OFF (opcode 11) -- jog manual conveyor ON/OFF
  - Cmd::TEST_SERVO1_CYCLE (97) / TEST_SERVO2_CYCLE (98) -- 1x siklus dorong-tahan-tarik
  - Reg::MIDDLE_PACKAGE_PRESENT (16) -- status live PROX_2 (TENGAH)
  - Reg::UJUNG_PACKAGE_PRESENT (17) -- status live PROX_1 (UJUNG)

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
PROX_WAIT_TIMEOUT_S = 120.0    # nunggu operator/objek fisik -- kasih waktu longgar
PROX_POLL_INTERVAL_S = 0.2

DRY_RUN = True   # WAJIB False dulu manual sebelum jalan ke hardware asli --
                  # saat True, skrip cuma PRINT rencana aksi, TIDAK kirim apapun ke bus,
                  # DAN prox dianggap langsung trigger/clear (biar alur kelihatan lengkap).

# --- Register base (SAMA di semua node) ---
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_MIDDLE_PACKAGE_PRESENT = 16   # PROX_2 (TENGAH), live
REG_UJUNG_PACKAGE_PRESENT = 17    # PROX_1 (UJUNG), live

# --- Opcode DISPENSER (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
CMD_SET_CONVEYOR_ON_OFF = 11
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


def wait_prox(instr, label, reg_addr, want_value, timeout=PROX_WAIT_TIMEOUT_S):
    """Poll register live PROX sampai bernilai want_value (1=trigger, 0=clear)."""
    kata = "trigger" if want_value == 1 else "CLEAR (gak ke-deteksi lagi)"
    print(f"  .. TUNGGU {label} {kata}")
    if DRY_RUN:
        print(f"  .. (DRY_RUN) anggap {label} sudah {kata}")
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            val = instr.read_register(reg_addr, functioncode=3)
            if val == want_value:
                print(f"  .. {label} {kata} -- OK")
                return True
        except Exception as e:
            print(f"  !! DISPENSER: gagal baca {label} ({e}), retry...")
        time.sleep(PROX_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak {kata} dalam {timeout}s")
    return False


def main():
    print(f"[BOOT] Test Dispenser Loop Cycle -- port={SERIAL_PORT} baud={BAUDRATE} DRY_RUN={DRY_RUN}")
    dispenser = connect(DISPENSER_SLAVE_ID)

    print("\n=== SETUP [1] Conveyor MATI ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
        sys.exit(1)

    print("\n=== SETUP [2] Servo1 dorong -> tahan -> tarik ===")
    if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE):
        sys.exit(1)

    print("\n=== SETUP [3] Conveyor MENYALA ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    loop_count = 0
    try:
        while True:
            loop_count += 1
            print(f"\n########## LOOP #{loop_count} (mulai dari poin 4) ##########")

            print("\n=== [4] Tunggu package melewati PROX_2 (TENGAH) ===")
            if not wait_prox(dispenser, "PROX_2 (TENGAH)", REG_MIDDLE_PACKAGE_PRESENT, 1):
                break

            print("\n=== [5] Conveyor MATI ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
                break

            print("\n=== [6] Servo2 dorong -> tahan -> tarik ===")
            if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE):
                break

            print("\n=== [7] Conveyor MENYALA ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
                break

            print("\n=== [8] Tunggu package maju melewati PROX_1 (UJUNG) ===")
            if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 1):
                break

            print("\n=== [9] Conveyor MATI ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
                break

            print("\n=== [10] Tunggu PROX_1 CLEAR (package sudah diambil) ===")
            if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 0):
                break

            print("\n=== [11] Conveyor MENYALA ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
                break

            print("\n=== [12] Balik ke poin 4 ===")
            if DRY_RUN:
                # DRY_RUN cuma 1x putaran demo, biar gak infinite print di simulasi
                print("\n(DRY_RUN) 1 putaran demo selesai, berhenti di sini.")
                break

    except KeyboardInterrupt:
        print("\n[STOP] Dihentikan manual (Ctrl+C)")
        sys.exit(0)

    print(f"\n=== BERHENTI setelah {loop_count} putaran ===")


if __name__ == '__main__':
    main()
