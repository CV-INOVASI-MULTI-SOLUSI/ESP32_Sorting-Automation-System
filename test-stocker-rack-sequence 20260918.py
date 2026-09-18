#!/usr/bin/env python3
"""
============================================================
 TEST STOCKER -- urutan AutoHome -> (Load Position <-> Rak N) berulang
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node STOCKER
(slave 4), buat verifikasi gerakan fisik Load Position -> Rak -> Push -> Tarik
-> Load Position lagi, PAKAI RUN_FULL_CYCLE (opcode 2) -- ini SUDAH otomatis
include push+tarik+balik ke Load Position dalam 1 command, jadi TIDAK perlu
kirim GOTO_LOAD_POSITION terpisah tiap abis 1 rak (itu udah nempel di akhir
RUN_FULL_CYCLE sendiri).

Urutan:
  AutoHome (HOME_ALL) -> Load Position (posisi awal) ->
  RUN_FULL_CYCLE(Rak 1) [rak->push->tarik->balik Load Position] ->
  RUN_FULL_CYCLE(Rak 2) -> RUN_FULL_CYCLE(Rak 3) -> RUN_FULL_CYCLE(Rak 4)

Tiap langkah: TUNGGU ack (Stocker pakai deferred-ack -- ack baru dikirim
firmware SETELAH gerakan fisik BENERAN tuntas -- utk RUN_FULL_CYCLE artinya
ack baru muncul PAS SUDAH balik ke Load Position lagi, bukan pas baru nyampe
rak doang), BARU setelah itu jeda 2 detik tambahan (buat amatan visual/settle)
sebelum lanjut ke rak berikutnya.

CATATAN index rak: firmware pakai rack_idx 0-5 (0-indexed, total 6 slot).
Skrip ini pakai arg 1,2,3,4 apa adanya sesuai permintaan -- kalau maksudnya
4 slot PERTAMA (0-indexed 0,1,2,3), ganti RACK_SEQUENCE di bawah.

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
STOCKER_SLAVE_ID = 4

STEP_DELAY_S = 2.0             # jeda SETELAH tiap langkah tuntas (sesuai permintaan)
ACK_TIMEOUT_S = 60.0           # gerakan fisik bisa lama (homing/rak jauh) -- kasih waktu longgar
ACK_POLL_INTERVAL_S = 0.2
HOMING_TIMEOUT_S = 90.0        # AutoHome 3 axis, biasanya paling lama

DRY_RUN = True   # WAJIB False dulu manual sebelum jalan ke hardware asli --
                  # saat True, skrip cuma PRINT rencana aksi, TIDAK kirim apapun ke bus.

# Urutan rak yang dites, sesuai permintaan (arg APA ADANYA, lihat catatan index di atas)
RACK_SEQUENCE = [1, 2, 3, 4]

# --- Register base (SAMA di semua node) ---
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5

# --- Opcode STOCKER (lihat ESP32_SortingAutomation_Lifter/include/registers.h) ---
CMD_HOME_ALL = 1
CMD_RUN_FULL_CYCLE = 2      # alternatif MOVE_TO_RACK kalau mau ikut push+tarik+balik
CMD_MOVE_TO_RACK = 3        # manual, TANPA auto-push -- yang dipakai skrip ini
CMD_GOTO_LOAD_POSITION = 6
CMD_START_MAIN = 9   # BARU -- WAJIB dikirim dulu sebelum RUN_FULL_CYCLE (produksi) diterima
CMD_STOP_MAIN = 10


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
    print(f"  -> STOCKER: {label} (opcode={opcode} arg={arg} seq={seq})")
    if DRY_RUN:
        return seq
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
    instr.write_register(REG_CMD, opcode, functioncode=6)
    return seq


def wait_ack(instr, label, seq, timeout=ACK_TIMEOUT_S):
    """Stocker pakai deferred-ack -- ack baru dikirim SETELAH gerakan fisik
    beneran tuntas (bukan cuma command diterima), jadi nunggu ack ini SAMA
    DENGAN nunggu fisiknya sampai."""
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
            print(f"  !! STOCKER: gagal baca CMD_ACK_SEQ ({e}), retry...")
        time.sleep(ACK_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak ack seq={seq} dalam {timeout}s -- CEK FAULT MANUAL, jangan lanjut otomatis")
    return False


def step(instr, label, opcode, arg=0, timeout=ACK_TIMEOUT_S, delay_after=True):
    """1 langkah lengkap: kirim command -> tunggu tuntas -> (opsional) jeda STEP_DELAY_S.
    delay_after=False dipakai utk langkah yang TIDAK berakhir di Load Position (mis. AutoHome)
    -- sesuai permintaan, jeda 2 detik cuma berlaku pas posisi akhir = Load Position."""
    seq = send_command(instr, label, opcode, arg)
    ok = wait_ack(instr, label, seq, timeout=timeout)
    if not ok:
        return False
    if delay_after:
        print(f"  .. di Load Position, jeda {STEP_DELAY_S}s sebelum lanjut")
        time.sleep(STEP_DELAY_S)
    return True


def main():
    print(f"[BOOT] Test Stocker Rack Sequence -- port={SERIAL_PORT} baud={BAUDRATE} "
          f"DRY_RUN={DRY_RUN} urutan_rak={RACK_SEQUENCE}")
    stocker = connect(STOCKER_SLAVE_ID)

    print("\n=== [1] AutoHome ===")
    if not step(stocker, "HOME_ALL (AutoHome)", CMD_HOME_ALL, timeout=HOMING_TIMEOUT_S, delay_after=False):
        print("!! AutoHome GAGAL -- cek fault Stocker manual sebelum ulang. Berhenti.")
        sys.exit(1)

    print("\n=== [2] Load Position (awal) ===")
    if not step(stocker, "GOTO_LOAD_POSITION", CMD_GOTO_LOAD_POSITION):
        print("!! Gagal ke Load Position -- berhenti.")
        sys.exit(1)

    print("\n=== [3] START_MAIN (WAJIB sebelum RUN_FULL_CYCLE) ===")
    if not step(stocker, "START_MAIN", CMD_START_MAIN, delay_after=False):
        print("!! Gagal START_MAIN -- berhenti.")
        sys.exit(1)

    for rack_idx in RACK_SEQUENCE:
        print(f"\n=== Rak {rack_idx} (push -> tarik -> balik Load Position) ===")
        if not step(stocker, f"RUN_FULL_CYCLE({rack_idx})", CMD_RUN_FULL_CYCLE, arg=rack_idx,
                    timeout=ACK_TIMEOUT_S):
            print(f"!! Gagal RUN_FULL_CYCLE Rak {rack_idx} -- cek fault manual. Berhenti.")
            sys.exit(1)

    print("\n=== [STOP_MAIN] balik ke TEST mode ===")
    step(stocker, "STOP_MAIN", CMD_STOP_MAIN, delay_after=False)

    print("\n=== SELESAI -- semua rak dites, Stocker parkir di Load Position ===")


if __name__ == '__main__':
    main()
