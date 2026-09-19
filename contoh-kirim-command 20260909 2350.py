#!/usr/bin/env python3
"""
============================================================
 CONTOH SEDERHANA -- Mengirim Command Modbus dari Orange Pi
============================================================

Instalasi dulu (sekali saja):
    pip3 install minimalmodbus pyserial

Ini versi MINIMAL untuk belajar konsep dasarnya (cuma SORTER, START/STOP +
tulis CLASSIFY_IS_REJECT). Untuk skrip PRODUKSI ASLI (4 node, MAIN/TEST mode,
batch penuh Sorter->Dispenser->Picker->Stocker), pakai orangepi-orchestrator-batch.py.
Untuk uji manual per-node, pakai test-stocker-rack-sequence*.py atau
test-dispenser-full-protocol*.py.
"""

import minimalmodbus
import serial
import time

# ============================================================
# KONFIGURASI -- SESUAIKAN DENGAN HARDWARE ANDA
# ============================================================
SERIAL_PORT = '/dev/ttyS3'   # GANTI sesuai port RS485 Orange Pi Anda
BAUDRATE = 19200
SORTER_SLAVE_ID = 1

# Alamat register (dari registers.h SORTER)
REG_CMD               = 2
REG_CMD_ARG           = 3
REG_CMD_SEQ           = 4
REG_CLASSIFY_IS_REJECT = 12

# Opcode SORTER (dari enum Cmd di registers.h)
CMD_START = 1
CMD_STOP  = 2


def connect(slave_id):
    """Buka koneksi Modbus RTU ke 1 node."""
    instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
    instr.serial.baudrate = BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout = 0.5
    instr.mode = minimalmodbus.MODE_RTU
    return instr


# ============================================================
# POLA 1: Register tunggal (klasifikasi REJECT/PASS)
# Dipakai untuk register yang BERDIRI SENDIRI, dipanggil sering
# (1x per objek) -- CUKUP 1 write_register, tidak perlu urutan apa pun.
# ============================================================
def kirim_klasifikasi(instr, is_reject):
    nilai = 1 if is_reject else 0
    instr.write_register(REG_CLASSIFY_IS_REJECT, nilai, functioncode=6)
    print(f"Klasifikasi terkirim: {'REJECT' if is_reject else 'PASS'}")


# ============================================================
# POLA 2: Command (opcode) -- WAJIB 3 langkah berurutan.
# Dipakai untuk command yang jarang dipanggil (START/STOP/RUN_FULL_CYCLE/dst)
# -- firmware pakai nomor urut (seq) untuk cegah command yang sama
# dieksekusi dua kali kalau ada retry komunikasi.
# ============================================================
_seq_counter = 0   # WAJIB naik terus, jangan pernah diulang nilainya

def kirim_command(instr, opcode, arg=0):
    global _seq_counter
    _seq_counter += 1
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)        # 1. argumen dulu
    instr.write_register(REG_CMD_SEQ, _seq_counter, functioncode=6)  # 2. nomor urut
    instr.write_register(REG_CMD, opcode, functioncode=6)         # 3. opcode PALING TERAKHIR
    print(f"Command terkirim: opcode={opcode} arg={arg} seq={_seq_counter}")


# ============================================================
# CONTOH PEMAKAIAN
# ============================================================
if __name__ == '__main__':
    sorter = connect(SORTER_SLAVE_ID)

    # Contoh 1: kirim klasifikasi REJECT (simulasi HuskyLens mendeteksi objek reject)
    kirim_klasifikasi(sorter, is_reject=True)
    time.sleep(1)

    # Contoh 2: kirim command START (opcode, pakai protokol 3-langkah)
    kirim_command(sorter, CMD_START)
    time.sleep(1)

    # Contoh 3: kirim command STOP
    kirim_command(sorter, CMD_STOP)
