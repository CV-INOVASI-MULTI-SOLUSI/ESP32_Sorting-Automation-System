#!/usr/bin/env python3
"""CEK ORANGE PI -- apakah board master siap dipakai (2026-09-30).

Dijalankan SETELAH setup-orangepi-one.sh dan reboot. Tidak mengirim satu pun
command ke node -- hanya membaca register STATE (0) dan mengirim permintaan baca
ke HuskyLens. Aman dijalankan kapan saja, selama tidak ada program Modbus lain
yang sedang memegang /dev/ttyS3.

Urutannya sengaja dari yang paling dasar: kalau port serialnya saja belum ada,
cek Modbus sesudahnya tidak ada artinya, dan hasilnya akan menyesatkan.

    python3 cek-orangepi.py
"""

import os
import subprocess
import sys
import time

RS485_PORT = '/dev/ttyS3'
HUSKYLENS_PORT = '/dev/ttyS1'
SLAVES = {1: 'SORTER', 2: 'PICKER', 3: 'DISPENSER', 4: 'STOCKER'}

hasil = []   # (ok, teks) -- dirangkum di akhir


def catat(ok, teks):
    hasil.append((ok, teks))
    print(("  OK   " if ok else "  !!   ") + teks)


print("== 1. Board")
try:
    with open('/proc/device-tree/model') as f:
        print("  " + f.read().strip('\x00\n'))
except OSError:
    print("  (model tidak terbaca)")

print("\n== 2. Port serial")
for port, guna in ((RS485_PORT, 'RS485 / Modbus -- UART3'), (HUSKYLENS_PORT, 'HuskyLens -- UART1')):
    if os.path.exists(port):
        catat(True, f"{port} ada ({guna})")
    else:
        catat(False, f"{port} TIDAK ADA ({guna}) -- overlay belum aktif? cek /boot/*Env.txt lalu reboot")

print("\n== 3. Library Python")
for modul, wajib in (('serial', True), ('minimalmodbus', True), ('wiringpi', False)):
    try:
        __import__(modul)
        catat(True, f"import {modul}")
    except ImportError:
        if wajib:
            catat(False, f"import {modul} GAGAL -- jalankan setup-orangepi-one.sh")
        else:
            print(f"  --   import {modul} tidak ada (opsional: LED/buzzer script HuskyLens)")

print("\n== 4. Port RS485 bebas?")
if os.path.exists(RS485_PORT):
    try:
        keluaran = subprocess.run(['fuser', RS485_PORT], capture_output=True, text=True)
        pid = (keluaran.stdout + keluaran.stderr).replace(RS485_PORT + ':', '').split()
        if pid:
            catat(False, f"{RS485_PORT} sedang dipegang proses {' '.join(pid)} -- hentikan dulu, "
                         "dua master di satu bus saling merusak pembacaan")
        else:
            catat(True, f"{RS485_PORT} tidak dipegang proses lain")
    except FileNotFoundError:
        print("  --   fuser tidak ada, dilewati")

print("\n== 5. Node Modbus (baca register STATE saja, tidak mengirim command)")
try:
    import minimalmodbus
    import serial
    for sid, nama in SLAVES.items():
        instr = minimalmodbus.Instrument(RS485_PORT, sid)
        instr.serial.baudrate = 19200
        instr.serial.bytesize = 8
        instr.serial.parity = serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.serial.timeout = 0.5
        instr.clear_buffers_before_each_transaction = True
        try:
            st = instr.read_register(0, functioncode=3)
            catat(True, f"slave {sid} {nama:10s} menjawab, STATE={st}")
        except minimalmodbus.NoResponseError:
            catat(False, f"slave {sid} {nama:10s} TIDAK MENJAWAB -- daya node, kabel A/B, atau "
                         "pin TX/RX UART3 tertukar")
        except Exception as e:
            catat(False, f"slave {sid} {nama:10s} error: {type(e).__name__}: {e}")
        # Port TIDAK ditutup per slave: minimalmodbus memakai satu objek serial bersama
        # untuk semua Instrument di port yang sama, jadi menutupnya di sini membuat slave
        # berikutnya gagal dengan error yang mirip "node tidak menjawab".
except ImportError:
    print("  --   dilewati, library belum ada")
except Exception as e:
    catat(False, f"port {RS485_PORT} tidak bisa dibuka: {e}")

print("\n== 6. HuskyLens")
try:
    import serial
    with serial.Serial(HUSKYLENS_PORT, 9600, timeout=0.1) as hl:
        hl.reset_input_buffer()
        hl.write(bytes([0x55, 0xAA, 0x11, 0x00, 0x20, 0x30]))   # COMMAND_REQUEST
        data = b''
        batas = time.monotonic() + 0.8
        while time.monotonic() < batas:
            data += hl.read(64)
        if b'\x55\xAA\x11' in data:
            catat(True, f"HuskyLens menjawab ({len(data)} byte, header 55 AA 11 ditemukan)")
        elif data:
            catat(False, f"ada {len(data)} byte masuk tapi bukan frame HuskyLens -- baud salah "
                         "(HuskyLens harus UART 9600) atau protokol bukan UART")
        else:
            catat(False, "HuskyLens diam -- cek daya, TX/RX tertukar, atau mode protokol "
                         "HuskyLens belum diset ke UART 9600")
except ImportError:
    print("  --   dilewati, pyserial belum ada")
except Exception as e:
    catat(False, f"port {HUSKYLENS_PORT} tidak bisa dibuka: {e}")

gagal = [t for ok, t in hasil if not ok]
print("\n" + "=" * 60)
if gagal:
    print(f" {len(gagal)} MASALAH -- selesaikan dari atas ke bawah, yang atas penyebab yang bawah:")
    for t in gagal:
        print("   - " + t)
    sys.exit(1)
print(" SEMUA OK. Lanjut: python3 /root/sorting/sorting_automation.py  (menu 'f')")
