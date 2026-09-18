#!/usr/bin/env python3
"""
============================================================
 TEST DISPENSER -- protokol lengkap conveyor + servo1/servo2 + prox1/prox2
 + konfirmasi jumlah objek (placeholder timing)
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node
DISPENSER (slave 3). Versi PALING LENGKAP, sesuai urutan yang diminta:

SETUP (1x, sebelum masuk loop):
  1. Conveyor MATI
  2. Servo1: titik awal -> tahan -> titik akhir -> (balik titik awal) [round-trip,
     TEST_SERVO1_CYCLE -- dikonfirmasi user round-trip, bukan one-way]
  3. (deskriptif -- jeda hold time servo1&2 SUDAH otomatis include di dalam
     TEST_SERVO1_CYCLE/TEST_SERVO2_CYCLE, BUKAN langkah terpisah)
  4. Servo2: sama pola, round-trip (TEST_SERVO2_CYCLE)
  5. Conveyor MENYALA

LOOP (mulai poin 6, balik ke 6 lagi di poin 17 -- terus-menerus):
  6.  TUNGGU package melewati PROX_2 (TENGAH)
  7.  Conveyor MATI
  8.  [TIMING, BUKAN command -- lihat catatan] tunggu konfirmasi jumlah objek
  9.  [TIMING, BUKAN command] terima "jumlah objek == target"
  10. Servo1 round-trip lagi (TEST_SERVO1_CYCLE)
  11. (deskriptif, sama seperti poin 3)
  12. Servo2 round-trip lagi (TEST_SERVO2_CYCLE)
  13. Conveyor MENYALA
  14. TUNGGU package (dari PROX_2) maju melewati PROX_1 (UJUNG)
  15. [TIMING, BUKAN command] kirim command "ambil package"
  16. TUNGGU PROX_1 balik ke 0 (package diambil) -- conveyor TETAP MENYALA selama
      nunggu ini (tidak ada command mati di sini, sesuai instruksi asli: "prox_1
      sudah 0 tapi prox_2 belum 0 -> conveyor tetap maju")
  17. -> balik ke poin 6 (nunggu PROX_2 trigger lagi utk objek baru)

CATATAN PENTING soal poin 8, 9, 15:
  Sesuai instruksi eksplisit -- 3 poin ini dibuat pakai DELAY WAKTU (time.sleep),
  BUKAN kirim/tunggu command Modbus beneran. ini PLACEHOLDER, ganti nilai
  *_PLACEHOLDER_DELAY_S di bawah kalau durasi aslinya sudah diketahui, atau
  ganti isinya ke command Modbus asli kalau nanti protokol jumlah-objek sudah
  didefinisikan sebagai register/opcode beneran.

FITUR BARU khusus buat kebutuhan "masukin package pertama kali" (di luar alur
loop di atas, dipanggil manual kalau perlu -- lihat fungsi manual_load_servo()):
  - Cmd::MOVE_SERVO1_TO / MOVE_SERVO2_TO (opcode 12/13) -- jog manual SATU ARAH
    (BUKAN round-trip), gerak ke titik awal (arg=0) ATAU titik akhir (arg=1),
    berhenti di situ (gak balik sendiri) -- biar operator bisa buka gerbang,
    taruh package manual, tutup lagi pakai command terpisah.

WAJIB firmware Dispenser sudah di-flash versi terbaru (opcode 11/12/13 + register
16/17) sebelum skrip ini jalan ke hardware asli.

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

# --- PLACEHOLDER delay utk poin 8, 9, 15 (TIMING, bukan command beneran) ---
KONFIRMASI_JUMLAH_OBJEK_DELAY_S = 2.0   # poin 8: "menunggu konfirmasi jumlah objek"
TERIMA_TARGET_JUMLAH_DELAY_S = 1.0      # poin 9: "menerima command jumlah objek == target"
KIRIM_AMBIL_PACKAGE_DELAY_S = 1.0       # poin 15: "mengirim command untuk ambil package"

DRY_RUN = True   # WAJIB False dulu manual sebelum jalan ke hardware asli --
                  # saat True, skrip cuma PRINT rencana aksi + anggap prox trigger instan.

# --- Register base (SAMA di semua node) ---
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_MIDDLE_PACKAGE_PRESENT = 16   # PROX_2 (TENGAH), live
REG_UJUNG_PACKAGE_PRESENT = 17    # PROX_1 (UJUNG), live

# --- Opcode DISPENSER (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
CMD_SET_CONVEYOR_ON_OFF = 11
CMD_MOVE_SERVO1_TO = 12    # arg: 0=titik awal, 1=titik akhir
CMD_MOVE_SERVO2_TO = 13    # arg: 0=titik awal, 1=titik akhir
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


def timing_placeholder(label, delay_s):
    """Poin 8/9/15 -- SENGAJA cuma delay waktu, BUKAN command/register beneran
    (sesuai instruksi eksplisit: 'dibuat menggunakan timing tidak menerima command')."""
    print(f"  .. [TIMING PLACEHOLDER] {label} -- jeda {delay_s}s (belum ada command/register asli)")
    time.sleep(delay_s)


def manual_load_servo(instr, which, to_end):
    """BARU -- dipanggil MANUAL (bukan bagian alur loop otomatis) kalau operator
    perlu buka gerbang servo1/servo2 buat masukin package pertama kali. Panggil
    lagi dengan to_end=False setelah selesai taruh package, biar gerbang nutup."""
    opcode = CMD_MOVE_SERVO1_TO if which == 1 else CMD_MOVE_SERVO2_TO
    label = f"MOVE_SERVO{which}_TO({'titik akhir' if to_end else 'titik awal'})"
    return do_command(instr, label, opcode, arg=1 if to_end else 0)


def main():
    print(f"[BOOT] Test Dispenser Full Protocol -- port={SERIAL_PORT} baud={BAUDRATE} DRY_RUN={DRY_RUN}")
    dispenser = connect(DISPENSER_SLAVE_ID)

    # Contoh pemakaian manual_load_servo() buat isi package PERTAMA KALI SEBELUM
    # loop mulai -- uncomment kalau perlu:
    # manual_load_servo(dispenser, 1, to_end=True)   # buka gerbang servo1
    # input("Taruh package, lalu Enter...")
    # manual_load_servo(dispenser, 1, to_end=False)  # tutup lagi

    print("\n=== SETUP [1] Conveyor MATI ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
        sys.exit(1)

    print("\n=== SETUP [2] Servo1 round-trip (titik awal->akhir->tahan->awal) ===")
    if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE):
        sys.exit(1)

    print("\n=== SETUP [4] Servo2 round-trip ===")
    if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE):
        sys.exit(1)

    print("\n=== SETUP [5] Conveyor MENYALA ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    loop_count = 0
    try:
        while True:
            loop_count += 1
            print(f"\n########## LOOP #{loop_count} (mulai dari poin 6) ##########")

            print("\n=== [6] Tunggu package melewati PROX_2 (TENGAH) ===")
            if not wait_prox(dispenser, "PROX_2 (TENGAH)", REG_MIDDLE_PACKAGE_PRESENT, 1):
                break

            print("\n=== [7] Conveyor MATI ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
                break

            print("\n=== [8] Tunggu konfirmasi jumlah objek (TIMING placeholder) ===")
            timing_placeholder("konfirmasi jumlah objek", KONFIRMASI_JUMLAH_OBJEK_DELAY_S)

            print("\n=== [9] Terima jumlah objek == target (TIMING placeholder) ===")
            timing_placeholder("jumlah objek == target", TERIMA_TARGET_JUMLAH_DELAY_S)

            print("\n=== [10] Servo1 round-trip ===")
            if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE):
                break

            print("\n=== [12] Servo2 round-trip ===")
            if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE):
                break

            print("\n=== [13] Conveyor MENYALA ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
                break

            print("\n=== [14] Tunggu package maju melewati PROX_1 (UJUNG) ===")
            if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 1):
                break

            print("\n=== [15] Kirim command ambil package (TIMING placeholder) ===")
            timing_placeholder("command ambil package", KIRIM_AMBIL_PACKAGE_DELAY_S)

            print("\n=== [16] Tunggu PROX_1 CLEAR -- conveyor TETAP MENYALA selama nunggu ===")
            if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 0):
                break

            print("\n=== [17] Balik ke poin 6 (tunggu PROX_2 trigger lagi) ===")
            if DRY_RUN:
                print("\n(DRY_RUN) 1 putaran demo selesai, berhenti di sini.")
                break

    except KeyboardInterrupt:
        print("\n[STOP] Dihentikan manual (Ctrl+C)")
        sys.exit(0)

    print(f"\n=== BERHENTI setelah {loop_count} putaran ===")


if __name__ == '__main__':
    main()
