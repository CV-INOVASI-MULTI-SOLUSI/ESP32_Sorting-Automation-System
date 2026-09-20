#!/usr/bin/env python3
"""
============================================================
 ORANGE PI -- Orkestrasi Batch Package (Sorter -> Dispenser -> Picker -> Stocker)
============================================================

DIUBAH TOTAL (bug ditemukan di versi sebelumnya): urutan lama salah -- Picker.MOVE_PACKAGE
dikirim SEBELUM Dispenser.REQUEST_REFILL, padahal REQUEST_REFILL itu justru yang memindahkan
package dari TENGAH ke UJUNG (titik ambil Picker). Versi ini juga menambah startup_sequence()
dan protokol PACKAGE_READY_FLAG/ACK_PACKAGE_TAKEN/MIDDLE_PACKAGE_PRESENT/FORCE_MIDDLE_REFILL
yang ditambahkan ke firmware Dispenser SETELAH versi awal script ini ditulis.

Alur fisik (2-proximity Dispenser, dikonfirmasi user):
  - Package duduk di TENGAH conveyor Dispenser (PACKAGE_MIDDLE_SENSOR), diisi objek PASS dari
    Sorter. Begitu Orange Pi anggap "full" (SORTER.PASS_COUNT capai BATCH_SIZE):
      1. DISPENSER.REQUEST_REFILL -- conveyor MAJU, bawa package dari TENGAH ke UJUNG.
      2. TUNGGU DISPENSER.PACKAGE_READY_FLAG == 1 (package sudah di UJUNG, siap diambil arm).
         Paralel di sisi Dispenser: begitu TENGAH kosong (PROX_2 clear), servo1+servo2
         otomatis jatuhin package baru ke TENGAH (sensor-confirmed, independen, TIDAK perlu
         command terpisah dari Orange Pi).
      3. STOCKER.GOTO_LOAD_POSITION -- siap terima package (biasanya sudah di situ dari
         siklus sebelumnya, command ini idempotent/aman dipanggil ulang).
      4. PICKER.MOVE_PACKAGE -- ambil package dari UJUNG Dispenser, taruh di Load Position.
      5. DISPENSER.ACK_PACKAGE_TAKEN -- WAJIB, tanpa ini REQUEST_REFILL berikutnya DITOLAK
         (lihat guard di firmware Dispenser applyCommand()).
      6. STOCKER.RUN_FULL_CYCLE(rack_idx) -- pindah package dari Load Position ke rak kosong.
      7. SORTER.RESET_COUNTERS -- batch berikutnya mulai dari 0.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial

PENTING SEBELUM DIPAKAI PRODUKSI:
  - Kalibrasi dulu pose 4 (PACKAGE_PICKUP) & pose 5 (LIFT_LOAD) di Picker lewat
    menu LCD (Setting Kalibrasi > Pose > pilih joint > JOG > # > slot 4/5) atau
    Serial (JOG lalu SAVEPOSE 4 / SAVEPOSE 5) -- default masih sama dgn home,
    BELUM dikalibrasi ke posisi fisik package/load position sungguhan.
  - Kalibrasi loadPos (X/Z) di STOCKER lewat menu kalibrasinya sendiri kalau
    belum -- pastikan titik itu SECARA FISIK align dgn pose LIFT_LOAD Picker.
  - Script ini BELUM pernah diuji ke hardware asli -- jalankan step-by-step
    manual dulu (mode DRY_RUN=True) sebelum lepas otomatis penuh.
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
BATCH_SIZE = 20                # jumlah PASS_COUNT sebelum trigger batch
POLL_INTERVAL_S = 0.5          # jeda antar poll PASS_COUNT
ACK_TIMEOUT_S = 15.0           # maksimum tunggu 1 command selesai (CMD_ACK_SEQ match)
ACK_POLL_INTERVAL_S = 0.2
PACKAGE_READY_TIMEOUT_S = 15.0     # maksimum tunggu conveyor Dispenser sampai UJUNG
MIDDLE_REFILL_TIMEOUT_S = 15.0     # maksimum tunggu FORCE_MIDDLE_REFILL saat startup
HOMING_TIMEOUT_S = 60.0            # STOCKER homing 3 axis bisa lumayan lama

DRY_RUN = True   # WAJIB False dulu manual-test 1x sebelum jalan otomatis penuh --
                  # saat True, script cuma PRINT rencana aksi, TIDAK kirim apapun ke bus.

SLAVE = {'SORTER': 1, 'PICKER': 2, 'DISPENSER': 3, 'STOCKER': 4}

# --- Register base (SAMA di semua node) ---
REG_STATE, REG_FAULT_CODE = 0, 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5

STATE_IDLE, STATE_RUNNING, STATE_FAULT, STATE_ESTOPPED = 1, 2, 3, 4

# --- Register/opcode spesifik per node ---
REG_SORTER_PASS_COUNT = 10
CMD_SORTER_START = 1
CMD_SORTER_STOP = 2    # BARU -- dipakai shutdown_sequence()
CMD_SORTER_RESET_COUNTERS = 7

CMD_PICKER_GOTO_HOME = 2
CMD_PICKER_MOVE_PACKAGE = 8   # BARU -- lihat ESP32_SortingAutomation_Picker/include/registers.h
CMD_PICKER_START_MAIN = 11    # BARU -- WAJIB sebelum MOVE_PACKAGE/RUN_SEQUENCE diterima
CMD_PICKER_STOP_MAIN = 12

REG_DISPENSER_PACKAGE_READY = 15     # BARU -- 1 = package di UJUNG siap diambil arm
REG_DISPENSER_MIDDLE_PRESENT = 16    # BARU -- 1 = ada package terdeteksi di TENGAH
CMD_DISPENSER_REQUEST_REFILL = 1
CMD_DISPENSER_ACK_PACKAGE_TAKEN = 5      # BARU
CMD_DISPENSER_FORCE_MIDDLE_REFILL = 6    # BARU -- utk startup, lihat startup_sequence()
CMD_DISPENSER_START_MAIN = 14   # BARU -- WAJIB sebelum REQUEST_REFILL/FORCE_MIDDLE_REFILL/ACK_PACKAGE_TAKEN diterima
CMD_DISPENSER_STOP_MAIN = 15

REG_STOCKER_ALL_HOMED = 11
REG_STOCKER_RACK_OCCUPIED = 12
CMD_STOCKER_HOME_ALL = 1
CMD_STOCKER_RUN_FULL_CYCLE = 2
CMD_STOCKER_GOTO_LOAD_POSITION = 6
CMD_STOCKER_START_MAIN = 9    # BARU -- WAJIB sebelum RUN_FULL_CYCLE diterima
CMD_STOCKER_STOP_MAIN = 10


def connect(slave_id):
    """Buka koneksi Modbus RTU ke 1 node (pola sama dgn contoh-kirim-command)."""
    instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
    instr.serial.baudrate = BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout = 0.5
    instr.mode = minimalmodbus.MODE_RTU
    return instr


_seq_counter = 0   # WAJIB naik terus lintas SEMUA node -- jangan reset per-node

def send_command(instr, node_name, opcode, arg=0):
    """Kirim command 3-langkah (CMD_ARG -> CMD_SEQ -> CMD). Return seq yang dipakai."""
    global _seq_counter
    _seq_counter += 1
    seq = _seq_counter
    print(f"  -> {node_name}: opcode={opcode} arg={arg} seq={seq}")
    if DRY_RUN:
        return seq
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
    instr.write_register(REG_CMD, opcode, functioncode=6)
    return seq


def wait_ack(instr, node_name, seq, timeout=ACK_TIMEOUT_S):
    """Poll CMD_ACK_SEQ sampai match `seq`, atau timeout. Return True kalau sukses.
    CATATAN: ack cuma nandain command DITERIMA/diproses secara sinkron -- utk command yang
    hasil fisiknya makan waktu (mis. Dispenser.REQUEST_REFILL, STOCKER.HOME_ALL/RUN_FULL_CYCLE),
    ack BISA datang duluan sebelum gerakan fisik tuntas (tergantung firmware node itu deferred-ack
    atau tidak) -- pakai wait_register() terpisah kalau perlu tunggu hasil fisiknya, bukan cuma ack.
    """
    if DRY_RUN:
        print(f"  .. (DRY_RUN) anggap {node_name} ack seq={seq}")
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            ack = instr.read_register(REG_CMD_ACK_SEQ, functioncode=3)
            if ack == seq:
                return True
        except Exception as e:
            print(f"  !! {node_name}: gagal baca CMD_ACK_SEQ ({e}), retry...")
        time.sleep(ACK_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {node_name} tidak ack seq={seq} dalam {timeout}s")
    return False


def wait_register(instr, node_name, reg_addr, expected_value, timeout, poll_interval=ACK_POLL_INTERVAL_S, label=""):
    """Poll 1 register sampai == expected_value, atau timeout. Dipakai utk nunggu HASIL FISIK
    (bukan cuma ack command diterima) -- mis. PACKAGE_READY_FLAG, ALL_HOMED_FLAG."""
    if DRY_RUN:
        print(f"  .. (DRY_RUN) anggap {node_name}.{label or reg_addr} == {expected_value}")
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            val = instr.read_register(reg_addr, functioncode=3)
            if val == expected_value:
                return True
        except Exception as e:
            print(f"  !! {node_name}: gagal baca reg {reg_addr} ({e}), retry...")
        time.sleep(poll_interval)
    print(f"  !! TIMEOUT: {node_name}.{label or reg_addr} tidak capai {expected_value} dalam {timeout}s")
    return False


def find_free_rack(stocker):
    """Baca RACK_OCCUPIED_BITMASK, cari rak 1-4 pertama yang kosong (bit=0).
    DIUBAH: fisik cuma ada 4 rack (Rak 1-4, dikonfirmasi user) -- dulu loop 0-5 salah, bisa
    balikin slot 0 (gak ada fisiknya) karena bit-nya emang selalu 0 (gak pernah kepakai).

    PENTING (diperbaiki 2026-09-20 di sisi FIRMWARE, bukan di sini): dulu firmware Stocker
    menomori bit pakai INDEX ARRAY RACK_LIM (RACK_LIM[0] -> bit0), padahal RACK_LIM[0] itu
    limit switch fisik RAK 1. Jadi bit yang dibaca loop di bawah geser 1: bit Rak 1 tidak
    pernah dibaca (Rak 1 selalu kelihatan kosong -> SEMUA box ditumpuk ke Rak 1 terus),
    sementara bit 4 yang dibaca justru LIM_8 yang tidak ada fisiknya. Firmware sekarang
    memakai konvensi bit N = Rak N (bit1..bit4 = Rak 1..4, bit0 selalu 0) -- persis yang
    diasumsikan loop di bawah. WAJIB flash ulang Stocker agar perbaikan ini berlaku."""
    if DRY_RUN:
        print("  .. (DRY_RUN) anggap rack 1 kosong")
        return 1
    bitmask = stocker.read_register(REG_STOCKER_RACK_OCCUPIED, functioncode=3)
    for slot in range(1, 5):
        if not (bitmask & (1 << slot)):
            return slot
    return None   # semua rack penuh


def startup_sequence(sorter, picker, dispenser, stocker):
    """Dijalankan SEKALI di awal, sebelum masuk loop monitoring batch. Urutan (dikonfirmasi user):
    0. Picker -> GOTO_HOME DULU (manual override, WAJIB masih TEST mode -- mainModeActive
       default FALSE saat boot, jangan kirim START_MAIN dulu sebelum ini).
    1. Dispenser -> START_MAIN (WAJIB sebelum FORCE_MIDDLE_REFILL/REQUEST_REFILL/ACK_PACKAGE_TAKEN
       diterima firmware -- lihat guard applyCommand() Dispenser), lalu pastikan package sudah
       ada di posisi TENGAH -- kalau belum, paksa isi (FORCE_MIDDLE_REFILL).
    2. Picker -> START_MAIN (baru SEKARANG, setelah GOTO_HOME tuntas -- supaya MOVE_PACKAGE
       diterima nanti pas run_batch_sequence()).
    3. Stocker -> pastikan HOMED + GOTO_LOAD_POSITION (2 command ini sengaja TIDAK di-gate
       firmware, aman kapan saja), lalu START_MAIN (WAJIB sebelum RUN_FULL_CYCLE diterima).
    4. Baru jalankan SORTER -- START-nya sendiri yang set mainModeActive=true di firmware Sorter.
    5. Semua node berjalan (MAIN mode aktif semua) -- lanjut ke loop monitoring batch.
    """
    print("\n=== STARTUP SEQUENCE ===")

    print("[0/5] Picker -> GOTO_HOME (masih TEST mode, sebelum START_MAIN)")
    seq = send_command(picker, "PICKER", CMD_PICKER_GOTO_HOME)
    if not wait_ack(picker, "PICKER", seq):
        return False

    print("[1/5] Dispenser -> START_MAIN, lalu cek MIDDLE_PACKAGE_PRESENT")
    seq = send_command(dispenser, "DISPENSER", CMD_DISPENSER_START_MAIN)
    if not wait_ack(dispenser, "DISPENSER", seq):
        return False
    middle_present = True if DRY_RUN else (dispenser.read_register(REG_DISPENSER_MIDDLE_PRESENT, functioncode=3) == 1)
    if not middle_present:
        print("      Tengah KOSONG -- FORCE_MIDDLE_REFILL")
        seq = send_command(dispenser, "DISPENSER", CMD_DISPENSER_FORCE_MIDDLE_REFILL)
        if not wait_ack(dispenser, "DISPENSER", seq):
            return False
        if not wait_register(dispenser, "DISPENSER", REG_DISPENSER_MIDDLE_PRESENT, 1,
                              MIDDLE_REFILL_TIMEOUT_S, label="MIDDLE_PACKAGE_PRESENT"):
            return False
    else:
        print("      Tengah sudah terisi, lanjut")

    print("[2/5] Picker -> START_MAIN")
    seq = send_command(picker, "PICKER", CMD_PICKER_START_MAIN)
    if not wait_ack(picker, "PICKER", seq):
        return False

    print("[3/5] Stocker -> pastikan HOMED, GOTO_LOAD_POSITION, lalu START_MAIN")
    all_homed = True if DRY_RUN else (stocker.read_register(REG_STOCKER_ALL_HOMED, functioncode=3) == 1)
    if not all_homed:
        print("      Belum homed -- HOME_ALL dulu (bisa lama, 3 axis berurutan)")
        seq = send_command(stocker, "STOCKER", CMD_STOCKER_HOME_ALL)
        if not wait_ack(stocker, "STOCKER", seq, timeout=HOMING_TIMEOUT_S):
            return False
    seq = send_command(stocker, "STOCKER", CMD_STOCKER_GOTO_LOAD_POSITION)
    if not wait_ack(stocker, "STOCKER", seq):
        return False
    seq = send_command(stocker, "STOCKER", CMD_STOCKER_START_MAIN)
    if not wait_ack(stocker, "STOCKER", seq):
        return False

    print("[4/5] Sorter -> START (otomatis set mainModeActive=true di firmware Sorter)")
    seq = send_command(sorter, "SORTER", CMD_SORTER_START)
    if not wait_ack(sorter, "SORTER", seq):
        return False

    print("=== STARTUP SELESAI -- semua node MAIN mode aktif ===\n")
    return True


def shutdown_sequence(sorter, picker, dispenser, stocker):
    """Dipanggil pas orchestrator berhenti (Ctrl+C) -- balikin SEMUA node ke TEST mode
    (STOP/STOP_MAIN), supaya script test manual (test-dispenser-*, test-stocker-*) bisa
    langsung dipakai lagi tanpa perlu power-cycle fisik tiap node. Best-effort (fire-and-
    forget, TIDAK nunggu ack) -- kalau ada node lagi fault/gak respon pas shutdown, tetap
    lanjut ke node berikutnya, jangan sampai shutdown sendiri nyangkut nunggu ack yg gak
    pernah datang."""
    print("\n=== SHUTDOWN SEQUENCE -- balik semua node ke TEST mode ===")
    send_command(sorter, "SORTER", CMD_SORTER_STOP)
    send_command(picker, "PICKER", CMD_PICKER_STOP_MAIN)
    send_command(dispenser, "DISPENSER", CMD_DISPENSER_STOP_MAIN)
    send_command(stocker, "STOCKER", CMD_STOCKER_STOP_MAIN)
    print("=== SHUTDOWN SELESAI ===\n")


def run_batch_sequence(sorter, picker, dispenser, stocker):
    print(f"\n=== BATCH TRIGGER: PASS_COUNT capai {BATCH_SIZE} ===")

    rack_idx = find_free_rack(stocker)
    if rack_idx is None:
        print("!! SEMUA RACK PENUH -- batch DITAHAN, tidak reset counter, "
              "tidak trigger Picker/Stocker. Kosongkan rack dulu.")
        return False

    # DIPERBAIKI (bug ditemukan): urutan lama kirim Picker.MOVE_PACKAGE SEBELUM
    # Dispenser.REQUEST_REFILL -- padahal REQUEST_REFILL itu yang MEMINDAHKAN package dari
    # TENGAH ke UJUNG. Sekarang REQUEST_REFILL dulu, TUNGGU PACKAGE_READY_FLAG, baru Picker jalan.
    print("[1/6] Dispenser -> REQUEST_REFILL (maju krn package TENGAH sudah full)")
    seq = send_command(dispenser, "DISPENSER", CMD_DISPENSER_REQUEST_REFILL)
    if not wait_ack(dispenser, "DISPENSER", seq):
        return False

    print("[2/6] Tunggu Dispenser.PACKAGE_READY_FLAG == 1 (package sampai UJUNG)")
    if not wait_register(dispenser, "DISPENSER", REG_DISPENSER_PACKAGE_READY, 1,
                          PACKAGE_READY_TIMEOUT_S, label="PACKAGE_READY_FLAG"):
        return False

    print("[3/6] Stocker -> Load Position (idempotent, aman kalau sudah di situ)")
    seq = send_command(stocker, "STOCKER", CMD_STOCKER_GOTO_LOAD_POSITION)
    if not wait_ack(stocker, "STOCKER", seq):
        return False

    print("[4/6] Picker -> angkat package dari UJUNG Dispenser, taruh di Load Position")
    seq = send_command(picker, "PICKER", CMD_PICKER_MOVE_PACKAGE)
    if not wait_ack(picker, "PICKER", seq):
        return False

    # WAJIB dikirim SEBELUM REQUEST_REFILL berikutnya -- firmware Dispenser MENOLAK REQUEST_REFILL
    # baru selama PACKAGE_READY_FLAG masih 1 (lihat guard di applyCommand()).
    print("[5/6] Dispenser -> ACK_PACKAGE_TAKEN (WAJIB, buka jalan refill berikutnya)")
    seq = send_command(dispenser, "DISPENSER", CMD_DISPENSER_ACK_PACKAGE_TAKEN)
    if not wait_ack(dispenser, "DISPENSER", seq):
        return False

    print(f"[6/6] Stocker -> simpan package ke rack slot {rack_idx}")
    seq = send_command(stocker, "STOCKER", CMD_STOCKER_RUN_FULL_CYCLE, arg=rack_idx)
    if not wait_ack(stocker, "STOCKER", seq, timeout=30.0):   # gerak X+Z+push, kasih waktu lebih
        return False

    print("Sorter -> RESET_COUNTERS")
    seq = send_command(sorter, "SORTER", CMD_SORTER_RESET_COUNTERS)
    if not wait_ack(sorter, "SORTER", seq):
        return False

    print("=== BATCH SELESAI ===\n")
    return True


def main():
    print(f"[BOOT] Orkestrator batch mulai -- port={SERIAL_PORT} baud={BAUDRATE} "
          f"batch_size={BATCH_SIZE} DRY_RUN={DRY_RUN}")
    sorter = connect(SLAVE['SORTER'])
    picker = connect(SLAVE['PICKER'])
    dispenser = connect(SLAVE['DISPENSER'])
    stocker = connect(SLAVE['STOCKER'])

    if not startup_sequence(sorter, picker, dispenser, stocker):
        print("!! STARTUP GAGAL -- cek fault tiap node manual sebelum lanjut.")
        if not DRY_RUN:
            sys.exit(1)

    while True:
        try:
            if DRY_RUN:
                count = BATCH_SIZE   # simulasi langsung capai batch tiap iterasi DRY_RUN
            else:
                count = sorter.read_register(REG_SORTER_PASS_COUNT, functioncode=3)

            if count >= BATCH_SIZE:
                ok = run_batch_sequence(sorter, picker, dispenser, stocker)
                if not ok:
                    print("!! Batch GAGAL di tengah jalan -- cek fault tiap node manual "
                          "sebelum lanjut (jangan auto-retry, bisa tabrakan mekanis).")
                    if DRY_RUN:
                        break
                    time.sleep(5)   # jeda sebelum re-poll, hindari spam retry saat fault
                if DRY_RUN:
                    break   # 1x demo cukup buat DRY_RUN

            time.sleep(POLL_INTERVAL_S)

        except KeyboardInterrupt:
            print("\n[STOP] Dihentikan manual (Ctrl+C)")
            shutdown_sequence(sorter, picker, dispenser, stocker)
            sys.exit(0)
        except Exception as e:
            print(f"!! Error tak terduga di loop utama: {e}")
            time.sleep(2)


if __name__ == '__main__':
    main()
