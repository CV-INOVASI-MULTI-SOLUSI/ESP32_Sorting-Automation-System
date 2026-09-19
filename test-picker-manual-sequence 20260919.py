#!/usr/bin/env python3
"""
============================================================
 TEST PICKER -- urutan manual GOTO_HOME -> GOTO_PASS -> PICK -> GOTO_HOME -> PLACE
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node PICKER
(slave 2). Nyoba SATU-SATU command "manual override" (TEST mode) yang ada di
firmware sekarang -- GOTO_HOME/GOTO_PASS/PICK/PLACE -- biar tiap gerakan bisa
diamati terpisah, bukan langsung full-cycle otomatis.

SETUP (1x):
  0. Pastikan Picker masih TEST mode (auto STOP_MAIN kalau kesangkut MAIN,
     sama pola dgn test-dispenser-full-protocol -- lihat ensure_test_mode())

URUTAN MANUAL (jalan sekali, bukan loop -- edit MANUAL_STEPS di bawah kalau
mau ubah urutan/tambah langkah):
  1. GOTO_HOME      -- arm ke posisi Home
  2. GOTO_PASS      -- arm ke posisi Pass (titik ambil objek)
  3. PICK           -- gripper ambil (di posisi SEKARANG, GOTO_PASS dulu di atas)
  4. GOTO_HOME      -- arm balik ke Home (bawa objek yang barusan diambil)
  5. PLACE          -- gripper lepas (di posisi SEKARANG, Home)

CATATAN:
  - GOTO_HOME/GOTO_PASS/PICK/PLACE ini SEMUA "manual override" -- cuma diterima
    firmware selama TEST mode (mainModeActive == FALSE). Kalau Orange Pi lagi
    START_MAIN Picker (mis. orchestrator produksi jalan), semua command ini
    DITOLAK DIAM-DIAM (ack tetap "sukses", gak ada efek fisik). ensure_test_mode()
    di bawah otomatis cek & kirim STOP_MAIN kalau perlu.
  - GOTO_REJECT SENGAJA TIDAK ADA di sini -- sudah dihapus dari firmware (Picker
    fisik cuma ambil dari jalur PASS, reject ditangani hopper SORTER).
  - MOVE_PACKAGE (produksi asli, full cycle home->pickup->place->home otomatis)
    BUTUH MAIN mode -- disediakan lewat run_move_package(), TAPI TIDAK dipanggil
    otomatis di main(). Uncomment manggilnya sendiri kalau mau test itu (lihat
    komentar di main()) -- gerakan penuh, pastikan area aman dulu.

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
PICKER_SLAVE_ID = 2

ACK_TIMEOUT_S = 15.0
ACK_POLL_INTERVAL_S = 0.2
MOVE_WATCH_S = 6.0             # intip ACTIVITY_CODE sekian detik abis tiap gerakan (arm 6 joint,
                                 # bisa makan beberapa detik tergantung trajStepUs/Interval)

DRY_RUN = False   # saat True, skrip cuma PRINT rencana aksi, TIDAK kirim apapun ke bus.

# --- Register (lihat ESP32_SortingAutomation_Picker/include/registers.h) ---
REG_STATE = 0
REG_FAULT_CODE = 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_CURRENT_POSE = 10
REG_ACTIVITY_CODE = 11
REG_MAIN_MODE_ACTIVE = 15

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING_OR_MOVING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM", 1: "MENUJU_HOME", 2: "MENUJU_PASS", 4: "MENUJU_LIFT",
    5: "MENGAMBIL", 6: "MELETAKKAN", 7: "NAIK_CLEARANCE", 8: "BERGERAK",
    9: "POST_PLACE_GERAK", 90: "FAULT_AKTIF", 91: "ESTOP_AKTIF",
}

# --- Opcode PICKER ---
CMD_RUN_SEQUENCE = 1
CMD_GOTO_HOME = 2
CMD_GOTO_PASS = 3
CMD_PICK = 5
CMD_PLACE = 6
CMD_RESET_FAULT = 7
CMD_MOVE_PACKAGE = 8
CMD_START_MAIN = 11
CMD_STOP_MAIN = 12


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
    print(f"  -> PICKER: {label} (opcode={opcode} arg={arg} seq={seq})")
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
            print(f"  !! PICKER: gagal baca CMD_ACK_SEQ ({e}), retry...")
        time.sleep(ACK_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak ack seq={seq} dalam {timeout}s")
    return False


def print_status(instr, label):
    """Debug otomatis -- baca STATE/FAULT_CODE/ACTIVITY_CODE/CURRENT_POSE tiap command,
    sama pola dgn test-dispenser-full-protocol.py (kepake buat diagnosa jarak jauh
    tanpa Serial USB kalau ada gerakan yang gak sesuai harapan)."""
    if DRY_RUN:
        return
    try:
        state = instr.read_register(REG_STATE, functioncode=3)
        fault = instr.read_register(REG_FAULT_CODE, functioncode=3)
        activity = instr.read_register(REG_ACTIVITY_CODE, functioncode=3)
        pose = instr.read_register(REG_CURRENT_POSE, functioncode=3)
        print(f"     [status] STATE={state}({STATE_NAMES.get(state, '?')}) "
              f"FAULT_CODE={fault} ACTIVITY_CODE={activity}({ACTIVITY_NAMES.get(activity, '?')}) "
              f"CURRENT_POSE={pose}")
    except Exception as e:
        print(f"     [status] gagal baca ({e})")


def do_command(instr, label, opcode, arg=0, timeout=ACK_TIMEOUT_S, watch_activity_s=0.0):
    seq = send_command(instr, label, opcode, arg)
    ok = wait_ack(instr, label, seq, timeout=timeout)
    print_status(instr, label)
    if ok and watch_activity_s > 0 and not DRY_RUN:
        # Gerakan arm (trajectory 6 joint) butuh waktu SETELAH ack -- ack cuma nandain
        # command DITERIMA, bukan gerakan SELESAI. Intip ACTIVITY_CODE biar transisi
        # MENUJU_HOME/MENGAMBIL/MELETAKKAN/dll kelihatan, sampai balik DIAM (selesai).
        last_val = None
        deadline = time.monotonic() + watch_activity_s
        while time.monotonic() < deadline:
            try:
                val = instr.read_register(REG_ACTIVITY_CODE, functioncode=3)
                if val != last_val:
                    print(f"     [watch] ACTIVITY_CODE = {val} ({ACTIVITY_NAMES.get(val, f'?({val})')})")
                    last_val = val
                    if val == 0 and last_val is not None:
                        break   # balik DIAM -- gerakan selesai, gak perlu nunggu sisa watch_activity_s
            except Exception as e:
                print(f"     [watch] gagal baca ({e})")
            time.sleep(0.2)
    return ok


def ensure_test_mode(instr):
    """Sama pola dgn test-dispenser-full-protocol.py -- GOTO_HOME/GOTO_PASS/PICK/PLACE
    cuma diterima firmware selama TEST mode. Cek MAIN_MODE_ACTIVE, kirim STOP_MAIN
    kalau kesangkut MAIN dari sesi produksi/orchestrator sebelumnya."""
    if DRY_RUN:
        print("  .. (DRY_RUN) lewati cek MAIN/TEST mode")
        return True
    try:
        main_active = instr.read_register(REG_MAIN_MODE_ACTIVE, functioncode=3)
    except Exception as e:
        print(f"  !! Gagal baca MAIN_MODE_ACTIVE ({e}) -- lanjut anggap TEST mode")
        return True
    if main_active == 1:
        print("  .. Picker masih MAIN mode (sisa sesi produksi sebelumnya) -- kirim STOP_MAIN")
        return do_command(instr, "STOP_MAIN", CMD_STOP_MAIN)
    print("  .. Picker sudah TEST mode, lanjut")
    return True


def run_move_package(instr):
    """BARU -- test PRODUKSI ASLI (MOVE_PACKAGE, opcode 8): full cycle
    home->PACKAGE_PICKUP(pose4)->pick->LIFT_LOAD(pose5)->place->home, OTOMATIS,
    TANPA jeda antar-tahap. BUTUH MAIN mode (kebalikan dari 5 langkah manual di
    atas) -- kirim START_MAIN dulu, jalanin, balik STOP_MAIN lagi di akhir biar
    Picker balik TEST mode (aman dipakai manual lagi setelahnya).
    TIDAK dipanggil otomatis dari main() -- panggil sendiri kalau mau test ini,
    pastikan area gerak arm AMAN dulu (full cycle, gak ada jeda konfirmasi)."""
    print("\n=== [MOVE_PACKAGE] START_MAIN dulu (wajib) ===")
    if not do_command(instr, "START_MAIN", CMD_START_MAIN):
        return False
    print("\n=== [MOVE_PACKAGE] Jalankan full cycle produksi ===")
    ok = do_command(instr, "MOVE_PACKAGE", CMD_MOVE_PACKAGE, watch_activity_s=15.0)
    print("\n=== [MOVE_PACKAGE] STOP_MAIN -- balik ke TEST mode ===")
    do_command(instr, "STOP_MAIN", CMD_STOP_MAIN)
    return ok


def main():
    print(f"[BOOT] Test Picker Manual Sequence -- port={SERIAL_PORT} baud={BAUDRATE} DRY_RUN={DRY_RUN}")
    picker = connect(PICKER_SLAVE_ID)

    print("\n=== [0] Pastikan Picker TEST mode ===")
    if not ensure_test_mode(picker):
        print("!! Gagal pastikan TEST mode -- cek fault Picker manual. Berhenti.")
        sys.exit(1)

    # Urutan manual -- (label, opcode, watch_activity_s). Edit di sini kalau mau
    # ubah urutan/tambah langkah.
    MANUAL_STEPS = [
        ("GOTO_HOME",  CMD_GOTO_HOME, 6.0),
        ("GOTO_PASS",  CMD_GOTO_PASS, 6.0),
        ("PICK",       CMD_PICK,      4.0),
        ("GOTO_HOME",  CMD_GOTO_HOME, 6.0),
        ("PLACE",      CMD_PLACE,     4.0),
    ]

    for i, (label, opcode, watch_s) in enumerate(MANUAL_STEPS, start=1):
        print(f"\n=== [{i}/{len(MANUAL_STEPS)}] {label} ===")
        if not do_command(picker, label, opcode, watch_activity_s=watch_s):
            print(f"!! Gagal di langkah {label} -- cek fault Picker manual. Berhenti.")
            sys.exit(1)

    print("\n=== SELESAI -- 5 langkah manual dijalankan ===")

    # Uncomment kalau mau LANJUT test produksi asli (MOVE_PACKAGE) setelah manual
    # sequence di atas -- PASTIKAN area gerak arm aman dulu:
    # run_move_package(picker)


if __name__ == '__main__':
    main()
