#!/usr/bin/env python3
"""TEST PICKER — pipeline lengkap (versi 2026-09-22).

Menggantikan `test-picker-manual-sequence 20260919.py`, disusun mengikuti pola
`test-dispenser-pipeline 20260920.py` yang sudah terbukti di lapangan.

Arm robot HANYA memindahkan package penuh dari ujung Dispenser ke Load Position
Stocker. Objek satuan tidak pernah disentuhnya, jadi hanya tiga pose yang berarti:

    0 = HOME             titik istirahat
    1 = PACKAGE_PICKUP   ambil package di ujung Dispenser
    2 = LIFT_LOAD        taruh package di Load Position Stocker

ALUR

  TAHAP A — verifikasi (TEST mode, sekali di awal)
    [1] Pastikan TEST mode
    [2] Pose 0 HOME
    [3] Pose 1 PACKAGE_PICKUP
    [4] PICK   gripper menutup di titik ambil
    [5] PLACE  gripper membuka lagi
    [6] Pose 2 LIFT_LOAD
    [7] Pose 0 HOME

  TAHAP B — produksi (MAIN mode, berulang)
    [8]  Tunggu tanda "package siap diambil"
    [9]  MOVE_PACKAGE -- satu siklus penuh, dikemudikan firmware
    [10] Tunggu siklus tuntas
    [11] Beri tahu Dispenser bahwa package sudah diambil  (kalau dirantai)
    [12] Kembali ke [8]

TANDA "PACKAGE SIAP" datang dari salah satu, tergantung PAKAI_DISPENSER:
  False (bawaan) -- Enter di Orange Pi. Picker diuji sendirian.
  True           -- register PACKAGE_READY_FLAG milik Dispenser dibaca langsung.
                    Ini uji gabungan dua node, persis seperti produksi nanti.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial

Jalankan LANGSUNG DI TERMINAL (bukan lewat pipe/nohup -- lihat catatan stdin):
    python3 'test-picker-pipeline 20260922.py'
"""

import select
import signal
import sys
import time

import minimalmodbus
import serial

# ============================================================
# KONFIGURASI
# ============================================================
SERIAL_PORT = '/dev/ttyS3'
BAUDRATE = 19200
PICKER_SLAVE_ID = 2
DISPENSER_SLAVE_ID = 3

# True = tanda "package siap" dibaca dari Dispenser, dan Dispenser diberi tahu setelah
# package diambil. Butuh Dispenser hidup di bus DAN sudah START_MAIN.
PAKAI_DISPENSER = False

ACK_TIMEOUT_S = 20.0            # MOVE_PACKAGE baru di-ack setelah SELURUH siklus tuntas
POLL_INTERVAL_S = 0.2
TUNGGU_TIMEOUT_S = 300.0
GERAK_TIMEOUT_S = 60.0

DRY_RUN = False

# --- Register PICKER ---
REG_STATE = 0
REG_FAULT_CODE = 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_CURRENT_POSE = 10
REG_ACTIVITY_CODE = 11
REG_MAIN_MODE_ACTIVE = 15
REG_MENU_ACTIVE = 16

# --- Register DISPENSER (dipakai hanya kalau PAKAI_DISPENSER) ---
REG_DISP_PACKAGE_READY = 15

# --- Opcode PICKER ---
CMD_GOTO_HOME = 2
CMD_PICK = 5
CMD_PLACE = 6
CMD_RESET_FAULT = 7
CMD_MOVE_PACKAGE = 8
CMD_START_MAIN = 11
CMD_STOP_MAIN = 12
CMD_GOTO_POSE_N = 13      # BARU -- arg = nomor slot pose

# --- Opcode DISPENSER ---
CMD_DISP_ACK_TAKEN = 5

POSE_NAMA = {0: "HOME", 1: "PACKAGE_PICKUP", 2: "LIFT_LOAD"}
STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING_OR_MOVING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM", 1: "MENUJU_HOME", 5: "MENGAMBIL", 6: "MELETAKKAN",
    7: "NAIK_CLEARANCE", 8: "BERGERAK", 9: "POST_PLACE_GERAK",
    10: "MENUJU_PACKAGE_PICKUP", 11: "MENUJU_LIFT_LOAD",
    90: "FAULT_AKTIF", 91: "ESTOP_AKTIF",
}


def connect(slave_id):
    instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
    instr.serial.baudrate = BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout = 0.5
    instr.mode = minimalmodbus.MODE_RTU
    instr.clear_buffers_before_each_transaction = True
    return instr


class RegisterTidakAda(Exception):
    """Register memang tidak ada di firmware -- beda dari gagal baca sesaat."""


def baca(instr, reg, nama, percobaan=3, diam=False):
    """Dibedakan dengan sengaja: 'register tidak ada' (firmware lama) versus 'pembacaan
    meleset sesaat' (gangguan bus). Menyamakan keduanya membuat satu kali meleset
    disimpulkan sebagai firmware salah -- diagnosis yang menyesatkan."""
    if DRY_RUN:
        return 0
    terakhir = None
    for _ in range(percobaan):
        try:
            return instr.read_register(reg, functioncode=3)
        except minimalmodbus.IllegalRequestError:
            raise RegisterTidakAda(f"register {reg} ({nama}) tidak ada di firmware")
        except Exception as e:
            terakhir = e
            time.sleep(0.05)
    if not diam:
        print(f"  !! gagal baca {nama} setelah {percobaan}x ({terakhir})")
    return None


_seq = 0


def kirim(instr, nama, opcode, arg=0, timeout=ACK_TIMEOUT_S, slave="PICKER"):
    """PENTING -- ACK BUKAN BERARTI BERHASIL. Firmware menulis CMD_ACK_SEQ tanpa syarat,
    termasuk untuk command yang baru saja ditolaknya. Keberhasilan dinilai dari
    STATE/FAULT_CODE/ACTIVITY_CODE dan dari gerakan fisiknya."""
    global _seq
    _seq += 1
    seq = _seq
    print(f"  -> {slave}: {nama} (opcode={opcode} arg={arg} seq={seq})")
    if DRY_RUN:
        return True
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
    instr.write_register(REG_CMD, opcode, functioncode=6)

    batas = time.monotonic() + timeout
    while time.monotonic() < batas:
        if baca(instr, REG_CMD_ACK_SEQ, "CMD_ACK_SEQ", diam=True) == seq:
            return True
        time.sleep(POLL_INTERVAL_S)
    print(f"  !! {nama} tidak di-ack dalam {timeout}s")
    return False


def status(instr, judul=""):
    if DRY_RUN:
        return
    st = baca(instr, REG_STATE, "STATE", diam=True)
    akt = baca(instr, REG_ACTIVITY_CODE, "ACTIVITY_CODE", diam=True)
    pose = baca(instr, REG_CURRENT_POSE, "CURRENT_POSE", diam=True)
    print(f"     STATE={st}({STATE_NAMES.get(st, '?')}) "
          f"FAULT={baca(instr, REG_FAULT_CODE, 'FAULT_CODE', diam=True)} "
          f"ACTIVITY={akt}({ACTIVITY_NAMES.get(akt, '?')}) "
          f"POSE={pose}({POSE_NAMA.get(pose, '?')})"
          f"{('  <- ' + judul) if judul else ''}")


try:
    KEYBOARD_ADA = sys.stdin.isatty()
except Exception:
    KEYBOARD_ADA = False


def stdin_siap():
    """HATI-HATI: kalau stdin bukan terminal (nohup, pipe, `< /dev/null`), select()
    melaporkannya SELALU siap karena sudah EOF. Kalau itu dianggap 'Enter ditekan', setiap
    konfirmasi langsung dilewati tanpa menunggu siapa pun. Karena itu keyboard dimatikan
    total di luar terminal, dan pembacaan kosong tidak pernah dihitung sebagai penekanan."""
    if not KEYBOARD_ADA:
        return False
    try:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return False
        return sys.stdin.readline() != ""
    except Exception:
        return False


def tunggu_diam(instr, judul, timeout=GERAK_TIMEOUT_S):
    """Menunggu lengan benar-benar berhenti.

    Ack sebuah perintah gerak hanya berarti perintahnya diterima; trajektori enam joint
    masih berjalan beberapa detik sesudahnya. Yang ditunggu di sini ACTIVITY_CODE kembali
    DIAM, sambil mencetak tiap transisi supaya tahapannya terlihat."""
    if DRY_RUN:
        return True
    print(f"  .. tunggu {judul} selesai")
    terakhir, batas = None, time.monotonic() + timeout
    time.sleep(0.4)   # beri waktu gerakan benar-benar mulai sebelum menilai "sudah diam"
    while time.monotonic() < batas:
        a = baca(instr, REG_ACTIVITY_CODE, "ACTIVITY_CODE", diam=True)
        if a is not None and a != terakhir:
            print(f"     [gerak] {a} ({ACTIVITY_NAMES.get(a, '?')})")
            terakhir = a
        if a == 0:
            return True
        if a in (90, 91):
            print("  !! Lengan FAULT/E-STOP di tengah gerakan.")
            return False
        time.sleep(POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {judul} tidak selesai dalam {timeout}s")
    return False


def pastikan_test_mode(instr):
    if DRY_RUN:
        return True
    st = baca(instr, REG_STATE, "STATE")
    if st is None:
        print("!! Picker tidak menjawab Modbus. Cek daya node dan kabel RS485 A/B.")
        print("   Pastikan juga tidak ada program Modbus lain yang jalan di /dev/ttyS3.")
        return False

    try:
        if baca(instr, REG_MENU_ACTIVE, "MENU_ACTIVE") == 1:
            print("!! Operator sedang di menu kalibrasi LCD -- SEMUA command Modbus diabaikan")
            print("   tanpa dibalas, jadi dari sini mirip node mati. Tekan D sampai layar utama.")
            return False
    except RegisterTidakAda:
        print("!! Register MENU_ACTIVE belum ada -- firmware Picker masih lama. Flash dulu.")
        return False

    fault = baca(instr, REG_FAULT_CODE, "FAULT_CODE")
    if fault:
        print(f"!! Picker FAULT (code={fault}).")
        kirim(instr, "RESET_FAULT", CMD_RESET_FAULT, timeout=5.0)
        if baca(instr, REG_FAULT_CODE, "FAULT_CODE"):
            print("!! Masih FAULT setelah reset. Berhenti.")
            return False

    if baca(instr, REG_MAIN_MODE_ACTIVE, "MAIN_MODE_ACTIVE") == 1:
        print("  .. MAIN aktif -> STOP_MAIN dulu supaya command manual diterima")
        kirim(instr, "STOP_MAIN", CMD_STOP_MAIN, timeout=5.0)
    print("  .. TEST mode OK")
    return True


def ke_pose(instr, slot):
    if not kirim(instr, f"GOTO pose {slot} ({POSE_NAMA[slot]})", CMD_GOTO_POSE_N, arg=slot):
        return False
    ok = tunggu_diam(instr, f"gerak ke {POSE_NAMA[slot]}")
    status(instr)
    return ok


# ============================================================
# TAHAP A -- verifikasi pose & gripper
# ============================================================
def tahap_verifikasi(picker):
    print("\n=== [2] Pose 0 HOME ===")
    if not ke_pose(picker, 0):
        return False

    print("\n=== [3] Pose 1 PACKAGE_PICKUP ===")
    if not ke_pose(picker, 1):
        return False

    print("\n=== [4] PICK -- gripper menutup di titik ambil ===")
    if not kirim(picker, "PICK", CMD_PICK) or not tunggu_diam(picker, "PICK"):
        return False

    print("\n=== [5] PLACE -- gripper membuka lagi ===")
    if not kirim(picker, "PLACE", CMD_PLACE) or not tunggu_diam(picker, "PLACE"):
        return False

    print("\n=== [6] Pose 2 LIFT_LOAD ===")
    if not ke_pose(picker, 2):
        return False

    print("\n=== [7] Kembali pose 0 HOME ===")
    return ke_pose(picker, 0)


# ============================================================
# TAHAP B -- siklus produksi
# ============================================================
def tunggu_package_siap(picker, dispenser, nomor):
    print(f"\n=== [8] Putaran #{nomor} -- tunggu tanda 'package siap diambil' ===")
    if DRY_RUN:
        return True

    if PAKAI_DISPENSER:
        print("     menunggu PACKAGE_READY_FLAG Dispenser jadi 1")
    if KEYBOARD_ADA:
        print("     atau tekan Enter di sini")
    else:
        print("     (stdin bukan terminal -- konfirmasi lewat Enter DIMATIKAN)")

    batas = time.monotonic() + TUNGGU_TIMEOUT_S
    lapor = time.monotonic() + 10.0
    while time.monotonic() < batas:
        if stdin_siap():
            print("  .. dikonfirmasi dari Orange Pi (Enter)")
            return True
        if PAKAI_DISPENSER and dispenser is not None:
            siap = baca(dispenser, REG_DISP_PACKAGE_READY, "PACKAGE_READY_FLAG", diam=True)
            if siap == 1:
                print("  .. Dispenser melaporkan package siap diambil")
                return True
        if time.monotonic() >= lapor:
            lapor = time.monotonic() + 10.0
            print(f"     .. masih menunggu, sisa {int(batas - time.monotonic())}s")
        time.sleep(POLL_INTERVAL_S)

    print("  !! TIMEOUT: tidak ada tanda package siap")
    return False


def satu_putaran(picker, dispenser, nomor):
    if not tunggu_package_siap(picker, dispenser, nomor):
        return False

    print("\n=== [9] MOVE_PACKAGE -- satu siklus penuh ===")
    print("     Urutannya dikemudikan FIRMWARE, bukan script ini:")
    print("     home -> naik -> pose 1 -> ambil -> pose 2 -> taruh -> mundur -> home")
    # MOVE_PACKAGE butuh MAIN. Ack-nya sengaja ditunda firmware sampai SELURUH antrian
    # tuntas, jadi ack di sini sudah berarti siklusnya benar-benar selesai.
    if not kirim(picker, "MOVE_PACKAGE", CMD_MOVE_PACKAGE):
        return False

    print("\n=== [10] Pastikan lengan sudah diam ===")
    if not tunggu_diam(picker, "siklus MOVE_PACKAGE"):
        return False
    status(picker, "akhir siklus")

    if PAKAI_DISPENSER and dispenser is not None:
        print("\n=== [11] Beri tahu Dispenser: package sudah diambil ===")
        kirim(dispenser, "ACK_PACKAGE_TAKEN", CMD_DISP_ACK_TAKEN, timeout=5.0, slave="DISPENSER")

    print("\n=== [12] Kembali ke [8] ===")
    return True


# ============================================================
class Dihentikan(Exception):
    """Supaya SIGTERM ikut menjalankan pembersihan, sama seperti Ctrl+C."""


def _on_sigterm(signum, frame):
    raise Dihentikan()


def main():
    signal.signal(signal.SIGTERM, _on_sigterm)

    print("=" * 70)
    print("TEST PICKER -- pipeline lengkap")
    print(f"port={SERIAL_PORT} slave={PICKER_SLAVE_ID} DRY_RUN={DRY_RUN} "
          f"PAKAI_DISPENSER={PAKAI_DISPENSER}")
    print("PASTIKAN AREA GERAK LENGAN KOSONG sebelum melanjutkan.")
    print("Ctrl+C kapan saja untuk berhenti.")
    print("=" * 70)

    picker = connect(PICKER_SLAVE_ID)
    dispenser = connect(DISPENSER_SLAVE_ID) if PAKAI_DISPENSER else None

    try:
        print("\n=== [1] Pastikan Picker TEST mode ===")
        if not pastikan_test_mode(picker):
            return 1
        status(picker, "kondisi awal")

        if not tahap_verifikasi(picker):
            print("\n!! Verifikasi pose/gripper gagal. Perbaiki kalibrasi dulu sebelum siklus penuh.")
            return 1

        print("\n" + "=" * 70)
        print("TAHAP A SELESAI -- ketiga pose dan gripper sudah terverifikasi.")
        print("Lanjut ke siklus produksi. MAIN akan diaktifkan.")
        print("=" * 70)

        if not kirim(picker, "START_MAIN", CMD_START_MAIN, timeout=5.0):
            return 1

        putaran = 0
        while True:
            putaran += 1
            if not satu_putaran(picker, dispenser, putaran):
                print(f"\n!! Putaran #{putaran} berhenti sebelum tuntas.")
                return 1

    except (KeyboardInterrupt, Dihentikan):
        print("\n\n>> Dihentikan operator.")
        return 130
    finally:
        # Sengaja TIDAK menggerakkan lengan saat keluar. Kalau gripper sedang memegang
        # package, memulangkannya ke home justru menjatuhkan muatan di jalan. Yang
        # dilakukan hanya mengembalikan node ke TEST mode supaya command produksi tidak
        # tertinggal aktif.
        print("\n>> Kembalikan Picker ke TEST mode...")
        try:
            kirim(picker, "STOP_MAIN", CMD_STOP_MAIN, timeout=5.0)
        except Exception as e:
            print(f"  !! gagal kirim STOP_MAIN ({e})")
        print(">> Lengan DIBIARKAN di posisinya. Periksa apakah gripper masih memegang package.")


if __name__ == '__main__':
    sys.exit(main())
