#!/usr/bin/env python3
"""TEST DISPENSER — alur pipeline 16 langkah (versi 2026-09-20).

Menggantikan `test-dispenser-full-protocol 20260919.py` untuk pengujian Dispenser.
Dua perbedaan mendasar dari script lama:

1. TIDAK ADA LAGI PLACEHOLDER TIMING. Dua konfirmasi ("package penuh" dan "package sudah
   diambil") sekarang datang dari kejadian NYATA: tombol fisik di panel, atau Enter di
   Orange Pi. Script lama menebaknya dengan delay tetap, yang tidak pernah benar-benar
   mencerminkan keadaan fisik.

2. TIDAK ADA LAGI GUARD PROX_2 REAKTIF. Di script lama, conveyor dinyalakan lalu ada
   pengawas terpisah yang bisa mematikannya sewaktu-waktu -- dua pihak mengatur satu
   aktuator yang sama, dan dari situ muncul gejala "conveyor baru jalan langsung mati lagi".
   Di sini conveyor hidup/mati HANYA pada langkah yang disebutkan eksplisit. Satu pengatur,
   satu aktuator, jadi jenis bentrok itu tidak bisa terjadi sama sekali.

3. SERVO TIDAK PERNAH BERGERAK SELAGI PACKAGE BERJALAN. Servo (PCA9685) dan kedua proximity
   (MCP23017) berbagi satu bus I2C. Kalau servo digerakkan bersamaan dengan package yang
   sedang melintasi sensor, bus paling ramai justru pada saat pembacaan paling menentukan.
   Seluruh gerakan servo karena itu ditempatkan di jendela saat conveyor BERHENTI.

ALUR (sesuai permintaan user):

  SETUP (sekali di awal — mengisi package PERTAMA)
    [1] Conveyor menyala
    [2] Servo 1 tarik
    [3] Package pertama jatuh
    [4] Servo 1 dorong
    [5] Servo 2 tarik
    [6] Servo 2 tahan
    [7] Servo 2 dorong

  LOOP (berulang)
    [8]  Tunggu package menyentuh PROX_2
    [9]  Conveyor mati
    [10] Servo 1 tarik
    [11] Tunggu konfirmasi "package sudah PENUH"      <- tombol BUTTON_2 atau Enter
    [12] Conveyor hidup -- package berikutnya mulai berjalan
    [13] (kosong -- sengaja TIDAK ada servo yang bergerak selagi package berjalan)
    [14] Tunggu package sampai PROX_1 -> kirim command ambil oleh robot, conveyor mati.
         Hanya kedatangan SAH yang dihitung: harus didahului kepergian dari PROX_2,
         karena satu-satunya jalan ke UJUNG adalah lewat TENGAH. PROX_1 yang tersentuh
         tangan atau benda lain diabaikan firmware, tidak menghentikan conveyor.
    [14b] REFILL gerbang, saat conveyor sudah berhenti:
          servo 1 dorong -> servo 2 tarik -> tahan -> servo 2 dorong
    [15] Tunggu package DIAMBIL -- angkat manual (PROX_1 jadi kosong) ATAU tekan BUTTON_3
         ATAU Enter, lalu conveyor hidup lagi
    [16] Kembali ke [8] -- di sana conveyor mati lagi dan servo 1 tarik lagi,
         jadi pola servo-nya mengulang [2] dan seterusnya

BUTUH FIRMWARE TERBARU. Script ini membaca register 19/21/22 yang baru ada setelah
Dispenser di-flash ulang. Kalau belum, langkah [8] akan langsung gagal baca register.

Jalankan di Orange Pi:  python3 'test-dispenser-pipeline 20260920.py'
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
DISPENSER_SLAVE_ID = 3

ACK_TIMEOUT_S = 15.0
ACK_POLL_INTERVAL_S = 0.2
POLL_INTERVAL_S = 0.2
PROX_WAIT_TIMEOUT_S = 180.0     # menunggu objek fisik -- longgar

# Arah servo. "Tarik" = gerbang membuka (objek boleh jatuh), "Dorong" = gerbang menutup.
# Opcode MOVE_SERVOx_TO: arg 0 = titik AWAL (servoxStartUs), arg 1 = titik AKHIR (servoxEndUs).
# Kalau ternyata terbalik secara fisik, cukup tukar dua angka di bawah ini.
SERVO_TARIK_ARG = 1
SERVO_DORONG_ARG = 0

# Jeda fisik, bukan tebakan proses: waktu objek benar-benar jatuh / servo sampai di posisi.
JATUH_SETTLE_S = 1.5        # [3] beri waktu objek jatuh setelah gerbang servo1 membuka
SERVO2_TAHAN_S = 1.0        # [6] lama servo2 menahan sebelum mendorong lagi
SERVO_GERAK_SETTLE_S = 0.8  # jeda singkat supaya servo sampai posisi sebelum perintah berikutnya

# Catatan urutan (klarifikasi user 2026-09-20): servo 1 yang dibuka di [10] TIDAK ditutup
# di situ juga. Gerbang sengaja terbuka selama menunggu konfirmasi "package penuh" -- itu
# memang jendela waktu objek berikutnya jatuh ke package baru.
#
# Kapan gerbang itu ditutup:
#   False (bawaan) -> ditutup di [14b], bersama rangkaian refill, SETELAH conveyor berhenti.
#                     Tidak ada servo yang bergerak selagi package berjalan, sehingga tidak
#                     ada tabrakan dengan pembacaan PROX_1/PROX_2 di bus I2C yang sama.
#   True           -> ditutup lebih awal di [13], tepat sesudah conveyor hidup. Pakai ini
#                     kalau secara fisik gerbang TIDAK BOLEH terbuka selama package berjalan
#                     (misalnya objek masih berjatuhan dan mengotori jalur). Risikonya servo
#                     bergerak bersamaan dengan package yang sedang melintasi sensor.
SERVO1_TUTUP_LEBIH_AWAL = False

DRY_RUN = False   # True = cuma cetak rencana, tidak mengirim apa pun ke bus

# --- Register (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
REG_STATE = 0
REG_FAULT_CODE = 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_ACTIVITY_CODE = 11
REG_UJUNG_PACKAGE_PRESENT = 17      # PROX_1 (UJUNG), live
REG_MIDDLE_PACKAGE_PRESENT = 16     # PROX_2 (TENGAH), live -- hanya untuk pelaporan, bukan keputusan
REG_MAIN_MODE_ACTIVE = 18
REG_MIDDLE_ARRIVAL_COUNT = 19       # counter latch PROX_2 -- tidak pernah kehilangan kejadian
REG_BTN_PACKAGE_FULL_COUNT = 21     # BARU -- counter latch BUTTON_2 ("package PENUH")
REG_BTN_PACKAGE_TAKEN_COUNT = 22    # BARU -- counter latch BUTTON_3 ("package DIAMBIL")
REG_CONVEYOR_AUTOSTOP_COUNT = 23    # BARU -- berapa kali FIRMWARE menghentikan conveyor sendiri
                                     # begitu PROX_2 mendeteksi package, tanpa menunggu perintah
REG_UJUNG_ARRIVAL_COUNT = 24        # BARU -- kedatangan SAH di PROX_1, yaitu yang memang didahului
                                     # kepergian dari PROX_2. Trigger PROX_1 tanpa itu diabaikan
                                     # firmware (tangan operator, benda tersenggol, pantulan sensor).

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING_OR_MOVING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM", 1: "CONVEYOR_JALAN", 2: "SERVO1_BUKA", 3: "SERVO1_TAHAN", 4: "SERVO1_TUTUP",
    5: "SERVO2_BUKA", 6: "SERVO2_TAHAN", 7: "SERVO2_TUTUP", 8: "SELESAI",
    9: "TUNGGU_KONFIRM_TENGAH", 10: "TEST_LOOP_AKTIF", 90: "FAULT_AKTIF", 91: "ESTOP_AKTIF",
}

# --- Opcode ---
CMD_SET_CONVEYOR_ON_OFF = 11
CMD_MOVE_SERVO1_TO = 12
CMD_MOVE_SERVO2_TO = 13
CMD_STOP_MAIN = 15


# ============================================================
# LAPISAN MODBUS
# ============================================================
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


_seq_counter = 0


def send_command(instr, label, opcode, arg=0):
    global _seq_counter
    _seq_counter += 1
    seq = _seq_counter
    print(f"  -> {label} (opcode={opcode} arg={arg} seq={seq})")
    if DRY_RUN:
        return seq
    instr.write_register(REG_CMD_ARG, arg, functioncode=6)
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
    instr.write_register(REG_CMD, opcode, functioncode=6)
    return seq


def wait_ack(instr, label, seq, timeout=ACK_TIMEOUT_S):
    """PENTING: ack hanya berarti "command DITERIMA", bukan "command BERHASIL". Firmware
    menulis CMD_ACK_SEQ tanpa syarat, termasuk untuk command yang ditolaknya. Keberhasilan
    dibaca dari STATE/FAULT_CODE/ACTIVITY_CODE, bukan dari ack."""
    if DRY_RUN:
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if instr.read_register(REG_CMD_ACK_SEQ, functioncode=3) == seq:
                return True
        except Exception as e:
            print(f"  !! gagal baca ack ({e}), retry...")
        time.sleep(ACK_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak di-ack dalam {timeout}s")
    return False


class RegisterTidakAda(Exception):
    """Register memang tidak ada di firmware -- beda dari sekadar gagal baca sesaat."""


def baca(instr, reg, nama, percobaan=3, diam=False):
    """Baca register dengan beberapa kali percobaan.

    Dibedakan dengan sengaja: 'register tidak ada di firmware' (IllegalRequestError, artinya
    firmware belum di-flash ulang) versus 'pembacaan meleset sesaat' (gangguan bus). Versi
    sebelumnya menyamakan keduanya, sehingga satu kali meleset saja sudah membuat script
    menyimpulkan firmware-nya salah lalu berhenti -- diagnosis yang menyesatkan."""
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


def print_status(instr, label=""):
    if DRY_RUN:
        return
    try:
        state = instr.read_register(REG_STATE, functioncode=3)
        fault = instr.read_register(REG_FAULT_CODE, functioncode=3)
        act = instr.read_register(REG_ACTIVITY_CODE, functioncode=3)
        print(f"     [status] STATE={state}({STATE_NAMES.get(state, '?')}) FAULT={fault} "
              f"ACTIVITY={act}({ACTIVITY_NAMES.get(act, '?')}){(' <- ' + label) if label else ''}")
    except Exception as e:
        print(f"     [status] gagal baca ({e})")


def kirim(instr, label, opcode, arg=0):
    ok = wait_ack(instr, label, send_command(instr, label, opcode, arg))
    print_status(instr)
    return ok


# ============================================================
# AKTUATOR
# ============================================================
def conveyor(instr, nyala):
    label = f"CONVEYOR {'HIDUP' if nyala else 'MATI'}"
    return kirim(instr, label, CMD_SET_CONVEYOR_ON_OFF, arg=1 if nyala else 0)


def servo(instr, nomor, tarik):
    opcode = CMD_MOVE_SERVO1_TO if nomor == 1 else CMD_MOVE_SERVO2_TO
    arg = SERVO_TARIK_ARG if tarik else SERVO_DORONG_ARG
    ok = kirim(instr, f"SERVO {nomor} {'TARIK' if tarik else 'DORONG'}", opcode, arg=arg)
    if ok and not DRY_RUN:
        time.sleep(SERVO_GERAK_SETTLE_S)
    return ok


# ============================================================
# MENUNGGU KEJADIAN
# ============================================================
try:
    KEYBOARD_ADA = sys.stdin.isatty()
except Exception:
    KEYBOARD_ADA = False


def stdin_siap():
    """True kalau operator benar-benar menekan Enter.

    HATI-HATI: kalau stdin bukan terminal (dijalankan lewat nohup, pipe, atau
    `< /dev/null`), select() melaporkan stdin SELALU siap karena sudah EOF -- dan versi
    sebelumnya menganggap itu sebagai "Enter ditekan", sehingga setiap konfirmasi langsung
    dilewati tanpa menunggu siapa pun. Karena itu keyboard dimatikan total kalau bukan
    terminal, dan pembacaan kosong (EOF) tidak pernah dihitung sebagai penekanan."""
    if not KEYBOARD_ADA:
        return False
    try:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return False
        return sys.stdin.readline() != ""   # "" berarti EOF, bukan Enter
    except Exception:
        return False


def tunggu_konfirmasi(instr, judul, reg_counter, nama_tombol, timeout=PROX_WAIT_TIMEOUT_S):
    """Menunggu konfirmasi NYATA, bukan delay. Dua sumber diterima sekaligus:
      - tombol fisik di panel (counter latch di firmware naik +1), atau
      - Enter di Orange Pi.

    Counter latch dipakai (bukan status tombol sesaat) supaya penekanan tombol tidak
    terlewat di sela-sela polling. Mengembalikan True kalau terkonfirmasi."""
    print(f"  .. TUNGGU KONFIRMASI: {judul}")
    if DRY_RUN:
        print("  .. (DRY_RUN) anggap sudah dikonfirmasi")
        return True

    try:
        baseline = baca(instr, reg_counter, nama_tombol)
    except RegisterTidakAda as e:
        print(f"  !! {e} -- firmware Dispenser belum di-flash ulang. Berhenti.")
        return False
    if baseline is None:
        print(f"  !! Counter {nama_tombol} tidak terbaca (gangguan bus, bukan firmware). Berhenti.")
        return False

    if KEYBOARD_ADA:
        print(f"     tekan tombol {nama_tombol} di panel, ATAU Enter di sini")
    else:
        print(f"     tekan tombol {nama_tombol} di panel")
        print("     (stdin bukan terminal -- konfirmasi lewat Enter DIMATIKAN)")
    print(f"     counter {nama_tombol} sekarang = {baseline}, menunggu angka ini berubah")

    deadline = time.monotonic() + timeout
    lapor_berikutnya = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if stdin_siap():
            print("  .. dikonfirmasi dari Orange Pi (Enter)")
            return True
        sekarang = baca(instr, reg_counter, nama_tombol, diam=True)
        if sekarang is not None and sekarang != baseline:
            print(f"  .. dikonfirmasi dari tombol {nama_tombol} (counter {baseline} -> {sekarang})")
            return True
        # Denyut berkala: supaya kalau macet, langsung kelihatan script memang ADA di langkah
        # ini dan angka apa yang dilihatnya -- bukan menebak-nebak dari layar yang diam.
        if time.monotonic() >= lapor_berikutnya:
            lapor_berikutnya = time.monotonic() + 5.0
            sisa = int(deadline - time.monotonic())
            ket = "TIDAK TERBACA" if sekarang is None else sekarang
            print(f"     .. masih menunggu {judul} -- {nama_tombol}={ket} "
                  f"(perlu != {baseline}), sisa {sisa}s")
        time.sleep(POLL_INTERVAL_S)

    print(f"  !! TIMEOUT: {judul} tidak dikonfirmasi dalam {timeout}s")
    return False


def tunggu_prox2(instr, baseline, timeout=PROX_WAIT_TIMEOUT_S):
    """Menunggu sampai ADA PACKAGE DI PROX_2 (TENGAH).

    Dua kondisi sama-sama diterima, dan itu disengaja:

      a) counter MIDDLE_ARRIVAL_COUNT berubah -- ada kedatangan baru sejak terakhir dilihat.
         Counter naik di firmware terlepas dari kapan script sempat membaca, jadi kedatangan
         yang terjadi ketika script sedang sibuk menggerakkan servo tidak akan hilang.

      b) sensor SEDANG TERTUTUP sekarang -- package memang sudah ada di situ.

    Syarat (b) WAJIB ada, dan ini yang dulu hilang sehingga script menggantung. Menunggu
    kedatangan BARU saja tidak cukup: begitu sebuah package berhenti di atas PROX_2, tidak
    akan pernah ada kedatangan berikutnya selama package itu belum pergi. Kalau kedatangannya
    kebetulan sudah terhitung sebelum langkah ini dimulai -- misalnya terjadi saat rangkaian
    setup [1]-[7] masih berjalan -- maka angka yang ditunggu tidak akan pernah berubah, dan
    conveyor pun sudah dihentikan firmware sehingga package tidak akan bergerak lagi. Buntu
    permanen, persis gejala "counter=1 (perlu != 1), sensor TERTUTUP" yang terlihat.

    Yang sebenarnya dipedulikan langkah ini memang "apakah ada package di PROX_2", bukan
    "apakah barusan ada yang datang". Mengembalikan (berhasil, nilai_counter_terbaru)."""
    print("  .. TUNGGU package di PROX_2 (TENGAH)")
    if DRY_RUN:
        return True, baseline
    print(f"     counter PROX_2 = {baseline}; lanjut kalau angka ini berubah "
          f"ATAU sensor sudah tertutup")
    deadline = time.monotonic() + timeout
    lapor_berikutnya = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            sekarang = baca(instr, REG_MIDDLE_ARRIVAL_COUNT, "MIDDLE_ARRIVAL_COUNT", diam=True)
        except RegisterTidakAda as e:
            print(f"  !! {e} -- firmware Dispenser belum di-flash ulang. Berhenti.")
            return False, baseline
        if sekarang is not None and sekarang != baseline:
            print(f"  .. PROX_2 tersentuh (counter {baseline} -> {sekarang})")
            return True, sekarang

        # Syarat kedua: package memang sudah ada di sensor sekarang. Lihat penjelasan panjang
        # di docstring -- tanpa ini script bisa menunggu selamanya untuk kedatangan yang
        # sudah terlanjur terhitung sebelum langkah ini dimulai.
        hadir_kini = baca(instr, REG_MIDDLE_PACKAGE_PRESENT, "MIDDLE_PACKAGE_PRESENT", diam=True)
        if hadir_kini == 1:
            terbaru = sekarang if sekarang is not None else baseline
            print(f"  .. package SUDAH ADA di PROX_2 (sensor tertutup, counter={terbaru})")
            return True, terbaru
        if time.monotonic() >= lapor_berikutnya:
            lapor_berikutnya = time.monotonic() + 5.0
            hadir = baca(instr, REG_MIDDLE_PACKAGE_PRESENT, "MIDDLE_PACKAGE_PRESENT", diam=True)
            # Nilai None berarti PEMBACAAN GAGAL, bukan "sensor terbuka". Versi sebelumnya
            # menyamakan keduanya, sehingga bus yang sedang bermasalah tampil sebagai
            # "sensor terbuka" -- laporan yang menyesatkan justru saat paling dibutuhkan.
            if hadir is None:
                ket_sensor = "TIDAK TERBACA"
            else:
                ket_sensor = "TERTUTUP" if hadir == 1 else "terbuka"
            ket_counter = "TIDAK TERBACA" if sekarang is None else sekarang
            sisa = int(deadline - time.monotonic())
            print(f"     .. masih menunggu PROX_2 -- counter={ket_counter} "
                  f"(perlu != {baseline} ATAU sensor tertutup), sensor {ket_sensor}, sisa {sisa}s")
            if sekarang is None:
                print("        (pembacaan gagal berulang -- pastikan tidak ada program Modbus"
                      " lain yang jalan bersamaan di /dev/ttyS3)")
        time.sleep(POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: PROX_2 tidak tersentuh dalam {timeout}s")
    return False, baseline


def tunggu_package_diambil(instr, timeout=PROX_WAIT_TIMEOUT_S):
    """Langkah [15] -- menunggu package di UJUNG benar-benar lepas.

    DUA sumber diterima, sesuai permintaan user (2026-09-20):
      a) BUTTON_3 ditekan (counter latch naik), atau
      b) PROX_1 menjadi kosong -- package diangkat langsung dengan tangan.

    Syarat (b) penting: saat menguji, package sering diambil manual tanpa menyentuh tombol.
    Tanpa itu script akan menunggu penekanan yang tidak akan pernah datang, padahal
    package-nya sendiri sudah tidak ada di sana.

    Enter di Orange Pi juga tetap diterima seperti konfirmasi lain."""
    print("  .. TUNGGU package DIAMBIL dari UJUNG")
    if DRY_RUN:
        return True

    try:
        baseline = baca(instr, REG_BTN_PACKAGE_TAKEN_COUNT, "BUTTON_3")
    except RegisterTidakAda as e:
        print(f"  !! {e} -- firmware Dispenser belum di-flash ulang. Berhenti.")
        return False
    if baseline is None:
        print("  !! Counter BUTTON_3 tidak terbaca (gangguan bus). Berhenti.")
        return False

    print(f"     angkat package-nya, ATAU tekan BUTTON_3"
          f"{', ATAU Enter di sini' if KEYBOARD_ADA else ''}")
    print(f"     counter BUTTON_3 = {baseline}; lanjut kalau angka ini berubah "
          f"ATAU PROX_1 menjadi kosong")

    deadline = time.monotonic() + timeout
    lapor_berikutnya = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if stdin_siap():
            print("  .. dikonfirmasi dari Orange Pi (Enter)")
            return True
        tombol = baca(instr, REG_BTN_PACKAGE_TAKEN_COUNT, "BUTTON_3", diam=True)
        if tombol is not None and tombol != baseline:
            print(f"  .. dikonfirmasi dari BUTTON_3 (counter {baseline} -> {tombol})")
            return True
        ujung = baca(instr, REG_UJUNG_PACKAGE_PRESENT, "UJUNG_PACKAGE_PRESENT", diam=True)
        if ujung == 0:
            print("  .. PROX_1 kosong -- package sudah diangkat")
            return True
        if time.monotonic() >= lapor_berikutnya:
            lapor_berikutnya = time.monotonic() + 5.0
            sisa = int(deadline - time.monotonic())
            ket = "TIDAK TERBACA" if ujung is None else ("masih ADA" if ujung == 1 else "kosong")
            print(f"     .. masih menunggu -- BUTTON_3={tombol} (perlu != {baseline}), "
                  f"package di UJUNG {ket}, sisa {sisa}s")
        time.sleep(POLL_INTERVAL_S)

    print(f"  !! TIMEOUT: package tidak diambil dalam {timeout}s")
    return False


def tunggu_prox1(instr, ujung_baseline, timeout=PROX_WAIT_TIMEOUT_S):
    """Menunggu package sampai di PROX_1 (UJUNG) -- kedatangan yang SAH saja.

    Yang diawasi adalah counter UJUNG_ARRIVAL_COUNT, bukan register live PROX_1. Firmware
    hanya menaikkan counter itu kalau kedatangannya memang didahului kepergian package dari
    PROX_2, karena satu-satunya jalan menuju UJUNG adalah lewat TENGAH. Dengan begitu PROX_1
    yang tersentuh tangan operator, benda tersenggol, atau pantulan sensor tidak akan membuat
    langkah ini lanjut -- dan conveyor pun tidak dihentikan firmware.

    Sambil menunggu, counter autostop ikut diawasi. Sejak firmware menghentikan conveyor
    sendiri begitu PROX_2 mendeteksi package, ada satu kemungkinan yang harus ketahuan dengan
    jelas, bukan berakhir sebagai timeout membisu: package BERIKUTNYA sampai di PROX_2 lebih
    dulu daripada package ini sampai di PROX_1. Conveyor lalu berhenti di tengah jalan, dan
    package ini tidak akan pernah mencapai UJUNG tanpa perintah baru."""
    print("  .. TUNGGU package sampai di PROX_1 (UJUNG)")
    if DRY_RUN:
        return True
    autostop_awal = baca(instr, REG_CONVEYOR_AUTOSTOP_COUNT, "CONVEYOR_AUTOSTOP_COUNT")
    deadline = time.monotonic() + timeout
    lapor_berikutnya = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        # Dipakai counter kedatangan SAH, bukan register live. Firmware hanya menaikkannya
        # kalau kedatangan itu memang didahului kepergian dari PROX_2 -- jadi PROX_1 yang
        # tersentuh tangan atau benda tersenggol tidak akan membuat langkah ini lanjut.
        nilai = baca(instr, REG_UJUNG_ARRIVAL_COUNT, "UJUNG_ARRIVAL_COUNT", diam=True)
        if nilai is not None and nilai != ujung_baseline:
            print(f"  .. package SAMPAI di PROX_1 (kedatangan sah {ujung_baseline} -> {nilai})")
            return True
        autostop = baca(instr, REG_CONVEYOR_AUTOSTOP_COUNT, "CONVEYOR_AUTOSTOP_COUNT")
        if autostop is not None and autostop_awal is not None and autostop != autostop_awal:
            print("\n  !! BENTROK URUTAN -- perlu keputusan Anda.")
            print("     Package BERIKUTNYA menyentuh PROX_2 sebelum package ini sampai PROX_1.")
            print("     Firmware sudah menghentikan conveyor (sesuai aturan 'PROX_2 trigger = stop').")
            print("     Akibatnya package ini berhenti di tengah dan tidak akan sampai UJUNG")
            print("     selama conveyor belum dinyalakan lagi.")
            print("     Artinya jarak PROX_1 dan PROX_2 tidak cocok dengan urutan langkah [14]-[15].")
            return False
        if time.monotonic() >= lapor_berikutnya:
            lapor_berikutnya = time.monotonic() + 5.0
            hadir = baca(instr, REG_UJUNG_PACKAGE_PRESENT, "UJUNG_PACKAGE_PRESENT", diam=True)
            sisa = int(deadline - time.monotonic())
            ket = "TIDAK TERBACA" if hadir is None else ("TERTUTUP" if hadir == 1 else "terbuka")
            print(f"     .. masih menunggu PROX_1 -- kedatangan sah={nilai} "
                  f"(perlu != {ujung_baseline}), sensor {ket}, sisa {sisa}s")
            if hadir == 1 and nilai == ujung_baseline:
                print("        (sensor tertutup tapi TIDAK diakui -- package ini tidak tercatat"
                      " lewat PROX_2 dulu, sengaja diabaikan oleh guard urutan)")
        time.sleep(POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: tidak ada kedatangan sah di PROX_1 dalam {timeout}s")
    return False


# ============================================================
# PERSIAPAN & PENGAMANAN
# ============================================================
def pastikan_test_mode(instr):
    """Semua command yang dipakai script ini (conveyor jog, MOVE_SERVOx_TO) adalah command
    TEST-only -- firmware menolaknya selama MAIN aktif. Jadi pastikan MAIN mati dulu."""
    if DRY_RUN:
        return True
    state = baca(instr, REG_STATE, "STATE")
    fault = baca(instr, REG_FAULT_CODE, "FAULT_CODE")
    if state is None:
        print("!! Dispenser tidak menjawab Modbus sama sekali. Cek daya node dan kabel RS485 A/B.")
        return False
    print(f"  .. STATE={state}({STATE_NAMES.get(state, '?')}) FAULT_CODE={fault}")
    if fault:
        print(f"!! Dispenser sedang FAULT (code={fault}). Reset dulu lewat menu LCD "
              f"(Setting Kalibrasi -> Reset Fault). Berhenti.")
        return False

    kirim(instr, "STOP_MAIN (pastikan TEST mode)", CMD_STOP_MAIN)
    if baca(instr, REG_MAIN_MODE_ACTIVE, "MAIN_MODE_ACTIVE") == 1:
        print("!! MAIN masih aktif padahal STOP_MAIN sudah dikirim. Berhenti.")
        return False
    print("  .. TEST mode OK")
    return True


def stop_darurat(instr):
    """Dipanggil di jalur keluar MANA PUN. Conveyor WAJIB mati saat script berhenti.

    Dicoba beberapa kali dan tanpa wait_ack: pada saat ini bus bisa sedang kacau, dan
    lebih baik perintah mati terkirim berkali-kali daripada conveyor ditinggal berputar.
    Keluhan "sudah di-terminate tapi conveyor masih menyala" berasal dari sini -- dulu
    perintah mati hanya dikirim sekali dan kalau gagal ya sudah."""
    print("\n>> PENGAMANAN: mematikan conveyor...")
    if DRY_RUN:
        return
    for percobaan in range(1, 4):
        try:
            send_command(instr, f"CONVEYOR MATI [pengaman {percobaan}/3]", CMD_SET_CONVEYOR_ON_OFF, arg=0)
            time.sleep(0.3)
        except Exception as e:
            print(f"  !! percobaan {percobaan} gagal ({e})")
    print(">> Selesai. Pastikan secara visual conveyor benar-benar berhenti.")


class Dihentikan(Exception):
    """Dipakai supaya SIGTERM ikut menjalankan blok pembersihan, sama seperti Ctrl+C."""


def _on_sigterm(signum, frame):
    raise Dihentikan()


# ============================================================
# ALUR UTAMA
# ============================================================
def setup_package_pertama(instr):
    """Langkah [1]-[7]: mengisi package PERTAMA."""
    print("\n=== [1] Conveyor menyala ===")
    if not conveyor(instr, True):
        return False

    print("\n=== [2] Servo 1 tarik ===")
    if not servo(instr, 1, tarik=True):
        return False

    print(f"\n=== [3] Package pertama jatuh (beri waktu {JATUH_SETTLE_S}s) ===")
    if not DRY_RUN:
        time.sleep(JATUH_SETTLE_S)

    print("\n=== [4] Servo 1 dorong ===")
    if not servo(instr, 1, tarik=False):
        return False

    print("\n=== [5] Servo 2 tarik ===")
    if not servo(instr, 2, tarik=True):
        return False

    print(f"\n=== [6] Servo 2 tahan ({SERVO2_TAHAN_S}s) ===")
    if not DRY_RUN:
        time.sleep(SERVO2_TAHAN_S)

    print("\n=== [7] Servo 2 dorong ===")
    return servo(instr, 2, tarik=False)


def satu_putaran(instr, prox2_baseline, nomor_putaran):
    """Langkah [8]-[16]. Mengembalikan (lanjut, prox2_baseline_terbaru)."""
    print(f"\n########## PUTARAN #{nomor_putaran} (mulai dari langkah [8]) ##########")

    print("\n=== [8] Tunggu package menyentuh PROX_2 ===")
    ok, prox2_baseline = tunggu_prox2(instr, prox2_baseline)
    if not ok:
        return False, prox2_baseline

    # Firmware sudah mematikan conveyor sendiri pada milidetik PROX_2 terdeteksi (lihat
    # handleMiddleSensor). Perintah di sini tetap dikirim sebagai penegasan -- idempoten,
    # dan berguna kalau autostop firmware kebetulan tidak berlaku (mode MAIN / TEST LOOP).
    print("\n=== [9] Conveyor mati (penegasan -- firmware sudah stop duluan) ===")
    autostop = baca(instr, REG_CONVEYOR_AUTOSTOP_COUNT, "CONVEYOR_AUTOSTOP_COUNT")
    if autostop is not None:
        print(f"  .. CONVEYOR_AUTOSTOP_COUNT = {autostop}")
    if not conveyor(instr, False):
        return False, prox2_baseline

    print("\n=== [10] Servo 1 tarik (gerbang dibuka, objek jatuh ke package baru) ===")
    if not servo(instr, 1, tarik=True):
        return False, prox2_baseline

    print("\n=== [11] Tunggu konfirmasi: package sudah PENUH ===")
    if not tunggu_konfirmasi(instr, "package sudah PENUH",
                             REG_BTN_PACKAGE_FULL_COUNT, "BUTTON_2"):
        return False, prox2_baseline

    # Acuan kedatangan UJUNG diambil SEBELUM conveyor hidup, supaya kedatangan yang terjadi
    # kapan pun sesudah ini tetap tertangkap -- termasuk kalau script sedang sibuk.
    ujung_baseline = baca(instr, REG_UJUNG_ARRIVAL_COUNT, "UJUNG_ARRIVAL_COUNT")
    if ujung_baseline is None:
        print("  !! Counter kedatangan UJUNG tidak terbaca. Berhenti.")
        return False, prox2_baseline

    print("\n=== [12] Conveyor hidup -- package berikutnya mulai berjalan ===")
    if not conveyor(instr, True):
        return False, prox2_baseline

    if SERVO1_TUTUP_LEBIH_AWAL:
        print("\n=== [13] Servo 1 dorong (menutup gerbang lebih awal) ===")
        if not servo(instr, 1, tarik=False):
            return False, prox2_baseline
    else:
        print("\n=== [13] (sengaja kosong -- tidak ada servo bergerak selagi package berjalan) ===")

    print("\n=== [14] Tunggu package sampai PROX_1, kirim command ambil, conveyor mati ===")
    if not tunggu_prox1(instr, ujung_baseline):
        return False, prox2_baseline
    # Placeholder, BUKAN delay: di sistem sebenarnya di sinilah Orange Pi menyuruh Picker
    # mengambil package (Cmd::MOVE_PACKAGE ke slave 2). Untuk test Dispenser sendirian,
    # cukup ditandai supaya terlihat jelas di log kapan hal itu seharusnya terjadi.
    print("  >> COMMAND AMBIL PACKAGE OLEH ROBOT (placeholder test -- di produksi: Picker MOVE_PACKAGE)")
    # Firmware sudah mematikan conveyor sendiri pada milidetik PROX_1 tersentuh, sama seperti
    # yang dilakukannya di PROX_2. Perintah di bawah tinggal penegasan -- idempoten. Sebelum
    # firmware ikut menghentikan, belt masih sempat berjalan selama perintah ini menempuh
    # perjalanan Modbus, dan package terlanjur bergeser melewati titik ambil.
    if not conveyor(instr, False):
        return False, prox2_baseline

    # [14b] REFILL gerbang -- dipindah ke SINI (usulan user, 2026-09-20).
    #
    # Dulu rangkaian ini dijalankan di [13], tepat sesudah conveyor hidup. Akibatnya servo
    # bergerak BERSAMAAN dengan dua package yang sedang berjalan: satu menuju PROX_1, satu
    # menuju PROX_2. Padahal servo (PCA9685) dan kedua proximity (MCP23017) berbagi satu bus
    # I2C, sehingga justru pada saat kedua sensor itu paling perlu dibaca dengan benar, bus
    # sedang paling ramai.
    #
    # Sekarang servo hanya bergerak saat conveyor BERHENTI dan tidak ada package yang
    # berjalan -- tidak ada tepi sensor yang ditunggu, jadi tidak ada yang bisa terlewat.
    # Ini menghilangkan tabrakannya, bukan sekadar menambal gejalanya.
    print("\n=== [14b] Refill gerbang (conveyor berhenti, aman dari tabrakan sensor) ===")
    if not SERVO1_TUTUP_LEBIH_AWAL:
        if not servo(instr, 1, tarik=False):
            return False, prox2_baseline
    if not servo(instr, 2, tarik=True):
        return False, prox2_baseline
    print(f"  .. servo 2 tahan ({SERVO2_TAHAN_S}s)")
    if not DRY_RUN:
        time.sleep(SERVO2_TAHAN_S)
    if not servo(instr, 2, tarik=False):
        return False, prox2_baseline

    print("\n=== [15] Tunggu package DIAMBIL (angkat manual ATAU tekan BUTTON_3) ===")
    if not tunggu_package_diambil(instr):
        return False, prox2_baseline

    print("\n=== [15b] Conveyor hidup lagi ===")
    if not conveyor(instr, True):
        return False, prox2_baseline

    print("\n=== [16] Kembali ke [8] -- di sana conveyor mati lagi & servo 1 tarik lagi ===")
    return True, prox2_baseline


def main():
    signal.signal(signal.SIGTERM, _on_sigterm)

    print("=" * 70)
    print("TEST DISPENSER -- alur pipeline 16 langkah")
    print(f"port={SERIAL_PORT} slave={DISPENSER_SLAVE_ID} DRY_RUN={DRY_RUN}")
    print("Konfirmasi lewat tombol panel (BUTTON_2 / BUTTON_3) atau Enter di sini.")
    print("Ctrl+C kapan saja untuk berhenti -- conveyor otomatis dimatikan.")
    print("=" * 70)

    dispenser = connect(DISPENSER_SLAVE_ID)

    # Potret awal semua counter. Kalau ada yang aneh nanti, ini titik bandingnya --
    # tanpa ini kita cuma bisa menebak apakah sebuah angka memang berubah atau tidak.
    if not DRY_RUN:
        print("\n=== Potret awal counter ===")
        for reg, nama in ((REG_MIDDLE_ARRIVAL_COUNT, "PROX_2 datang"),
                          (REG_BTN_PACKAGE_FULL_COUNT, "BUTTON_2 (package PENUH)"),
                          (REG_BTN_PACKAGE_TAKEN_COUNT, "BUTTON_3 (package DIAMBIL)"),
                          (REG_CONVEYOR_AUTOSTOP_COUNT, "conveyor autostop firmware")):
            try:
                print(f"  {nama:<30} = {baca(dispenser, reg, nama)}")
            except RegisterTidakAda as e:
                print(f"  {nama:<30} = TIDAK ADA -- {e}")
                print("  !! Firmware Dispenser belum di-flash ulang. Berhenti.")
                return 1
        print(f"  Konfirmasi lewat Enter: {'AKTIF' if KEYBOARD_ADA else 'MATI (stdin bukan terminal)'}")

    try:
        print("\n=== Persiapan: pastikan TEST mode ===")
        if not pastikan_test_mode(dispenser):
            return 1

        if not setup_package_pertama(dispenser):
            print("\n!! Setup [1]-[7] gagal. Berhenti.")
            return 1

        prox2_baseline = baca(dispenser, REG_MIDDLE_ARRIVAL_COUNT, "MIDDLE_ARRIVAL_COUNT")
        if prox2_baseline is None:
            print("\n!! Register 19 tidak terbaca -- firmware Dispenser belum di-flash ulang. Berhenti.")
            return 1

        putaran = 0
        while True:
            putaran += 1
            lanjut, prox2_baseline = satu_putaran(dispenser, prox2_baseline, putaran)
            if not lanjut:
                print(f"\n!! Putaran #{putaran} berhenti sebelum tuntas. Keluar.")
                return 1

    except (KeyboardInterrupt, Dihentikan):
        print("\n\n>> Dihentikan operator.")
        return 130
    finally:
        stop_darurat(dispenser)


if __name__ == "__main__":
    sys.exit(main())
