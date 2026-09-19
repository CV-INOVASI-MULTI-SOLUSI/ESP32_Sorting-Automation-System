#!/usr/bin/env python3
"""
============================================================
 TEST DISPENSER -- protokol lengkap conveyor + servo1/servo2 + prox1/prox2
 + konfirmasi jumlah objek (placeholder timing)
============================================================
Skrip test MANUAL (bukan orchestrator produksi) -- cuma nyentuh node
DISPENSER (slave 3). Versi terbaru (gantikan "20260918", sudah dihapus --
isinya sama, ditulis ulang + ditambah pengecekan MAIN/TEST mode).

SETUP (1x, sebelum masuk loop):
  0. Pastikan Dispenser masih TEST mode (lihat catatan MAIN/TEST di bawah)
  1. Conveyor MENYALA (DIUBAH -- dulu MATI dulu baru nyala di poin 5, sekarang
     langsung HIDUP dari awal, konsisten sama fix [9b] di loop: conveyor harus
     jalan SEBELUM servo round-trip biar objek gak numpuk di bawah gerbang)
  2. Servo1: titik awal -> tahan -> titik akhir -> (balik titik awal) [round-trip,
     TEST_SERVO1_CYCLE -- dikonfirmasi user round-trip, bukan one-way]
  3. (deskriptif -- jeda hold time servo1&2 SUDAH otomatis include di dalam
     TEST_SERVO1_CYCLE/TEST_SERVO2_CYCLE, BUKAN langkah terpisah)
  4. Servo2: sama pola, round-trip (TEST_SERVO2_CYCLE)
  5. Pastikan conveyor tetap MENYALA (sudah nyala dari poin 1, idempotent/jaga-jaga)

LOOP (mulai poin 6, balik ke 6 lagi di poin 17 -- terus-menerus):
  6.  TUNGGU package melewati PROX_2 (TENGAH) -- BARU pakai counter MIDDLE_ARRIVAL_COUNT
      (latch di firmware), bukan live state -- anti-kelewatan kalau package lewat SELAGI
      servo round-trip [9b]-[13] masih jalan (~7 detik gak sempat kepoll)
  7.  Conveyor MATI
  8.  [TIMING, BUKAN command -- lihat catatan] tunggu konfirmasi jumlah objek
  9.  [TIMING, BUKAN command] terima "jumlah objek == target"
  9b. BARU -- Conveyor MENYALA DULU sebelum round-trip servo (kalau diam, objek yang
      dijatuhin servo1/servo2 numpuk di bawah gerbang, gak kebawa maju conveyor)
  10. Servo1 round-trip lagi (TEST_SERVO1_CYCLE)
  11. (deskriptif, sama seperti poin 3)
  12. Servo2 round-trip lagi (TEST_SERVO2_CYCLE)
  13. Pastikan conveyor tetap MENYALA (sudah nyala dari 9b, ini cuma jaga-jaga/idempotent)
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

CATATAN PENTING soal MAIN/TEST mode (BARU, firmware ditambah sesi ini):
  SET_CONVEYOR_ON_OFF / MOVE_SERVO1_TO / MOVE_SERVO2_TO / TEST_SERVO1_CYCLE /
  TEST_SERVO2_CYCLE -- yang dipakai skrip ini -- SEMUA cuma diterima firmware
  selama Dispenser masih TEST mode (mainModeActive == FALSE, default pas boot).
  Kalau sebelumnya `orangepi-orchestrator-batch.py` pernah START_MAIN Dispenser
  dan belum di-STOP_MAIN (mis. proses-nya di-kill paksa, bukan Ctrl+C normal),
  SEMUA command di skrip ini bakal DITOLAK DIAM-DIAM (CMD_ACK_SEQ tetap jalan
  normal, tapi gak ada efek fisik apapun). Skrip ini SEKARANG cek register
  MAIN_MODE_ACTIVE di awal dan otomatis kirim STOP_MAIN kalau perlu (lihat
  ensure_test_mode()) -- gak perlu langkah manual lagi.

FITUR "masukin package pertama kali" (di luar alur loop di atas, dipanggil
manual kalau perlu -- lihat fungsi manual_load_servo()):
  - Cmd::MOVE_SERVO1_TO / MOVE_SERVO2_TO (opcode 12/13) -- jog manual SATU ARAH
    (BUKAN round-trip), gerak ke titik awal (arg=0) ATAU titik akhir (arg=1),
    berhenti di situ (gak balik sendiri) -- biar operator bisa buka gerbang,
    taruh package manual, tutup lagi pakai command terpisah.

WAJIB firmware Dispenser sudah di-flash versi terbaru (MAIN/TEST mode +
opcode 11/12/13/14/15 + register 16/17/18) sebelum skrip ini jalan ke
hardware asli.

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

DRY_RUN = False  # BARU default False (minta user) -- langsung kirim command ke hardware asli.
                  # Balikin ke True kalau mau simulasi doang (cuma PRINT rencana aksi + anggap
                  # prox trigger instan, gak kirim apapun ke bus).

# --- Register base (SAMA di semua node) ---
REG_STATE = 0
REG_FAULT_CODE = 1
REG_CMD, REG_CMD_ARG, REG_CMD_SEQ, REG_CMD_ACK_SEQ = 2, 3, 4, 5
REG_ACTIVITY_CODE = 11            # BARU -- dipakai print_status() buat debug tiap command
REG_MIDDLE_PACKAGE_PRESENT = 16   # PROX_2 (TENGAH), live
REG_UJUNG_PACKAGE_PRESENT = 17    # PROX_1 (UJUNG), live
REG_MAIN_MODE_ACTIVE = 18         # BARU -- 1 = MAIN aktif, 0 = TEST mode
REG_MIDDLE_ARRIVAL_COUNT = 19     # BARU -- counter latch PROX_2, naik tiap package baru dateng,
                                   # anti-kelewatan (beda dari REG_MIDDLE_PACKAGE_PRESENT yg cuma live)

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING_OR_MOVING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM", 1: "CONVEYOR_JALAN", 2: "SERVO1_BUKA", 3: "SERVO1_TAHAN", 4: "SERVO1_TUTUP",
    5: "SERVO2_BUKA", 6: "SERVO2_TAHAN", 7: "SERVO2_TUTUP", 8: "SELESAI",
    9: "TUNGGU_KONFIRM_TENGAH", 10: "TEST_LOOP_AKTIF", 90: "FAULT_AKTIF", 91: "ESTOP_AKTIF",
}

# --- Opcode DISPENSER (lihat ESP32_SortingAutomation_Dispenser/include/registers.h) ---
CMD_SET_CONVEYOR_ON_OFF = 11
CMD_MOVE_SERVO1_TO = 12    # arg: 0=titik awal, 1=titik akhir
CMD_MOVE_SERVO2_TO = 13    # arg: 0=titik awal, 1=titik akhir
CMD_STOP_MAIN = 15         # BARU -- dipakai ensure_test_mode()
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


def print_status(instr, label):
    """BARU -- debug otomatis: baca STATE/FAULT_CODE/ACTIVITY_CODE, dipanggil tiap
    do_command() (kalau bukan DRY_RUN). Ini yang kepake buat nyari akar masalah
    'servo1 gak gerak pas lewat full-protocol, padahal jalan normal pas ditest sendirian'."""
    if DRY_RUN:
        return
    try:
        state = instr.read_register(REG_STATE, functioncode=3)
        fault = instr.read_register(REG_FAULT_CODE, functioncode=3)
        activity = instr.read_register(REG_ACTIVITY_CODE, functioncode=3)
        print(f"     [status] STATE={state}({STATE_NAMES.get(state, '?')}) "
              f"FAULT_CODE={fault} ACTIVITY_CODE={activity}({ACTIVITY_NAMES.get(activity, '?')})")
    except Exception as e:
        print(f"     [status] gagal baca ({e})")


def middle_triggered(instr):
    """BARU -- baca live PROX_2 (TENGAH). True = lagi trigger = ADA package sedang diisi
    objek di situ SEKARANG. Dasar buat aturan UNIVERSAL: conveyor GAK BOLEH gerak selama
    ini True, di step MANAPUN (bukan cuma poin 14/16 kayak sebelumnya)."""
    if DRY_RUN:
        return False
    try:
        return instr.read_register(REG_MIDDLE_PACKAGE_PRESENT, functioncode=3) == 1
    except Exception as e:
        print(f"  !! Gagal baca PROX_2 (guard universal) ({e})")
        return False


def conveyor_on_if_safe(instr, label="SET_CONVEYOR_ON_OFF(1)", wait_timeout_s=PROX_WAIT_TIMEOUT_S):
    """Dipakai di poin [13] (BUKAN [9b], lihat catatan di situ). Kalau PROX_2 lagi trigger --
    artinya guard universal (stop_conveyor_if_middle_triggered, dipanggil selama servo watch
    [10]/[12]) BARU AJA matiin conveyor gara-gara package baru dateng -- TUNGGU dulu sampai
    PROX_2 CLEAR, baru nyalain lagi.

    DIPERBAIKI (Celah A, ditemukan 2026-09-20): versi lama cuma CEK SEKALI, langsung nyerah
    kalau masih trigger (conveyor dibiarkan mati selamanya, gak pernah dicoba nyalain lagi).
    Akibatnya package 1 gak pernah kebawa maju, [14] nunggu PROX_1 yang GAK AKAN PERNAH trigger
    (conveyor mati), sampai timeout 120 detik baru seluruh script ke-abort. Sekarang nunggu
    PROX_2 clear dulu (self-healing begitu package berikutnya udah lewat/diisi), baru nyalain --
    cuma gagal beneran (return False, caller break) kalau PROX_2 GENUINELY macet/gak pernah
    clear sampai wait_timeout_s (indikasi hardware, bukan cuma dua package numpuk deket-deketan)."""
    if middle_triggered(instr):
        print("  !! PROX_2 masih/baru trigger -- tunggu CLEAR dulu sebelum nyalain conveyor lagi...")
        if not wait_prox(instr, "PROX_2 (TENGAH) clear", REG_MIDDLE_PACKAGE_PRESENT, 0, timeout=wait_timeout_s):
            print("  !! TIMEOUT nunggu PROX_2 clear -- conveyor TETAP mati, kemungkinan sensor macet/hardware.")
            return False
        print("  .. PROX_2 clear, lanjut nyalain conveyor")
    return do_command(instr, label, CMD_SET_CONVEYOR_ON_OFF, arg=1)


_middle_guard_already_stopped = False   # BARU -- lihat catatan Celah B di stop_conveyor_if_middle_triggered()

def stop_conveyor_if_middle_triggered(instr):
    """Cek PROX_2, kalau trigger kirim SET_CONVEYOR_ON_OFF(0) SEKALI SAJA per episode trigger
    (bukan tiap poll -- lihat Celah B, ditemukan 2026-09-20: dulu fungsi ini SPAM
    SET_CONVEYOR_ON_OFF(0) tiap 0.2 detik SELAMA PROX_2 masih trigger, karena gak ada
    penanda 'udah pernah kirim'. Sekarang edge-triggered -- flag di-reset begitu PROX_2
    balik clear, biar episode trigger BERIKUTNYA tetap kedeteksi & direaksi)."""
    global _middle_guard_already_stopped
    if middle_triggered(instr):
        if not _middle_guard_already_stopped:
            print("  !! [guard universal] PROX_2 trigger -- conveyor di-stop SEKARANG (package sedang diisi).")
            do_command(instr, "SET_CONVEYOR_ON_OFF(0) [guard]", CMD_SET_CONVEYOR_ON_OFF, arg=0)
            _middle_guard_already_stopped = True
        return True
    _middle_guard_already_stopped = False
    return False


def do_command(instr, label, opcode, arg=0, timeout=ACK_TIMEOUT_S, watch_activity_s=0.0):
    seq = send_command(instr, label, opcode, arg)
    ok = wait_ack(instr, label, seq, timeout=timeout)
    print_status(instr, label)
    if ok and watch_activity_s > 0 and not DRY_RUN:
        # BARU -- khusus command yang efeknya BARU kelihatan SETELAH ack (servo cycle
        # butuh ~3-4 detik gerak, ack cuma nandain command DITERIMA bukan SELESAI) --
        # intip ACTIVITY_CODE beberapa detik biar transisi SERVO1_BUKA/TAHAN/TUTUP kelihatan.
        last_val = None
        deadline = time.monotonic() + watch_activity_s
        while time.monotonic() < deadline:
            # BARU -- guard universal PROX_2 diselipin di sini juga (servo round-trip
            # [10]/[12] bisa makan ~3.5 detik SENDIRI-SENDIRI -- dulu PROX_2 kepencet di
            # tengah jendela ini SAMA SEKALI gak ke-react sampai poin 14).
            if opcode != CMD_SET_CONVEYOR_ON_OFF:   # cegah rekursi tak berguna pas guard sendiri manggil conveyor-off
                stop_conveyor_if_middle_triggered(instr)
            try:
                val = instr.read_register(REG_ACTIVITY_CODE, functioncode=3)
                if val != last_val:
                    print(f"     [watch] ACTIVITY_CODE = {val} ({ACTIVITY_NAMES.get(val, f'?({val})')})")
                    last_val = val
            except Exception as e:
                print(f"     [watch] gagal baca ({e})")
            time.sleep(0.2)
    return ok


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


def wait_prox_with_middle_guard(instr, label, reg_addr, want_value, timeout=PROX_WAIT_TIMEOUT_S):
    """Dipakai KHUSUS di poin [14] (BUKAN [16] lagi -- lihat catatan di situ), selama conveyor
    lagi jalan nungguin package sampai UJUNG. Sensor PROX_2 (TENGAH) dipantau PARALEL (bareng
    PROX_1) -- kalau PROX_2 kedeteksi trigger LAGI (package berikutnya udah sampai TENGAH
    selagi package SEBELUMNYA masih menuju UJUNG), conveyor di-stop SEMENTARA.

    DIPERBAIKI (Celah A-2, kelas masalah SAMA dgn conveyor_on_if_safe -- ditemukan 2026-09-20):
    dulu abis conveyor di-stop SEKALI, TIDAK PERNAH dinyalain lagi -- padahal kondisi UTAMA yang
    ditunggu di sini (PROX_1 trigger) SENDIRI butuh conveyor jalan biar package sampai ke situ.
    "Lanjut nunggu" versi lama tetap ujung-ujungnya timeout 120 detik, cuma nundur waktunya.
    Sekarang: begitu PROX_2 clear lagi, conveyor otomatis DINYALAKAN ULANG, baru lanjut nunggu
    kondisi utama seperti biasa -- self-healing, gak butuh intervensi manual."""
    kata = "trigger" if want_value == 1 else "CLEAR (gak ke-deteksi lagi)"
    print(f"  .. TUNGGU {label} {kata} (PARAREL: pantau PROX_2 juga, stop+resume conveyor otomatis kalau kepencet lagi)")
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

        if middle_triggered(instr):
            print("  !! PROX_2 (TENGAH) kepencet LAGI -- package berikutnya udah sampai. Conveyor di-stop sementara.")
            do_command(instr, "SET_CONVEYOR_ON_OFF(0) [PROX_2 guard]", CMD_SET_CONVEYOR_ON_OFF, arg=0)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if not wait_prox(instr, "PROX_2 (TENGAH) clear", REG_MIDDLE_PACKAGE_PRESENT, 0, timeout=remaining):
                print("  !! TIMEOUT nunggu PROX_2 clear -- kemungkinan sensor macet/hardware, bukan cuma dua package numpuk.")
                break
            print("  .. PROX_2 clear -- nyalain conveyor lagi, lanjut nunggu")
            do_command(instr, "SET_CONVEYOR_ON_OFF(1) [resume after guard]", CMD_SET_CONVEYOR_ON_OFF, arg=1)

        time.sleep(PROX_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: {label} tidak {kata} dalam {timeout}s")
    return False


def wait_middle_arrival(instr, last_count, timeout=PROX_WAIT_TIMEOUT_S):
    """BARU -- ganti wait_prox biasa di poin [6]. Pakai counter MIDDLE_ARRIVAL_COUNT (latch di
    firmware) BUKAN baca live state PROX_2 -- robust walau ada jendela lama gak ke-poll (servo
    round-trip [9b]-[13] ~7 detik + timing placeholder), soalnya counter naik di FIRMWARE
    persis pas sensor ke-trigger, terlepas kapan Orange Pi sempat baca. Kalau package 2 udah
    lewat PROX_2 SELAGI servo masih jatuhin objek buat package 1 (counter udah naik duluan
    sebelum kita sempat cek), poin [6] BERIKUTNYA langsung ke-detect seketika (gak nunggu lagi)
    karena counter SAAT INI udah beda dari `last_count` yang di-bawa dari iterasi sebelumnya.
    Return (ok, new_count) -- new_count WAJIB dioper balik ke panggilan berikutnya (state nyambung
    antar-iterasi loop, BUKAN baca ulang dari awal tiap kali)."""
    print(f"  .. TUNGGU PROX_2 (TENGAH) trigger -- counter MIDDLE_ARRIVAL_COUNT (anti-kelewatan, baseline={last_count})")
    if DRY_RUN:
        print("  .. (DRY_RUN) anggap PROX_2 sudah trigger")
        return True, last_count + 1
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            cur = instr.read_register(REG_MIDDLE_ARRIVAL_COUNT, functioncode=3)
            if cur != last_count:
                print(f"  .. PROX_2 trigger -- OK (counter {last_count} -> {cur})")
                return True, cur
        except Exception as e:
            print(f"  !! DISPENSER: gagal baca MIDDLE_ARRIVAL_COUNT ({e}), retry...")
        time.sleep(PROX_POLL_INTERVAL_S)
    print(f"  !! TIMEOUT: PROX_2 (counter) tidak berubah dari {last_count} dalam {timeout}s")
    return False, last_count


def timing_placeholder(instr, label, delay_s):
    """Poin 8/9/15 -- SENGAJA cuma delay waktu, BUKAN command/register beneran
    (sesuai instruksi eksplisit: 'dibuat menggunakan timing tidak menerima command').
    DIUBAH: dulu satu time.sleep() utuh -- sekarang dipecah jadi potongan kecil DAN
    diselipin guard universal PROX_2 (lihat stop_conveyor_if_middle_triggered()) tiap
    potongan, biar reaktif matiin conveyor walau lagi di tengah jeda placeholder ini."""
    print(f"  .. [TIMING PLACEHOLDER] {label} -- jeda {delay_s}s (belum ada command/register asli)")
    deadline = time.monotonic() + delay_s
    while time.monotonic() < deadline:
        if not DRY_RUN:
            stop_conveyor_if_middle_triggered(instr)
        time.sleep(min(PROX_POLL_INTERVAL_S, max(0.0, deadline - time.monotonic())))


def ensure_test_mode(instr):
    """BARU -- semua command di skrip ini (SET_CONVEYOR_ON_OFF dkk) cuma diterima
    selama Dispenser TEST mode. Kalau ketinggalan MAIN mode dari sesi orchestrator
    sebelumnya (mis. di-kill paksa, bukan Ctrl+C normal), command bakal ditolak
    diam-diam. Cek MAIN_MODE_ACTIVE, kirim STOP_MAIN kalau perlu."""
    if DRY_RUN:
        print("  .. (DRY_RUN) lewati cek MAIN/TEST mode")
        return True
    try:
        main_active = instr.read_register(REG_MAIN_MODE_ACTIVE, functioncode=3)
    except Exception as e:
        print(f"  !! Gagal baca MAIN_MODE_ACTIVE ({e}) -- lanjut anggap TEST mode")
        return True
    if main_active == 1:
        print("  .. Dispenser masih MAIN mode (sisa sesi produksi sebelumnya) -- kirim STOP_MAIN")
        return do_command(instr, "STOP_MAIN", CMD_STOP_MAIN)
    print("  .. Dispenser sudah TEST mode, lanjut")
    return True


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

    print("\n=== [0] Pastikan Dispenser TEST mode ===")
    if not ensure_test_mode(dispenser):
        print("!! Gagal pastikan TEST mode -- cek fault Dispenser manual. Berhenti.")
        sys.exit(1)

    # Contoh pemakaian manual_load_servo() buat isi package PERTAMA KALI SEBELUM
    # loop mulai -- uncomment kalau perlu:
    # manual_load_servo(dispenser, 1, to_end=True)   # buka gerbang servo1
    # input("Taruh package, lalu Enter...")
    # manual_load_servo(dispenser, 1, to_end=False)  # tutup lagi

    print("\n=== SETUP [1] Conveyor MENYALA (DIUBAH -- dulu MATI, sekarang langsung HIDUP dari awal) ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    print("\n=== SETUP [2] Servo1 round-trip (titik awal->akhir->tahan->awal) ===")
    if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE, watch_activity_s=5.0):
        sys.exit(1)

    print("\n=== SETUP [4] Servo2 round-trip ===")
    if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE, watch_activity_s=5.0):
        sys.exit(1)

    print("\n=== SETUP [5] Pastikan conveyor tetap MENYALA (sudah nyala dari [1], idempotent) ===")
    if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
        sys.exit(1)

    # BARU -- baseline counter MIDDLE_ARRIVAL_COUNT SEBELUM masuk loop, dioper+diupdate
    # tiap iterasi lewat wait_middle_arrival() (lihat komentar fungsinya).
    try:
        middle_count = 0 if DRY_RUN else dispenser.read_register(REG_MIDDLE_ARRIVAL_COUNT, functioncode=3)
    except Exception as e:
        print(f"  !! Gagal baca MIDDLE_ARRIVAL_COUNT awal ({e}) -- pakai baseline 0")
        middle_count = 0

    loop_count = 0
    try:
        while True:
            loop_count += 1
            print(f"\n########## LOOP #{loop_count} (mulai dari poin 6) ##########")

            print("\n=== [6] Tunggu package melewati PROX_2 (TENGAH) ===")
            ok, middle_count = wait_middle_arrival(dispenser, middle_count)
            if not ok:
                break

            print("\n=== [7] Conveyor MATI ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(0)", CMD_SET_CONVEYOR_ON_OFF, arg=0):
                break

            print("\n=== [8] Tunggu konfirmasi jumlah objek (TIMING placeholder) ===")
            timing_placeholder(dispenser, "konfirmasi jumlah objek", KONFIRMASI_JUMLAH_OBJEK_DELAY_S)

            print("\n=== [9] Terima jumlah objek == target (TIMING placeholder) ===")
            timing_placeholder(dispenser, "jumlah objek == target", TERIMA_TARGET_JUMLAH_DELAY_S)

            # BARU: conveyor WAJIB nyala DULU sebelum servo round-trip -- kalau conveyor
            # diam pas servo1/servo2 jatuhin objek, objeknya numpuk di bawah gerbang servo
            # (gak kebawa maju). Beda dari urutan lama (servo dulu, conveyor baru nyala di [13]).
            # SENGAJA unconditional (gak pakai conveyor_on_if_safe seperti [13]) -- package YANG
            # BARU AJA ke-detect di [6] sendiri masih nempatin PROX_2 (baru mati [7], belum
            # sempat kegeser), jadi kalau di-gate PROX_2 di sini bakal DEADLOCK (gak pernah
            # nyala sama sekali). Guard universal PROX_2 buat package BERIKUTNYA dipantau
            # reaktif selama servo watch [10]/[12] (lihat stop_conveyor_if_middle_triggered).
            print("\n=== [9b] Conveyor MENYALA (sebelum round-trip, biar objek jatuh gak numpuk) ===")
            if not do_command(dispenser, "SET_CONVEYOR_ON_OFF(1)", CMD_SET_CONVEYOR_ON_OFF, arg=1):
                break

            print("\n=== [10] Servo1 round-trip -- PROX_2 dipantau reaktif selama gerak ===")
            if not do_command(dispenser, "TEST_SERVO1_CYCLE", CMD_TEST_SERVO1_CYCLE, watch_activity_s=5.0):
                break

            print("\n=== [12] Servo2 round-trip -- PROX_2 dipantau reaktif selama gerak ===")
            if not do_command(dispenser, "TEST_SERVO2_CYCLE", CMD_TEST_SERVO2_CYCLE, watch_activity_s=5.0):
                break

            print("\n=== [13] Nyalain conveyor LAGI -- KECUALI PROX_2 lagi trigger (guard [10]/[12] baru stop) ===")
            if not conveyor_on_if_safe(dispenser, "SET_CONVEYOR_ON_OFF(1)"):
                break

            print("\n=== [14] Tunggu package maju melewati PROX_1 (UJUNG) -- PROX_2 dipantau pararel, self-healing ===")
            if not wait_prox_with_middle_guard(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 1):
                break

            print("\n=== [15] Kirim command ambil package (TIMING placeholder) ===")
            timing_placeholder(dispenser, "command ambil package", KIRIM_AMBIL_PACKAGE_DELAY_S)

            # DIUBAH (2026-09-20): [16] BALIK ke wait_prox biasa (TANPA guard PROX_2) --
            # sesuai instruksi ASLI yang eksplisit soal poin ini: "prox_1 sudah 0 tapi prox_2
            # belum 0 -> conveyor tetap maju". Temuan #12 kemarin salah perluas guard ke sini
            # juga, padahal poin 16 punya pengecualian sendiri dari awal.
            print("\n=== [16] Tunggu PROX_1 CLEAR -- conveyor TETAP MENYALA apapun status PROX_2 ===")
            if not wait_prox(dispenser, "PROX_1 (UJUNG)", REG_UJUNG_PACKAGE_PRESENT, 0):
                break

            print("\n=== [17] Balik ke poin 6 (tunggu PROX_2 trigger lagi) ===")
            if DRY_RUN:
                print("\n(DRY_RUN) 1 putaran demo selesai, berhenti di sini.")
                break

    except KeyboardInterrupt:
        print("\n[STOP] Dihentikan manual (Ctrl+C)")
    finally:
        # BARU (bug ditemukan): DULU safety-stop cuma jalan di path KeyboardInterrupt --
        # keluar loop lewat `break` biasa (timeout wait_prox, ack gagal, dst) TIDAK dapat
        # safety-stop sama sekali, conveyor tetap nyala nyangkut walau skrip Python udah
        # berhenti. Sekarang di `finally` -- SELALU jalan apapun cara keluarnya (break,
        # exception, atau Ctrl+C). ESP32 gak tau/gak peduli skrip Python-nya masih hidup
        # atau enggak -- kalau ini gak dikirim, conveyor TETAP jalan terus di node.
        print("\n[SAFETY-STOP] Kirim SET_CONVEYOR_ON_OFF(0) -- ESP32 gak otomatis berhenti sendiri.")
        do_command(dispenser, "SET_CONVEYOR_ON_OFF(0) [safety-stop]", CMD_SET_CONVEYOR_ON_OFF, arg=0)

    print(f"\n=== BERHENTI setelah {loop_count} putaran ===")


if __name__ == '__main__':
    main()
