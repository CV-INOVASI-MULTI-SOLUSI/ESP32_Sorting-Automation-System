#!/usr/bin/env python3
"""
============================================================
 CONTOH -- Menggerakkan PALANG di SORTER dari Orange Pi
============================================================

Palang adalah actuator penolak objek REJECT. Secara fisik ia linear actuator
(motor DC) di channel Motor A, bukan solenoid: harus di-drive AKTIF dua arah,
maju (push) lalu mundur (retract). Tidak ada pegas yang menariknya balik.

Ada EMPAT cara menggerakkannya, dan tiga yang pertama berbeda maksud:

  1. Jalur PRODUKSI  -- tulis CLASSIFY_IS_REJECT. Inilah yang dipakai sungguhan.
  2. Jalur TEST      -- TEST_TRIGGER_PALANG, simulasi 1 objek reject.
  3. Jog MANUAL      -- SET_MOTOR_A, gerakkan motornya langsung tanpa siklus.
  4. Tuning          -- SET_PALANG_SPEED, cuma mengubah angka, tidak menggerakkan.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial

Jalankan:
    python3 'contoh-kirim-command-palang 20260921.py'
"""

import time

import minimalmodbus
import serial

# ============================================================
# KONFIGURASI
# ============================================================
SERIAL_PORT = '/dev/ttyS3'
BAUDRATE = 19200
SORTER_SLAVE_ID = 1

# --- Register SORTER (lihat ESP32_SortingAutomation_Sorter/include/registers.h) ---
REG_STATE               = 0
REG_FAULT_CODE          = 1
REG_CMD                 = 2
REG_CMD_ARG             = 3
REG_CMD_SEQ             = 4
REG_CMD_ACK_SEQ         = 5
REG_PASS_COUNT          = 10
REG_REJECT_COUNT        = 11
REG_CLASSIFY_IS_REJECT  = 12
REG_ACTIVITY_CODE       = 13
REG_MAIN_MODE_ACTIVE    = 17
REG_MENU_ACTIVE         = 18
REG_REJECT_MISSED_COUNT = 19

# --- Opcode SORTER ---
CMD_START               = 1
CMD_STOP                = 2
CMD_RESET_FAULT         = 3
CMD_SET_MOTOR_A         = 8    # arg: 0=stop, 1=maju, 2=mundur
CMD_SET_PALANG_SPEED    = 9    # arg: PWM 0-255
CMD_TEST_TRIGGER_PALANG = 98

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM",
    1: "CONVEYOR_JALAN",
    2: "CONVEYOR_JALAN_PALANG_AKTIF",   # <-- inilah tanda palang sedang mendorong
    3: "MOTOR_A_JALAN",
    4: "TEST_HOPPER_AKTIF",
    90: "FAULT_AKTIF",
    91: "ESTOP_AKTIF",
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


# ============================================================
# MENGIRIM COMMAND -- protokol 3 langkah
# ============================================================
_seq = 0


def kirim_command(instr, nama, opcode, arg=0, timeout=5.0):
    """Kirim opcode lalu tunggu ack.

    PENTING -- ACK BUKAN BERARTI BERHASIL. Firmware menulis CMD_ACK_SEQ tanpa
    syarat, termasuk untuk command yang baru saja DITOLAKNYA. Jadi ack hanya
    berarti "sudah sampai dan sudah diproses". Berhasil atau tidaknya dibaca
    dari STATE / FAULT_CODE / ACTIVITY_CODE, atau dari efek fisiknya.
    """
    global _seq
    _seq += 1
    seq = _seq
    print(f"  -> {nama} (opcode={opcode} arg={arg} seq={seq})")

    instr.write_register(REG_CMD_ARG, arg, functioncode=6)   # 1. argumen
    instr.write_register(REG_CMD_SEQ, seq, functioncode=6)   # 2. nomor urut
    instr.write_register(REG_CMD, opcode, functioncode=6)    # 3. opcode TERAKHIR

    batas = time.monotonic() + timeout
    while time.monotonic() < batas:
        try:
            if instr.read_register(REG_CMD_ACK_SEQ, functioncode=3) == seq:
                return True
        except Exception as e:
            print(f"     gagal baca ack ({e}), diulang...")
        time.sleep(0.1)

    # Tidak ada ack sama sekali punya arti khusus -- lihat cek_prasyarat().
    print(f"  !! {nama} tidak di-ack dalam {timeout}s")
    return False


def baca(instr, reg, bawaan=None):
    try:
        return instr.read_register(reg, functioncode=3)
    except Exception:
        return bawaan


def status(instr, judul=""):
    st = baca(instr, REG_STATE)
    print(f"     STATE={st}({STATE_NAMES.get(st, '?')}) "
          f"FAULT={baca(instr, REG_FAULT_CODE)} "
          f"ACTIVITY={baca(instr, REG_ACTIVITY_CODE)} "
          f"PASS={baca(instr, REG_PASS_COUNT)} "
          f"REJECT={baca(instr, REG_REJECT_COUNT)}"
          f"{('  <- ' + judul) if judul else ''}")


def cek_prasyarat(instr):
    """Tiga hal yang diam-diam membuat command palang tidak berpengaruh."""
    st = baca(instr, REG_STATE)
    if st is None:
        print("!! Sorter tidak menjawab Modbus. Cek daya node dan kabel RS485 A/B.")
        return False

    # (1) Menu kalibrasi LCD. Selama operator berada di menu, SEMUA command Modbus
    #     diabaikan DAN tidak di-ack -- dari sini kelihatan persis seperti node mati.
    #     Register MENU_ACTIVE ada supaya dua kondisi itu bisa dibedakan.
    if baca(instr, REG_MENU_ACTIVE) == 1:
        print("!! Operator sedang di menu kalibrasi LCD. Semua command diabaikan.")
        print("   Keluar dulu dari menu (tombol D sampai kembali ke layar utama).")
        return False

    # (2) FAULT/ESTOP memblokir command gerak.
    fault = baca(instr, REG_FAULT_CODE)
    if fault:
        print(f"!! Sorter FAULT (code={fault}). Kirim RESET_FAULT dulu, atau lewat")
        print("   menu LCD: Setting Kalibrasi -> Reset Fault.")
        return False

    # (3) MAIN vs TEST. Command test ditolak selama produksi berjalan, dan sebaliknya.
    mode = "MAIN (produksi)" if baca(instr, REG_MAIN_MODE_ACTIVE) == 1 else "TEST"
    print(f"  .. mode sekarang: {mode}")
    return True


def pantau_palang(instr, detik=4.0):
    """Palang TIDAK mendorong seketika setelah command dikirim.

    Firmware menjadwalkannya sejauh waktu tempuh objek (TOF = jarak scan->palang
    dibagi kecepatan conveyor), jadi jedanya tergantung kalibrasi. Karena itu
    verifikasinya dengan MENGAMATI, bukan dengan menganggap ack sebagai selesai.

    Dua tanda bahwa palang benar-benar bekerja:
      - ACTIVITY_CODE sempat bernilai 2 (CONVEYOR_JALAN_PALANG_AKTIF), dan
      - REJECT_COUNT naik 1.
    """
    reject_awal = baca(instr, REG_REJECT_COUNT)
    terlihat = set()
    batas = time.monotonic() + detik
    while time.monotonic() < batas:
        a = baca(instr, REG_ACTIVITY_CODE)
        if a is not None and a not in terlihat:
            terlihat.add(a)
            print(f"     [pantau] ACTIVITY_CODE = {a} ({ACTIVITY_NAMES.get(a, '?')})")
        time.sleep(0.1)

    reject_akhir = baca(instr, REG_REJECT_COUNT)
    print(f"     REJECT_COUNT {reject_awal} -> {reject_akhir}")
    if reject_akhir is not None and reject_awal is not None and reject_akhir > reject_awal:
        print("     PALANG BEKERJA.")
        return

    print("     Palang belum terlihat bekerja dalam jendela pengamatan.")
    print("     Kemungkinan: TOF lebih lama dari jendela ini, atau command ditolak")
    print("     (lihat Serial USB Sorter untuk pesan '[CMD] ... ditolak').")
    terlewat = baca(instr, REG_REJECT_MISSED_COUNT)
    if terlewat:
        print(f"     Catatan: REJECT_MISSED_COUNT = {terlewat} -- ada reject yang dibuang")
        print("     karena antrian penuh atau gilirannya sudah basi.")


# ============================================================
# CARA 1 -- JALUR PRODUKSI: tulis hasil klasifikasi
# ============================================================
def cara1_klasifikasi_reject(instr):
    """Inilah jalur yang sesungguhnya dipakai saat produksi.

    Cukup SATU write_register, bukan protokol 3 langkah -- register ini berdiri
    sendiri dan ditulis sangat sering (sekali per objek), jadi sengaja dibuat
    semurah mungkin. Konsekuensinya tidak ada ack: keberhasilan dinilai dari
    REJECT_COUNT yang naik.

    Firmware mengantre klasifikasi ini lalu mendorong palang setelah objek
    diperkirakan sampai (TOF). Jalur ini TIDAK dibatasi MAIN/TEST -- ia memang
    input produksi dari HuskyLens/Orange Pi.
    """
    print("\n=== CARA 1: klasifikasi REJECT (jalur produksi) ===")
    instr.write_register(REG_CLASSIFY_IS_REJECT, 1, functioncode=6)
    print("  -> CLASSIFY_IS_REJECT = 1 (register tunggal, tanpa ack)")
    pantau_palang(instr)


# ============================================================
# CARA 2 -- JALUR TEST: simulasi satu objek reject
# ============================================================
def cara2_test_trigger(instr):
    """Sama efeknya dengan cara 1, tapi lewat opcode sehingga ada ack-nya.

    DITOLAK kalau MAIN sedang aktif. Alasannya: selama produksi berjalan, objek
    palsu tidak boleh menyelinap ke antrian klasifikasi yang sesungguhnya.
    Kirim STOP dulu kalau memang perlu menguji.
    """
    print("\n=== CARA 2: TEST_TRIGGER_PALANG (jalur test) ===")
    if baca(instr, REG_MAIN_MODE_ACTIVE) == 1:
        print("  .. MAIN aktif -> kirim STOP dulu supaya command test diterima")
        kirim_command(instr, "STOP", CMD_STOP)

    kirim_command(instr, "TEST_TRIGGER_PALANG", CMD_TEST_TRIGGER_PALANG)
    pantau_palang(instr)


# ============================================================
# CARA 3 -- JOG MANUAL Motor A
# ============================================================
def cara3_jog_manual(instr):
    """Menggerakkan motor palang langsung, tanpa siklus push/retract otomatis.

    Dua hal yang membuat command ini ditolak:
      - MAIN sedang aktif (ini command TEST), dan
      - palang otomatis sedang memakai Motor A (sedang push/retract, atau ada
        reject yang menunggu giliran). Keduanya menulis ke output fisik yang
        sama, jadi jog manual yang mengalah -- produksi menang atas diagnostik.

    Karena ini jog mentah, JANGAN lupa mengembalikannya ke 0. Motor tidak
    berhenti sendiri.
    """
    print("\n=== CARA 3: jog manual Motor A ===")
    if baca(instr, REG_MAIN_MODE_ACTIVE) == 1:
        print("  .. MAIN aktif -> kirim STOP dulu")
        kirim_command(instr, "STOP", CMD_STOP)

    kirim_command(instr, "SET_MOTOR_A maju", CMD_SET_MOTOR_A, arg=1)
    status(instr, "harusnya ACTIVITY=3 (MOTOR_A_JALAN)")
    time.sleep(0.4)

    kirim_command(instr, "SET_MOTOR_A mundur", CMD_SET_MOTOR_A, arg=2)
    time.sleep(0.4)

    kirim_command(instr, "SET_MOTOR_A stop", CMD_SET_MOTOR_A, arg=0)
    status(instr, "motor sudah berhenti")


# ============================================================
# CARA 4 -- TUNING kecepatan palang
# ============================================================
def cara4_atur_kecepatan(instr, pwm=180):
    """Mengubah angka saja, tidak menggerakkan apa pun.

    Karena itu command ini NETRAL: tidak dibatasi MAIN/TEST dan boleh dikirim
    kapan saja, termasuk selagi produksi berjalan. Nilainya berlaku seketika
    untuk dorongan berikutnya, tapi hanya di RAM -- supaya bertahan setelah
    reboot, simpan lewat menu LCD (tombol '#').
    """
    print(f"\n=== CARA 4: SET_PALANG_SPEED = {pwm} ===")
    kirim_command(instr, f"SET_PALANG_SPEED({pwm})", CMD_SET_PALANG_SPEED, arg=pwm)


# ============================================================
if __name__ == '__main__':
    sorter = connect(SORTER_SLAVE_ID)

    print("=" * 60)
    print("CONTOH KIRIM COMMAND PALANG -- SORTER (slave 1)")
    print("=" * 60)
    status(sorter, "kondisi awal")

    if not cek_prasyarat(sorter):
        raise SystemExit(1)

    cara2_test_trigger(sorter)        # paling aman untuk mulai
    cara1_klasifikasi_reject(sorter)
    cara3_jog_manual(sorter)
    cara4_atur_kecepatan(sorter, 180)

    print("\nSelesai.")
    status(sorter, "kondisi akhir")
