"""
============================================================
 APPLIB -- pustaka Sorting Automation (2026-10-07)
============================================================

SEMUA fungsi ada di file ini; program yang dijalankan adalah sorting-automation.py.
Menggantikan tiga file lama:
  orangepi-orchestrator.py   -> bagian MESIN PRODUKSI
  sorting_automation.py      -> bagian TABEL NODE & MENU UJI (--uji)
  sorting-panel.py           -> bagian LAYAR (panel LCD sentuh)

File pendamping (satu folder):
  sorting.config     produksi: port, batch, siklus, delay, rak, pin LED/buzzer, kamera, batas waktu
  display.config     layar: pin & SPI LCD/touch, rotasi, judul, warna tema
  rak_terisi.txt, alarm_riwayat.json, touch_kalibrasi.json   -- dibuat otomatis

Isi file, berurutan:
  1. KONFIGURASI      memuat sorting.config (+ warna display.config)
  2. MESIN PRODUKSI   Bus Modbus, pemeriksaan awal, HuskyLens, startup, batch, rak, shutdown
  3. TABEL NODE       command & register tiap node (ACUAN, sama dengan panel ESP32)
  4. MENU UJI         menu terminal per node (--uji)
  5. LAYAR            driver ILI9341 + XPT2046, tampilan HMI, jalankan_panel()

Hanya SATU program yang boleh memegang /dev/ttyS3 pada satu waktu.
"""

import collections
import configparser
import json
import os
import queue
import random
import re
import signal
import sys
import threading
import time

try:
    import minimalmodbus
    import serial
except ImportError:
    print("ERROR: library belum terinstal:  pip3 install minimalmodbus pyserial")
    sys.exit(1)

try:
    from PIL import Image, ImageDraw, ImageFont
    ADA_PIL = True
except ImportError:     # mode terminal (--tanpa-lcd, --uji) tetap jalan tanpa Pillow
    ADA_PIL = False

FOLDER = os.path.dirname(os.path.abspath(__file__))
FILE_DISPLAY = os.path.join(FOLDER, 'display.config')
FILE_KALIBRASI = os.path.join(FOLDER, 'touch_kalibrasi.json')
LEBAR, TINGGI = 320, 240


def _warna_display():
    """[warna] di display.config, opsional: nama = RRGGBB (tanpa #, karena # = komentar).
    Yang tidak ditulis memakai warna bawaan."""
    hasil = {}
    if not os.path.exists(FILE_DISPLAY):
        return hasil
    cp = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cp.read(FILE_DISPLAY, encoding='utf-8-sig')
    if not cp.has_section('warna'):
        return hasil
    for kunci, nilai in cp.items('warna'):
        m = re.fullmatch(r'#?([0-9a-fA-F]{6})', nilai.strip())
        if not m:
            sys.exit(f"ERROR di {FILE_DISPLAY}: [warna] {kunci} = {nilai!r} -- tulis RRGGBB, mis. 1E78E6")
        h = m.group(1)
        hasil[kunci.upper()] = (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))
    return hasil




# ############################################################################
# 1-2. KONFIGURASI & MESIN PRODUKSI (dulu orangepi-orchestrator.py)
# ############################################################################
# ============================================================
#  ORANGE PI ORCHESTRATOR -- program produksi (2026-09-30)
#  HuskyLens + Sorter + Dispenser + Picker + Stocker dalam SATU program
# ============================================================
#
# Menggantikan DUA program sekaligus:
#   - orangepi-orchestrator-batch.py   (urutan batch package)
#   - script HuskyLens produksi         (klasifikasi objek -> palang)
#
# KENAPA DIGABUNG. Keduanya memakai /dev/ttyS3. Dua program di satu bus RS485 saling
# merusak pembacaan -- gejalanya mirip kabel rusak. Di sini seluruh akses Modbus lewat SATU
# objek `Bus` dengan kunci (lock), dan setiap command 3-langkah (ARG -> SEQ -> CMD) ditulis
# utuh di bawah kunci itu, jadi tidak pernah bisa disisipi tulisan lain di tengahnya.
#
# TIGA THREAD:
#   kamera     HuskyLens -> keputusan -> CLASSIFY_IS_REJECT. Tidak pernah menunggu batch.
#              (Kalau digabung secara berurutan, satu batch yang makan puluhan detik akan
#              membuat semua objek lewat tanpa diklasifikasi -- reject lolos semua.)
#   indikator  LED & buzzer. Terpisah supaya bunyi tidak menunda kamera.
#   keepalive  membaca STATE tiap node berkala, supaya watchdog komunikasi node tidak
#              menganggap master mati saat sedang menunggu gerakan panjang.
#   (utama)    startup -> pantau PASS_COUNT -> batch -> shutdown.
#
# YANG DIPERBAIKI DARI DUA PROGRAM LAMA
#   1. Klasifikasi ditulis SEBELUM LED/buzzer. Script HuskyLens lama membunyikan buzzer dulu
#      (~0,9 s) baru menulis REJECT. Firmware mencatat waktu scan saat tulisan TIBA, jadi
#      palang mendorong 0,9 s terlambat dari seharusnya.
#   2. Satu objek = satu klasifikasi. Script lama langsung mengambil sampel lagi begitu satu
#      keputusan selesai; objek yang masih di depan kamera dihitung DUA kali, palang mendorong
#      dua kali, dan antrian cepat penuh. Sekarang kamera harus melihat bidang kosong dulu
#      (ID kosong / tidak ada objek) selama MIN_KOSONG_S sebelum objek berikutnya diterima.
#   3. Startup Dispenser tidak lagi mengirim FORCE_MIDDLE_REFILL. START_MAIN sudah menjalankan
#      pipeline yang menjatuhkan package pertama sendiri; FORCE_MIDDLE_REFILL di atasnya
#      membuat dua mekanisme menggerakkan servo & conveyor yang sama. Sekarang: START_MAIN,
#      lalu tunggu DISPENSER_READY = 1.
#   4. REQUEST_REFILL hanya dikirim setelah DISPENSER_READY = 1 (firmware menolaknya di tahap
#      lain, dan penolakan tetap di-ack -- dulu batch berikutnya gagal tanpa sebab terlihat).
#   5. Batas waktu mengikuti firmware: package ke UJUNG 30 s (firmware 20 s), MOVE_PACKAGE
#      180 s (ack Picker baru dikirim setelah lima gerakan lengan selesai dan lengan kembali
#      di READY).
#   6. PASS_COUNT tidak lagi di-reset tiap batch. Objek yang lewat SELAMA batch dulu ikut
#      terhapus, sehingga package berikutnya berisi lebih dari BATCH_SIZE. Sekarang dihitung
#      dari titik acuan yang maju BATCH_SIZE per batch.
#   7. Batch gagal = BERHENTI, bukan diulang. Program lama mengulang batch tiap ~5 detik karena
#      PASS_COUNT tetap di atas batas -- berlawanan dengan komentarnya sendiri.
#   8. Pemeriksaan awal menolak jalan kalau ada node dengan firmware lama, sedang FAULT,
#      E-stop, atau operatornya sedang di menu LCD.
#
# SEBELUM PRODUKSI -- tidak bisa diperiksa dari sini:
#   - SEMUA node di-flash dengan firmware >= 2026-09-30. Firmware lebih tua belum menghitung
#     request baca sebagai tanda master hidup, jadi Sorter jatuh FAULT COMM_TIMEOUT 30 detik
#     setelah START.
#   - Adegan keempat gerakan Picker sudah disetel, dan pose tujuannya disimpan dari posisi
#     akhir adegan (Test Adegan, CC) -- tanpa itu lengan melompat sesudah adegan terakhir.
#   - Pose READY Picker (pose 3, titik siaga dekat package) sudah disetel lewat gerakan Home>Ready.
#   - READY (Load Position) Stocker sejajar dengan pose 2 (LIFT) Picker.
#   - Protocol Type HuskyLens = UART dengan baud = HUSKYLENS_BAUD.

# ============================================================
# KONFIGURASI -- dibaca dari sorting.config (dulu orchestrator.config): baud rate, jumlah batch,
# delay, pin out, batas waktu. File teks biasa, satu folder dengan program ini.
#
# Setiap nilai punya tipe yang diperiksa SAAT MULAI. Salah ketik (huruf di tempat angka,
# nama yang hilang) menghentikan program dengan pesan yang menyebut bagian dan namanya --
# tidak pernah ketahuan baru di tengah produksi.
# ============================================================
def _pin(teks):
    return None if teks.strip().lower() in ('', 'none', '-') else int(teks)


def _nol_satu(teks):
    t = teks.strip()
    if t == '1':
        return True
    if t == '0':
        return False
    raise ValueError(teks)


def _jeda_hopper(teks):
    nilai = int(teks)
    if nilai not in (0, 1, 2):
        raise ValueError(teks)
    return nilai


def _urutan_rak(teks):
    urutan = _daftar_int(teks)
    if not urutan or any(r not in (1, 2, 3, 4) for r in urutan) or len(set(urutan)) != len(urutan):
        raise ValueError(teks)
    return urutan


def _daftar_int(teks):
    return tuple(int(x) for x in teks.split(',') if x.strip())


SKEMA_CONFIG = {
    'port': {'RS485_PORT': str, 'RS485_BAUD': int, 'HUSKYLENS_PORT': str, 'HUSKYLENS_BAUD': int},
    'produksi': {'BATCH_SIZE': int, 'JUMLAH_SIKLUS': int, 'STOCKER_PARALEL': _nol_satu,
                 'JEDA_HOPPER': _jeda_hopper, 'LANJUT_HOPPER': _nol_satu},
    'rak': {'RAK_RANDOM': _nol_satu, 'URUTAN_RAK': _urutan_rak, 'PAKAI_SENSOR_RAK': _nol_satu},
    'delay': {'DELAY_BATCH_S': float, 'DELAY_PICKER_S': float, 'DELAY_DISPENSER_S': float, 'DELAY_STOCKER_S': float},
    'pin': {'LED_RED': _pin, 'LED_BLUE': _pin, 'BUZZER': _pin, 'LED_OPR': _pin, 'LED_ALARM': _pin},
    'huskylens': {'PAKAI_KAMERA': _nol_satu, 'JUMLAH_SAMPEL': int, 'INTERVAL_SAMPEL_S': float, 'BATAS_AMBIL_S': float,
                  'ID_KOSONG': int, 'ID_VALID': int, 'ID_INVALID': _daftar_int, 'MIN_KOSONG_S': float},
    'pemantauan': {'POLL_S': float, 'KEEPALIVE_S': float},
    'batas_waktu': {'T_ACK': float, 'T_PICKER_HOME': float, 'T_STOCKER_HOMING': float,
                    'T_STOCKER_GERAK': float, 'T_DISPENSER_SIAP_AWAL': float, 'T_DISPENSER_SIAP': float,
                    'T_PACKAGE_UJUNG': float, 'T_MOVE_PACKAGE': float, 'T_FULL_CYCLE': float,
                    'MIN_DURASI_MOVE_PACKAGE_S': float, 'MIN_DURASI_FULL_CYCLE_S': float},
}
FILE_CONFIG = os.path.join(FOLDER, 'sorting.config')


def muat_config(path=FILE_CONFIG):
    cp = configparser.ConfigParser(inline_comment_prefixes=('#',))
    try:
        # utf-8-sig: file yang disimpan Notepad Windows diawali BOM, dan tanpa ini baris
        # pertamanya tidak dikenali sebagai apa pun.
        if not cp.read(path, encoding='utf-8-sig'):
            sys.exit(f"ERROR: {path} tidak ada. Salin sorting.config ke folder yang sama "
                     f"dengan program ini.")
    except configparser.Error as e:
        sys.exit(f"ERROR: format {path} rusak -- {e}")
    hasil, salah = {}, []
    for bagian, isi in SKEMA_CONFIG.items():
        for nama, ubah in isi.items():
            kunci = nama.lower()
            if not cp.has_option(bagian, kunci):
                salah.append(f"[{bagian}] {kunci} tidak ada")
                continue
            teks = cp.get(bagian, kunci)
            try:
                hasil[nama] = ubah(teks)
            except ValueError:
                salah.append(f"[{bagian}] {kunci} = {teks!r} bukan nilai yang sah")
    if salah:
        sys.exit("ERROR di " + path + ":\n  " + "\n  ".join(salah))
    return hasil


globals().update(muat_config())

# ============================================================
# PETA MODBUS (registers.h tiap node, 2026-09-30)
# ============================================================
SORTER, PICKER, DISPENSER, STOCKER = 1, 2, 3, 4
NAMA = {SORTER: 'SORTER', PICKER: 'PICKER', DISPENSER: 'DISPENSER', STOCKER: 'STOCKER'}

R_STATE, R_FAULT, R_CMD, R_ARG, R_SEQ, R_ACK = 0, 1, 2, 3, 4, 5
ST_IDLE, ST_RUN, ST_FAULT, ST_ESTOP = 1, 2, 3, 4

R_MAIN = {SORTER: 17, PICKER: 15, DISPENSER: 18, STOCKER: 17}
R_MENU = {SORTER: 18, PICKER: 16, DISPENSER: 20, STOCKER: 18}

R_SORTER_BLOK = 10             # 10..19 dibaca sekaligus
FAULT_SORTER = {1: 'COMM_TIMEOUT', 5: 'ESTOP_ACTIVE', 10: 'HOPPER_JAM', 20: 'IO_EXPANDER_MISSING'}
R_SORTER_CLASSIFY = 12
R_PICKER_POSE, R_PICKER_ACTIVITY = 10, 11
R_DISP_PACKAGE_READY, R_DISP_READY, R_DISP_STAGE = 15, 25, 26
R_UPTIME = {SORTER: 16, PICKER: 14, DISPENSER: 14, STOCKER: 16}   # detik sejak node menyala
R_DISP_UJUNG = 17              # UJUNG_PACKAGE_PRESENT (PROX_1, sudah di-debounce firmware)
STAGE_TUNGGU_DIAMBIL = 15      # PIPELINE_STAGE 'tunggu diambil arm' -- hanya di sini ACK diterima
POSE_HOME, POSE_LIFT = 0, 2    # CURRENT_POSE Picker
R_STOCKER_HOMED, R_STOCKER_RAK = 11, 12

OP_START_MAIN = {SORTER: 1, PICKER: 11, DISPENSER: 14, STOCKER: 9}   # Sorter: START
OP_STOP_MAIN = {SORTER: 2, PICKER: 12, DISPENSER: 15, STOCKER: 10}   # Sorter: STOP
OP_SORTER_RESET_COUNTERS = 7
OP_SORTER_HOPPER_JEDA = 11       # arg 1 = jeda umpan hopper, 0 = lanjut (firmware Sorter >= 2026-10-05)
R_SORTER_HOPPER_DIJEDA = 24
OP_PICKER_GOTO_HOME, OP_PICKER_MOVE_PACKAGE, OP_PICKER_GOTO_READY = 2, 8, 15
POSE_READY = 3                 # Picker: 0=HOME 1=PICKUP 2=LIFT 3=READY
NAMA_POSE = {0: 'HOME', 1: 'PICKUP', 2: 'LIFT', 3: 'READY'}
GERAKAN_HOME_READY = 0         # GERAKAN_AKTIF Picker (reg 17), firmware >= 2026-10-01
NAMA_GERAKAN = {0: 'Home>Ready', 1: 'Ready>Pick', 2: 'Pick>Home', 3: 'Home>Place', 4: 'Place>Home',
                0xFF: '-', None: '?'}
OP_DISP_REQUEST_REFILL, OP_DISP_ACK_TAKEN = 1, 5
OP_STOCKER_HOME_ALL, OP_STOCKER_FULL_CYCLE, OP_STOCKER_GOTO_READY = 1, 2, 6   # READY = Load Position

PIPELINE_NAMES = [
    'MATI', 'INIT servo1 buka', 'INIT servo1 tahan', 'INIT servo1 tutup',
    'INIT servo2 buka', 'INIT servo2 tahan', 'INIT servo2 tutup', 'INIT tunggu PROX_2',
    'buka gerbang', 'SIAP ISI (ready)', 'maju ke ujung', 'refill servo1 tutup',
    'refill servo2 buka', 'refill servo2 tahan', 'refill servo2 tutup', 'tunggu diambil arm',
]

berhenti = threading.Event()      # Ctrl+C / SIGTERM
T_MULAI = time.monotonic()        # diisi ulang main() -- dibanding uptime node untuk deteksi restart
rak_kosong_awal = 0               # diisi periksa() -- target bawaan kalau jumlah_siklus = 0
produksi = threading.Event()      # set = kamera boleh menulis klasifikasi
alarm = threading.Event()         # set = produksi DITAHAN / gagal -- LED ALARM nyala


# BARU (2026-10-06): status & log terakhir untuk panel LCD (bagian 5 file ini). Mode terminal tidak
# memakainya, tapi ikut mengisinya -- murah, dan satu jalur log untuk keduanya.
LOG_BARU = collections.deque(maxlen=300)
STATUS = {'fase': 'SIAP', 'batch': 0, 'target': 0, 'pass_batch': 0, 'rak': None,
          'langkah': '', 'mulai': None, 'alasan': ''}


def log(teks):
    baris = time.strftime('%H:%M:%S ') + teks
    print(baris, flush=True)
    LOG_BARU.append(baris)
    if teks.strip():
        STATUS['langkah'] = teks.strip()


class Gagal(Exception):
    """Langkah produksi tidak bisa dilanjutkan dengan aman."""


class RegisterTidakAda(Exception):
    """Register tidak ada di firmware node itu -- firmware belum di-flash ulang."""


# ============================================================
# BUS -- satu-satunya jalan ke RS485
# ============================================================
class Bus:
    def __init__(self):
        self.kunci = threading.Lock()
        self.seq = 0
        self.instr = {}
        for sid in NAMA:
            i = minimalmodbus.Instrument(RS485_PORT, sid)
            i.serial.baudrate = RS485_BAUD
            i.serial.bytesize = 8
            i.serial.parity = serial.PARITY_NONE
            i.serial.stopbits = 1
            i.serial.timeout = 0.5
            i.mode = minimalmodbus.MODE_RTU
            i.clear_buffers_before_each_transaction = True
            self.instr[sid] = i

    def baca(self, sid, reg, jumlah=1, percobaan=3):
        """Satu nilai (jumlah=1) atau list. NoResponse dan IllegalRequest SENGAJA dibedakan:
        yang pertama node tidak menjawab, yang kedua register tidak ada di firmware."""
        terakhir = None
        with self.kunci:
            for _ in range(percobaan):
                try:
                    if jumlah == 1:
                        return self.instr[sid].read_register(reg, functioncode=3)
                    return self.instr[sid].read_registers(reg, jumlah, functioncode=3)
                except minimalmodbus.IllegalRequestError:
                    raise RegisterTidakAda(f"{NAMA[sid]}: register {reg} tidak ada di firmware")
                except Exception as e:
                    terakhir = e
                    time.sleep(0.05)
        raise Gagal(f"{NAMA[sid]} tidak menjawab ({type(terakhir).__name__})")

    def tulis(self, sid, reg, nilai, percobaan=2):
        terakhir = None
        with self.kunci:
            for _ in range(percobaan):
                try:
                    self.instr[sid].write_register(reg, nilai, functioncode=6)
                    return
                except Exception as e:
                    terakhir = e
                    time.sleep(0.03)
        raise Gagal(f"{NAMA[sid]}: gagal menulis register {reg} ({type(terakhir).__name__})")

    def command(self, sid, opcode, arg=0):
        """ARG -> SEQ -> CMD ditulis UTUH di bawah kunci; tidak ada tulisan lain yang bisa
        menyelinap di antaranya dan mengubah argumen command ini."""
        with self.kunci:
            self.seq = (self.seq % 65534) + 1
            seq = self.seq
            i = self.instr[sid]
            try:
                i.write_register(R_ARG, arg, functioncode=6)
                i.write_register(R_SEQ, seq, functioncode=6)
                i.write_register(R_CMD, opcode, functioncode=6)
            except Exception as e:
                raise Gagal(f"{NAMA[sid]}: gagal mengirim opcode {opcode} ({type(e).__name__})")
        return seq


def tunggu(bus, sid, fungsi_cek, timeout, label):
    """Tunggu sampai fungsi_cek() benar. Sambil menunggu, FAULT node itu ikut diawasi --
    menunggu 3 menit untuk node yang sudah FAULT di detik pertama tidak ada gunanya."""
    batas = time.monotonic() + timeout
    while time.monotonic() < batas:
        if berhenti.is_set():
            raise Gagal("dihentikan operator")
        state, fault = bus.baca(sid, R_STATE, 2)
        if fault or state in (ST_FAULT, ST_ESTOP):
            raise Gagal(f"{NAMA[sid]} FAULT (code={fault}, state={state}) saat menunggu {label}")
        if fungsi_cek():
            return
        time.sleep(0.3)
    raise Gagal(f"{NAMA[sid]}: {label} tidak tercapai dalam {timeout:.0f} s")


def kirim(bus, sid, opcode, nama, arg=0, timeout=T_ACK):
    """Kirim command lalu tunggu ack-nya. Mengembalikan durasi sampai ack (detik).
    INGAT: ack BUKAN berarti berhasil -- pemanggil wajib memeriksa hasilnya."""
    t0 = time.monotonic()
    seq = bus.command(sid, opcode, arg)
    tunggu(bus, sid, lambda: bus.baca(sid, R_ACK) == seq, timeout, f"ack {nama}")
    return time.monotonic() - t0


def pastikan_main(bus, sid, nyala):
    nilai = bus.baca(sid, R_MAIN[sid])
    if nilai != (1 if nyala else 0):
        raise Gagal(f"{NAMA[sid]}: {'START' if nyala else 'STOP'}_MAIN di-ack tapi MAIN_MODE_ACTIVE={nilai}")


# ============================================================
# PEMERIKSAAN AWAL
# ============================================================
def periksa(bus, pakai_kamera):
    log("== PEMERIKSAAN AWAL ==")
    ok = True
    for sid in NAMA:
        try:
            state, fault = bus.baca(sid, R_STATE, 2)
            menu = bus.baca(sid, R_MENU[sid])
            bus.baca(sid, R_MAIN[sid])
        except RegisterTidakAda:
            log(f"  !! {NAMA[sid]:10s} FIRMWARE LAMA -- MAIN_MODE_ACTIVE/MENU_ACTIVE tidak ada. Flash ulang.")
            ok = False
            continue
        except Gagal as e:
            log(f"  !! {e}")
            ok = False
            continue
        masalah = []
        if menu == 1:
            masalah.append("operator di menu LCD (semua command diabaikan)")
        if fault:
            masalah.append(f"FAULT_CODE={fault} -- periksa lalu RESET_FAULT dari panel")
        if state == ST_ESTOP:
            masalah.append("E-STOP ditekan")
        if masalah:
            ok = False
            log(f"  !! {NAMA[sid]:10s} " + "; ".join(masalah))
        else:
            log(f"  OK {NAMA[sid]:10s} state={state}")
        # Titik acuan deteksi restart -- lihat laporan_node().
        try:
            UPTIME_AWAL[sid] = (bus.baca(sid, R_UPTIME[sid]), time.monotonic())
        except (Gagal, RegisterTidakAda):
            UPTIME_AWAL.pop(sid, None)

    try:
        global rak_kosong_awal
        # Hanya rak yang ada di URUTAN_RAK (config) yang dipakai.
        kosong = daftar_rak_kosong(bus)
        rak_kosong_awal = len(kosong)
        if rak_terisi:
            log(f"  -- rak terisi menurut catatan ({os.path.basename(FILE_RAK)}): "
                f"{', '.join(map(str, sorted(rak_terisi)))}. Sudah dikosongkan? jalankan --kosongkan-rak")
        if not kosong:
            log("  !! SEMUA RAK PENUH (lihat catatan di atas)")
            ok = False
        elif JUMLAH_SIKLUS > len(kosong):
            log(f"  !! jumlah_siklus = {JUMLAH_SIKLUS}, tapi rak kosong hanya {len(kosong)} "
                f"({', '.join(map(str, kosong))}) -- kosongkan rak atau kecilkan jumlah_siklus")
            ok = False
        else:
            log(f"  OK rak kosong: {', '.join(map(str, kosong))}")
    except (Gagal, RegisterTidakAda):
        pass   # sudah dilaporkan di atas

    if JEDA_HOPPER:
        try:
            bus.baca(SORTER, R_SORTER_HOPPER_DIJEDA)
            log(f"  OK jeda hopper aktif (jeda_hopper = {JEDA_HOPPER})")
        except RegisterTidakAda:
            log("  !! jeda_hopper = 1/2 butuh firmware Sorter >= 2026-10-05 -- flash Sorter, "
                "atau set jeda_hopper = 0")
            ok = False
        except Gagal:
            pass   # sudah dilaporkan di atas

    if pakai_kamera:
        if not cek_huskylens():
            ok = False
    else:
        log("  -- kamera DIMATIKAN (pakai_kamera = 0 / --tanpa-kamera): semua objek dianggap PASS, palang tidak bekerja")

    log("  -- tidak bisa diperiksa dari sini: firmware >= 2026-09-30 di SEMUA node, kalibrasi")
    log("     adegan & pose Picker (termasuk READY), READY Stocker. Lihat docstring.")
    return ok


# ============================================================
# HUSKYLENS
# ============================================================
REQUEST_BLOCKS = bytes([0x55, 0xAA, 0x11, 0x00, 0x20, 0x30])
KNOCK = bytes([0x55, 0xAA, 0x11, 0x00, 0x2C, 0x3C])   # dijawab HuskyLens dengan frame OK (0x2E)


def _tanya_huskylens(port, baud):
    """Kirim KNOCK + REQUEST_BLOCKS, kembalikan semua byte yang datang dalam ~1 detik."""
    with serial.Serial(port, baud, timeout=0.1) as hl:
        time.sleep(0.2)              # adapter USB-serial kadang belum siap tepat setelah dibuka
        hl.reset_input_buffer()
        data = b''
        for perintah in (KNOCK, REQUEST_BLOCKS):
            hl.write(perintah)
            batas = time.monotonic() + 0.5
            while time.monotonic() < batas:
                data += hl.read(64)
        return data


def cek_huskylens():
    """DIPERLUAS (2026-10-05): kalau HuskyLens diam, cari tahu SEBABNYA, bukan sekadar
    'diam'. Port tidak ada, baud berbeda, dan kabel/protocol salah butuh tindakan berbeda."""
    import glob
    if not os.path.exists(HUSKYLENS_PORT):
        ada = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        log(f"  !! {HUSKYLENS_PORT} tidak ada. Port USB-serial yang terdeteksi: {', '.join(ada) or 'tidak ada'}")
        return False
    try:
        data = _tanya_huskylens(HUSKYLENS_PORT, HUSKYLENS_BAUD)
    except Exception as e:
        log(f"  !! {HUSKYLENS_PORT} tidak bisa dibuka: {e}")
        return False
    if b'\x55\xAA\x11' in data:
        log(f"  OK HuskyLens menjawab di {HUSKYLENS_PORT} ({HUSKYLENS_BAUD} baud)")
        return True
    log(f"  !! HuskyLens diam di {HUSKYLENS_PORT} @ {HUSKYLENS_BAUD} ({len(data)} byte"
        f"{': ' + data[:16].hex(' ') if data else ''})")
    for baud in (9600, 115200, 1000000):
        if baud == HUSKYLENS_BAUD:
            continue
        try:
            if b'\x55\xAA\x11' in _tanya_huskylens(HUSKYLENS_PORT, baud):
                log(f"  !! ...tapi MENJAWAB di {baud} baud -> ubah huskylens_baud = {baud} di sorting.config")
                return False
        except Exception:
            pass
    log("     Tidak menjawab di baud mana pun. Periksa:")
    log("     - Menu HuskyLens: General Settings -> Protocol Type = UART (BUKAN Auto Detect / I2C)")
    log("     - Kabel 4-pin HuskyLens: T -> RX adapter, R -> TX adapter, GND bersama. Coba tukar T/R.")
    log("     - Port micro-USB HuskyLens BUKAN jalur data protokol (hanya daya & update firmware):")
    log("       kalau /dev/ttyUSB0 itu kabel USB HuskyLens langsung, pakai adapter USB-TTL ke kabel 4-pin.")
    return False


class HuskyLens:
    """Buffer byte DISIMPAN antar pembacaan -- frame yang terpotong di tengah (normal pada
    serial) tersambung dengan sisa byte berikutnya, tidak dibuang seperti di script lama."""

    def __init__(self):
        self.ser = serial.Serial(HUSKYLENS_PORT, HUSKYLENS_BAUD, timeout=0.05)
        self.buf = bytearray()

    def tutup(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def baca_id(self):
        """ID objek pertama dalam ~50 ms, atau None (tidak ada objek dikenali)."""
        self.ser.write(REQUEST_BLOCKS)
        batas = time.monotonic() + 0.05
        while time.monotonic() < batas:
            data = self.ser.read(64)
            if data:
                self.buf += data
            oid = self._frame()
            if oid is not None:
                return oid
        return None

    def _frame(self):
        while True:
            a = self.buf.find(b'\x55\xAA\x11')
            if a < 0:
                if len(self.buf) > 2:
                    del self.buf[:-2]
                return None
            if a:
                del self.buf[:a]
            if len(self.buf) < 4:
                return None
            total = 4 + self.buf[3] + 2
            if len(self.buf) < total:
                return None
            paket = bytes(self.buf[:total])
            del self.buf[:total]
            if paket[3] == 0x0A and paket[4] == 0x2A:    # COMMAND_RETURN_BLOCK
                return paket[13] | (paket[14] << 8)

    def ambil_sampel(self):
        sampel, mulai = [], time.monotonic()
        while len(sampel) < JUMLAH_SAMPEL and time.monotonic() - mulai < BATAS_AMBIL_S:
            if berhenti.is_set():
                break
            oid = self.baca_id()
            if oid is None:
                continue
            if oid == ID_VALID or oid in ID_INVALID:
                sampel.append(oid)
            time.sleep(INTERVAL_SAMPEL_S)
        return sampel

    def tunggu_kosong(self):
        """Kembali setelah bidang pandang kosong MIN_KOSONG_S berturut-turut."""
        kosong_sejak = time.monotonic()
        while not berhenti.is_set():
            oid = self.baca_id()
            if oid is None or oid == ID_KOSONG:
                if time.monotonic() - kosong_sejak >= MIN_KOSONG_S:
                    return
            else:
                kosong_sejak = time.monotonic()


statistik = {'valid': 0, 'invalid': 0, 'gagal_tulis': 0}


def thread_kamera(bus, indikator):
    try:
        hl = HuskyLens()
    except Exception as e:
        log(f"!! KAMERA: {HUSKYLENS_PORT} tidak bisa dibuka ({e}) -- klasifikasi MATI")
        return
    log("[kamera] mulai")
    try:
        while not berhenti.is_set():
            sampel = hl.ambil_sampel()
            if not sampel:
                continue
            valid = sampel.count(ID_VALID)
            invalid = len(sampel) - valid
            reject = invalid >= valid

            # Tulisan DULU, lampu & bunyi sesudahnya -- firmware mencatat waktu scan saat
            # tulisan tiba, jadi setiap penundaan di sini menggeser dorongan palang.
            catatan = ''
            if reject and produksi.is_set():
                try:
                    bus.tulis(SORTER, R_SORTER_CLASSIFY, 1)
                except Gagal as e:
                    statistik['gagal_tulis'] += 1
                    catatan = '  !! TIDAK TERKIRIM: ' + str(e)
            elif reject:
                catatan = '  (produksi tidak aktif -- tidak dikirim)'
            statistik['invalid' if reject else 'valid'] += 1
            try:
                # put_nowait: kalau antrian lampu penuh, isyaratnya yang dibuang -- kamera
                # tidak boleh tertahan menunggu buzzer selesai berbunyi.
                indikator.put_nowait('INVALID' if reject else 'VALID')
            except queue.Full:
                pass
            log(f"[kamera] {sampel} -> {'REJECT' if reject else 'pass'}{catatan}")

            hl.tunggu_kosong()   # satu objek = satu klasifikasi
    finally:
        hl.tutup()
        log("[kamera] berhenti")


# ============================================================
# INDIKATOR (LED & buzzer) -- thread sendiri supaya tidak menunda kamera
# ============================================================
POLA = {
    'VALID':   (LED_BLUE, [(0.08, 0.12), (0.08, 0.12)]),
    'INVALID': (LED_RED,  [(0.30, 0.12), (0.08, 0.12), (0.08, 0.0)]),
}


def thread_indikator(antrian):
    try:
        import wiringpi
        wiringpi.wiringPiSetup()
        # Pin None (config) = tidak dipasang -- dilewati, bukan mematikan seluruh indikator.
        pins = [p for p in (LED_RED, LED_BLUE, BUZZER) if p is not None]
        for pin in pins:
            wiringpi.pinMode(pin, 1)
            wiringpi.digitalWrite(pin, 0)
    except Exception as e:
        log(f"-- indikator dimatikan ({e})")
        while not berhenti.is_set():   # tetap kosongkan antrian
            try:
                antrian.get(timeout=0.5)
            except queue.Empty:
                pass
        return
    try:
        while not berhenti.is_set():
            try:
                jenis = antrian.get(timeout=0.5)
            except queue.Empty:
                continue
            _, bunyi = POLA[jenis]
            # Pin dibaca SAAT INI (bukan dari POLA saat modul dimuat): panel LCD bisa memuat
            # ulang config tanpa menjalankan ulang program.
            led = LED_BLUE if jenis == 'VALID' else LED_RED
            if led is not None:
                wiringpi.digitalWrite(led, 1)
            for on_s, off_s in bunyi:
                if BUZZER is not None:
                    wiringpi.digitalWrite(BUZZER, 1)
                time.sleep(on_s)
                if BUZZER is not None:
                    wiringpi.digitalWrite(BUZZER, 0)
                time.sleep(off_s)
            time.sleep(0.2)
            if led is not None:
                wiringpi.digitalWrite(led, 0)
    finally:
        for pin in pins:
            wiringpi.digitalWrite(pin, 0)


def thread_status_led():
    """LED OPR: nyala tetap = produksi berjalan, kedip = startup / menunggu. LED ALARM: nyala =
    produksi DITAHAN atau gagal. Pin None di sorting.config = LED itu tidak dipakai."""
    pins = [p for p in (LED_OPR, LED_ALARM) if p is not None]
    if not pins:
        return
    try:
        import wiringpi
        wiringpi.wiringPiSetup()
        for p in pins:
            wiringpi.pinMode(p, 1)
            wiringpi.digitalWrite(p, 0)
    except Exception as e:
        log(f"-- LED OPR/ALARM dimatikan ({e})")
        return
    kedip = False
    while not berhenti.wait(0.5):
        kedip = not kedip
        if LED_OPR is not None:
            nyala = produksi.is_set() or (kedip and not alarm.is_set())
            wiringpi.digitalWrite(LED_OPR, 1 if nyala else 0)
        if LED_ALARM is not None:
            wiringpi.digitalWrite(LED_ALARM, 1 if alarm.is_set() else 0)
    # Program berhenti: OPR padam. ALARM dibiarkan menyala kalau berhenti karena gagal,
    # supaya operator yang datang belakangan masih melihat ada masalah.
    if LED_OPR is not None:
        wiringpi.digitalWrite(LED_OPR, 0)
    if LED_ALARM is not None:
        wiringpi.digitalWrite(LED_ALARM, 1 if alarm.is_set() else 0)


# ============================================================
# KEEPALIVE
# ============================================================
def thread_keepalive(bus):
    """Membaca STATE tiap node berkala. Firmware >= 2026-09-30 menghitung request APA PUN
    sebagai tanda master hidup; tanpa ini, node yang sedang menunggu gerakan panjang tanpa
    dibaca master bisa jatuh FAULT COMM_TIMEOUT."""
    while not berhenti.wait(KEEPALIVE_S):
        for sid in NAMA:
            try:
                bus.baca(sid, R_STATE, percobaan=1)
            except Exception:
                pass   # kegagalan node dilaporkan oleh alur utama, bukan di sini


# ============================================================
# STARTUP / BATCH / SHUTDOWN
# ============================================================
def startup(bus):
    log("== STARTUP ==")

    log("[0] semua node ke TEST mode (keadaan awal yang pasti)")
    for sid in NAMA:
        kirim(bus, sid, OP_STOP_MAIN[sid], "STOP_MAIN")
        pastikan_main(bus, sid, False)

    log("[1] Picker -> READY lewat HOME (masih TEST mode; GOTO_READY ditolak setelah MAIN)")
    kirim(bus, PICKER, OP_PICKER_GOTO_READY, "GOTO_READY")
    # GOTO_READY di-ack SEKETIKA, gerakannya baru mulai -- tunggu sampai benar-benar diam di READY.
    # Jeda sebentar dulu: tepat setelah ack, antrian gerak belum tentu diambil, sehingga
    # ACTIVITY masih DIAM dan CURRENT_POSE masih nilai lama -- terlihat "sudah sampai".
    time.sleep(0.5)
    tunggu(bus, PICKER,
           lambda: bus.baca(PICKER, R_PICKER_ACTIVITY) == 0 and bus.baca(PICKER, R_PICKER_POSE) == POSE_READY,
           T_PICKER_HOME, "Picker diam di READY")

    log("[2] Dispenser -> START_MAIN, tunggu DISPENSER_READY")
    kirim(bus, DISPENSER, OP_START_MAIN[DISPENSER], "START_MAIN")
    pastikan_main(bus, DISPENSER, True)
    # START_MAIN sendiri yang menjalankan urutan awal (jatuhkan package pertama sampai
    # PROX_2). JANGAN kirim FORCE_MIDDLE_REFILL di atasnya -- dua mekanisme, satu aktuator.
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_READY) == 1,
           T_DISPENSER_SIAP_AWAL, "DISPENSER_READY")

    log("[3] Picker -> START_MAIN")
    kirim(bus, PICKER, OP_START_MAIN[PICKER], "START_MAIN")
    pastikan_main(bus, PICKER, True)

    log("[4] Stocker -> homing (kalau perlu), READY, START_MAIN")
    if bus.baca(STOCKER, R_STOCKER_HOMED) != 1:
        kirim(bus, STOCKER, OP_STOCKER_HOME_ALL, "HOME_ALL", timeout=T_STOCKER_HOMING)
        if bus.baca(STOCKER, R_STOCKER_HOMED) != 1:
            raise Gagal("STOCKER: HOME_ALL di-ack tapi ALL_HOMED_FLAG masih 0")
    kirim(bus, STOCKER, OP_STOCKER_GOTO_READY, "GOTO_READY", timeout=T_STOCKER_GERAK)
    kirim(bus, STOCKER, OP_START_MAIN[STOCKER], "START_MAIN")
    pastikan_main(bus, STOCKER, True)

    log("[5] Sorter -> RESET_COUNTERS, START")
    kirim(bus, SORTER, OP_SORTER_RESET_COUNTERS, "RESET_COUNTERS")
    kirim(bus, SORTER, OP_START_MAIN[SORTER], "START")
    pastikan_main(bus, SORTER, True)
    log("== STARTUP SELESAI -- produksi berjalan ==")


# ============================================================
# CATATAN RAK -- Stocker belum punya sensor rak (2026-10-01), jadi rak mana yang sudah terisi
# DICATAT program ini di file rak_terisi.txt (satu folder dengan program). Catatan itu
# bertahan saat program dijalankan ulang -- tanpa itu, setelah restart program memilih Rak 1
# lagi dan Stocker mendorong package ke rak yang sudah berisi.
#
# Setelah rak dikosongkan secara fisik, hapus catatannya:  --kosongkan-rak  (semua) atau
# --kosongkan-rak 2,3  (rak tertentu saja).
# ============================================================
FILE_RAK = os.path.join(FOLDER, 'rak_terisi.txt')


def muat_rak_terisi():
    try:
        with open(FILE_RAK, encoding='utf-8') as f:
            return {int(x) for x in f.read().replace(',', ' ').split() if x.strip() in ('1', '2', '3', '4')}
    except FileNotFoundError:
        return set()


def simpan_rak_terisi():
    # Tulis ke file sementara lalu ganti nama: listrik padam di tengah penulisan tidak
    # meninggalkan file setengah jadi yang membuat catatan rak hilang.
    sementara = FILE_RAK + '.tmp'
    with open(sementara, 'w', encoding='utf-8') as f:
        f.write(', '.join(map(str, sorted(rak_terisi))) + '\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(sementara, FILE_RAK)


rak_terisi = muat_rak_terisi()

# BARU (2026-10-05): umpan hopper Sorter dijeda selama package diganti (JEDA_HOPPER di config).
# Conveyor & palang Sorter tetap jalan -- objek yang sudah di belt tetap diklasifikasi.
hopper_dijeda = False


def jeda_hopper(bus, jeda):
    global hopper_dijeda
    kirim(bus, SORTER, OP_SORTER_HOPPER_JEDA, "SET_HOPPER_JEDA", arg=1 if jeda else 0)
    if bus.baca(SORTER, R_SORTER_HOPPER_DIJEDA) != (1 if jeda else 0):
        raise Gagal("SORTER: SET_HOPPER_JEDA di-ack tapi HOPPER_DIJEDA tidak berubah")
    hopper_dijeda = jeda
    log(f"    hopper Sorter {'DIJEDA' if jeda else 'dilanjutkan'}")


def daftar_rak_kosong(bus):
    """Rak di URUTAN_RAK yang belum tercatat terisi. Sensor rak Stocker hanya ikut dipakai
    kalau PAKAI_SENSOR_RAK = 1 -- tanpa sensor terpasang, input yang mengambang bisa
    terbaca 'terisi' secara acak."""
    bitmask = bus.baca(STOCKER, R_STOCKER_RAK) if PAKAI_SENSOR_RAK else 0
    return [r for r in URUTAN_RAK                  # bit N = Rak N (firmware >= 2026-09-20)
            if r not in rak_terisi and not bitmask & (1 << r)]


def cari_rak_kosong(bus):
    """Urutan: URUTAN_RAK di config, atau acak di antara yang kosong (RAK_RANDOM = 1)."""
    kosong = daftar_rak_kosong(bus)
    if not kosong:
        return None
    return random.choice(kosong) if RAK_RANDOM else kosong[0]

def batch(bus, nomor):
    log(f"== BATCH {nomor} ==")
    for sid in NAMA:
        state, fault = bus.baca(sid, R_STATE, 2)
        if fault or state in (ST_FAULT, ST_ESTOP):
            raise Gagal(f"{NAMA[sid]} FAULT (code={fault}) sebelum batch dimulai")

    rak = cari_rak_kosong(bus)
    if rak is None:
        raise Gagal("SEMUA RAK PENUH -- kosongkan rak dulu")
    STATUS['rak'] = rak

    log("[1] tunggu Dispenser SIAP ISI")
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_READY) == 1,
           T_DISPENSER_SIAP, "DISPENSER_READY")

    if JEDA_HOPPER == 1:
        jeda_hopper(bus, True)
    log("[2] Dispenser -> REQUEST_REFILL, tunggu package sampai UJUNG")
    kirim(bus, DISPENSER, OP_DISP_REQUEST_REFILL, "REQUEST_REFILL")
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_PACKAGE_READY) == 1,
           T_PACKAGE_UJUNG, "PACKAGE_READY_FLAG")

    log("[3] Stocker -> READY")
    kirim(bus, STOCKER, OP_STOCKER_GOTO_READY, "GOTO_READY", timeout=T_STOCKER_GERAK)

    # DIUBAH (2026-10-01): langkah [4]-[6] sekarang BERJALAN BERSAMAAN, masing-masing dipicu
    # kejadian nyata -- bukan menunggu siklus lengan selesai seluruhnya:
    #   [5] Dispenser ACK_PACKAGE_TAKEN  begitu package terangkat dari UJUNG (PROX_1 kosong).
    #       Dispenser langsung membawa package kosong berikutnya ke TENGAH.
    #   [6] Stocker RUN_FULL_CYCLE       begitu Picker selesai menaruh package dan kembali ke
    #       HOME (sendok sudah keluar dari bawah package), selagi lengan menuju READY.
    # Batch selesai setelah Picker kembali di READY DAN Stocker selesai menyimpan.
    if DELAY_PICKER_S > 0:
        log(f"    jeda Picker {DELAY_PICKER_S:.1f} s")
        time.sleep(DELAY_PICKER_S)
    log("[4] Picker -> MOVE_PACKAGE: Ready-Pick-Home-Place-Home-Ready")
    t0 = time.monotonic()
    seq_picker = bus.command(PICKER, OP_PICKER_MOVE_PACKAGE)
    durasi_picker = None
    sudah_ack = False
    sudah_di_lift = False            # Picker pernah mencapai pose LIFT (package diletakkan)
    sudah_di_home = False            # ... lalu kembali ke HOME (sendok sudah keluar dari package)
    pose, gerakan = None, None
    seq_stocker, t_stocker, durasi_stocker = None, None, None
    while durasi_picker is None or durasi_stocker is None:
        if berhenti.is_set():
            raise Gagal("dihentikan operator")
        sekarang = time.monotonic()
        if durasi_picker is None and sekarang - t0 > T_MOVE_PACKAGE:
            raise Gagal(f"PICKER: MOVE_PACKAGE tidak selesai dalam {T_MOVE_PACKAGE:.0f} s")
        if t_stocker is not None and durasi_stocker is None and sekarang - t_stocker > T_FULL_CYCLE:
            raise Gagal(f"STOCKER: RUN_FULL_CYCLE tidak selesai dalam {T_FULL_CYCLE:.0f} s")
        for sid in (PICKER, DISPENSER, STOCKER):
            state, fault = bus.baca(sid, R_STATE, 2)
            if fault or state in (ST_FAULT, ST_ESTOP):
                raise Gagal(f"{NAMA[sid]} FAULT (code={fault}, state={state}) selama batch")

        # [5] Dispenser -- firmware hanya menerima ACK di tahap 'tunggu diambil arm'. Kalau
        # lengan mengangkat package selagi servo refill masih bergerak, ACK yang terlalu cepat
        # DIBUANG firmware dan Dispenser menunggu selamanya -- jadi tahapnya ditunggu dulu.
        if (not sudah_ack and bus.baca(DISPENSER, R_DISP_UJUNG) == 0
                and bus.baca(DISPENSER, R_DISP_STAGE) == STAGE_TUNGGU_DIAMBIL):
            log("[5] package terangkat dari UJUNG (PROX_1 kosong) -> Dispenser ACK_PACKAGE_TAKEN")
            if DELAY_DISPENSER_S > 0:
                time.sleep(DELAY_DISPENSER_S)
            kirim(bus, DISPENSER, OP_DISP_ACK_TAKEN, "ACK_PACKAGE_TAKEN")
            sudah_ack = True

        if durasi_picker is None:
            # Pose (reg 10) dan gerakan aktif (reg 17) dibaca satu blok. Firmware Picker lama
            # belum punya reg 17 -- cukup pose saja.
            try:
                blok = bus.baca(PICKER, R_PICKER_POSE, 9)
                pose_baru, gerakan_baru = blok[0], blok[7]
            except RegisterTidakAda:
                pose_baru, gerakan_baru = bus.baca(PICKER, R_PICKER_POSE), None
            if pose_baru != pose or gerakan_baru != gerakan:
                log(f"    lengan: pose {NAMA_POSE.get(pose_baru, pose_baru)}, "
                    f"gerakan {NAMA_GERAKAN.get(gerakan_baru, gerakan_baru)}")
            pose, gerakan = pose_baru, gerakan_baru
            if pose == POSE_LIFT:
                sudah_di_lift = True
            # DIPERBAIKI (2026-10-07): 'kembali ke HOME setelah menaruh' dulu hanya dikenali dari
            # CURRENT_POSE = HOME, yang bisa terlewat polling. Sekarang juga dari gerakan
            # Home>Ready yang sudah dimulai, atau pose READY -- keduanya PASTI sesudah HOME.
            if sudah_di_lift and not sudah_di_home and (
                    pose in (POSE_HOME, POSE_READY) or gerakan == GERAKAN_HOME_READY):
                sudah_di_home = True
                log("[6] lengan kembali di HOME setelah menaruh")
            if bus.baca(PICKER, R_ACK) == seq_picker:
                durasi_picker = time.monotonic() - t0
                if durasi_picker < MIN_DURASI_MOVE_PACKAGE_S:
                    raise Gagal(f"PICKER: MOVE_PACKAGE di-ack dalam {durasi_picker:.1f} s "
                                f"-- mustahil selesai, berarti DITOLAK")
                log(f"    Picker kembali di READY, siklus lengan {durasi_picker:.1f} s")

        # [6] Stocker -- CURRENT_POSE berurutan PICKUP, HOME, LIFT, HOME, READY. HOME sesudah
        # LIFT = Place>Home selesai: package sudah di lift dan sendok sudah keluar. Kalau
        # tahap itu terlewat oleh polling, ack Picker (lengan sudah di READY) dipakai.
        # Hopper dilanjutkan: LANJUT_HOPPER = 1 begitu lengan kembali di HOME setelah menaruh,
        # 0 begitu package KOSONG berikutnya siap di TENGAH (DISPENSER_READY kembali 1).
        if hopper_dijeda:
            if LANJUT_HOPPER and (sudah_di_home or durasi_picker is not None):
                jeda_hopper(bus, False)
            elif not LANJUT_HOPPER and sudah_ack and bus.baca(DISPENSER, R_DISP_READY) == 1:
                jeda_hopper(bus, False)

        stocker_boleh = sudah_di_home if STOCKER_PARALEL else False
        if seq_stocker is None and (stocker_boleh or durasi_picker is not None):
            if not sudah_ack:
                # Package harus sudah terangkat dari UJUNG sebelum Stocker membawanya pergi.
                if durasi_picker is not None and bus.baca(DISPENSER, R_DISP_UJUNG) == 1:
                    raise Gagal("siklus lengan selesai tapi package MASIH di UJUNG -- tidak terambil")
            else:
                log(f"[6] Picker selesai menaruh -> Stocker simpan ke Rak {rak}")
                if DELAY_STOCKER_S > 0:
                    time.sleep(DELAY_STOCKER_S)
                t_stocker = time.monotonic()
                seq_stocker = bus.command(STOCKER, OP_STOCKER_FULL_CYCLE, rak)
        if seq_stocker is not None and durasi_stocker is None and bus.baca(STOCKER, R_ACK) == seq_stocker:
            durasi_stocker = time.monotonic() - t_stocker
            if durasi_stocker < MIN_DURASI_FULL_CYCLE_S:
                raise Gagal(f"STOCKER: RUN_FULL_CYCLE di-ack dalam {durasi_stocker:.1f} s -- berarti DITOLAK")
            log(f"    Stocker selesai, {durasi_stocker:.1f} s")
        time.sleep(0.3)

    # Firmware membersihkan flag pada putaran pipeline BERIKUTNYA, bukan saat command
    # diterima -- dibaca seketika bisa masih 1. Karena itu ditunggu, bukan dibaca sekali.
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_PACKAGE_READY) == 0,
           5.0, "PACKAGE_READY_FLAG kembali 0")
    if hopper_dijeda:
        log("    tunggu package kosong siap di TENGAH sebelum hopper dilanjutkan")
        tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_READY) == 1,
               T_DISPENSER_SIAP, "DISPENSER_READY")
        jeda_hopper(bus, False)
    rak_terisi.add(rak)
    simpan_rak_terisi()
    log(f"== BATCH {nomor} SELESAI -> Rak {rak} (rak terisi: {sorted(rak_terisi)}) ==")


def shutdown(bus):
    """Best-effort, tanpa menunggu ack: node yang FAULT tidak boleh membuat shutdown macet.
    Sorter dulu, supaya objek berhenti mengalir sebelum yang lain dihentikan."""
    log("== SHUTDOWN -- semua node ke TEST mode ==")
    for sid in (SORTER, DISPENSER, PICKER, STOCKER):
        try:
            bus.command(sid, OP_STOP_MAIN[sid])
        except Exception as e:
            log(f"   {NAMA[sid]}: {e}")
    log("== SHUTDOWN SELESAI ==")


def pantau(bus):
    """Loop utama: tunggu PASS_COUNT naik BATCH_SIZE dari titik acuan, lalu jalankan batch.
    PASS_COUNT TIDAK di-reset per batch -- objek yang lewat selama batch tetap terhitung."""
    acuan = bus.baca(SORTER, R_SORTER_BLOK)
    missed_awal = bus.baca(SORTER, R_SORTER_BLOK, 10)[9]
    nomor, cetak_berikut = 0, time.monotonic() + 60
    pass_terakhir = acuan
    # BARU (2026-10-01): produksi berhenti RAPI setelah sekian siklus (batch). 0 di config =
    # sampai semua rak yang kosong saat mulai terisi.
    target = JUMLAH_SIKLUS if JUMLAH_SIKLUS > 0 else rak_kosong_awal
    STATUS.update(target=target, batch=0, pass_batch=0)
    log(f"[pass] mulai dari PASS_COUNT={acuan}, batch tiap {BATCH_SIZE} objek, target {target} siklus")
    while not berhenti.is_set():
        state, fault = bus.baca(SORTER, R_STATE, 2)
        if fault or state != ST_RUN:
            sebab = FAULT_SORTER.get(fault, '')
            raise Gagal(f"SORTER berhenti (state={state}, fault={fault}{' ' + sebab if sebab else ''}, "
                        f"pass terakhir={pass_terakhir})")
        blok = bus.baca(SORTER, R_SORTER_BLOK, 10)
        passc, rejectc, missed = blok[0], blok[1], blok[9]
        STATUS['pass_batch'] = (passc - acuan) % 65536
        if passc != pass_terakhir:   # setiap objek lewat sensor PASS langsung terlihat
            pass_terakhir = passc
            log(f"[pass] PASS_COUNT={passc} ({(passc - acuan) % 65536}/{BATCH_SIZE} menuju batch {nomor + 1})")
        if missed != missed_awal:
            log(f"!! REJECT_MISSED_COUNT naik ke {missed} -- ada reject yang lolos tanpa didorong")
            missed_awal = missed
        if (passc - acuan) % 65536 >= BATCH_SIZE:
            nomor += 1
            if JEDA_HOPPER == 2:
                jeda_hopper(bus, True)
            if DELAY_BATCH_S > 0:
                # Sensor PASS ada di depan ujung conveyor Sorter: objek terakhir yang terhitung
                # masih berjalan beberapa saat sebelum jatuh ke package. Tanpa jeda ini package
                # sudah dibawa pergi padahal objek terakhir belum masuk.
                log(f"[batch] {BATCH_SIZE} objek terhitung -- jeda {DELAY_BATCH_S:.1f} s sampai objek terakhir jatuh")
                if berhenti.wait(DELAY_BATCH_S):
                    break
            STATUS['batch'] = nomor
            batch(bus, nomor)
            acuan = (acuan + BATCH_SIZE) % 65536
            log(f"[siklus] {nomor}/{target} selesai")
            if nomor >= target:
                log(f"== TARGET {target} SIKLUS TERCAPAI -- produksi dihentikan rapi ==")
                return
        if time.monotonic() >= cetak_berikut:
            cetak_berikut = time.monotonic() + 60
            log(f"[status] pass={passc} reject={rejectc} missed={missed} batch={nomor} "
                f"kamera valid={statistik['valid']} invalid={statistik['invalid']} "
                f"gagal_tulis={statistik['gagal_tulis']}")
        time.sleep(POLL_S)


UPTIME_AWAL = {}   # sid -> (uptime node, waktu monotonic) saat pemeriksaan awal


def _restart(sid, up, umur):
    """DIPERBAIKI (2026-10-07): restart = uptime node MUNDUR dibanding yang tercatat saat
    pemeriksaan awal. Dulu dibandingkan dengan lama program berjalan -- salah kalau Orange Pi
    dan ESP32 dinyalakan bersamaan (uptime node memang pendek, padahal tidak restart).
    Register uptime 16 bit (kembali ke 0 tiap 18 jam), jadi dihitung modulo 65536."""
    if sid not in UPTIME_AWAL:
        return up + 5 < umur            # tidak ada acuan -- pakai cara lama
    up0, t0 = UPTIME_AWAL[sid]
    harapan = (up0 + int(time.monotonic() - t0)) % 65536
    selisih = (harapan - up) % 65536
    return 3 < selisih < 65536 - 3      # toleransi 3 s: uptime dibaca dalam detik bulat

def laporan_node(bus):
    """Dipanggil saat produksi ditahan: node mana yang menjawab, dan apakah ada yang RESTART
    selama program berjalan (uptime-nya mundur dibanding saat pemeriksaan awal) -- node itu
    menyala ulang di tengah jalan -- penyebab paling umum: tegangan turun saat motor/servo
    menarik arus besar. 'Tidak menjawab' biasa tidak bisa membedakan itu dari kabel putus."""
    umur = time.monotonic() - T_MULAI
    log(f"     Status node (program sudah berjalan {umur:.0f} s):")
    for sid in NAMA:
        up = None
        for _ in range(5):            # node yang restart butuh beberapa detik untuk boot
            try:
                up = bus.baca(sid, R_UPTIME[sid])
                break
            except (Gagal, RegisterTidakAda):
                time.sleep(1)
        if up is None:
            log(f"       {NAMA[sid]:10s} TIDAK MENJAWAB -- cek daya, kabel RS485, layar LCD-nya")
        elif _restart(sid, up, umur):
            log(f"       {NAMA[sid]:10s} uptime {up} s -- RESTART selama produksi! "
                f"(kemungkinan tegangan turun saat motor/servo jalan)")
        else:
            log(f"       {NAMA[sid]:10s} uptime {up} s -- tidak restart")


def tahan(bus, alasan):
    """Batch gagal: hentikan umpan Sorter (package tidak kebanjiran), lalu TUNGGU operator.
    Tidak ada pengulangan otomatis -- bisa tabrakan mekanis."""
    produksi.clear()
    alarm.set()
    STATUS.update(fase='DITAHAN', alasan=alasan)
    log(f"!!!! PRODUKSI DITAHAN: {alasan}")
    try:
        bus.command(SORTER, OP_STOP_MAIN[SORTER])
        log("     Sorter dihentikan. Node lain dibiarkan apa adanya -- periksa panelnya.")
    except Exception as e:
        log(f"     gagal menghentikan Sorter: {e}")
    laporan_node(bus)
    log("     Ctrl+C (atau STOP di panel) untuk shutdown penuh setelah diperiksa.")
    berhenti.wait()


# ============================================================
def jalankan_produksi(bus, pakai_kamera, hanya_cek=False):
    """SATU kali produksi, dari pemeriksaan awal sampai shutdown. Dipakai CLI main() DAN
    panel LCD (jalankan_panel), yang menjalankannya di thread sendiri dan menghentikannya
    lewat berhenti.set(). Mengembalikan True kalau pemeriksaan awal lolos.

    Event berhenti/produksi/alarm di-reset di sini, supaya produksi bisa dimulai ulang dari
    panel tanpa menjalankan ulang program."""
    global T_MULAI
    berhenti.clear(); produksi.clear(); alarm.clear()
    T_MULAI = time.monotonic()
    STATUS.update(fase='PERIKSA', batch=0, target=0, pass_batch=0, rak=None, mulai=time.time())
    log(f"BATCH_SIZE = {BATCH_SIZE}, JUMLAH_SIKLUS = {JUMLAH_SIKLUS or 'sampai rak kosong habis'}, "
        f"kamera = {'ya' if pakai_kamera else 'TIDAK'}")
    # LED OPR/ALARM dinyalakan PALING AWAL, supaya pemeriksaan awal yang gagal juga terlihat.
    led = threading.Thread(target=thread_status_led, daemon=True)
    led.start()

    if not periksa(bus, pakai_kamera):
        log("!! Pemeriksaan awal GAGAL -- tidak ada yang digerakkan.")
        STATUS.update(fase='GAGAL CEK', alasan='pemeriksaan awal gagal -- lihat LOG')
        alarm.set()
        berhenti.set()
        led.join(timeout=2)
        return False
    if hanya_cek:
        log("Pemeriksaan awal OK. (cek saja: berhenti di sini)")
        STATUS['fase'] = 'CEK OK'
        berhenti.set()
        led.join(timeout=2)
        return True

    antrian = queue.Queue(maxsize=20)
    threads = [threading.Thread(target=thread_keepalive, args=(bus,), daemon=True),
               threading.Thread(target=thread_indikator, args=(antrian,), daemon=True)]
    if pakai_kamera:
        threads.append(threading.Thread(target=thread_kamera, args=(bus, antrian), daemon=True))
    for t in threads:
        t.start()

    sudah_jalan = False
    try:
        STATUS['fase'] = 'STARTUP'
        startup(bus)
        sudah_jalan = True
        STATUS['fase'] = 'JALAN'
        produksi.set()
        pantau(bus)
        if not berhenti.is_set():
            STATUS['fase'] = 'SELESAI'
    except Exception as e:
        # Bukan hanya Gagal: RegisterTidakAda atau error tak terduga lain di thread panel
        # tidak boleh lewat diam-diam sambil mesin dibiarkan berjalan.
        alasan = str(e) if isinstance(e, Gagal) else f"{type(e).__name__}: {e}"
        if berhenti.is_set():
            log(f"dihentikan: {alasan}")
        elif sudah_jalan:
            tahan(bus, alasan)
        else:
            alarm.set()
            STATUS.update(fase='GAGAL STARTUP', alasan=alasan)
            log(f"!! STARTUP GAGAL: {alasan}")
    finally:
        produksi.clear()
        berhenti.set()
        for t in threads + [led]:
            t.join(timeout=2)
        shutdown(bus)
        if STATUS['fase'] in ('JALAN', 'STARTUP', 'DITAHAN'):
            STATUS['fase'] = 'BERHENTI'
    return True


# ############################################################################
# 3. TABEL NODE -- ACUAN command & register (dulu sorting_automation.py)
# ############################################################################
# ============================================================
# REGISTER UNIVERSAL (ALAMAT SAMA DI SEMUA 4 NODE)
# ============================================================
REG_STATE       = 0
REG_FAULT_CODE  = 1
REG_CMD         = 2
REG_CMD_ARG     = 3
REG_CMD_SEQ     = 4
REG_CMD_ACK_SEQ = 5
REG_HEARTBEAT   = 6

STATE_NAMES = {0: 'INIT', 1: 'IDLE', 2: 'RUNNING/MOVING', 3: 'FAULT', 4: 'ESTOPPED'}

# Urutan node = urutan slave ID.
NODES = {
    '1': {'name': 'SORTER',    'slave_id': 1},
    '2': {'name': 'PICKER',    'slave_id': 2},
    '3': {'name': 'DISPENSER', 'slave_id': 3},
    '4': {'name': 'STOCKER',   'slave_id': 4},
}

# ============================================================
# COMMAND per node
#
# Satu entri: (label, opcode, arg, gate, kelompok)
#   arg      None       = tanpa argumen
#            int        = argumen TETAP, sama dengan yang dikirim panel
#            str        = argumen ditanyakan; teksnya jadi petunjuk
#   gate     'netral'   = tidak dibatasi mode
#            'main'     = hanya diterima setelah START_MAIN
#            'test'     = ditolak selama MAIN
#            'lokal'    = BUKAN command Modbus -- hanya ada di panel. Tetap
#                         didaftar supaya nomornya sejajar dengan panel.
#   kelompok 'mode' 'produksi' 'aksi' 'uji'            -> juga ada di panel
#            'uji+' 'setting'                           -> hanya lewat Modbus
#
# Urutan list = urutan tampil = nomor. JANGAN disisipkan di tengah tanpa
# menyamakan panel (CMD_TEST_ITEMS di src/main.cpp node yang sama).
#
# Opcode yang SENGAJA DIKOSONGKAN dan tidak boleh dipakai ulang:
#   SORTER  99 (bekas TEST_FAULT)
#   PICKER  1 (bekas RUN_SEQUENCE), 3 (bekas GOTO_PASS), 4 (bekas GOTO_REJECT)
# ============================================================
OPCODES = {
    'SORTER': [
        ('START_MAIN',        1,  None, 'netral', 'mode'),
        ('STOP_MAIN',         2,  None, 'netral', 'mode'),
        ('RESET_FAULT',       3,  None, 'netral', 'mode'),
        ('RESET_COUNTERS',    7,  None, 'netral', 'aksi'),
        ('HOPPER_CYCLE',      97, None, 'test',   'uji'),
        ('TRIGGER_PALANG',    98, None, 'test',   'uji'),
        ('SET_MOTOR_A',       8,  '0=stop, 1=maju, 2=mundur -- MENTAH, tanpa batas waktu', 'test', 'uji+'),
        ('SET_HOPPER_JEDA',   11, '1=jeda umpan hopper, 0=lanjut (fw >= 2026-10-05)', 'netral', 'uji+'),
        ('SET_CONVEYOR_SPEED', 5, 'PWM 0-255',               'netral', 'setting'),
        ('SET_CONVEYOR_DIR',  6,  '0=reverse, 1=forward',    'netral', 'setting'),
        ('SET_PALANG_SPEED',  9,  'PWM 0-255',               'netral', 'setting'),
        ('SET_HOPPER_INTERVAL', 4, 'step interval ms 0-500', 'netral', 'setting'),
        ('SET_HOPPER_STEP',   10, 'hopperStepUs 1-2500',     'netral', 'setting'),
    ],
    'PICKER': [
        ('START_MAIN',        11, None, 'netral', 'mode'),
        ('STOP_MAIN',         12, None, 'netral', 'mode'),
        ('RESET_FAULT',       7,  None, 'netral', 'mode'),
        ('MOVE_PACKAGE',      8,  None, 'main',   'produksi'),
        ('GOTO_HOME',         2,  None, 'test',   'uji'),
        ('GOTO_READY',        15, None, 'test',   'uji'),
        ('PICK',              5,  None, 'test',   'uji'),
        ('PLACE',             6,  None, 'test',   'uji'),
        ('GOTO_POSE_N',       13, '0=HOME, 1=PICKUP, 2=LIFT, 3=READY', 'test', 'uji+'),
        ('RUN_GERAKAN',       14, '0=Home>Ready 1=Ready>Pick 2=Pick>Home 3=Home>Place 4=Place>Home', 'test', 'uji+'),
        ('SET_TRAJ_STEP',     9,  'trajStepUs 1-500',          'netral', 'setting'),
        ('SET_TRAJ_STEP_INTERVAL', 10, 'trajStepIntervalMs 5-200', 'netral', 'setting'),
    ],
    'DISPENSER': [
        ('START_MAIN',        14, None, 'netral', 'mode'),
        ('STOP_MAIN',         15, None, 'netral', 'mode'),
        ('RESET_FAULT',       2,  None, 'netral', 'mode'),
        ('REQUEST_REFILL',    1,  None, 'main',   'produksi'),
        ('ACK_TAKEN',         5,  None, 'main',   'produksi'),
        ('FORCE_REFILL',      6,  None, 'main',   'produksi'),
        ('SERVO1_CYCLE',      97, None, 'test',   'uji'),
        ('SERVO2_CYCLE',      98, None, 'test',   'uji'),
        ('REFILL_LOOP',       None, None, 'lokal', 'uji'),
        ('CONVEYOR_ON_OFF',   11, '0=mati, 1=nyala',             'test', 'uji+'),
        ('MOVE_SERVO1_TO',    12, '0=titik awal, 1=titik akhir', 'test', 'uji+'),
        ('MOVE_SERVO2_TO',    13, '0=titik awal, 1=titik akhir', 'test', 'uji+'),
        ('SET_CONVEYOR_SPEED', 3, 'PWM 0-255',                   'netral', 'setting'),
        ('SET_CONVEYOR_DIR',  4,  '0=reverse, 1=forward',        'netral', 'setting'),
        ('SET_SERVO1_STEP',   7,  'servo1StepUs 1-2500',         'netral', 'setting'),
        ('SET_SERVO1_STEP_INTERVAL', 8, 'servo1StepIntervalMs 0-500', 'netral', 'setting'),
        ('SET_SERVO2_STEP',   9,  'servo2StepUs 1-2500',         'netral', 'setting'),
        ('SET_SERVO2_STEP_INTERVAL', 10, 'servo2StepIntervalMs 0-500', 'netral', 'setting'),
    ],
    'STOCKER': [
        ('START_MAIN',        9,  None, 'netral', 'mode'),
        ('STOP_MAIN',         10, None, 'netral', 'mode'),
        ('RESET_FAULT',       5,  None, 'netral', 'mode'),
        ('FULL_CYCLE',        2,  'rak 1-4 (panel: rak 1)', 'main', 'produksi'),
        ('HOME_ALL',          1,  None, 'netral', 'aksi'),
        ('GOTO_READY',        6,  None, 'netral', 'aksi'),   # READY = Load Position
        ('MOVE_TO_RACK',      3,  'rak 1-4 (panel: rak 1)', 'test', 'uji'),
        ('PUSH_BOX',          4,  None, 'test',   'uji'),
        ('SET_STEP_INTERVAL', 7,  'stepIntervalUs 0-5000',        'netral', 'setting'),
        ('SET_HOMING_STEP_INTERVAL', 8, 'homingStepIntervalUs 20-5000', 'netral', 'setting'),
    ],
}

JUDUL_KELOMPOK = {
    'mode':     'Mode & Reset',
    'produksi': 'Produksi (hanya saat MAIN)',
    'aksi':     'Aksi (bebas mode)',
    'uji':      'Uji (ditolak saat MAIN)',
    'uji+':     'Uji tambahan -- HANYA lewat Modbus, tidak ada di panel',
    'setting':  'Setting -- HANYA lewat Modbus, tidak menggerakkan apa pun',
}
KELOMPOK_PANEL = ('mode', 'produksi', 'aksi', 'uji')

# ============================================================
# REGISTER STATUS
#
# Ditampilkan dengan urutan yang SAMA di semua node:
#   1. universal (alamat sama)   STATE, FAULT_CODE, CMD_ACK_SEQ, HEARTBEAT
#   2. diagnostik bersama        nama sama di semua node, alamatnya berbeda
#   3. khusus node
# ============================================================
DIAGNOSTIK_BERSAMA = ['MAIN_MODE_ACTIVE', 'MENU_ACTIVE', 'ACTIVITY_CODE',
                      'LAST_FAULT_CODE', 'I2C_ERROR_COUNT', 'UPTIME_SEC']

ALAMAT = {
    'SORTER': {
        'MAIN_MODE_ACTIVE': 17, 'MENU_ACTIVE': 18, 'ACTIVITY_CODE': 13,
        'LAST_FAULT_CODE': 15, 'I2C_ERROR_COUNT': 14, 'UPTIME_SEC': 16,
    },
    'PICKER': {
        'MAIN_MODE_ACTIVE': 15, 'MENU_ACTIVE': 16, 'ACTIVITY_CODE': 11,
        'LAST_FAULT_CODE': 13, 'I2C_ERROR_COUNT': 12, 'UPTIME_SEC': 14,
    },
    'DISPENSER': {
        'MAIN_MODE_ACTIVE': 18, 'MENU_ACTIVE': 20, 'ACTIVITY_CODE': 11,
        'LAST_FAULT_CODE': 13, 'I2C_ERROR_COUNT': 12, 'UPTIME_SEC': 14,
    },
    'STOCKER': {
        'MAIN_MODE_ACTIVE': 17, 'MENU_ACTIVE': 18, 'ACTIVITY_CODE': 13,
        'LAST_FAULT_CODE': 15, 'I2C_ERROR_COUNT': 14, 'UPTIME_SEC': 16,
    },
}

KHUSUS_NODE = {
    'SORTER': [
        ('PASS_COUNT', 10), ('REJECT_COUNT', 11), ('REJECT_MISSED_COUNT', 19),
        ('SPEED_LAST_MM_S', 20), ('SPEED_LAST_MS', 21),
        ('SPEED_SAMPLE_COUNT', 22), ('SPEED_MM_S_AT_MAX_PWM', 23),
    ],
    'PICKER': [
        ('CURRENT_POSE', 10), ('GERAKAN_AKTIF', 17), ('ADEGAN_KE', 18),
    ],
    'DISPENSER': [
        ('DISPENSER_READY', 25), ('PIPELINE_STAGE', 26), ('PACKAGE_READY_FLAG', 15),
        ('MIDDLE_PACKAGE_PRESENT', 16), ('UJUNG_PACKAGE_PRESENT', 17),
        ('MIDDLE_ARRIVAL_COUNT', 19), ('UJUNG_ARRIVAL_COUNT', 24),
        ('CONVEYOR_AUTOSTOP_COUNT', 23),
        ('BTN_PACKAGE_FULL_COUNT', 21), ('BTN_PACKAGE_TAKEN_COUNT', 22),
        ('STOCK_EMPTY_FLAG', 10),
    ],
    'STOCKER': [
        ('CURRENT_RACK_IDX', 10), ('ALL_HOMED_FLAG', 11), ('RACK_OCCUPIED_BITMASK', 12),
    ],
}

# Register yang BARU ADA setelah flash 2026-09-20 ke atas. Dipakai untuk membedakan
# node yang sudah di-flash dari yang belum -- pertanyaan yang selalu muncul duluan
# setiap kali "tidak ada yang berubah".
REG_PENANDA_FIRMWARE_BARU = {
    'SORTER':    [('MENU_ACTIVE', 18), ('REJECT_MISSED_COUNT', 19), ('SPEED_MM_S_AT_MAX_PWM', 23)],
    'PICKER':    [('MENU_ACTIVE', 16)],
    'DISPENSER': [('MENU_ACTIVE', 20), ('DISPENSER_READY', 25), ('PIPELINE_STAGE', 26)],
    'STOCKER':   [('MENU_ACTIVE', 18)],
}

ACTIVITY_NAMES = {
    'SORTER': {0: 'DIAM', 1: 'CONVEYOR_JALAN', 2: 'PALANG_AKTIF', 3: 'PALANG_MANUAL',
               4: 'TEST_HOPPER_AKTIF', 90: 'FAULT', 91: 'ESTOP'},
    'PICKER': {0: 'DIAM', 1: 'MENUJU_HOME', 5: 'MENGAMBIL', 6: 'MELETAKKAN',
               7: 'NAIK_CLEARANCE', 8: 'BERGERAK', 9: 'POST_PLACE_GERAK',
               10: 'MENUJU_PACKAGE_PICKUP', 11: 'MENUJU_LIFT_LOAD', 12: 'ADEGAN_GERAK', 13: 'MENUJU_READY',
               90: 'FAULT', 91: 'ESTOP'},
    'DISPENSER': {0: 'DIAM', 1: 'CONVEYOR_JALAN', 8: 'SELESAI',
                  9: 'TUNGGU_KONFIRM_TENGAH', 10: 'TEST_LOOP_AKTIF', 90: 'FAULT', 91: 'ESTOP'},
    'STOCKER': {0: 'DIAM', 1: 'HOMING', 2: 'MENUJU_RAK', 3: 'MENDORONG_BOX',
                4: 'MENARIK_PUSHER', 5: 'KEMBALI_KE_HOME', 6: 'BERGERAK_MANUAL',
                7: 'TEST_DORONG', 8: 'TEST_TARIK', 90: 'FAULT', 91: 'ESTOP'},
}

# Tahap pipeline Dispenser (enum PipelineStage, urutan sama persis dengan firmware).
PIPELINE_NAMES = [
    'MATI', 'INIT servo1 buka', 'INIT servo1 tahan', 'INIT servo1 tutup',
    'INIT servo2 buka', 'INIT servo2 tahan', 'INIT servo2 tutup', 'INIT tunggu PROX_2',
    'buka gerbang', 'SIAP ISI (ready)', 'maju ke ujung', 'refill servo1 tutup',
    'refill servo2 buka', 'refill servo2 tahan', 'refill servo2 tutup', 'tunggu diambil arm',
]


# ############################################################################
# 4. MENU UJI NODE DI TERMINAL (--uji)
# ############################################################################
# Lapisan Modbus menu uji: memakai Bus yang SAMA dengan produksi & panel (satu port, satu kunci).
_BUS_UJI = None
ACK_TIMEOUT_S = 5.0


def baca(slave_id, reg, percobaan=3):
    """Bedakan: RegisterTidakAda (firmware belum punya register itu) vs ConnectionError
    (node tidak menjawab -- mati, atau kabel RS485 A/B lepas/tertukar)."""
    try:
        return _BUS_UJI.baca(slave_id, reg, percobaan=percobaan)
    except Gagal as e:
        raise ConnectionError(str(e))


def send_command(slave_id, opcode, arg=0, tunggu_ack=True):
    """Urutan WAJIB: CMD_ARG -> CMD_SEQ -> CMD (opcode ditulis PALING TERAKHIR)."""
    try:
        seq = _BUS_UJI.command(slave_id, opcode, arg)
        print(f"  -> Terkirim: opcode={opcode} arg={arg} seq={seq}")
    except Exception as e:
        print(f"  !! GAGAL kirim command: {e}")
        return False
    if not tunggu_ack:
        return True
    batas = time.monotonic() + ACK_TIMEOUT_S
    while time.monotonic() < batas:
        try:
            if baca(slave_id, REG_CMD_ACK_SEQ, percobaan=1) == seq:
                print("  .. di-ack. INGAT: ack = 'diterima & diproses', BUKAN 'berhasil'.")
                print("     Berhasil atau ditolak dibaca dari STATE/FAULT_CODE/ACTIVITY_CODE")
                print("     atau dari efek fisiknya.")
                return True
        except Exception:
            pass
        time.sleep(0.1)
    print(f"  !! TIDAK di-ack dalam {ACK_TIMEOUT_S}s -- node mati / kabel lepas, atau operator")
    print("     sedang di menu kalibrasi LCD node itu (MENU_ACTIVE = 1).")
    return False


def cetak(label, nilai, catatan=''):
    print(f"  {label:24s} = {nilai}{catatan}")


def catatan_untuk(node_name, label, val, firmware_lama):
    if label == 'ACTIVITY_CODE':
        return '  ' + ACTIVITY_NAMES.get(node_name, {}).get(val, '?')
    if label == 'MAIN_MODE_ACTIVE':
        return '  <- MAIN (produksi)' if val == 1 else '  <- TEST mode'
    if label == 'MENU_ACTIVE' and val == 1:
        return '  <- OPERATOR DI MENU LCD, SEMUA COMMAND DIABAIKAN'
    if label == 'PIPELINE_STAGE':
        return '  ' + (PIPELINE_NAMES[val] if val < len(PIPELINE_NAMES) else '?')
    if label == 'CURRENT_RACK_IDX' and val == 0xFF:
        return '  (sedang pindah / bukan di rak)'
    if label == 'GERAKAN_AKTIF':
        return '  ' + {0: 'Home>Ready', 1: 'Ready>Pick', 2: 'Pick>Home', 3: 'Home>Place', 4: 'Place>Home',
                       0xFF: '(tidak ada)'}.get(val, '?')
    if label == 'CURRENT_POSE':
        return '  ' + {0: 'HOME', 1: 'PICKUP', 2: 'LIFT', 3: 'READY'}.get(val, '?')
    if label == 'RACK_OCCUPIED_BITMASK':
        # Penomoran bit DIPERBAIKI 2026-09-20 (temuan #22): dulu bit0-3 = Rak 1-4,
        # sekarang bit1-4 = Rak 1-4. Menerjemahkan firmware lama dengan aturan baru
        # menggeser hasilnya satu rak -- salah tanpa terlihat salah.
        if firmware_lama:
            return '  (penomoran bit firmware LAMA, tidak diterjemahkan)'
        terisi = [str(b) for b in range(1, 5) if val & (1 << b)]
        return '  rak terisi: ' + (', '.join(terisi) if terisi else 'tidak ada')
    return ''


def read_status(slave_id, node_name):
    try:
        state = baca(slave_id, REG_STATE)
    except Exception as e:
        print(f"  !! Node tidak menjawab: {e}")
        print("     Cek daya node, kabel RS485 A/B, dan pastikan tidak ada program")
        print("     Modbus lain yang sedang memegang " + RS485_PORT + ".")
        return

    # Ditentukan SEBELUM register dicetak: penerjemahan RACK_OCCUPIED_BITMASK bergantung
    # padanya, dan register penandanya ada di alamat yang lebih tinggi.
    firmware_lama = False
    for _, addr in REG_PENANDA_FIRMWARE_BARU.get(node_name, []):
        try:
            baca(slave_id, addr)
        except RegisterTidakAda:
            firmware_lama = True
            break
        except Exception:
            pass

    print("  -- universal --")
    cetak('STATE', state, f"  ({STATE_NAMES.get(state, '?')})")
    for label, reg in (('FAULT_CODE', REG_FAULT_CODE), ('CMD_ACK_SEQ', REG_CMD_ACK_SEQ),
                       ('HEARTBEAT', REG_HEARTBEAT)):
        try:
            cetak(label, baca(slave_id, reg))
        except Exception as e:
            cetak(label, f"?? ({e})")

    def tampilkan(daftar):
        for label, addr in daftar:
            try:
                val = baca(slave_id, addr)
            except RegisterTidakAda:
                cetak(label, "(tidak ada di firmware node ini)")
                continue
            except Exception as e:
                cetak(label, f"?? ({e})")
                continue
            cetak(label, val, catatan_untuk(node_name, label, val, firmware_lama))

    print("  -- diagnostik (urutan sama di semua node) --")
    tampilkan([(n, ALAMAT[node_name][n]) for n in DIAGNOSTIK_BERSAMA])
    print(f"  -- khusus {node_name} --")
    tampilkan(KHUSUS_NODE[node_name])

    if firmware_lama:
        print("\n  !! Sebagian register tidak ada -- node ini menjalankan firmware LAMA.")
        print("     Flash ulang dulu, kalau tidak perubahan firmware apa pun tidak akan")
        print("     terlihat dari sini.")


def cek_firmware():
    """Node mana yang sudah di-flash, mana yang belum.

    Tidak ada register versi firmware di protokol, jadi yang dipakai adalah
    keberadaan register yang baru ditambahkan.
    """
    print("\n=== CEK FIRMWARE TIAP NODE ===")
    print("Dinilai dari ada/tidaknya register yang baru ditambahkan 2026-09-20 ke atas.\n")
    for node in NODES.values():
        name = node['name']
        sid = node['slave_id']
        try:
            baca(sid, REG_STATE)
        except Exception:
            print(f"  {name:10s} (slave {sid}) : TIDAK MENJAWAB -- node mati atau kabel lepas")
            continue

        hilang = []
        for label, addr in REG_PENANDA_FIRMWARE_BARU.get(name, []):
            try:
                baca(sid, addr)
            except RegisterTidakAda:
                hilang.append(label)
            except Exception:
                pass

        if hilang:
            print(f"  {name:10s} (slave {sid}) : FIRMWARE LAMA -- belum ada {', '.join(hilang)}")
        else:
            print(f"  {name:10s} (slave {sid}) : firmware baru (register penanda lengkap)")
    print("\nCatatan: ini memeriksa keberadaan register, bukan nomor versi. Node yang")
    print("sudah punya register penanda tapi di-flash lagi dengan perubahan yang lebih")
    print("baru tetap terlihat sama di sini. Versi firmware lengkap ada di panel LCD:")
    print("menu utama -> Info Sistem.")


def cetak_daftar(name, slave_id, daftar):
    print(f"\n=== {name} (Slave {slave_id}) ===")
    print(" s. Baca STATUS")
    kelompok_sebelum = None
    for i, (label, opcode, arg, gate, kelompok) in enumerate(daftar, start=1):
        if kelompok != kelompok_sebelum:
            if kelompok_sebelum in KELOMPOK_PANEL and kelompok not in KELOMPOK_PANEL:
                print("   " + "-" * 60)
            print(f"   [{JUDUL_KELOMPOK[kelompok]}]")
            kelompok_sebelum = kelompok
        huruf = f"({chr(ord('a') + i - 1)})" if kelompok in KELOMPOK_PANEL else "   "
        op = f"op{opcode:<3d}" if opcode is not None else "  -  "
        if gate == 'lokal':
            ket = "hanya dari panel, bukan command Modbus"
        elif isinstance(arg, str):
            ket = f"arg: {arg}"
        elif isinstance(arg, int):
            ket = f"arg tetap {arg}"
        else:
            ket = ""
        print(f"{i:2d}. {huruf} {label:26s} {op}  {ket}")
    print(" 0. Kembali ke menu utama")
    print("(a), (b), ... = huruf item yang sama di menu Test Command panel LCD node ini.")


def menu_node(node_key):
    node = NODES[node_key]
    name = node['name']
    slave_id = node['slave_id']
    daftar = OPCODES[name]

    while True:
        cetak_daftar(name, slave_id, daftar)
        choice = input("Pilih: ").strip()

        if choice == '0':
            break
        if choice.lower() == 's':
            read_status(slave_id, name)
            time.sleep(0.3)
            continue

        try:
            idx = int(choice) - 1
            if idx < 0 or idx >= len(daftar):
                raise IndexError
        except (ValueError, IndexError):
            print("  Pilihan tidak valid.")
            time.sleep(0.3)
            continue

        label, opcode, arg, gate, kelompok = daftar[idx]

        if gate == 'lokal':
            print(f"  {label} adalah fungsi lokal panel, bukan command Modbus -- tidak bisa")
            print("  dikirim dari sini. Jalankan dari panel: menu Test Command node ini.")
            time.sleep(0.3)
            continue

        # Peringatan mode SEBELUM mengirim. Tanpa ini, command yang ditolak tetap
        # di-ack dan tidak ada apa pun di layar ini yang menunjukkan kenapa tidak
        # terjadi apa-apa.
        if gate in ('main', 'test'):
            try:
                mode = baca(slave_id, ALAMAT[name]['MAIN_MODE_ACTIVE'])
            except Exception:
                mode = None
            if mode == 1 and gate == 'test':
                print(f"  !! {label} ditolak selama MAIN, dan node sedang MAIN.")
                print("     Akan di-ack lalu DIABAIKAN. Kirim STOP_MAIN (nomor 2) dulu.")
            elif mode == 0 and gate == 'main':
                print(f"  !! {label} hanya diterima saat MAIN, dan node sedang TEST.")
                print("     Akan di-ack lalu DIABAIKAN. Kirim START_MAIN (nomor 1) dulu.")

        if isinstance(arg, str):
            arg_str = input(f"  Masukkan arg ({arg}): ").strip()
            try:
                nilai_arg = int(arg_str)
            except ValueError:
                print("  Arg tidak valid, dibatalkan.")
                time.sleep(0.3)
                continue
        elif isinstance(arg, int):
            nilai_arg = arg
        else:
            nilai_arg = 0

        send_command(slave_id, opcode, nilai_arg)
        time.sleep(0.3)


def menu_uji(bus):
    """Menu uji node di terminal (dulu sorting_automation.py)."""
    global _BUS_UJI
    _BUS_UJI = bus
    while True:
        print("\n============================================")
        print(" SORTING AUTOMATION -- MENU UJI NODE (--uji)")
        print(f" Port: {RS485_PORT}  Baud: {RS485_BAUD}")
        print("============================================")
        for key, node in NODES.items():
            print(f"{key}. {node['name']} (Slave {node['slave_id']})")
        print("m. Monitor semua node (baca status sekali)")
        print("f. Cek firmware tiap node (sudah di-flash atau belum)")
        print("q. Keluar")
        choice = input("Pilih: ").strip()

        if choice == 'q':
            print("Keluar.")
            return
        elif choice == 'm':
            for node in NODES.values():
                print(f"\n--- {node['name']} (Slave {node['slave_id']}) ---")
                read_status(node['slave_id'], node['name'])
        elif choice == 'f':
            cek_firmware()
        elif choice in NODES:
            menu_node(choice)
        else:
            print("Pilihan tidak valid.")


# ############################################################################
# 5. LAYAR -- panel LCD sentuh (dulu sorting-panel.py)
# ############################################################################
# ============================================================
# KONFIGURASI LAYAR -- display.config [lcd] & [tampilan]
# ============================================================
SKEMA_LCD = {
    'lcd_spi_bus': int, 'lcd_spi_device': int, 'lcd_spi_hz': int, 'lcd_rotasi': int,
    'lcd_bgr': _nol_satu, 'lcd_invert': _nol_satu,
    'lcd_pin_dc': _pin, 'lcd_pin_cs': _pin, 'lcd_pin_rst': _pin, 'lcd_pin_led': _pin,
    'touch_clk': _pin, 'touch_cs': _pin, 'touch_din': _pin,
    'touch_out': _pin, 'touch_irq': _pin,
}


def muat_config_lcd():
    if not os.path.exists(FILE_DISPLAY):
        sys.exit(f"ERROR: {FILE_DISPLAY} tidak ada -- salin display.config ke folder program.")
    cp = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cp.read(FILE_DISPLAY, encoding='utf-8-sig')
    hasil, salah = {}, []
    for kunci, ubah in SKEMA_LCD.items():
        if not cp.has_option('lcd', kunci):
            salah.append(f"[lcd] {kunci} tidak ada")
            continue
        try:
            hasil[kunci] = ubah(cp.get('lcd', kunci))
        except ValueError:
            salah.append(f"[lcd] {kunci} = {cp.get('lcd', kunci)!r} bukan nilai yang sah")
    # Opsional -- nama di header layar.
    hasil['lcd_judul'] = cp.get('tampilan', 'judul', fallback='SORTING').strip() or 'SORTING'
    if hasil.get('lcd_rotasi') not in (1, 3, None):
        salah.append("[lcd] lcd_rotasi harus 1 atau 3 (layar mendatar)")
    # Satu pin dipakai dua fungsi = salah satunya diam-diam tidak bekerja.
    dipakai = {}
    for nama in ('LED_RED', 'LED_BLUE', 'BUZZER', 'LED_OPR', 'LED_ALARM'):
        if globals()[nama] is not None:
            dipakai.setdefault(globals()[nama], []).append(nama.lower())
    if hasil.get('lcd_pin_dc') is None or hasil.get('touch_cs') is None:
        salah.append("[lcd] lcd_pin_dc dan touch_cs wajib diisi")
    if hasil.get('touch_clk') is None:
        # Touch berbagi bus SPI: CS LCD WAJIB lewat GPIO. Kalau LCD di CE0, setiap pembacaan
        # touch ikut mengaktifkan LCD dan perintah touch terbaca sebagai perintah layar.
        if hasil.get('lcd_pin_cs') is None:
            salah.append("[lcd] touch berbagi SPI (touch_clk = none) butuh lcd_pin_cs berupa GPIO")
    elif hasil.get('touch_din') is None or hasil.get('touch_out') is None:
        salah.append("[lcd] touch bit-bang (touch_clk diisi) butuh touch_din dan touch_out")
    for kunci in ('lcd_pin_dc', 'lcd_pin_cs', 'lcd_pin_rst', 'lcd_pin_led', 'touch_clk', 'touch_cs',
                  'touch_din', 'touch_out', 'touch_irq'):
        if hasil.get(kunci) is not None:
            dipakai.setdefault(hasil[kunci], []).append(kunci)
    for pin, nama in dipakai.items():
        if len(nama) > 1:
            salah.append(f"pin wPi {pin} dipakai dua kali: {', '.join(nama)}")
    if salah:
        sys.exit("ERROR di " + FILE_DISPLAY + ":\n  " + "\n  ".join(salah))
    return hasil


# ============================================================
# GPIO (wiringOP, nomor wPi -- sama dengan LED/buzzer di sorting.config)
# ============================================================
class Gpio:
    def __init__(self):
        import wiringpi
        self.w = wiringpi
        wiringpi.wiringPiSetup()

    def keluar(self, pin, nilai=0):
        self.w.pinMode(pin, 1)
        self.w.digitalWrite(pin, nilai)

    def masuk(self, pin, pullup=False):
        self.w.pinMode(pin, 0)
        if pullup:
            self.w.pullUpDnControl(pin, 2)

    def tulis(self, pin, nilai):
        self.w.digitalWrite(pin, nilai)

    def baca(self, pin):
        return self.w.digitalRead(pin)


# ============================================================
# LCD ILI9341 lewat spidev
# ============================================================
class LayarILI9341:
    # Urutan inisialisasi baku ILI9341 (sama dengan driver Adafruit).
    INIT = [
        (0xEF, [0x03, 0x80, 0x02]), (0xCF, [0x00, 0xC1, 0x30]), (0xED, [0x64, 0x03, 0x12, 0x81]),
        (0xE8, [0x85, 0x00, 0x78]), (0xCB, [0x39, 0x2C, 0x00, 0x34, 0x02]), (0xF7, [0x20]),
        (0xEA, [0x00, 0x00]), (0xC0, [0x23]), (0xC1, [0x10]), (0xC5, [0x3E, 0x28]), (0xC7, [0x86]),
        (0x37, [0x00]), (0x3A, [0x55]), (0xB1, [0x00, 0x18]), (0xB6, [0x08, 0x82, 0x27]),
        (0xF2, [0x00]), (0x26, [0x01]),
        (0xE0, [0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00]),
        (0xE1, [0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F]),
    ]

    def __init__(self, k, gpio):
        import spidev
        self.g = gpio
        self.dc, self.rst, self.cs = k['lcd_pin_dc'], k['lcd_pin_rst'], k['lcd_pin_cs']
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(k['lcd_spi_bus'], k['lcd_spi_device'])
        except FileNotFoundError:
            dev = f"/dev/spidev{k['lcd_spi_bus']}.{k['lcd_spi_device']}"
            sys.exit(f"ERROR: {dev} tidak ada -- SPI belum diaktifkan.\n"
                     f"  Tambahkan spi-spidev ke baris overlays= di /boot/armbianEnv.txt (atau\n"
                     f"  /boot/orangepiEnv.txt), plus param_spidev_spi_bus=0 dan\n"
                     f"  param_spidev_max_freq=32000000, lalu reboot. Atau: setup-orangepi-one.sh --panel")
        self.spi.mode = 0
        self.spi.max_speed_hz = k['lcd_spi_hz']
        gpio.keluar(self.dc, 0)
        if self.cs is not None:
            gpio.keluar(self.cs, 1)
        if self.rst is not None:
            gpio.keluar(self.rst, 1)
            time.sleep(0.01); gpio.tulis(self.rst, 0); time.sleep(0.02); gpio.tulis(self.rst, 1)
            time.sleep(0.15)
        if k['lcd_pin_led'] is not None:
            gpio.keluar(k['lcd_pin_led'], 1)
        self._pilih(True)
        self._perintah(0x01); time.sleep(0.15)          # software reset (juga kalau RESET ke 3.3V)
        for c, data in self.INIT:
            self._perintah(c, data)
        # MADCTL: rotasi 1 = mendatar (MV), 3 = mendatar terbalik (MX|MY|MV). Bit BGR untuk
        # modul yang warna merah-birunya tertukar.
        madctl = {1: 0x20, 3: 0xE0}[k['lcd_rotasi']] | (0x08 if k['lcd_bgr'] else 0)
        self._perintah(0x36, [madctl])
        self._perintah(0x21 if k['lcd_invert'] else 0x20)
        self._perintah(0x11); time.sleep(0.12)          # sleep out
        self._perintah(0x29)                            # display on
        self._pilih(False)
        self.sebelumnya = None

    def _pilih(self, aktif):
        # CS LCD lewat GPIO (mode SPI berbagi). Tanpa lcd_pin_cs, CE0 hardware yang dipakai.
        if self.cs is not None:
            self.g.tulis(self.cs, 0 if aktif else 1)

    def _perintah(self, c, data=None):
        self.g.tulis(self.dc, 0)
        self.spi.writebytes([c])
        if data:
            self.g.tulis(self.dc, 1)
            self.spi.writebytes(data)

    def _jendela(self, x0, y0, x1, y1):
        self._perintah(0x2A, [x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF])
        self._perintah(0x2B, [y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF])
        self.g.tulis(self.dc, 0)
        self.spi.writebytes([0x2C])
        self.g.tulis(self.dc, 1)

    def tampilkan(self, gambar):
        """Hanya kotak yang BERUBAH yang dikirim -- satu layar penuh 150 kB, sedangkan
        perubahan angka status biasanya hanya beberapa ratus byte."""
        import numpy as np
        a = np.asarray(gambar.convert('RGB'), dtype=np.uint16)
        if self.sebelumnya is None:
            y0, y1, x0, x1 = 0, TINGGI - 1, 0, LEBAR - 1
        else:
            beda = np.any(a != self.sebelumnya, axis=2)
            baris, kolom = np.where(beda.any(axis=1))[0], np.where(beda.any(axis=0))[0]
            if len(baris) == 0:
                return
            y0, y1, x0, x1 = int(baris[0]), int(baris[-1]), int(kolom[0]), int(kolom[-1])
        self.sebelumnya = a
        bagian = a[y0:y1 + 1, x0:x1 + 1]
        rgb565 = ((bagian[..., 0] & 0xF8) << 8) | ((bagian[..., 1] & 0xFC) << 3) | (bagian[..., 2] >> 3)
        data = rgb565.astype('>u2').tobytes()
        self._pilih(True)
        self._jendela(x0, y0, x1, y1)
        if hasattr(self.spi, 'writebytes2'):
            self.spi.writebytes2(data)
        else:
            for i in range(0, len(data), 4096):
                self.spi.writebytes(list(data[i:i + 4096]))
        self._pilih(False)


# ============================================================
# TOUCH XPT2046 -- dua cara sambung:
#   SPI berbagi (touch_clk = none)  T_CLK/T_DIN/T_DO ke SCLK/MOSI/MISO yang sama dengan LCD,
#                                   hanya T_CS yang GPIO sendiri. Paling sedikit pin.
#   bit-bang (touch_clk diisi)      kelima pin touch ke GPIO sendiri, bus SPI hanya untuk LCD.
# ============================================================
class SentuhXPT2046:
    CMD_X, CMD_Y, CMD_Z1, CMD_Z2 = 0xD0, 0x90, 0xB0, 0xC0
    HZ_TOUCH = 2000000   # XPT2046 maksimal ~2,5 MHz -- jauh di bawah kecepatan LCD

    def __init__(self, k, gpio, spi=None):
        self.g = gpio
        self.clk, self.cs, self.din, self.do, self.irq = (k['touch_clk'], k['touch_cs'],
                                                          k['touch_din'], k['touch_out'], k['touch_irq'])
        self.spi = spi if self.clk is None else None
        gpio.keluar(self.cs, 1)
        if self.spi is None:
            gpio.keluar(self.clk, 0)
            gpio.keluar(self.din, 0)
            gpio.masuk(self.do)
        if self.irq is not None:
            gpio.masuk(self.irq, pullup=True)

    def _xfer(self, cmd):
        g = self.g
        if self.spi is not None:
            g.tulis(self.cs, 0)
            r = self.spi.xfer2([cmd, 0, 0], self.HZ_TOUCH)
            g.tulis(self.cs, 1)
            return ((r[1] << 8) | r[2]) >> 3   # 1 bit busy, 12 bit data, 3 bit nol
        g.tulis(self.cs, 0)
        for i in range(7, -1, -1):
            g.tulis(self.din, (cmd >> i) & 1)
            g.tulis(self.clk, 1)
            g.tulis(self.clk, 0)
        g.tulis(self.din, 0)
        nilai = 0
        for _ in range(16):
            g.tulis(self.clk, 1)
            g.tulis(self.clk, 0)
            nilai = (nilai << 1) | g.baca(self.do)
        g.tulis(self.cs, 1)
        return nilai >> 4   # 12 bit

    def mentah(self):
        """(x, y) mentah 0-4095, atau None kalau tidak disentuh."""
        if self.irq is not None and self.g.baca(self.irq):
            return None
        z = self._xfer(self.CMD_Z1) + 4095 - self._xfer(self.CMD_Z2)
        if z < 400:          # tekanan terlalu kecil -- bukan sentuhan
            return None
        xs = sorted(self._xfer(self.CMD_X) for _ in range(5))
        ys = sorted(self._xfer(self.CMD_Y) for _ in range(5))
        return xs[2], ys[2]   # median: buang lonjakan


class Kalibrasi:
    """Pemetaan mentah -> piksel: x = ax*u + bx, y = ay*v + by, (u, v) = mentah atau
    ditukar (swap) tergantung bagaimana panel sentuhnya dipasang."""

    def __init__(self, data=None):
        self.data = data

    @staticmethod
    def muat():
        try:
            with open(FILE_KALIBRASI, encoding='utf-8') as f:
                return Kalibrasi(json.load(f))
        except (FileNotFoundError, ValueError):
            return Kalibrasi(None)

    def ada(self):
        return self.data is not None

    def simpan(self):
        with open(FILE_KALIBRASI, 'w', encoding='utf-8') as f:
            json.dump(self.data, f)

    def ke_layar(self, mentah):
        d = self.data
        u, v = (mentah[1], mentah[0]) if d['swap'] else mentah
        return int(d['ax'] * u + d['bx']), int(d['ay'] * v + d['by'])

    @staticmethod
    def hitung(titik, mentah):
        """titik: 3 posisi layar A, B (kanan A), C (bawah A); mentah: hasil sentuhannya.
        Sumbu mentah yang paling berubah dari A ke B adalah sumbu X layar."""
        (ax_, ay_), (bx_, _), (_, cy_) = titik
        (ra, rb, rc) = mentah
        swap = abs(rb[1] - ra[1]) > abs(rb[0] - ra[0])
        u = lambda r: r[1] if swap else r[0]
        v = lambda r: r[0] if swap else r[1]
        if u(rb) == u(ra) or v(rc) == v(ra):
            return None
        kx = (bx_ - ax_) / (u(rb) - u(ra))
        ky = (cy_ - ay_) / (v(rc) - v(ra))
        return {'swap': swap, 'ax': kx, 'bx': ax_ - kx * u(ra), 'ay': ky, 'by': ay_ - ky * v(ra)}


# ============================================================
# TAMPILAN (2026-10-06, gaya HMI): header + isi + tab bar bawah
#
#   y   0- 23  header     logo, nama mesin, tanggal & jam
#   y  26-199  isi        satu layar per tab (atau sub-layar)
#   y 202-239  tab bar    HOME  AUTO  NODE  ALARM  SETTING
# ============================================================
def _font(ukuran, tebal=False):
    nama = 'DejaVuSans-Bold.ttf' if tebal else 'DejaVuSans.ttf'
    for folder in ('/usr/share/fonts/truetype/dejavu', '/usr/share/fonts/dejavu', FOLDER):
        try:
            return ImageFont.truetype(os.path.join(folder, nama), ukuran)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=ukuran)
    except TypeError:
        return ImageFont.load_default()


if ADA_PIL:
    F_MINI, F_KECIL, F_BIASA, F_TEBAL = _font(9), _font(10), _font(12), _font(12, True)
    F_JUDUL, F_ANGKA, F_BESAR = _font(13, True), _font(22, True), _font(30, True)
else:   # mode terminal tanpa Pillow
    F_MINI = F_KECIL = F_BIASA = F_TEBAL = F_JUDUL = F_ANGKA = F_BESAR = None

# Warna -- ganti di sini untuk mengubah tema seluruh layar.
HITAM, PUTIH = (0, 0, 0), (255, 255, 255)
LATAR = (11, 19, 31)           # latar layar
KARTU = (22, 34, 52)           # kartu / panel
KARTU2 = (32, 47, 70)          # tombol netral, kartu di dalam kartu
GARIS = (48, 66, 94)
ABU = (140, 156, 178)          # teks keterangan
ABU_GELAP = (70, 82, 100)      # tombol tidak aktif
BIRU = (30, 120, 230)          # tab aktif, tombol utama
HIJAU = (34, 170, 80)
MERAH = (220, 60, 60)
KUNING = (240, 190, 40)
ORANYE = (235, 125, 30)
# Warna di atas bisa ditimpa bagian [warna] display.config (RRGGBB).
globals().update(_warna_display())

FASE = {   # fase produksi -> (warna, baris 1, baris 2, ikon, label pendek)
    'SIAP': (KARTU2, 'MESIN', 'SIAP', 'stop', 'STOPPED'),
    'PERIKSA': (KUNING, 'PEMERIKSAAN', 'AWAL', 'info', 'CHECKING'),
    'STARTUP': (KUNING, 'MENYIAPKAN', 'NODE', 'ulang', 'STARTING'),
    'JALAN': (HIJAU, 'MESIN', 'BERJALAN', 'play', 'RUNNING'),
    'DITAHAN': (MERAH, 'PRODUKSI', 'DITAHAN', 'alarm', 'DITAHAN'),
    'GAGAL CEK': (MERAH, 'CEK AWAL', 'GAGAL', 'alarm', 'GAGAL'),
    'GAGAL STARTUP': (MERAH, 'STARTUP', 'GAGAL', 'alarm', 'GAGAL'),
    'SELESAI': (BIRU, 'TARGET', 'SELESAI', 'cek', 'SELESAI'),
    'BERHENTI': (KARTU2, 'MESIN', 'BERHENTI', 'stop', 'STOPPED'),
    'CEK OK': (BIRU, 'CEK AWAL', 'OK', 'cek', 'CEK OK'),
}
BULAN = ['Jan', 'Feb', 'Mar', 'Apr', 'Mei', 'Jun', 'Jul', 'Agu', 'Sep', 'Okt', 'Nov', 'Des']
SINGKAT = {SORTER: 'SORTER', PICKER: 'PICKER', DISPENSER: 'DISPENSER', STOCKER: 'STOCKER'}
IKON_NODE = {SORTER: 'hopper', DISPENSER: 'kotak', PICKER: 'lengan', STOCKER: 'rak'}
ALUR = [SORTER, DISPENSER, PICKER, STOCKER]   # urutan barang mengalir


def kotak(d, xy, r, fill, outline=None):
    # rounded_rectangle baru ada di Pillow 8.2 -- Armbian lama bisa membawa 8.1.
    if hasattr(d, 'rounded_rectangle'):
        d.rounded_rectangle(xy, r, fill=fill, outline=outline)
    else:
        d.rectangle(xy, fill=fill, outline=outline)


def teks_tengah(d, cx, cy, teks, font, warna):
    b = d.textbbox((0, 0), teks, font=font)
    d.text((cx - (b[2] - b[0]) // 2 - b[0], cy - (b[3] - b[1]) // 2 - b[1]), teks, font=font, fill=warna)


def teks_kanan(d, xkanan, y, teks, font, warna):
    d.text((xkanan - d.textlength(teks, font=font), y), teks, font=font, fill=warna)


def potong(d, teks, font, lebar):
    if d.textlength(teks, font=font) <= lebar:
        return teks
    while teks and d.textlength(teks + '..', font=font) > lebar:
        teks = teks[:-1]
    return teks + '..'


def bar(d, x, y, w, h, frac, warna):
    kotak(d, (x, y, x + w, y + h), h // 2, KARTU2)
    isi = int(w * max(0.0, min(1.0, frac)))
    if isi > h:
        kotak(d, (x, y, x + isi, y + h), h // 2, warna)


def ikon(d, nama, cx, cy, s, w=PUTIH):
    """Ikon sederhana dari garis & bidang -- tidak butuh file gambar. s = setengah ukuran."""
    lw = max(1, s // 6)
    if nama == 'rumah':
        d.polygon([(cx - s, cy), (cx, cy - s), (cx + s, cy)], fill=w)
        d.rectangle((cx - s * 2 // 3, cy, cx + s * 2 // 3, cy + s), fill=w)
    elif nama == 'play':
        d.polygon([(cx - s * 2 // 3, cy - s), (cx - s * 2 // 3, cy + s), (cx + s, cy)], fill=w)
    elif nama == 'stop':
        d.rectangle((cx - s * 3 // 4, cy - s * 3 // 4, cx + s * 3 // 4, cy + s * 3 // 4), fill=w)
    elif nama == 'cek':
        d.line([(cx - s, cy), (cx - s // 3, cy + s * 2 // 3), (cx + s, cy - s * 2 // 3)], fill=w, width=lw + 1)
    elif nama == 'silang':
        d.line([(cx - s * 2 // 3, cy - s * 2 // 3), (cx + s * 2 // 3, cy + s * 2 // 3)], fill=w, width=lw + 1)
        d.line([(cx - s * 2 // 3, cy + s * 2 // 3), (cx + s * 2 // 3, cy - s * 2 // 3)], fill=w, width=lw + 1)
    elif nama == 'gear':
        import math
        for i in range(8):
            a = i * math.pi / 4
            d.line([(cx, cy), (cx + math.cos(a) * s, cy + math.sin(a) * s)], fill=w, width=lw + 2)
        d.ellipse((cx - s * 3 // 4, cy - s * 3 // 4, cx + s * 3 // 4, cy + s * 3 // 4), fill=w)
        d.ellipse((cx - s // 3, cy - s // 3, cx + s // 3, cy + s // 3), fill=LATAR)
    elif nama == 'daftar':
        for i in (-1, 0, 1):
            y = cy + i * s * 2 // 3
            d.ellipse((cx - s, y - lw, cx - s + lw * 2, y + lw), fill=w)
            d.line([(cx - s // 2, y), (cx + s, y)], fill=w, width=lw + 1)
    elif nama == 'alarm':
        d.polygon([(cx, cy - s), (cx + s, cy + s * 4 // 5), (cx - s, cy + s * 4 // 5)], fill=w)
        d.line([(cx, cy - s // 3), (cx, cy + s // 4)], fill=LATAR, width=lw + 1)
        d.ellipse((cx - lw, cy + s // 2 - lw, cx + lw, cy + s // 2 + lw), fill=LATAR)
    elif nama == 'chip':
        d.rectangle((cx - s * 2 // 3, cy - s * 2 // 3, cx + s * 2 // 3, cy + s * 2 // 3), outline=w, width=lw + 1)
        for i in (-1, 0, 1):
            o = i * s // 3
            d.line([(cx + o, cy - s), (cx + o, cy - s * 2 // 3)], fill=w, width=lw)
            d.line([(cx + o, cy + s), (cx + o, cy + s * 2 // 3)], fill=w, width=lw)
            d.line([(cx - s, cy + o), (cx - s * 2 // 3, cy + o)], fill=w, width=lw)
            d.line([(cx + s, cy + o), (cx + s * 2 // 3, cy + o)], fill=w, width=lw)
    elif nama == 'ulang':
        d.arc((cx - s, cy - s, cx + s, cy + s), 30, 330, fill=w, width=lw + 1)
        d.polygon([(cx + s, cy - s // 2), (cx + s // 4, cy - s // 2), (cx + s * 3 // 4, cy + s // 6)], fill=w)
    elif nama == 'info':
        d.ellipse((cx - s, cy - s, cx + s, cy + s), outline=w, width=lw + 1)
        d.line([(cx, cy - s // 6), (cx, cy + s // 2)], fill=w, width=lw + 1)
        d.ellipse((cx - lw, cy - s // 2 - lw, cx + lw, cy - s // 2 + lw), fill=w)
    elif nama == 'grafik':
        for i, h in enumerate((0.45, 0.75, 1.0)):
            x = cx - s + i * s * 2 // 3
            d.rectangle((x, cy + s - int(2 * s * h), x + s // 2, cy + s), fill=w)
    elif nama == 'hopper':
        d.polygon([(cx - s, cy - s), (cx + s, cy - s), (cx + s // 4, cy + s // 4), (cx - s // 4, cy + s // 4)], fill=w)
        d.rectangle((cx - s // 4, cy + s // 4, cx + s // 4, cy + s), fill=w)
    elif nama == 'kotak':
        d.rectangle((cx - s, cy - s // 2, cx + s, cy + s), fill=w)
        d.polygon([(cx - s, cy - s // 2), (cx - s // 2, cy - s), (cx + s // 2, cy - s), (cx + s, cy - s // 2)], fill=w)
        d.line([(cx - s, cy - s // 2), (cx + s, cy - s // 2)], fill=LATAR, width=lw)
    elif nama == 'lengan':
        d.rectangle((cx - s, cy + s * 2 // 3, cx, cy + s), fill=w)
        d.line([(cx - s // 2, cy + s * 2 // 3), (cx - s // 4, cy - s // 3), (cx + s * 2 // 3, cy - s * 2 // 3)],
               fill=w, width=lw + 2)
        d.line([(cx + s * 2 // 3, cy - s * 2 // 3), (cx + s, cy - s // 6)], fill=w, width=lw + 1)
    elif nama == 'rak':
        d.rectangle((cx - s, cy - s, cx + s, cy + s), outline=w, width=lw + 1)
        d.line([(cx - s, cy), (cx + s, cy)], fill=w, width=lw + 1)
        d.line([(cx, cy - s), (cx, cy + s)], fill=w, width=lw + 1)
    elif nama == 'tanya':
        d.ellipse((cx - s, cy - s, cx + s, cy + s), fill=w)
        teks_tengah(d, cx, cy, '?', _font(int(s * 1.4), True), LATAR)
    elif nama == 'logo':
        import math
        titik = [(cx + math.cos(math.pi / 3 * i + math.pi / 6) * s, cy + math.sin(math.pi / 3 * i + math.pi / 6) * s)
                 for i in range(6)]
        d.polygon(titik, fill=BIRU)
        d.polygon([(cx, cy - s // 2), (cx + s // 2, cy + s // 3), (cx - s // 2, cy + s // 3)], fill=PUTIH)
    elif nama == 'kembali':
        d.polygon([(cx - s, cy), (cx, cy - s), (cx, cy + s)], fill=w)


class Tombol:
    """Bidang sentuh. Teks/ikon digambar oleh pemanggil kalau bentuknya khusus."""

    def __init__(self, x, y, w, h, teks, aksi, warna=KARTU2, aktif=True, font=None, ikon_=None):
        self.kotak, self.teks, self.aksi = (x, y, w, h), teks, aksi
        self.warna, self.aktif, self.font, self.ikon = warna, aktif, font or F_TEBAL, ikon_

    def kena(self, p):
        x, y, w, h = self.kotak
        return self.aktif and x <= p[0] < x + w and y <= p[1] < y + h

    def gambar(self, d):
        x, y, w, h = self.kotak
        kotak(d, (x, y, x + w - 1, y + h - 1), 6, self.warna if self.aktif else ABU_GELAP)
        warna_teks = PUTIH if self.aktif else ABU
        if self.ikon and h >= 40:      # ikon di atas, teks di bawah
            ikon(d, self.ikon, x + w // 2, y + h // 2 - 7, 9, warna_teks)
            teks_tengah(d, x + w // 2, y + h - 10, self.teks, F_KECIL, warna_teks)
        else:
            teks_tengah(d, x + w // 2, y + h // 2, self.teks, self.font, warna_teks)


# ============================================================
# TULIS CONFIG -- ganti nilai satu kunci, komentar & susunan file DIPERTAHANKAN
# ============================================================
def tulis_config(bagian, kunci, nilai):
    with open(FILE_CONFIG, encoding='utf-8-sig') as f:
        baris = f.read().split('\n')
    sekarang, ketemu = None, False
    pola = re.compile(r'^(\s*' + re.escape(kunci) + r'\s*=\s*)(.*?)(\s*(#.*)?)$')
    for i, b in enumerate(baris):
        m = re.match(r'^\s*\[(.+?)\]', b)
        if m:
            sekarang = m.group(1).strip()
            continue
        if sekarang == bagian and not ketemu:
            m = pola.match(b)
            if m and not b.lstrip().startswith('#'):
                lama = m.group(2)
                baru = str(nilai)
                if m.group(4):   # jaga kolom komentar tetap lurus
                    baru = baru.ljust(len(lama))
                baris[i] = m.group(1) + baru + (m.group(3) or '')
                ketemu = True
    if not ketemu:
        raise ValueError(f"[{bagian}] {kunci} tidak ditemukan di config")
    sementara = FILE_CONFIG + '.tmp'
    with open(sementara, 'w', encoding='utf-8') as f:
        f.write('\n'.join(baris))
        f.flush()
        os.fsync(f.fileno())
    os.replace(sementara, FILE_CONFIG)


def muat_ulang_config():
    """Config baru langsung dipakai mesin produksi berikutnya. Kembalikan pesan error atau None."""
    try:
        globals().update(muat_config(FILE_CONFIG))
        return None
    except SystemExit as e:
        return str(e)


# Kategori Setting -> daftar (bagian, kunci, label). Tipe diambil dari skema config mesin
# produksi, jadi aturan valid/tidaknya SATU sumber. Maksimal 6 baris per kategori.
KATEGORI_SETTING = [
    ('Produksi', [('produksi', 'batch_size', 'Objek per batch'),
                  ('produksi', 'jumlah_siklus', 'Siklus (0=semua rak)'),
                  ('produksi', 'jeda_hopper', 'Jeda hopper 0/1/2'),
                  ('produksi', 'lanjut_hopper', 'Hopper lanjut di HOME'),
                  ('produksi', 'stocker_paralel', 'Stocker paralel')]),
    ('Rak', [('rak', 'rak_random', 'Rak acak'),
             ('rak', 'pakai_sensor_rak', 'Pakai sensor rak'),
             ('rak', 'urutan_rak', 'Urutan rak')]),
    ('Delay', [('delay', 'delay_batch_s', 'Batch (detik)'),
               ('delay', 'delay_picker_s', 'Picker (detik)'),
               ('delay', 'delay_dispenser_s', 'Dispenser (detik)'),
               ('delay', 'delay_stocker_s', 'Stocker (detik)')]),
    ('Kamera', [('huskylens', 'pakai_kamera', 'Pakai kamera'),
                ('huskylens', 'jumlah_sampel', 'Jumlah sampel'),
                ('huskylens', 'batas_ambil_s', 'Batas ambil (s)'),
                ('huskylens', 'min_kosong_s', 'Min kosong (s)'),
                ('huskylens', 'id_valid', 'ID valid'),
                ('huskylens', 'id_kosong', 'ID kosong')]),
    ('Port', [('port', 'rs485_baud', 'Baud RS485'),
              ('port', 'huskylens_baud', 'Baud HuskyLens'),
              ('port', 'rs485_port', 'Port RS485'),
              ('port', 'huskylens_port', 'Port HuskyLens')]),
    ('Batas', [('batas_waktu', 't_move_package', 'MOVE_PACKAGE (s)'),
               ('batas_waktu', 't_full_cycle', 'FULL_CYCLE (s)'),
               ('batas_waktu', 't_package_ujung', 'Ke UJUNG (s)'),
               ('batas_waktu', 't_dispenser_siap', 'Dispenser siap (s)')]),
    ('Sistem', []),
]
PILIHAN_BAUD = [9600, 19200, 38400, 57600, 115200]


def jenis_setting(bagian, kunci):
    ubah = SKEMA_CONFIG[bagian][kunci.upper()]
    if kunci.endswith('_baud'):
        return 'pilih', PILIHAN_BAUD
    if ubah is _nol_satu:
        return 'pilih', [0, 1]
    if ubah is _jeda_hopper:
        return 'pilih', [0, 1, 2]
    if ubah is int:
        return 'int', [1, 5, 10]
    if ubah is float:
        return 'float', [0.1, 0.5, 1.0, 5.0]
    return 'baca', None   # port, daftar -- ubah lewat file


def nilai_sekarang(kunci):
    v = globals()[kunci.upper()]
    if isinstance(v, bool):
        return int(v)
    return v


def tampil_nilai(v):
    if isinstance(v, float):
        return f"{v:g}"
    if isinstance(v, tuple):
        return ', '.join(map(str, v))
    return str(v)


def lama_waktu(detik):
    detik = int(detik)
    if detik >= 86400:
        return f"{detik // 86400}h {detik % 86400 // 3600:02d}:{detik % 3600 // 60:02d}"
    return f"{detik // 3600:02d}:{detik % 3600 // 60:02d}:{detik % 60:02d}"


# ============================================================
# PEMANTAU NODE (thread) -- Modbus TIDAK PERNAH dibaca dari thread layar, supaya node yang
# tidak menjawab (timeout 0,5 s) tidak membekukan sentuhan.
#
# Per node hanya DUA pembacaan blok per detik: STATE+FAULT, lalu satu blok register khusus
# yang di dalamnya sudah ada MAIN_MODE_ACTIVE & MENU_ACTIVE. Bus dipakai bersama mesin
# produksi, jadi pembacaan dibuat sehemat mungkin.
# ============================================================
BLOK_NODE = {SORTER: (10, 10), PICKER: (10, 9), DISPENSER: (15, 12), STOCKER: (10, 9)}


class Pemantau(threading.Thread):
    def __init__(self, bus):
        super().__init__(daemon=True)
        self.bus = bus
        self.node = {sid: None for sid in NAMA}   # None = tidak terbaca
        self.detail_sid = None                          # node yang sedang dibuka di layar detail
        self.detail = []                                # [(label, nilai, catatan)]
        self.henti = threading.Event()

    def run(self):
        while not self.henti.wait(1.0):
            if self.bus is None:
                continue
            for sid in NAMA:
                self.node[sid] = self._baca_node(sid)
            if self.detail_sid is not None:
                self.detail = self._baca_detail(self.detail_sid)

    def _baca_node(self, sid):
        try:
            state, fault = self.bus.baca(sid, R_STATE, 2, percobaan=1)
        except Exception:
            return None
        awal, n = BLOK_NODE[sid]
        r = {}
        try:
            r = dict(zip(range(awal, awal + n), self.bus.baca(sid, awal, n, percobaan=1)))
        except RegisterTidakAda:   # firmware lama: sebagian register blok belum ada
            for reg in (R_MAIN[sid], R_MENU[sid]):
                try:
                    r[reg] = self.bus.baca(sid, reg, percobaan=1)
                except Exception:
                    pass
        except Exception:
            pass
        return {'state': state, 'fault': fault, 'main': r.get(R_MAIN[sid], 0),
                'menu': r.get(R_MENU[sid], 0), 'r': r}

    def _baca_detail(self, sid):
        nama = NAMA[sid]
        daftar = [('STATE', R_STATE), ('FAULT_CODE', R_FAULT)]
        daftar += [(n, ALAMAT[nama][n]) for n in DIAGNOSTIK_BERSAMA]
        daftar += KHUSUS_NODE[nama]
        hasil = []
        for label, reg in daftar:
            try:
                v = self.bus.baca(sid, reg, percobaan=1)
                catatan = (STATE_NAMES.get(v, '?') if label == 'STATE'
                           else catatan_untuk(nama, label, v, False).strip())
                hasil.append((label, v, catatan))
            except RegisterTidakAda:
                hasil.append((label, '-', 'tidak ada di firmware'))
            except Exception:
                hasil.append((label, '?', 'tidak menjawab'))
        return hasil


# ============================================================
# RIWAYAT ALARM -- dicatat panel dari perubahan status node & fase produksi, disimpan ke
# alarm_riwayat.json supaya tetap ada setelah Orange Pi dinyalakan ulang.
#   E = error (merah)   W = peringatan (kuning)   I = informasi (biru)
# ============================================================
FILE_ALARM = os.path.join(FOLDER, 'alarm_riwayat.json')


class Riwayat:
    MAKS = 200

    def __init__(self):
        try:
            with open(FILE_ALARM, encoding='utf-8') as f:
                self.isi = json.load(f)[-self.MAKS:]
        except (FileNotFoundError, ValueError):
            self.isi = []
        self.belum_dilihat = 0
        self._node = {}
        self._fase = STATUS['fase']

    def catat(self, tingkat, kode, pesan):
        self.isi.append({'t': time.time(), 'tingkat': tingkat, 'kode': kode, 'pesan': pesan})
        self.isi = self.isi[-self.MAKS:]
        if tingkat in ('E', 'W'):
            self.belum_dilihat += 1
        try:
            with open(FILE_ALARM + '.tmp', 'w', encoding='utf-8') as f:
                json.dump(self.isi, f)
            os.replace(FILE_ALARM + '.tmp', FILE_ALARM)
        except OSError:
            pass

    def hapus(self):
        self.isi, self.belum_dilihat = [], 0
        self.catat('I', 'I-00', 'Riwayat alarm dihapus')
        self.belum_dilihat = 0

    def periksa(self, node, berjalan):
        """Dipanggil tiap gambar: catat hanya PERUBAHAN, bukan keadaan yang sama berulang."""
        for sid, n in node.items():
            nama, lama = SINGKAT[sid], self._node.get(sid, 'awal')
            if lama == 'awal':
                self._node[sid] = n
                continue
            if n is None and lama is not None and berjalan:
                self.catat('W', f'W-{sid}0', f'{nama} tidak menjawab')
            if n is not None:
                f_lama = (lama or {}).get('fault', 0)
                if n['fault'] and n['fault'] != f_lama:
                    sebab = FAULT_SORTER.get(n['fault'], '') if sid == SORTER else ''
                    self.catat('E', f'E-{sid}{n["fault"]:02d}', f'{nama} fault {n["fault"]} {sebab}'.strip())
                if n['state'] == ST_ESTOP and (lama or {}).get('state') != ST_ESTOP:
                    self.catat('E', f'E-{sid}ES', f'{nama} E-STOP ditekan')
                if n['menu'] and not (lama or {}).get('menu'):
                    self.catat('I', f'I-{sid}M', f'{nama}: operator di menu LCD')
            self._node[sid] = n
        fase = STATUS['fase']
        if fase != self._fase:
            self._fase = fase
            alasan = STATUS.get('alasan') or STATUS.get('langkah', '')
            if fase in ('DITAHAN', 'GAGAL CEK', 'GAGAL STARTUP'):
                self.catat('E', 'E-PRD', f'{fase}: {alasan}')
            elif fase == 'JALAN':
                self.catat('I', 'I-RUN', 'Produksi berjalan')
            elif fase == 'SELESAI':
                self.catat('I', 'I-END', 'Target siklus tercapai')
            elif fase == 'BERHENTI':
                self.catat('I', 'I-STP', 'Produksi dihentikan')


# ============================================================
# INFO SISTEM (Orange Pi)
# ============================================================
_ip_cache = (0, '-')


def info_sistem():
    import shutil
    import socket
    global _ip_cache
    hasil = [('Nama', socket.gethostname())]
    if time.monotonic() - _ip_cache[0] > 10:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('10.255.255.255', 1))
            _ip_cache = (time.monotonic(), s.getsockname()[0])
            s.close()
        except OSError:
            _ip_cache = (time.monotonic(), 'tidak ada jaringan')
    hasil.append(('IP', _ip_cache[1]))
    try:
        with open('/proc/uptime') as f:
            hasil.append(('Uptime', lama_waktu(float(f.read().split()[0]))))
    except OSError:
        hasil.append(('Uptime', '-'))
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            hasil.append(('Suhu CPU', f"{int(f.read()) / 1000:.1f} C"))
    except (OSError, ValueError):
        hasil.append(('Suhu CPU', '-'))
    try:
        du = shutil.disk_usage('/')
        hasil.append(('Penyimpanan', f"{du.used * 100 // du.total}% ({du.free // 2**30} GB sisa)"))
    except OSError:
        hasil.append(('Penyimpanan', '-'))
    hasil.append(('RS485', f"{RS485_PORT} @ {RS485_BAUD}"))
    return hasil


# ============================================================
# APLIKASI
# ============================================================
TAB = [('home', 'HOME', 'rumah'), ('auto', 'AUTO', 'play'), ('node', 'NODE', 'chip'),
       ('alarm', 'ALARM', 'alarm'), ('setting', 'SETTING', 'gear')]
Y_ISI, Y_TAB = 26, 202


class Panel:
    def __init__(self, layar, sentuh, kalibrasi, bus, judul='SORTING'):
        self.layar, self.sentuh, self.kal, self.bus, self.judul = layar, sentuh, kalibrasi, bus, judul
        self.pemantau = Pemantau(bus)
        self.pemantau.start()
        self.riwayat = Riwayat()
        self.produksi = None            # thread mesin produksi
        self.tab, self.sub = 'home', None
        self.halaman = 0
        self.kategori = 0               # kategori Setting yang terbuka
        self.edit = None                # nilai yang sedang diubah (popup edit)
        self.popup = None               # dict popup konfirmasi
        self.pesan = None               # (teks, warna, sampai)
        self.node_sid = SORTER
        self.cmd = None
        self.kirim_hasil = None
        self.tombol = []
        self.kalibrasi_aktif = not kalibrasi.ada()
        self.kal_titik = [(30, 30), (290, 30), (30, 210)]
        self.kal_mentah = []
        self.log_geser = 0

    # ---------- bantu ----------
    def berjalan(self):
        return self.produksi is not None and self.produksi.is_alive()

    def ke(self, tab=None, sub=None, halaman=0):
        if tab:
            self.tab = tab
        self.sub, self.halaman = sub, halaman
        if tab == 'alarm' and sub is None:
            self.riwayat.belum_dilihat = 0
        if sub == 'detail' and self.pemantau.detail_sid != self.node_sid:
            self.pemantau.detail = []
        self.pemantau.detail_sid = self.node_sid if sub == 'detail' else None

    def mulai_kalibrasi(self):
        self.kalibrasi_aktif, self.kal_mentah = True, []

    def beri_pesan(self, teks, warna=ORANYE, detik=3.0):
        self.pesan = (teks, warna, time.monotonic() + detik)

    def tanya(self, judul, baris, aksi, label_ya='YA', warna_ya=MERAH, ikon_='tanya'):
        self.popup = {'judul': judul, 'baris': baris, 'aksi': aksi, 'ya': label_ya, 'warna': warna_ya,
                      'ikon': ikon_}

    def sorter(self, reg):
        n = self.pemantau.node.get(SORTER)
        return None if n is None else n['r'].get(reg)

    def status_node(self, sid):
        n = self.pemantau.node.get(sid)
        if n is None:
            return '--', ABU_GELAP
        if n['menu']:
            return 'MENU', ORANYE
        if n['state'] == ST_ESTOP:
            return 'ESTOP', MERAH
        if n['fault'] or n['state'] == ST_FAULT:
            return f"F{n['fault']}", MERAH
        return ('MAIN', HIJAU) if n['main'] else ('TEST', BIRU)

    # ---------- aksi produksi ----------
    def mulai(self, hanya_cek=False):
        if self.bus is None:
            self.beri_pesan("RS485 tidak bisa dibuka", MERAH)
            return
        err = muat_ulang_config()
        if err:
            self.beri_pesan("Config salah -- lihat ALARM > LOG", MERAH)
            log(err)
            return
        STATUS['alasan'] = ''
        self.produksi = threading.Thread(target=jalankan_produksi,
                                         args=(self.bus, bool(PAKAI_KAMERA), hanya_cek), daemon=True)
        self.produksi.start()

    def konfirmasi_mulai(self):
        kosong = [r for r in URUTAN_RAK if r not in rak_terisi]
        if not kosong:
            self.beri_pesan("Tidak ada rak kosong -- SETTING > Rak", MERAH)
            return
        siklus = JUMLAH_SIKLUS or len(kosong)
        self.tanya("Mulai produksi?",
                   [f"{BATCH_SIZE} objek/batch, {siklus} siklus",
                    f"Rak kosong: {', '.join(map(str, kosong))}",
                    f"Kamera: {'YA' if PAKAI_KAMERA else 'TIDAK (semua PASS)'}"],
                   self.mulai, 'START', HIJAU, 'play')

    def stop(self):
        berhenti.set()
        self.beri_pesan("Menghentikan... tunggu shutdown", KUNING, 5)

    def reset_counter(self):
        def kerja():
            try:
                self.bus.command(SORTER, OP_SORTER_RESET_COUNTERS)
                log("[panel] counter Sorter direset")
            except Exception as e:
                log(f"!! [panel] reset counter gagal: {e}")
        threading.Thread(target=kerja, daemon=True).start()
        self.beri_pesan("Counter direset", HIJAU, 1.5)

    def kirim_command(self):
        sid, (label, op, arg, gate, kelompok) = self.node_sid, self.cmd
        nilai = arg if isinstance(arg, int) else (self.edit['nilai'] if self.edit else 0)
        self.kirim_hasil = ('mengirim', f"{label} ...")

        def kerja():
            try:
                seq = self.bus.command(sid, op, nilai)
                batas = time.monotonic() + T_ACK
                while time.monotonic() < batas:
                    try:
                        if self.bus.baca(sid, R_ACK, percobaan=1) == seq:
                            self.kirim_hasil = ('ok', f"{label}: DITERIMA (ack). Cek efeknya.")
                            log(f"[panel] {NAMA[sid]} <- {label} (arg {nilai}) di-ack")
                            return
                    except Exception:
                        pass
                    time.sleep(0.1)
                self.kirim_hasil = ('gagal', f"{label}: TIDAK di-ack -- node mati / di menu LCD")
            except Exception as e:
                self.kirim_hasil = ('gagal', f"{label}: gagal kirim ({e})")
        threading.Thread(target=kerja, daemon=True).start()

    # ---------- sentuhan ----------
    def tekan(self, p):
        for t in self.tombol:
            if t.kena(p):
                t.aksi()
                return

    def tekan_kalibrasi(self, mentah):
        self.kal_mentah.append(mentah)
        if len(self.kal_mentah) == 3:
            data = Kalibrasi.hitung(self.kal_titik, self.kal_mentah)
            if data is None:
                self.kal_mentah = []
                self.beri_pesan("Kalibrasi gagal, ulangi", MERAH)
                return
            self.kal.data = data
            self.kal.simpan()
            self.kalibrasi_aktif = False
            self.beri_pesan("Kalibrasi sentuh tersimpan", HIJAU)

    # ---------- gambar ----------
    def gambar(self):
        self.riwayat.periksa(self.pemantau.node, self.berjalan())
        img = Image.new('RGB', (LEBAR, TINGGI), LATAR)
        d = ImageDraw.Draw(img)
        self.tombol = []
        if self.kalibrasi_aktif:
            self._kalibrasi(d)
        else:
            self._header(d)
            getattr(self, '_' + (self.sub or self.tab))(d)
            self._tab_bar(d)
            if self.popup or self.edit_popup():
                img = self._redup(img)
                d = ImageDraw.Draw(img)
                self.tombol = []
                if self.popup:
                    self._popup(d)
                else:
                    self._popup_edit(d)
        if self.pesan and time.monotonic() < self.pesan[2]:
            teks = potong(d, self.pesan[0], F_TEBAL, 290)
            w = int(d.textlength(teks, font=F_TEBAL)) + 24
            kotak(d, (160 - w // 2, 172, 160 + w // 2, 196), 12, self.pesan[1])
            teks_tengah(d, 160, 184, teks, F_TEBAL, PUTIH)
        return img

    def edit_popup(self):
        return self.edit is not None and self.edit.get('popup')

    @staticmethod
    def _redup(img):
        gelap = Image.new('RGB', img.size, HITAM)
        return Image.blend(img, gelap, 0.6)

    def _header(self, d):
        d.rectangle((0, 0, LEBAR, 23), fill=KARTU)
        ikon(d, 'logo', 13, 12, 9)
        d.text((27, 4), self.judul, font=F_JUDUL, fill=PUTIH)
        sekarang = time.localtime()
        teks_kanan(d, 316, 5, time.strftime('%H:%M:%S', sekarang), F_TEBAL, PUTIH)
        tanggal = f"{sekarang.tm_mday} {BULAN[sekarang.tm_mon - 1]} {sekarang.tm_year}"
        teks_kanan(d, 256, 7, tanggal, F_MINI, ABU)

    def _tab_bar(self, d):
        d.rectangle((0, Y_TAB - 2, LEBAR, TINGGI), fill=KARTU)
        w = LEBAR // len(TAB)
        for i, (kunci, label, ik) in enumerate(TAB):
            x = i * w
            aktif = self.tab == kunci
            if aktif:
                kotak(d, (x + 2, Y_TAB, x + w - 3, TINGGI - 2), 6, BIRU)
            warna = PUTIH if aktif else ABU
            ikon(d, ik, x + w // 2, Y_TAB + 12, 7, warna)
            teks_tengah(d, x + w // 2, Y_TAB + 29, label, F_MINI, warna)
            if kunci == 'alarm' and self.riwayat.belum_dilihat and not aktif:
                d.ellipse((x + w // 2 + 8, Y_TAB + 2, x + w // 2 + 16, Y_TAB + 10), fill=MERAH)
            self.tombol.append(Tombol(x, Y_TAB - 2, w, TINGGI - Y_TAB + 2, '',
                                      lambda k=kunci: self.ke(k)))

    def _judul_isi(self, d, ik, teks, kembali=None):
        """Baris judul di dalam isi: ikon + teks, dan tombol kembali kalau sub-layar."""
        x = 6
        if kembali:
            t = Tombol(4, Y_ISI + 2, 30, 22, '', kembali, KARTU2)
            t.gambar(d)
            ikon(d, 'kembali', 19, Y_ISI + 13, 5)
            self.tombol.append(t)
            x = 40
        if ik:
            ikon(d, ik, x + 8, Y_ISI + 13, 7)
            x += 20
        d.text((x, Y_ISI + 6), teks, font=F_JUDUL, fill=PUTIH)

    # ---------- HOME ----------
    def _home(self, d):
        fase = STATUS['fase']
        warna, b1, b2, ik, _ = FASE.get(fase, FASE['SIAP'])
        teks_w = HITAM if warna == KUNING else PUTIH
        kotak(d, (4, 28, 122, 128), 8, warna)
        ikon(d, ik, 63, 58, 16, teks_w)
        teks_tengah(d, 63, 95, b1, F_TEBAL, teks_w)
        teks_tengah(d, 63, 112, b2, F_JUDUL, teks_w)
        self.tombol.append(Tombol(4, 28, 118, 100, '', lambda: self.ke('auto')))

        ok, rej = self.sorter(10), self.sorter(11)
        total = None if ok is None or rej is None else ok + rej
        kotak(d, (126, 28, 316, 74), 8, KARTU)
        d.text((136, 33), 'TOTAL COUNT', font=F_KECIL, fill=ABU)
        d.text((136, 46), '-' if total is None else f"{total:,}", font=F_ANGKA, fill=PUTIH)
        ikon(d, 'grafik', 294, 52, 11, BIRU)
        for x0, label, nilai, w, ik2 in ((126, 'OK', ok, HIJAU, 'cek'), (223, 'REJECT', rej, MERAH, 'silang')):
            kotak(d, (x0, 78, x0 + 93, 128), 8, KARTU)
            d.ellipse((x0 + 7, 86, x0 + 27, 106), fill=w)
            ikon(d, ik2, x0 + 17, 96, 6)
            d.text((x0 + 33, 84), label, font=F_KECIL, fill=ABU)
            d.text((x0 + 33, 99), '-' if nilai is None else f"{nilai:,}", font=_font(17, True), fill=PUTIH)

        kotak(d, (4, 132, 316, 198), 8, KARTU)
        for i, sid in enumerate(ALUR):
            cx = 42 + i * 78
            teks, w = self.status_node(sid)
            ikon(d, IKON_NODE[sid], cx, 150, 10)
            teks_tengah(d, cx, 170, SINGKAT[sid], F_MINI, PUTIH)
            kotak(d, (cx - 26, 178, cx + 26, 193), 7, w)
            teks_tengah(d, cx, 185, teks, F_MINI, PUTIH)
            if i < len(ALUR) - 1:
                d.polygon([(cx + 33, 147), (cx + 33, 153), (cx + 39, 150)], fill=ABU)
            self.tombol.append(Tombol(cx - 38, 134, 76, 62, '', lambda s=sid: self._buka_node(s)))

    def _buka_node(self, sid):
        self.node_sid = sid
        self.ke('node', 'detail')

    # ---------- AUTO ----------
    def _auto(self, d):
        fase = STATUS['fase']
        warna, _, _, _, pendek = FASE.get(fase, FASE['SIAP'])
        d.ellipse((6, 30, 26, 50), fill=HIJAU if fase == 'JALAN' else KARTU2)
        ikon(d, 'play', 17, 40, 6)
        d.text((32, 33), 'AUTO MODE', font=F_JUDUL, fill=PUTIH)
        kotak(d, (196, 30, 316, 50), 10, warna)
        teks_tengah(d, 256, 40, pendek, F_TEBAL, HITAM if warna == KUNING else PUTIH)

        s = STATUS
        kosong = len([r for r in URUTAN_RAK if r not in rak_terisi])
        target = s['target'] or JUMLAH_SIKLUS or kosong
        kotak(d, (4, 54, 196, 140), 8, KARTU)
        d.text((12, 58), 'PRODUKSI', font=F_TEBAL, fill=PUTIH)
        for y, label, a, b, w in ((74, 'Siklus', s['batch'], target, BIRU),
                                  (102, 'Batch (objek)', s['pass_batch'], BATCH_SIZE, HIJAU)):
            d.text((12, y), label, font=F_KECIL, fill=ABU)
            teks_kanan(d, 188, y, f"{a} / {b}", F_TEBAL, PUTIH)
            bar(d, 12, y + 15, 176, 7, a / b if b else 0, w)
        d.text((12, 126), potong(d, f"Rak {s['rak'] or '-'}   Kamera {'YA' if PAKAI_KAMERA else 'TIDAK'}",
                                 F_KECIL, 180), font=F_KECIL, fill=ABU)

        ok, rej = self.sorter(10), self.sorter(11)
        kotak(d, (200, 54, 316, 140), 8, KARTU)
        d.text((208, 58), 'STATISTIK', font=F_TEBAL, fill=PUTIH)
        total = None if ok is None or rej is None else ok + rej
        rate = '-' if not total else f"{ok * 100 / total:.1f}%"
        for i, (label, nilai, w) in enumerate((('Total', total, PUTIH), ('OK', ok, HIJAU),
                                               ('Reject', rej, MERAH), ('Rate', rate, PUTIH))):
            y = 76 + i * 15
            d.text((208, y), label, font=F_KECIL, fill=ABU)
            teks_kanan(d, 309, y, '-' if nilai is None else (f"{nilai:,}" if isinstance(nilai, int) else nilai),
                       F_TEBAL, w)

        jalan = self.berjalan()
        tombol = [
            ('START', 'play', HIJAU, not jalan and self.bus is not None, self.konfirmasi_mulai),
            ('STOP', 'stop', MERAH, jalan,
             lambda: self.tanya("Hentikan produksi?", ["Sorter berhenti dulu, lalu", "semua node ke mode TEST."],
                                self.stop, 'STOP')),
            ('CEK', 'cek', BIRU, not jalan and self.bus is not None,
             lambda: self.tanya("Cek awal saja?", ["Memeriksa node, rak & kamera.", "Tidak ada yang bergerak."],
                                lambda: self.mulai(hanya_cek=True), 'CEK', BIRU, 'info')),
            ('RESET', 'ulang', KARTU2, not jalan and self.bus is not None,
             lambda: self.tanya("Reset counter?", ["PASS & REJECT Sorter", "kembali ke 0."], self.reset_counter,
                                'RESET')),
        ]
        for i, (label, ik, w, aktif, aksi) in enumerate(tombol):
            t = Tombol(4 + i * 79, 144, 75, 54, label, aksi, w if aktif else KARTU, aktif, ikon_=ik)
            t.gambar(d)
            self.tombol.append(t)

    # ---------- NODE ----------
    def _ringkas_node(self, sid):
        n = self.pemantau.node.get(sid)
        if n is None:
            return ['Tidak menjawab', 'Cek daya & kabel RS485']
        r, nama = n['r'], NAMA[sid]
        akt = lambda reg: ACTIVITY_NAMES.get(nama, {}).get(r.get(reg), '-')
        if sid == SORTER:
            return [f"Pass {r.get(10, '-')}  Reject {r.get(11, '-')}", f"Missed {r.get(19, '-')}",
                    f"Akt: {akt(13)}"]
        if sid == PICKER:
            g = r.get(17)
            gerak = '-' if g in (None, 0xFF) else catatan_untuk(nama, 'GERAKAN_AKTIF', g, False).strip()
            return [f"Pose {catatan_untuk(nama, 'CURRENT_POSE', r.get(10, -1), False).strip() or '-'}",
                    f"Gerakan {gerak}", f"Akt: {akt(11)}"]
        if sid == DISPENSER:
            tahap = r.get(26)
            nama_tahap = PIPELINE_NAMES[tahap] if tahap is not None and tahap < len(PIPELINE_NAMES) else '-'
            ada = lambda v: 'ADA' if v else '--'
            return [f"{nama_tahap}", f"Tengah {ada(r.get(16))}  Ujung {ada(r.get(17))}",
                    f"Siap isi: {'YA' if r.get(25) else 'tidak'}"]
        rak = r.get(10)
        return [f"Homing: {'OK' if r.get(11) else 'BELUM'}",
                f"Posisi rak: {'-' if rak in (None, 0xFF) else rak}", f"Akt: {akt(13)}"]

    def _node(self, d):
        for i, sid in enumerate(ALUR):
            x, y = 4 + (i % 2) * 158, 28 + (i // 2) * 86
            kotak(d, (x, y, x + 154, y + 82), 8, KARTU)
            ikon(d, IKON_NODE[sid], x + 14, y + 13, 7)
            d.text((x + 26, y + 6), SINGKAT[sid], font=F_TEBAL, fill=PUTIH)
            teks, w = self.status_node(sid)
            kotak(d, (x + 104, y + 5, x + 149, y + 20), 7, w)
            teks_tengah(d, x + 126, y + 12, teks, F_MINI, PUTIH)
            for j, baris in enumerate(self._ringkas_node(sid)):
                d.text((x + 8, y + 27 + j * 16), potong(d, baris, F_KECIL, 140), font=F_KECIL, fill=ABU)
            self.tombol.append(Tombol(x, y, 154, 82, '', lambda s=sid: self._buka_node(s)))

    def _detail(self, d):
        nama = NAMA[self.node_sid]
        teks, w = self.status_node(self.node_sid)
        self._judul_isi(d, IKON_NODE[self.node_sid], nama, lambda: self.ke('node'))
        kotak(d, (250, 29, 316, 47), 8, w)
        teks_tengah(d, 283, 38, teks, F_TEBAL, PUTIH)
        isi = self.pemantau.detail or [('membaca...', '', '')]
        per = 6
        hal_maks = (len(isi) - 1) // per
        hal = min(self.halaman, hal_maks)
        kotak(d, (4, 52, 316, 172), 8, KARTU)
        for i, (label, v, cat) in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 56 + i * 19
            d.text((10, y), potong(d, label, F_KECIL, 130), font=F_KECIL, fill=ABU)
            d.text((145, y - 1), str(v), font=F_TEBAL, fill=PUTIH)
            if cat:
                d.text((195, y), potong(d, cat, F_KECIL, 117), font=F_KECIL, fill=KUNING)
        for i, (teks_t, aksi, aktif, w2) in enumerate((
                ('<', lambda: self.ke('node', 'detail', max(0, hal - 1)), hal > 0, KARTU2),
                ('>', lambda: self.ke('node', 'detail', min(hal_maks, hal + 1)), hal < hal_maks, KARTU2),
                ('PERINTAH', lambda: self.ke('node', 'perintah'), not self.berjalan() and self.bus is not None, BIRU))):
            lebar = 50 if i < 2 else 204
            x = 4 + i * 54
            t = Tombol(x, 176, lebar, 22, teks_t, aksi, w2, aktif)
            t.gambar(d)
            self.tombol.append(t)

    def _perintah(self, d):
        nama = NAMA[self.node_sid]
        daftar = [c for c in OPCODES[nama] if c[1] is not None]   # 'lokal' = hanya di panel ESP32
        per = 5
        hal_maks = (len(daftar) - 1) // per
        self._judul_isi(d, None, f"PERINTAH {nama}", lambda: self.ke('node', 'detail'))
        teks_kanan(d, 312, Y_ISI + 7, f"{self.halaman + 1}/{hal_maks + 1}", F_KECIL, ABU)
        n = self.pemantau.node.get(self.node_sid)
        main = bool(n and n['main'])
        for i, c in enumerate(daftar[self.halaman * per:(self.halaman + 1) * per]):
            label, op, arg, gate, kelompok = c
            y = 52 + i * 25
            salah_mode = (gate == 'main' and not main) or (gate == 'test' and main)

            def pilih(c=c):
                self.cmd, self.kirim_hasil = c, None
                label, op, arg, gate, _ = c
                if isinstance(arg, str):   # argumen ditanyakan -- popup angka dulu
                    self.edit = {'popup': True, 'bagian': '', 'kunci': label, 'label': label, 'catatan': arg,
                                 'jenis': 'int', 'pilihan': [1, 10, 100], 'nilai': 0, 'langkah': 0,
                                 'simpan': lambda: self.ke('node', 'kirim')}
                else:
                    self.edit = None
                    self.ke('node', 'kirim')
            t = Tombol(4, y, 312, 22, '', pilih, KARTU)
            t.gambar(d)
            self.tombol.append(t)
            d.text((12, y + 4), label, font=F_TEBAL, fill=ABU if salah_mode else PUTIH)
            tag = {'main': 'MAIN', 'test': 'TEST'}.get(gate, '')
            if tag:
                teks_kanan(d, 308, y + 5, tag, F_KECIL, ORANYE if salah_mode else ABU)
        for i, (teks_t, aksi, aktif) in enumerate((
                ('<', lambda: self.ke('node', 'perintah', max(0, self.halaman - 1)), self.halaman > 0),
                ('>', lambda: self.ke('node', 'perintah', min(hal_maks, self.halaman + 1)), self.halaman < hal_maks))):
            t = Tombol(4 + i * 158, 178, 154, 20, teks_t, aksi, KARTU2, aktif)
            t.gambar(d)
            self.tombol.append(t)

    def _kirim(self, d):
        label, op, arg, gate, kelompok = self.cmd
        nama = NAMA[self.node_sid]
        nilai = arg if isinstance(arg, int) else (self.edit['nilai'] if self.edit else 0)
        self._judul_isi(d, None, f"KIRIM KE {nama}", lambda: self.ke('node', 'perintah'))
        n = self.pemantau.node.get(self.node_sid)
        main = bool(n and n['main'])
        kotak(d, (4, 52, 316, 110), 8, KARTU)
        d.text((12, 57), label, font=F_JUDUL, fill=PUTIH)
        d.text((12, 75), f"opcode {op}   arg {nilai}", font=F_KECIL, fill=ABU)
        peringatan = None
        if gate == 'main' and not main:
            peringatan = "Node belum MAIN -- akan DITOLAK"
        elif gate == 'test' and main:
            peringatan = "Node sedang MAIN -- akan DITOLAK"
        if n and n['menu']:
            peringatan = "Operator di menu LCD -- DIABAIKAN"
        if peringatan:
            d.text((12, 91), peringatan, font=F_TEBAL, fill=ORANYE)
        if self.kirim_hasil:
            jenis, teks = self.kirim_hasil
            w = {'ok': HIJAU, 'gagal': MERAH}.get(jenis, KUNING)
            kotak(d, (4, 114, 316, 146), 8, w)
            teks_tengah(d, 160, 130, potong(d, teks, F_TEBAL, 300), F_TEBAL, HITAM if w == KUNING else PUTIH)
        sibuk = bool(self.kirim_hasil and self.kirim_hasil[0] == 'mengirim')
        t = Tombol(4, 152, 312, 46, 'KIRIM', self.kirim_command, MERAH, not sibuk and not self.berjalan())
        t.gambar(d)
        self.tombol.append(t)

    # ---------- ALARM ----------
    def _alarm(self, d):
        ikon(d, 'alarm', 14, Y_ISI + 13, 8, MERAH)
        d.text((28, Y_ISI + 6), 'ALARM', font=F_JUDUL, fill=PUTIH)
        isi = list(reversed(self.riwayat.isi))
        per = 6
        hal_maks = max(0, (len(isi) - 1) // per)
        hal = min(self.halaman, hal_maks)
        for i, (teks_t, aksi, aktif, w, lebar, x) in enumerate((
                ('<', lambda: self.ke('alarm', None, max(0, hal - 1)), hal > 0, KARTU2, 30, 112),
                ('>', lambda: self.ke('alarm', None, min(hal_maks, hal + 1)), hal < hal_maks, KARTU2, 30, 146),
                ('LOG', lambda: self.ke('alarm', 'log'), True, KARTU2, 50, 180),
                ('HAPUS', lambda: self.tanya("Hapus riwayat?", ["Semua catatan alarm dihapus."],
                                             self.riwayat.hapus, 'HAPUS'), bool(isi), KARTU2, 82, 234))):
            t = Tombol(x, Y_ISI + 2, lebar, 22, teks_t, aksi, w, aktif)
            t.gambar(d)
            self.tombol.append(t)
        kotak(d, (4, 52, 316, 198), 8, KARTU)
        if not isi:
            teks_tengah(d, 160, 125, 'Belum ada alarm', F_BIASA, ABU)
        warna = {'E': MERAH, 'W': KUNING, 'I': BIRU}
        simbol = {'E': 'silang', 'W': 'alarm', 'I': 'info'}
        for i, a in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 56 + i * 23
            w = warna.get(a['tingkat'], ABU)
            d.ellipse((9, y + 3, 23, y + 17), fill=w)
            ikon(d, simbol.get(a['tingkat'], 'info'), 16, y + 10, 4, HITAM if w == KUNING else PUTIH)
            d.text((28, y + 4), time.strftime('%d/%m %H:%M', time.localtime(a['t'])), font=F_KECIL, fill=ABU)
            d.text((106, y + 3), a['kode'], font=F_TEBAL, fill=w)
            d.text((150, y + 4), potong(d, a['pesan'], F_KECIL, 162), font=F_KECIL, fill=PUTIH)
            if i < per - 1:
                d.line([(10, y + 22), (310, y + 22)], fill=GARIS)

    def _log(self, d):
        self._judul_isi(d, 'daftar', 'LOG', lambda: (setattr(self, 'log_geser', 0), self.ke('alarm')))
        semua = list(LOG_BARU)
        per = 11
        akhir = max(0, len(semua) - self.log_geser)
        kotak(d, (4, 52, 316, 198), 8, KARTU)
        for i, b in enumerate(semua[max(0, akhir - per):akhir]):
            d.text((8, 55 + i * 13), potong(d, b[9:], F_MINI, 304), font=F_MINI,
                   fill=MERAH if '!!' in b else PUTIH)

        def naik():
            self.log_geser = min(self.log_geser + per, max(0, len(semua) - per))

        def turun():
            self.log_geser = max(0, self.log_geser - per)
        for i, (teks_t, aksi) in enumerate((('NAIK', naik), ('TURUN', turun))):
            t = Tombol(178 + i * 70, Y_ISI + 2, 66, 22, teks_t, aksi, KARTU2)
            t.gambar(d)
            self.tombol.append(t)

    # ---------- SETTING ----------
    def _setting(self, d):
        for i, (nama, _) in enumerate(KATEGORI_SETTING):
            y = 28 + i * 24
            aktif = i == self.kategori
            t = Tombol(4, y, 86, 22, nama, lambda i=i: setattr(self, 'kategori', i), BIRU if aktif else KARTU,
                       font=F_TEBAL)
            t.gambar(d)
            self.tombol.append(t)
        nama, daftar = KATEGORI_SETTING[self.kategori]
        kotak(d, (94, 28, 316, 198), 8, KARTU)
        d.text((102, 32), nama.upper(), font=F_TEBAL, fill=PUTIH)
        if nama == 'Sistem':
            for i, (k, v) in enumerate(info_sistem()):
                y = 50 + i * 17
                d.text((102, y), k, font=F_KECIL, fill=ABU)
                d.text((175, y), potong(d, v, F_KECIL, 136), font=F_KECIL, fill=PUTIH)
            t = Tombol(100, 168, 210, 26, 'KALIBRASI SENTUH', self.mulai_kalibrasi, BIRU)
            t.gambar(d)
            self.tombol.append(t)
            return
        for i, (bagian, kunci, label) in enumerate(daftar):
            y = 50 + i * 24
            jenis, _ = jenis_setting(bagian, kunci)
            nilai = tampil_nilai(nilai_sekarang(kunci))
            d.text((102, y + 5), potong(d, label, F_KECIL, 132), font=F_KECIL,
                   fill=ABU if jenis == 'baca' else PUTIH)
            kotak(d, (238, y + 1, 310, y + 20), 5, KARTU2 if jenis != 'baca' else KARTU,
                  outline=GARIS if jenis != 'baca' else None)
            teks_tengah(d, 274, y + 10, potong(d, nilai, F_TEBAL, 68), F_TEBAL,
                        KUNING if jenis != 'baca' else ABU)
            self.tombol.append(Tombol(98, y, 216, 22, '', lambda b=bagian, k=kunci, l=label: self._buka_edit(b, k, l)))
        if nama == 'Rak':
            t = Tombol(100, 168, 210, 26, 'KELOLA RAK', lambda: self.ke('setting', 'rak'), BIRU)
            t.gambar(d)
            self.tombol.append(t)

    def _buka_edit(self, bagian, kunci, label):
        if self.berjalan():
            self.beri_pesan("Hentikan produksi dulu", ORANYE)
            return
        jenis, pilihan = jenis_setting(bagian, kunci)
        if jenis == 'baca':
            self.beri_pesan("Ubah lewat file sorting.config", ORANYE)
            return
        self.edit = {'popup': True, 'bagian': bagian, 'kunci': kunci, 'label': label, 'catatan': f"[{bagian}] {kunci}",
                     'jenis': jenis, 'pilihan': pilihan, 'nilai': nilai_sekarang(kunci), 'langkah': 0,
                     'simpan': self._simpan_setting}

    def _simpan_setting(self):
        e = self.edit
        lama = nilai_sekarang(e['kunci'])
        try:
            tulis_config(e['bagian'], e['kunci'], tampil_nilai(e['nilai']))
        except (OSError, ValueError) as ex:
            self.beri_pesan(f"Gagal simpan: {ex}", MERAH)
            return
        err = muat_ulang_config()
        if err:
            tulis_config(e['bagian'], e['kunci'], tampil_nilai(lama))   # kembalikan
            muat_ulang_config()
            self.beri_pesan("Nilai ditolak config -- dibatalkan", MERAH)
            return
        log(f"[panel] {e['kunci']} = {tampil_nilai(e['nilai'])} (dari LCD)")
        self.edit = None
        self.beri_pesan("Tersimpan", HIJAU, 1.5)

    def _rak(self, d):
        self._judul_isi(d, 'rak', 'KELOLA RAK', lambda: self.ke('setting'))
        d.text((10, 54), 'Sentuh rak untuk ubah tanda KOSONG / TERISI', font=F_KECIL, fill=ABU)
        for i, r in enumerate((1, 2, 3, 4)):
            terisi = r in rak_terisi
            x = 4 + i * 79

            def ubah(r=r, terisi=terisi):
                if self.berjalan():
                    self.beri_pesan("Hentikan produksi dulu", ORANYE)
                    return

                def lakukan():
                    (rak_terisi.discard if terisi else rak_terisi.add)(r)
                    simpan_rak_terisi()
                    log(f"[panel] Rak {r} ditandai {'KOSONG' if terisi else 'TERISI'}")
                self.tanya(f"Rak {r} -> {'KOSONG' if terisi else 'TERISI'}?",
                           ["Pastikan sesuai kondisi fisik rak."], lakukan, 'UBAH', BIRU)
            t = Tombol(x, 70, 75, 88, '', ubah, ORANYE if terisi else HIJAU)
            t.gambar(d)
            self.tombol.append(t)
            ikon(d, 'kotak' if terisi else 'rak', x + 38, 92, 10)
            teks_tengah(d, x + 38, 118, f"RAK {r}", F_TEBAL, PUTIH)
            teks_tengah(d, x + 38, 135, "TERISI" if terisi else "KOSONG", F_KECIL, PUTIH)
            if r not in URUTAN_RAK:
                teks_tengah(d, x + 38, 149, "(tdk dipakai)", F_MINI, PUTIH)

        def semua():
            if self.berjalan():
                self.beri_pesan("Hentikan produksi dulu", ORANYE)
                return

            def lakukan():
                rak_terisi.clear()
                simpan_rak_terisi()
                log("[panel] Semua rak ditandai KOSONG")
            self.tanya("Kosongkan semua rak?", ["Pastikan rak sudah", "dikosongkan secara fisik."], lakukan,
                       'KOSONGKAN', ORANYE)
        t = Tombol(4, 164, 312, 34, 'SEMUA KOSONG', semua, KARTU2)
        t.gambar(d)
        self.tombol.append(t)

    # ---------- POPUP ----------
    def _popup(self, d):
        p = self.popup
        kotak(d, (34, 22, 286, 218), 12, KARTU, outline=GARIS)
        ikon(d, p['ikon'], 160, 46, 14, KUNING if p['ikon'] in ('tanya', 'info') else PUTIH)
        teks_tengah(d, 160, 76, potong(d, p['judul'], F_JUDUL, 240), F_JUDUL, PUTIH)
        for i, b in enumerate(p['baris']):
            teks_tengah(d, 160, 98 + i * 16, potong(d, b, F_KECIL, 240), F_KECIL, ABU)

        def ya():
            self.popup = None
            p['aksi']()

        def batal():
            self.popup = None
        for x, teks_t, aksi, w in ((44, 'BATAL', batal, ABU_GELAP), (164, p['ya'], ya, p['warna'])):
            t = Tombol(x, 168, 112, 40, teks_t, aksi, w)
            t.gambar(d)
            self.tombol.append(t)

    def _popup_edit(self, d):
        e = self.edit
        kotak(d, (14, 18, 306, 222), 12, KARTU, outline=GARIS)
        teks_tengah(d, 160, 32, potong(d, e['label'], F_JUDUL, 280), F_JUDUL, PUTIH)
        teks_tengah(d, 160, 48, potong(d, e['catatan'], F_KECIL, 280), F_KECIL, ABU)
        teks_tengah(d, 160, 80, tampil_nilai(e['nilai']), F_BESAR, KUNING)
        if e['jenis'] == 'pilih':
            pil = e['pilihan']
            lebar = (276 - 4 * (len(pil) - 1)) // len(pil)
            for i, v in enumerate(pil):
                t = Tombol(22 + i * (lebar + 4), 108, lebar, 40, str(v), lambda v=v: e.update(nilai=v),
                           BIRU if v == e['nilai'] else KARTU2)
                t.gambar(d)
                self.tombol.append(t)
        else:
            langkah = e['pilihan'][e['langkah']]
            minimum = 1 if e['kunci'] == 'batch_size' else 0

            def ubah(arah):
                v = e['nilai'] + arah * langkah
                v = round(v, 2) if e['jenis'] == 'float' else int(v)
                e['nilai'] = min(65535, max(minimum, v))
            for i, (teks_t, aksi) in enumerate(((f"- {langkah:g}", lambda: ubah(-1)),
                                                (f"langkah {langkah:g}",
                                                 lambda: e.update(langkah=(e['langkah'] + 1) % len(e['pilihan']))),
                                                (f"+ {langkah:g}", lambda: ubah(1)))):
                t = Tombol(22 + i * 94, 108, 90, 40, teks_t, aksi, KARTU2)
                t.gambar(d)
                self.tombol.append(t)

        def batal():
            self.edit = None

        def simpan():
            e['popup'] = False
            e['simpan']()
        t = Tombol(22, 166, 134, 46, 'BATAL', batal, ABU_GELAP)
        t.gambar(d)
        self.tombol.append(t)
        t = Tombol(164, 166, 134, 46, 'SIMPAN' if e.get('bagian') else 'LANJUT', simpan, HIJAU)
        t.gambar(d)
        self.tombol.append(t)

    # ---------- KALIBRASI SENTUH ----------
    def _kalibrasi(self, d):
        n = len(self.kal_mentah)
        teks_tengah(d, LEBAR // 2, 105, "KALIBRASI SENTUH", _font(20, True), PUTIH)
        teks_tengah(d, LEBAR // 2, 135, f"Sentuh tepat di tengah tanda + ({n + 1}/3)", F_BIASA, ABU)
        x, y = self.kal_titik[min(n, 2)]
        d.line((x - 12, y, x + 12, y), fill=MERAH, width=2)
        d.line((x, y - 12, x, y + 12), fill=MERAH, width=2)
        d.ellipse((x - 4, y - 4, x + 4, y + 4), outline=PUTIH)


# ============================================================
# LOOP UTAMA
# ============================================================
def jalankan(panel, layar, sentuh):
    ditekan_sejak = None
    terakhir_gambar = 0
    perlu = True
    while not BERHENTI_PANEL.is_set():
        mentah = sentuh.mentah() if sentuh else None
        sekarang = time.monotonic()
        if mentah is not None:
            if ditekan_sejak is None:
                ditekan_sejak = sekarang
                time.sleep(0.03)                       # biarkan tekanan stabil dulu
                mentah = sentuh.mentah() or mentah
                if panel.kalibrasi_aktif:
                    panel.tekan_kalibrasi(mentah)
                elif panel.kal.ada():
                    panel.tekan(panel.kal.ke_layar(mentah))
                perlu = True
            elif sekarang - ditekan_sejak > 5 and not panel.kalibrasi_aktif:
                panel.mulai_kalibrasi()                # jalan keluar kalau kalibrasi rusak
                perlu = True
        else:
            ditekan_sejak = None
        if perlu or sekarang - terakhir_gambar > 0.5:
            layar.tampilkan(panel.gambar())
            terakhir_gambar, perlu = sekarang, False
        time.sleep(0.03)


BERHENTI_PANEL = threading.Event()


def jalankan_panel(kalibrasi=False, judul_tambahan=''):
    """Mode utama: panel LCD sentuh. Produksi dimulai/dihentikan dari layar."""
    if not ADA_PIL:
        sys.exit("ERROR: panel butuh Pillow & numpy:  apt install python3-pil python3-numpy fonts-dejavu-core")
    k = muat_config_lcd()
    gpio = Gpio()
    layar = LayarILI9341(k, gpio)
    sentuh = SentuhXPT2046(k, gpio, layar.spi)
    kal = Kalibrasi(None) if kalibrasi else Kalibrasi.muat()
    try:
        bus = Bus()
    except Exception as e:
        log(f"!! RS485 {RS485_PORT} tidak bisa dibuka: {e}")
        bus = None
    panel = Panel(layar, sentuh, kal, bus, (k.get('lcd_judul') or 'SORTING') + judul_tambahan)

    def keluar(*_):
        BERHENTI_PANEL.set()
    signal.signal(signal.SIGTERM, keluar)
    signal.signal(signal.SIGINT, keluar)
    try:
        jalankan(panel, layar, sentuh)
    finally:
        if panel.berjalan():
            log("[panel] program ditutup -- produksi dihentikan")
            berhenti.set()
            panel.produksi.join(timeout=60)
        img = Image.new('RGB', (LEBAR, TINGGI), HITAM)
        teks_tengah(ImageDraw.Draw(img), LEBAR // 2, TINGGI // 2, "Panel berhenti", F_TEBAL, ABU)
        layar.tampilkan(img)
