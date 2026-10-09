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
  assets/            JPEG 320x240 untuk HMI (splash, loading, system check, latar);
                     opsional -- tanpa folder ini layar memakai latar polos
  rak_terisi.txt, alarm_riwayat.json, touch_kalibrasi.json,
  resep.json, produksi_riwayat.json                           -- dibuat otomatis

Isi file, berurutan:
  1. KONFIGURASI      memuat sorting.config (+ warna display.config)
  2. MESIN PRODUKSI   Bus Modbus, pemeriksaan awal, HuskyLens, startup, batch, rak, shutdown
  3. TABEL NODE       command & register tiap node (ACUAN, sama dengan panel ESP32)
  4. MENU UJI         menu terminal per node (--uji)
  5. LAYAR            driver ILI9341 + XPT2046, HMI SECURA (boot, menu, resep, riwayat), jalankan_panel()

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


def _opsi_font():
    """[tampilan] font_tajam di display.config (opsional, bawaan 1)."""
    if not os.path.exists(FILE_DISPLAY):
        return 1
    cp = configparser.ConfigParser(inline_comment_prefixes=('#',))
    cp.read(FILE_DISPLAY, encoding='utf-8-sig')
    nilai = cp.get('tampilan', 'font_tajam', fallback='1').strip()
    if nilai not in ('0', '1'):
        sys.exit(f"ERROR di {FILE_DISPLAY}: [tampilan] font_tajam = {nilai!r} -- tulis 0 atau 1")
    return int(nilai)


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
#   indikator  LED, buzzer & SW1 (INDIKATOR, hidup sepanjang program). Bunyi tidak menunda kamera.
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
    'pin': {'LED_OPR': _pin, 'LED_RUN': _pin, 'LED_ALARM': _pin, 'BUZZER': _pin, 'SW1': _pin},
    'indikator': {'BUZZER_VALID': _nol_satu, 'BUZZER_INVALID': _nol_satu, 'BUZZER_ALARM': _nol_satu,
                  'BUZZER_BOOT': _nol_satu, 'BUNYI_PENDEK_MS': int, 'JEDA_BUNYI_MS': int,
                  'ALARM_NYALA_S': float, 'ALARM_MATI_S': float, 'HEARTBEAT_PERIODE_S': float,
                  'HEARTBEAT_NYALA_S': float, 'HEARTBEAT_BATAS_S': float, 'SW1_TAHAN_S': float},
    'huskylens': {'PAKAI_KAMERA': _nol_satu, 'JUMLAH_SAMPEL': int, 'INTERVAL_SAMPEL_S': float, 'BATAS_AMBIL_S': float,
                  'ID_KOSONG': int, 'ID_VALID': int, 'ID_INVALID': _daftar_int, 'MIN_KOSONG_S': float,
                  'MIN_DOMINAN': int},
    'pemantauan': {'POLL_S': float, 'KEEPALIVE_S': float},
    'ota': {'OTA_PORT': int, 'FOLDER_FIRMWARE': str},
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
    if not salah and not 1 <= hasil['MIN_DOMINAN'] <= hasil['JUMLAH_SAMPEL']:
        salah.append(f"[huskylens] min_dominan = {hasil['MIN_DOMINAN']} harus 1..jumlah_sampel "
                     f"({hasil['JUMLAH_SAMPEL']})")
    if salah:
        sys.exit("ERROR di " + path + ":\n  " + "\n  ".join(salah))
    return hasil


def putuskan_reject(sampel):
    """True = objek di-REJECT. VALID hanya kalau sampel VALID minimal MIN_DOMINAN DAN lebih banyak
    dari INVALID. Selain itu -- termasuk sampel kurang karena objek cepat lewat, atau seri --
    REJECT: kalau kamera tidak yakin, objek lebih aman disingkirkan daripada lolos.
    Contoh jumlah_sampel 5, min_dominan 3: 3 VALID + 2 INVALID -> VALID; 2 VALID + 3 INVALID ->
    REJECT; hanya 4 sampel 2 + 2 -> REJECT."""
    valid = sampel.count(ID_VALID)
    return not (valid >= MIN_DOMINAN and valid > len(sampel) - valid)


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
# BARU (2026-10-07): 7 register berurutan mulai alamat ini -- FW_TAHUN, FW_BULAN_HARI,
# FW_JAM_MENIT (waktu build), FW_VERSI (nomor rilis, 200 = v2.00), WIFI_IP_HI, WIFI_IP_LO,
# WIFI_RSSI. Firmware sebelum 2026-10-07 belum punya (ILLEGAL DATA ADDRESS).
R_FW = {SORTER: 25, PICKER: 19, DISPENSER: 27, STOCKER: 19}

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
          'langkah': '', 'mulai': None, 'alasan': '', 'kamera': None}


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
                    if INDIKATOR.sumber_detak == 'bus':
                        INDIKATOR.detak()     # mode terminal: loop produksi hidup = bus dipakai
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

    def tulis_banyak(self, sid, reg, nilai, percobaan=2):
        """Beberapa register berurutan dalam SATU request (FC16) -- jendela data kalibrasi."""
        terakhir = None
        with self.kunci:
            for _ in range(percobaan):
                try:
                    self.instr[sid].write_registers(reg, list(nilai))
                    return
                except Exception as e:
                    terakhir = e
                    time.sleep(0.03)
        raise Gagal(f"{NAMA[sid]}: gagal menulis register {reg}..{reg + len(nilai) - 1} "
                    f"({type(terakhir).__name__})")

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
def versi_node(bus, sid):
    """Versi firmware (waktu build) + diagnostik satu node, untuk log & MORE > SYSTEM."""
    nama = NAMA[sid]
    hasil = {'fw': '-', 'jawab': True, 'uptime': None, 'i2c': None, 'last_fault': None,
             'ip': None, 'rssi': None, 'baru': False}
    try:
        try:
            th, bh, jm, versi, ip_hi, ip_lo, rssi = bus.baca(sid, R_FW[sid], 7, percobaan=2)
        except RegisterTidakAda:                 # build sementara: waktu build saja
            (th, bh, jm), versi, ip_hi, ip_lo, rssi = bus.baca(sid, R_FW[sid], 3, percobaan=2), None, 0, 0, 0
        build = f"{th:04d}-{bh // 100:02d}-{bh % 100:02d} {jm // 100:02d}:{jm % 100:02d}"
        hasil['fw'] = (f"v{versi // 100}.{versi % 100:02d}  " if versi is not None else '') + build
        hasil['baru'] = True
        if ip_hi:
            hasil['ip'] = f"{ip_hi >> 8}.{ip_hi & 0xFF}.{ip_lo >> 8}.{ip_lo & 0xFF}"
            hasil['rssi'] = rssi - 65536 if rssi > 32767 else rssi
    except RegisterTidakAda:
        # Firmware tanpa register versi: perkirakan umurnya dari register yang ditambahkan belakangan.
        lama = False
        for _, reg in REG_PENANDA_FIRMWARE_BARU.get(nama, []):
            try:
                bus.baca(sid, reg, percobaan=1)
            except RegisterTidakAda:
                lama = True
                break
            except Exception:
                pass
        hasil['fw'] = 'LAMA, sebelum 2026-09-20' if lama else 'LAMA, sebelum 2026-10-07'
    except Exception:
        return dict(hasil, jawab=False, fw='tidak menjawab')
    for kunci, label in (('uptime', 'UPTIME_SEC'), ('i2c', 'I2C_ERROR_COUNT'), ('last_fault', 'LAST_FAULT_CODE')):
        try:
            hasil[kunci] = bus.baca(sid, ALAMAT[nama][label], percobaan=1)
        except Exception:
            pass
    return hasil


# ============================================================
# UPDATE FIRMWARE OTA -- protokol ArduinoOTA (sama dengan espota.py arduino-esp32 2.x),
# ditulis sendiri supaya Orange Pi tidak butuh PlatformIO / espota.py.
#   1. UDP ke node:3232  "0 <port_lokal> <ukuran> <md5>"   2. node membalas OK atau AUTH <nonce>
#   3. AUTH: kirim "200 <cnonce> md5(md5(password):nonce:cnonce)"
#   4. node MENYAMBUNG BALIK lewat TCP ke Orange Pi, file dikirim per 1460 byte, node menjawab
#      jumlah byte tiap potongan dan "OK" setelah flash ditulis & diverifikasi (MD5), lalu reboot.
# Password OTA dibaca dari ota_password.txt (TIDAK di sorting.config yang ikut ke git).
# ============================================================
FILE_OTA_PASSWORD = os.path.join(FOLDER, 'ota_password.txt')
NAMA_FILE_FIRMWARE = {SORTER: 'sorter.bin', PICKER: 'picker.bin', DISPENSER: 'dispenser.bin', STOCKER: 'stocker.bin'}
UKURAN_APP_MAKS = 0x140000    # partisi app0 esp32dev (default.csv) = 1,25 MB


def file_firmware(sid):
    """(path, keterangan, galat) file firmware node. galat None = siap dikirim."""
    path = os.path.join(FOLDER, FOLDER_FIRMWARE, NAMA_FILE_FIRMWARE[sid])
    if not os.path.exists(path):
        return path, 'tidak ada', f"salin firmware ke {FOLDER_FIRMWARE}/{NAMA_FILE_FIRMWARE[sid]}"
    ukuran = os.path.getsize(path)
    ket = f"{ukuran // 1024} KB, {time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(path)))}"
    with open(path, 'rb') as f:
        awal = f.read(1)
    if awal != b'\xe9' or not 100_000 < ukuran <= UKURAN_APP_MAKS:
        return path, ket, "bukan firmware ESP32 (pakai .pio/build/esp32dev/firmware.bin)"
    return path, ket, None


def password_ota():
    try:
        with open(FILE_OTA_PASSWORD, encoding='utf-8') as f:
            return f.read().strip() or None
    except OSError:
        return None


def ota_kirim(ip, path, password, progres=None, port=None, batal=None):
    import hashlib
    import socket
    port = port or OTA_PORT
    with open(path, 'rb') as f:
        data = f.read()
    ukuran, md5 = len(data), hashlib.md5(data).hexdigest()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', 0))
    srv.listen(1)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.settimeout(10)
    try:
        udp.sendto(f"0 {srv.getsockname()[1]} {ukuran} {md5}\n".encode(), (ip, port))
        try:
            jawab = udp.recv(37).decode(errors='replace').strip()
        except OSError:            # timeout, atau ditolak langsung (tidak ada yang mendengar di port itu)
            raise Gagal(f"node {ip}:{port} tidak menjawab undangan OTA (WiFi / jaringan berbeda?)")
        if jawab.startswith('AUTH'):
            if not password:
                raise Gagal(f"node minta password OTA -- isi {os.path.basename(FILE_OTA_PASSWORD)}")
            nonce = jawab.split()[1]
            cnonce = hashlib.md5(f"{os.path.basename(path)}{ukuran}{md5}{ip}".encode()).hexdigest()
            hasil = hashlib.md5(f"{hashlib.md5(password.encode()).hexdigest()}:{nonce}:{cnonce}".encode()).hexdigest()
            udp.sendto(f"200 {cnonce} {hasil}\n".encode(), (ip, port))
            try:
                jawab = udp.recv(32).decode(errors='replace').strip()
            except OSError:
                raise Gagal("node tidak menjawab password OTA")
            if jawab != 'OK':
                raise Gagal("password OTA ditolak node")
        elif jawab != 'OK':
            raise Gagal(f"jawaban OTA tidak dikenal: {jawab!r}")
        srv.settimeout(15)
        try:
            kon, _ = srv.accept()
        except OSError:
            raise Gagal("node tidak menyambung balik ke Orange Pi (firewall / beda jaringan)")
        with kon:
            kon.settimeout(15)
            ok = False
            for i in range(0, ukuran, 1460):
                if batal is not None and batal.is_set():
                    raise Gagal('dibatalkan')
                kon.sendall(data[i:i + 1460])
                ok = b'OK' in kon.recv(32)
                if progres:
                    progres(min(i + 1460, ukuran) / ukuran)
            kon.settimeout(60)         # node memverifikasi MD5 & menulis partisi
            while not ok:
                r = kon.recv(32)
                if not r:
                    raise Gagal("koneksi putus sebelum node mengonfirmasi (update gagal)")
                ok = b'OK' in r
    finally:
        udp.close()
        srv.close()


_INFO_TETAP = None


def _sidik(path):
    """8 karakter awal SHA-256 isi file -- berbeda = file berbeda, walau tanggalnya sama."""
    import hashlib
    try:
        with open(path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()[:8]
    except OSError:
        return '-'


def info_orangepi():
    """[(label, teks)] versi program & sistem Orange Pi. Bagian tetap dihitung sekali."""
    global _INFO_TETAP
    import platform
    import shutil
    import socket

    def baca(path, bawaan='-'):
        try:
            with open(path, encoding='utf-8', errors='replace') as f:
                return f.read().strip('\x00\n ')
        except OSError:
            return bawaan
    if _INFO_TETAP is None:
        app, utama = os.path.join(FOLDER, 'applib.py'), os.path.join(FOLDER, 'sorting-automation.py')
        try:
            tgl = time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(app)))
        except OSError:
            tgl = '-'
        os_nama = next((b.split('=', 1)[1].strip('"') for b in baca('/etc/os-release', '').splitlines()
                        if b.startswith('PRETTY_NAME=')), '-')
        _INFO_TETAP = [('Program', f"HMI {VERSI_HMI}, applib {_sidik(app)} ({tgl})"),
                       ('Main', f"sorting-automation.py {_sidik(utama)}"),
                       ('Board', baca('/proc/device-tree/model')),
                       ('OS', os_nama),
                       ('Kernel', f"{platform.release()}, Python {platform.python_version()}")]
    hasil = list(_INFO_TETAP)
    sistem = dict(info_sistem())
    hasil.append(('Host / IP', f"{socket.gethostname()} / {sistem.get('IP', '-')}"))
    hasil.append(('Uptime', f"{sistem.get('Uptime', '-')}, CPU {sistem.get('Suhu CPU', '-')}"))
    try:
        with open('/proc/meminfo') as f:
            m = {b.split(':')[0]: int(b.split()[1]) for b in f if ':' in b}
        ram = f"RAM {m.get('MemAvailable', 0) // 1024}/{m.get('MemTotal', 0) // 1024} MB bebas"
    except (OSError, ValueError):
        ram = 'RAM -'
    try:
        du = shutil.disk_usage('/')
        disk = f"SD {du.free // 2**30} GB bebas"
    except OSError:
        disk = 'SD -'
    hasil.append(('Memori', f"{ram}, {disk}"))
    hasil.append(('Port', f"RS485 {RS485_PORT} @ {RS485_BAUD}, kamera {HUSKYLENS_PORT if PAKAI_KAMERA else 'OFF'}"))
    return hasil


def log_info_awal(mode):
    """Satu blok di awal log: versi program & sistem -- untuk melacak update."""
    log("=" * 64)
    log(f"SECURA sorting automation -- mode {mode}")
    for label, teks in info_orangepi():
        log(f"  {label:10s} {teks}")
    log(f"  Config     {FILE_CONFIG}")
    log("=" * 64)


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
            log(f"  OK {NAMA[sid]:10s} state={state}  firmware {versi_node(bus, sid)['fw']}")

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
# BACKUP / RESTORE KALIBRASI NODE (2026-10-09) -- firmware v2.01+, protokol lengkap di
# include/kalibrasi_modbus.h tiap proyek ESP32. Kalibrasi (isi NVS) tiap node disimpan ke
# kalibrasi/<NODE>_<waktu>.json di Orange Pi, dan bisa dimuat ke ESP32 pengganti.
#   R_CAL+0 FORMAT, +1 UKURAN, +2 CRC, +3 OFFSET, +4 HASIL, +5..+36 DATA (64 byte)
# Restore hanya diterima kalau FORMAT & UKURAN sama (layout struct firmware sama). Setelah
# restore node menyimpan ke NVS lalu restart sendiri; hasilnya diverifikasi dengan membaca
# ulang CRC dari NVS node.
# ============================================================
R_CAL = {SORTER: 32, PICKER: 26, DISPENSER: 34, STOCKER: 26}
OP_CAL_BACA, OP_CAL_TULIS, OP_CAL_TERAPKAN = 60, 61, 62
CAL_JENDELA = 64
CAL_HASIL = {10: 'ditolak: MAIN aktif / node sedang bergerak', 11: 'urutan data salah',
             12: 'data belum lengkap', 13: 'CRC tidak cocok', 14: 'isi kalibrasi tidak valid',
             15: 'memori node habis'}
FOLDER_KALIBRASI = os.path.join(FOLDER, 'kalibrasi')
SIMPAN_BACKUP_PER_NODE = 30      # file lama dihapus melewati jumlah ini


def crc_kalibrasi(data):
    """CRC16-CCITT init 0xFFFF -- sama dengan calCrc16() di firmware."""
    import binascii
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def _cal_perintah(bus, sid, opcode, arg, nama):
    """Command CAL + tunggu ack + cek CAL_HASIL. Sengaja TIDAK lewat kirim()/tunggu(): node
    yang FAULT (mis. MCP hilang) tetap boleh di-backup / di-restore."""
    seq = bus.command(sid, opcode, arg)
    batas = time.monotonic() + 3
    while bus.baca(sid, R_ACK) != seq:
        if time.monotonic() > batas:
            raise Gagal(f"{NAMA[sid]}: {nama} tidak dijawab (operator di menu LCD node?)")
        time.sleep(0.05)
    h = bus.baca(sid, R_CAL[sid] + 4)
    if h >= 10:
        raise Gagal(f"{NAMA[sid]}: {nama} ditolak node -- {CAL_HASIL.get(h, f'kode {h}')}")


def kalibrasi_info(bus, sid):
    """(format, ukuran). RegisterTidakAda = firmware belum v2.01."""
    fmt, ukuran = bus.baca(sid, R_CAL[sid], 2, percobaan=2)
    return fmt, ukuran


def kalibrasi_baca(bus, sid, progres=None):
    """Baca kalibrasi node (isi NVS). Kembalikan rekaman siap disimpan ke file."""
    r = R_CAL[sid]
    fmt, ukuran = kalibrasi_info(bus, sid)
    data, crc = bytearray(), None
    for off in range(0, ukuran, CAL_JENDELA):
        _cal_perintah(bus, sid, OP_CAL_BACA, off, 'CAL_BACA')
        if off == 0:
            crc = bus.baca(sid, r + 2)
        if bus.baca(sid, r + 3) != off:
            raise Gagal(f"{NAMA[sid]}: jendela data bukan offset {off}")
        for w in bus.baca(sid, r + 5, CAL_JENDELA // 2):
            data += bytes((w >> 8, w & 0xFF))
        if progres:
            progres(min(1.0, (off + CAL_JENDELA) / ukuran))
    data = bytes(data[:ukuran])
    if crc_kalibrasi(data) != crc:
        raise Gagal(f"{NAMA[sid]}: CRC hasil baca tidak cocok -- ulangi (gangguan RS485?)")
    return {'node': NAMA[sid], 'sid': sid, 'format': fmt, 'ukuran': ukuran, 'crc': crc,
            'fw': versi_node(bus, sid)['fw'], 'waktu': time.strftime('%Y-%m-%d %H:%M:%S'), 'data': data.hex()}


def kalibrasi_simpan(rekam):
    """Tulis ke kalibrasi/<NODE>_<waktu>.json; backup lama melewati SIMPAN_BACKUP_PER_NODE dihapus."""
    os.makedirs(FOLDER_KALIBRASI, exist_ok=True)
    nama = f"{rekam['node']}_{rekam['waktu'].replace('-', '').replace(':', '').replace(' ', '-')}.json"
    path = os.path.join(FOLDER_KALIBRASI, nama)
    with open(path + '.tmp', 'w', encoding='utf-8') as f:
        json.dump(rekam, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(path + '.tmp', path)
    for lama in kalibrasi_daftar(rekam['node'])[SIMPAN_BACKUP_PER_NODE:]:
        try:
            os.remove(lama['path'])
        except OSError:
            pass
    return path


def kalibrasi_daftar(node):
    """Backup satu node, TERBARU di depan: [rekaman + 'path']. File rusak dilewati."""
    hasil = []
    try:
        nama_file = os.listdir(FOLDER_KALIBRASI)
    except OSError:
        return hasil
    for nama in nama_file:
        if not (nama.startswith(node + '_') and nama.endswith('.json')):
            continue
        path = os.path.join(FOLDER_KALIBRASI, nama)
        try:
            with open(path, encoding='utf-8') as f:
                r = json.load(f)
            if r.get('node') != node or crc_kalibrasi(bytes.fromhex(r['data'])) != r['crc']:
                continue
            hasil.append(dict(r, path=path))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return sorted(hasil, key=lambda r: r['waktu'], reverse=True)


def kalibrasi_tulis(bus, sid, rekam, progres=None, tunggu_restart=None):
    """Muat backup ke node: data -> CAL_TULIS per 64 byte -> CAL_TERAPKAN (node simpan NVS lalu
    restart) -> tunggu node menjawab -> baca ulang dan bandingkan CRC. Gagal = Exception."""
    r = R_CAL[sid]
    fmt, ukuran = kalibrasi_info(bus, sid)
    if (fmt, ukuran) != (rekam['format'], rekam['ukuran']):
        raise Gagal(f"{NAMA[sid]}: format backup ({rekam['format']}, {rekam['ukuran']} byte) berbeda dengan "
                    f"firmware node ({fmt}, {ukuran} byte) -- flash firmware yang sama dengan saat backup")
    data = bytes.fromhex(rekam['data'])
    if len(data) != ukuran or crc_kalibrasi(data) != rekam['crc']:
        raise Gagal("file backup rusak (CRC)")
    for off in range(0, ukuran, CAL_JENDELA):
        potong = data[off:off + CAL_JENDELA].ljust(CAL_JENDELA, b'\0')
        bus.tulis_banyak(sid, r + 5, [potong[i] << 8 | potong[i + 1] for i in range(0, CAL_JENDELA, 2)])
        _cal_perintah(bus, sid, OP_CAL_TULIS, off, 'CAL_TULIS')
        if progres:
            progres(0.8 * min(1.0, (off + CAL_JENDELA) / ukuran))
    _cal_perintah(bus, sid, OP_CAL_TERAPKAN, rekam['crc'], 'CAL_TERAPKAN')
    log(f"[kalibrasi] {NAMA[sid]}: disimpan ke NVS node, node restart")
    if tunggu_restart:
        tunggu_restart()
    else:
        time.sleep(3)
    batas = time.monotonic() + 30
    while True:
        try:
            baru = kalibrasi_baca(bus, sid)
            break
        except Gagal:
            if time.monotonic() > batas:
                raise Gagal(f"{NAMA[sid]}: tersimpan, tapi node tidak menjawab 30 s setelah restart")
            time.sleep(1)
    if progres:
        progres(1.0)
    if baru['crc'] != rekam['crc']:
        raise Gagal(f"{NAMA[sid]}: setelah restart kalibrasi node BERBEDA dengan backup (CRC)")
    return baru


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
        self.pantau = None     # fungsi(byte_diterima, oid) tiap bacaan -- tampilan LIVE uji kamera

    def tutup(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def baca_id(self):
        """ID objek pertama dalam ~50 ms, atau None (tidak ada objek dikenali)."""
        self.ser.write(REQUEST_BLOCKS)
        batas, dapat, oid = time.monotonic() + 0.05, 0, None
        while time.monotonic() < batas:
            data = self.ser.read(64)
            if data:
                dapat += len(data)
                self.buf += data
            oid = self._frame()
            if oid is not None:
                break
        if self.pantau is not None:
            self.pantau(dapat, oid)
        return oid

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

    def ambil_sampel(self, henti=None):
        """Sampel SATU objek; [] hanya kalau dihentikan sebelum ada objek.
        DIPERBAIKI 2026-10-08: tunggu objek dulu, jendela BATAS_AMBIL_S dihitung sejak objek
        PERTAMA terlihat, dan bacaan pertama itu ikut dihitung. Dulu jendela dimulai saat fungsi
        dipanggil -- kamera masih kosong -- jadi objek yang lewat di akhir jendela hanya dapat
        1-2 sampel, dan dengan min_dominan 3 objek VALID pun di-REJECT."""
        henti = henti or berhenti
        dikenali = lambda oid: oid == ID_VALID or oid in ID_INVALID
        oid = None
        while not henti.is_set() and not dikenali(oid):
            oid = self.baca_id()
        if henti.is_set():
            return []
        sampel, mulai = [oid], time.monotonic()
        while len(sampel) < JUMLAH_SAMPEL and time.monotonic() - mulai < BATAS_AMBIL_S:
            if oid is not None:            # jeda hanya setelah kamera melihat sesuatu
                henti.wait(INTERVAL_SAMPEL_S)
            if henti.is_set():
                break
            oid = self.baca_id()
            if dikenali(oid):
                sampel.append(oid)
        return sampel

    def tunggu_kosong(self, henti=None):
        """Kembali setelah bidang pandang kosong MIN_KOSONG_S berturut-turut."""
        henti = henti or berhenti
        kosong_sejak = time.monotonic()
        while not henti.is_set():
            oid = self.baca_id()
            if oid is None or oid == ID_KOSONG:
                if time.monotonic() - kosong_sejak >= MIN_KOSONG_S:
                    return
            else:
                kosong_sejak = time.monotonic()


statistik = {'valid': 0, 'invalid': 0, 'gagal_tulis': 0}


FILE_HASIL_KAMERA = os.path.join(FOLDER, 'kamera_hasil.csv')
JUDUL_HASIL_KAMERA = 'No;Timestamp;Sampel;Hasil'


class UjiKamera:
    """Uji HuskyLens dari MANUAL > CAMERA. Port kamera dibuka HANYA selama uji berjalan dan
    ditutup lagi sesudahnya -- produksi memakai port yang sama, jadi uji diblokir selama produksi.
      live         baca terus: ID yang terlihat, jumlah jawaban per detik
      klasifikasi  LIVE + klasifikasi TERUS sampai STOP: tiap objek diambil sampelnya seperti
                   produksi, diputuskan VALID / INVALID (min_dominan), bunyi buzzer/LED, dicatat ke
                   kamera_hasil.csv, lalu tunggu bidang pandang kosong sebelum objek berikutnya.
                   Opsional INVALID dikirim ke Sorter (CLASSIFY_IS_REJECT -> palang)
      diagnosa     cek_huskylens(): port ada? baud cocok? protokol UART?

    kamera_hasil.csv (titik koma, bisa dibuka Excel), satu baris per objek:
        No;Timestamp;Sampel;Hasil
        1;2026-10-08 14:30:05;1,1,1,1,1;VALID       sampel: 1 = ID VALID, 0 = ID INVALID"""

    def __init__(self):
        self.thread, self.henti = None, threading.Event()
        self.mode, self.pesan = None, None          # pesan: (teks, warna)
        self.id_terakhir, self.t_id = None, 0.0
        self.jawaban = collections.deque(maxlen=40)  # waktu tiap balasan kamera
        self.sampel, self.hitung = [], {'VALID': 0, 'INVALID': 0}
        self.kirim_palang, self.bus = False, None
        # 200 hasil terakhir untuk tabel HASIL di panel; nomor berlanjut dari isi file
        self.hasil, self.no = collections.deque(maxlen=200), 0
        try:
            with open(FILE_HASIL_KAMERA, encoding='utf-8') as f:
                for baris in f:
                    bagian = baris.strip().split(';')
                    if len(bagian) == 4 and bagian[0].isdigit():
                        self.hasil.append(tuple(bagian))
        except OSError:
            pass
        if self.hasil:
            self.no = int(self.hasil[-1][0])

    def daftar_hasil(self):
        """[(no, waktu, sampel, hasil)], terbaru di akhir."""
        return list(self.hasil)

    def _catat(self, sampel, jenis):
        self.no += 1
        rekam = (str(self.no), time.strftime('%Y-%m-%d %H:%M:%S'),
                 ','.join('1' if s == ID_VALID else '0' for s in sampel), jenis)
        self.hasil.append(rekam)
        try:
            baru = not os.path.exists(FILE_HASIL_KAMERA)
            with open(FILE_HASIL_KAMERA, 'a', encoding='utf-8') as f:
                if baru:
                    f.write(JUDUL_HASIL_KAMERA + '\n')
                f.write(';'.join(rekam) + '\n')
        except OSError as e:
            log(f"!! [kamera uji] gagal mencatat {FILE_HASIL_KAMERA}: {e}")
        return rekam

    def aktif(self):
        return self.thread is not None and self.thread.is_alive()

    def mulai(self, mode, bus=None):
        if self.aktif():
            return
        self.mode, self.bus, self.pesan = mode, bus, None
        self.henti.clear()
        self.thread = threading.Thread(target=self._jalan, daemon=True)
        self.thread.start()

    def stop(self):
        self.henti.set()
        if self.thread is not None:
            self.thread.join(timeout=2)

    def jawaban_per_detik(self):
        t = time.monotonic()
        return sum(1 for x in self.jawaban if t - x < 2) / 2

    def _pantau(self, dapat, oid):
        """Dipanggil HuskyLens.baca_id tiap bacaan: jawaban/s dan ID terakhir untuk tampilan LIVE."""
        if dapat:
            self.jawaban.append(time.monotonic())
        if oid is not None:
            self.id_terakhir, self.t_id = oid, time.monotonic()

    def _jalan(self):
        if self.mode == 'diagnosa':
            log(f"[kamera uji] DIAGNOSA {HUSKYLENS_PORT} @ {HUSKYLENS_BAUD}")
            ok = cek_huskylens()
            self.pesan = ('HuskyLens menjawab -- port & baud OK' if ok else 'GAGAL -- penyebab rinci di LOG',
                          HIJAU if ok else MERAH)
            return
        try:
            hl = HuskyLens()
        except Exception as e:
            self.pesan = (f"{HUSKYLENS_PORT} tidak bisa dibuka: {e}", MERAH)
            log(f"!! [kamera uji] {self.pesan[0]}")
            return
        log(f"[kamera uji] {self.mode} -- {HUSKYLENS_PORT} @ {HUSKYLENS_BAUD}")
        hl.pantau = self._pantau
        try:
            if self.mode == 'live':
                while not self.henti.wait(0.05):
                    hl.baca_id()
            else:
                self._klasifikasi(hl)
        finally:
            hl.tutup()
            log("[kamera uji] port ditutup")

    def _klasifikasi(self, hl):
        """Terus sampai STOP, memakai fungsi PRODUKSI yang sama (HuskyLens.ambil_sampel +
        putuskan_reject + tunggu_kosong) -- hasilnya sama dengan yang akan terjadi saat produksi.
        Jalankan conveyor (tombol CONVEYOR) supaya objek lewat dengan kecepatan produksi."""
        self.pesan = ('Menunggu objek lewat kamera...', KUNING)
        while not self.henti.is_set():
            sampel = hl.ambil_sampel(self.henti)
            if self.henti.is_set():
                return                           # dihentikan di tengah sampel: tidak dicatat
            self.sampel = sampel
            reject = putuskan_reject(sampel)
            jenis = 'INVALID' if reject else 'VALID'
            catatan = ''
            if reject and self.kirim_palang and self.bus is not None:   # tulisan DULU, seperti produksi
                try:
                    self.bus.tulis(SORTER, R_SORTER_CLASSIFY, 1)
                    catatan = ' -> palang'
                except Exception as e:
                    catatan = f' (palang gagal: {e})'
            self.hitung[jenis] += 1
            INDIKATOR.put_nowait(jenis)
            no, _, teks_sampel, _ = self._catat(sampel, jenis)
            hasil = (f"#{no} {jenis}{catatan}  {teks_sampel}", MERAH if reject else HIJAU)
            log(f"[kamera uji] #{no} {sampel} -> {jenis}{catatan}")
            self.pesan = (hasil[0] + '  - tunggu kosong', hasil[1])
            hl.tunggu_kosong(self.henti)         # satu objek = satu klasifikasi
            self.pesan = hasil

UJI_KAMERA = UjiKamera()


def thread_kamera(bus, indikator):
    try:
        hl = HuskyLens()
    except Exception as e:
        STATUS['kamera'] = 'GAGAL'
        log(f"!! KAMERA: {HUSKYLENS_PORT} tidak bisa dibuka ({e}) -- klasifikasi MATI")
        return
    STATUS['kamera'] = 'OK'
    log("[kamera] mulai")
    try:
        while not berhenti.is_set():
            sampel = hl.ambil_sampel()
            if not sampel:
                continue
            reject = putuskan_reject(sampel)     # aturan min_dominan (sorting.config)

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
        STATUS['kamera'] = None
        log("[kamera] berhenti")


# ============================================================
# INDIKATOR (2026-10-07) -- 3 LED + 1 buzzer + tombol SW1. SATU thread untuk seluruh umur
# program (bukan hanya saat produksi), dijadwalkan dengan waktu -- tidak ada sleep() panjang,
# jadi heartbeat, LED, buzzer dan SW1 berjalan bersamaan.
#
#   LED OPR    heartbeat: kedip pendek selama loop utama program hidup. Loop macet lebih dari
#              heartbeat_batas_s atau program berhenti -> PADAM.
#   LED RUN    nyala = produksi berjalan, kedip = pemeriksaan awal / startup node
#   LED ALARM  nyala = fault / E-STOP / node tidak menjawab / produksi ditahan / gagal
#   BUZZER     objek VALID 1x, INVALID 2x; alarm: nyala alarm_nyala_s, mati alarm_mati_s,
#              berulang sampai di-diamkan (SW1 / layar ALARM dibuka) atau alarm hilang --
#              alarm BARU membunyikan lagi; boot: 2x pendek = normal, 3x panjang = ada error
#   SW1        tekan singkat = diamkan buzzer alarm; tahan >= sw1_tahan_s = STOP produksi
#              (shutdown aman yang sama dengan tombol STOP); ditahan saat program mulai =
#              kalibrasi sentuh
# Pin None di sorting.config = komponen itu tidak dipasang, sisanya tetap bekerja.
# ============================================================
FASE_ALARM = ('DITAHAN', 'GAGAL CEK', 'GAGAL STARTUP')

# GPIO BERSAMA (2026-10-07). wiringOP mengubah satu pin dengan baca-ubah-tulis SELURUH register
# port; LED/buzzer dan D/C-CS LCD/touch sebagian besar di port A yang sama. Dua thread yang
# melakukannya bersamaan bisa saling menimpa bit pin lain. Karena itu: wiringPiSetup() SEKALI
# untuk seluruh program, dan SEMUA akses GPIO lewat satu kunci.
_GPIO = {'w': None}
GPIO_KUNCI = threading.RLock()


def wiringpi_siap():
    with GPIO_KUNCI:
        if _GPIO['w'] is None:
            import wiringpi
            wiringpi.wiringPiSetup()
            _GPIO['w'] = wiringpi
        return _GPIO['w']


def pasang_pullup(pin):
    """JANGAN memakai wiringpi.pullUpDnControl() dari Python: di Orange Pi One (OrangePiH3,
    binding wiringOP-Python) panggilan itu MEMBEKUKAN seluruh board sampai harus hard reset --
    terbukti 2026-10-07 di pin wPi 10 (PC7), sedangkan pinMode/digitalRead dari Python dan
    `gpio mode 10 up` dari perintah gpio aman. Pull-up karena itu diatur lewat perintah `gpio`
    (wiringOP C), sekali saat pin disiapkan."""
    import subprocess
    try:
        subprocess.run(['gpio', 'mode', str(pin), 'up'], check=True, timeout=5,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        return True
    except (OSError, subprocess.SubprocessError) as e:
        log(f"!! pull-up pin wPi {pin} lewat perintah `gpio` gagal ({e}) -- "
            f"pasang resistor pull-up 10k ke 3.3V di pin itu")
        return False


class Indikator:
    def __init__(self):
        self.w = None                     # modul wiringpi, None = GPIO tidak tersedia
        self.thread = None
        self.henti = threading.Event()
        self.kunci = threading.Lock()
        self.pola, self.pola_i, self.pola_sejak = [], 0, 0.0   # bunyi sekali jalan [(nyala, mati)]
        self.alarm_panel = set()          # kode alarm aktif dari panel (fault, E-STOP, tidak menjawab)
        self.alarm_lalu = set()
        self.alarm_diam = False           # di-ACK: diam sampai ada alarm BARU
        self.alarm_mulai = 0.0
        self.detak_t = time.monotonic()
        self.sumber_detak = 'bus'         # 'bus' (mode terminal) atau 'panel' (loop layar)
        self.on_pendek = None             # fungsi tambahan saat SW1 ditekan singkat (panel)
        self.terakhir = {}                # nilai terakhir tiap pin keluaran
        self.sw1_mulai, self.sw1_panjang = None, False
        self._lepas_sejak = None

    # ---------- dipanggil dari luar ----------
    def mulai(self):
        if self.thread is not None and self.thread.is_alive():
            return
        try:
            wiringpi = wiringpi_siap()
            with GPIO_KUNCI:
                for p in (LED_OPR, LED_RUN, LED_ALARM, BUZZER):
                    if p is not None:
                        wiringpi.pinMode(p, 1)
                        wiringpi.digitalWrite(p, 0)
                if SW1 is not None:
                    wiringpi.pinMode(SW1, 0)
            if SW1 is not None:
                pasang_pullup(SW1)                     # tombol ke GND = 0 saat ditekan
            self.terakhir = {}
            self.w = wiringpi
        except Exception as e:
            log(f"-- indikator LED/buzzer/SW1 tidak aktif ({e})")
            self.w = None
        self.henti.clear()
        self.thread = threading.Thread(target=self._jalan, daemon=True)
        self.thread.start()

    def berhenti(self):
        self.henti.set()
        if self.thread is not None:
            self.thread.join(timeout=1)

    def detak(self):
        self.detak_t = time.monotonic()

    def sw1_ditekan(self):
        """Dibaca langsung -- dipakai saat program mulai (tahan SW1 = kalibrasi sentuh)."""
        if SW1 is None:
            return False
        try:
            wiringpi = wiringpi_siap()
            with GPIO_KUNCI:
                wiringpi.pinMode(SW1, 0)
            pasang_pullup(SW1)
            time.sleep(0.05)
            with GPIO_KUNCI:
                return wiringpi.digitalRead(SW1) == 0
        except Exception:
            return False

    def bunyi(self, jenis):
        b, j = BUNYI_PENDEK_MS / 1000, JEDA_BUNYI_MS / 1000
        pola = {'valid': [(b, 0)] if BUZZER_VALID else [],
                'invalid': [(b, j), (b, 0)] if BUZZER_INVALID else [],
                'boot_ok': [(b, j), (b, 0)] if BUZZER_BOOT else [],
                'boot_error': [(0.6, 0.3), (0.6, 0.3), (0.6, 0)] if BUZZER_BOOT else [],
                'konfirmasi': [(0.05, 0)],
                'stop': [(0.5, 0)]}[jenis]
        if pola:
            with self.kunci:
                self.pola, self.pola_i, self.pola_sejak = pola, 0, time.monotonic()

    def put_nowait(self, jenis):
        """Dipakai thread kamera (dulu antrian queue): 'VALID' / 'INVALID'."""
        self.bunyi('valid' if jenis == 'VALID' else 'invalid')

    def set_alarm(self, kode):
        self.alarm_panel = set(kode)

    def diamkan(self):
        self.alarm_diam = True

    # ---------- loop ----------
    def _alarm(self):
        kode = set(self.alarm_panel)
        if STATUS['fase'] in FASE_ALARM:
            kode.add('fase:' + STATUS['fase'])
        if alarm.is_set():
            kode.add('alarm')
        if kode - self.alarm_lalu:          # ada alarm BARU -> bunyi lagi walau sudah didiamkan
            self.alarm_diam = False
            self.alarm_mulai = time.monotonic()  # pola alarm dihitung dari awal alarm ini
        self.alarm_lalu = kode
        return bool(kode)

    def _tulis(self, pin, nilai, paksa=False):
        """Hanya menulis saat nilainya BERUBAH -- dulu 4 pin ditulis tiap 20 ms walau sama."""
        if pin is None or self.w is None:
            return
        nilai = 1 if nilai else 0
        if not paksa and self.terakhir.get(pin) == nilai:
            return
        with GPIO_KUNCI:
            self.w.digitalWrite(pin, nilai)
        self.terakhir[pin] = nilai

    def _jalan(self):
        try:
            while not self.henti.wait(0.02):
                sekarang = time.monotonic()
                ada_alarm = self._alarm()
                # LED OPR -- heartbeat hanya selama loop utama masih berdetak
                hidup = sekarang - self.detak_t < HEARTBEAT_BATAS_S
                self._tulis(LED_OPR, hidup and (sekarang % HEARTBEAT_PERIODE_S) < HEARTBEAT_NYALA_S)
                # LED RUN
                fase = STATUS['fase']
                self._tulis(LED_RUN, fase == 'JALAN' or (fase in ('PERIKSA', 'STARTUP') and sekarang % 1.0 < 0.5))
                # LED ALARM
                self._tulis(LED_ALARM, ada_alarm)
                # BUZZER -- bunyi sekali jalan didahulukan, lalu pola alarm
                self._tulis(BUZZER, self._buzzer(sekarang, ada_alarm))
                self._baca_sw1(sekarang)
        finally:
            # Program berhenti: OPR, RUN, buzzer padam. ALARM dibiarkan menyala kalau ada
            # masalah, supaya operator yang datang belakangan masih melihatnya.
            for p in (LED_OPR, LED_RUN, BUZZER):
                self._tulis(p, 0, paksa=True)
            self._tulis(LED_ALARM, bool(self.alarm_lalu), paksa=True)

    def _buzzer(self, sekarang, ada_alarm):
        with self.kunci:
            while self.pola_i < len(self.pola):
                nyala, mati = self.pola[self.pola_i]
                t = sekarang - self.pola_sejak
                if t < nyala:
                    return True
                if t < nyala + mati:
                    return False
                self.pola_i += 1
                self.pola_sejak += nyala + mati
        if ada_alarm and BUZZER_ALARM and not self.alarm_diam:
            return ((sekarang - self.alarm_mulai) % (ALARM_NYALA_S + ALARM_MATI_S)) < ALARM_NYALA_S
        return False

    def _baca_sw1(self, sekarang):
        if SW1 is None or self.w is None:
            return
        with GPIO_KUNCI:
            ditekan = self.w.digitalRead(SW1) == 0
        if ditekan:
            self._lepas_sejak = None
            if self.sw1_mulai is None:
                self.sw1_mulai, self.sw1_panjang = sekarang, False
            elif not self.sw1_panjang and sekarang - self.sw1_mulai >= SW1_TAHAN_S:
                self.sw1_panjang = True
                if STATUS['fase'] in ('PERIKSA', 'STARTUP', 'JALAN'):
                    log("[SW1] ditahan -- STOP produksi (shutdown aman)")
                    berhenti.set()
                    self.bunyi('stop')
                else:
                    self.bunyi('konfirmasi')
        elif self.sw1_mulai is not None:
            if self._lepas_sejak is None:           # debounce: lepas harus stabil 40 ms
                self._lepas_sejak = sekarang
                return
            if sekarang - self._lepas_sejak < 0.04:
                return
            lama = self._lepas_sejak - self.sw1_mulai
            self.sw1_mulai, self._lepas_sejak = None, None
            if not self.sw1_panjang and lama >= 0.04:
                log("[SW1] ditekan -- buzzer alarm didiamkan")
                self.diamkan()
                self.bunyi('konfirmasi')
                if self.on_pendek:
                    self.on_pendek()


INDIKATOR = Indikator()


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


def laporan_node(bus):
    """Dipanggil saat produksi ditahan: node mana yang menjawab, dan apakah ada yang RESTART
    selama program berjalan. Node yang uptime-nya lebih pendek dari umur program ini pasti
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
        elif up < umur:
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
def jalankan_produksi(bus, pakai_kamera, hanya_cek=False, bunyi_cek=False):
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
    # Indikator hidup PALING AWAL (idempoten -- panel sudah menyalakannya sejak program mulai),
    # supaya pemeriksaan awal yang gagal juga terlihat & terdengar.
    INDIKATOR.mulai()

    lolos = periksa(bus, pakai_kamera)
    if bunyi_cek:              # mode terminal: bunyi tanda program siap / ada error
        INDIKATOR.bunyi('boot_ok' if lolos else 'boot_error')
    if not lolos:
        log("!! Pemeriksaan awal GAGAL -- tidak ada yang digerakkan.")
        STATUS.update(fase='GAGAL CEK', alasan='pemeriksaan awal gagal -- lihat LOG')
        alarm.set()
        berhenti.set()
        return False
    if hanya_cek:
        log("Pemeriksaan awal OK. (cek saja: berhenti di sini)")
        STATUS['fase'] = 'CEK OK'
        berhenti.set()
        return True

    threads = [threading.Thread(target=thread_keepalive, args=(bus,), daemon=True)]
    if pakai_kamera:
        threads.append(threading.Thread(target=thread_kamera, args=(bus, INDIKATOR), daemon=True))
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
        for t in threads:
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

for _nama, _sid in (('SORTER', SORTER), ('PICKER', PICKER), ('DISPENSER', DISPENSER), ('STOCKER', STOCKER)):
    KHUSUS_NODE[_nama] += [('FW_TAHUN', R_FW[_sid]), ('FW_BULAN_HARI', R_FW[_sid] + 1),
                           ('FW_JAM_MENIT', R_FW[_sid] + 2), ('FW_VERSI', R_FW[_sid] + 3),
                           ('WIFI_IP_HI', R_FW[_sid] + 4), ('WIFI_IP_LO', R_FW[_sid] + 5),
                           ('WIFI_RSSI', R_FW[_sid] + 6)]

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
    for nama in ('LED_OPR', 'LED_RUN', 'LED_ALARM', 'BUZZER', 'SW1'):
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
    """GPIO LCD & touch. wiringPiSetup() dan kunci DIBAGI dengan INDIKATOR (lihat GPIO BERSAMA)."""

    def __init__(self):
        self.w = wiringpi_siap()

    def keluar(self, pin, nilai=0):
        with GPIO_KUNCI:
            self.w.pinMode(pin, 1)
            self.w.digitalWrite(pin, nilai)

    def masuk(self, pin, pullup=False):
        with GPIO_KUNCI:
            self.w.pinMode(pin, 0)
        if pullup:
            pasang_pullup(pin)          # BUKAN pullUpDnControl() -- lihat pasang_pullup()

    def tulis(self, pin, nilai):
        with GPIO_KUNCI:
            self.w.digitalWrite(pin, nilai)

    def baca(self, pin):
        with GPIO_KUNCI:
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
# TAMPILAN -- alat gambar dasar (font, warna, ikon, tombol). Susunan layar: lihat HMI SECURA.
# ============================================================
_FONT_CACHE = {}


def _font(ukuran, tebal=False):
    """Font dimuat SEKALI per ukuran -- dulu sebagian layar memuat file TTF dari SD tiap frame."""
    kunci = (ukuran, tebal)
    if kunci not in _FONT_CACHE:
        _FONT_CACHE[kunci] = _muat_font(ukuran, tebal)
    return _FONT_CACHE[kunci]


def _muat_font(ukuran, tebal):
    nama = 'DejaVuSans-Bold.ttf' if tebal else 'DejaVuSans.ttf'
    # BASIC: posisi huruf dibulatkan ke piksel. Raqm (kalau terpasang) memakai posisi pecahan
    # -- di layar 320x240 tepi huruf jadi kabur.
    dasar = ImageFont.Layout.BASIC if hasattr(ImageFont, 'Layout') else ImageFont.LAYOUT_BASIC
    for folder in ('/usr/share/fonts/truetype/dejavu', '/usr/share/fonts/dejavu', FOLDER):
        try:
            return ImageFont.truetype(os.path.join(folder, nama), ukuran, layout_engine=dasar)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=ukuran)
    except TypeError:
        return ImageFont.load_default()


# font_tajam = 1: huruf digambar TANPA antialias (piksel penuh, mengikuti hinting font). Di
# layar 320x240 (~125 ppi) huruf kecil 8-12 px jauh lebih tajam; huruf besar sedikit bergerigi.
FONT_TAJAM = _opsi_font()


def kanvas(img, mode=None):
    """ImageDraw untuk semua layar panel -- satu tempat mengatur cara huruf digambar."""
    d = ImageDraw.Draw(img, mode)
    d.fontmode = '1' if FONT_TAJAM else 'L'
    return d


if ADA_PIL:
    F_MINI, F_KECIL, F_BIASA, F_TEBAL = _font(9), _font(10), _font(12), _font(12, True)
    F_JUDUL, F_ANGKA, F_BESAR = _font(13, True), _font(22, True), _font(30, True)
    F_LABEL, F_KECIL_TEBAL, F_SEDANG = _font(8), _font(10, True), _font(17, True)
else:   # mode terminal tanpa Pillow
    F_MINI = F_KECIL = F_BIASA = F_TEBAL = F_JUDUL = F_ANGKA = F_BESAR = None
    F_LABEL = F_KECIL_TEBAL = F_SEDANG = None

# Warna -- ganti di sini untuk mengubah tema seluruh layar.
HITAM, PUTIH = (0, 0, 0), (255, 255, 255)
LATAR = (11, 19, 31)           # latar layar
KARTU = (22, 34, 52)           # kartu / panel
KARTU2 = (32, 47, 70)          # tombol netral, kartu di dalam kartu
GARIS = (48, 66, 94)
ABU = (160, 174, 194)          # teks keterangan (lebih terang = lebih terbaca di LCD)
ABU_GELAP = (70, 82, 100)      # tombol tidak aktif
BIRU = (30, 120, 230)          # tab aktif, tombol utama
BIRU_MUDA = (90, 175, 255)     # angka penting di atas kartu gelap
HIJAU = (34, 170, 80)
HIJAU_MUDA = (90, 215, 120)
MERAH = (220, 60, 60)
KUNING = (240, 190, 40)
ORANYE = (235, 125, 30)
# Warna di atas bisa ditimpa bagian [warna] display.config (RRGGBB).
globals().update(_warna_display())

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
    elif nama == 'maju':
        d.polygon([(cx + s, cy), (cx, cy - s), (cx, cy + s)], fill=w)
    elif nama == 'konveyor':      # sabuk dengan 3 roller
        if hasattr(d, 'rounded_rectangle'):
            d.rounded_rectangle((cx - s, cy - s // 2, cx + s, cy + s // 2), s // 2, outline=w, width=lw + 1)
        else:
            d.rectangle((cx - s, cy - s // 2, cx + s, cy + s // 2), outline=w, width=lw + 1)
        for i in (-1, 0, 1):
            x = cx + i * s * 2 // 3
            d.ellipse((x - s // 4, cy - s // 4, x + s // 4, cy + s // 4), fill=w)
        d.line([(cx - s * 2 // 3, cy + s // 2), (cx - s * 2 // 3, cy + s)], fill=w, width=lw)
        d.line([(cx + s * 2 // 3, cy + s // 2), (cx + s * 2 // 3, cy + s)], fill=w, width=lw)
    elif nama == 'kamera':
        d.rectangle((cx - s // 3, cy - s * 4 // 5, cx + s // 3, cy - s // 2), fill=w)
        kotak(d, (cx - s, cy - s // 2, cx + s, cy + s * 3 // 4), max(2, s // 4), w)
        d.ellipse((cx - s // 2, cy - s // 3, cx + s // 2, cy + s * 2 // 3), fill=LATAR)
        d.ellipse((cx - s // 4, cy - s // 12, cx + s // 4, cy + s * 5 // 12), fill=w)
    elif nama == 'diverter':      # palang: kotak + panah lolos (hijau) & tolak (merah)
        d.rectangle((cx - s, cy, cx + s, cy + s * 3 // 4), outline=w, width=lw + 1)
        d.rectangle((cx - s // 2, cy + s // 4, cx + s // 2, cy + s // 2), fill=w)
        xg = cx - s // 2
        d.polygon([(xg, cy - s), (xg - s // 3, cy - s // 2), (xg + s // 3, cy - s // 2)], fill=HIJAU_MUDA)
        d.rectangle((xg - s // 8, cy - s // 2, xg + s // 8, cy - s // 8), fill=HIJAU_MUDA)
        xr = cx + s // 2
        d.line([(xr - s // 4, cy - s // 6), (xr + s // 3, cy - s * 3 // 4)], fill=MERAH, width=lw + 1)
        d.polygon([(xr + s // 2, cy - s), (xr, cy - s * 3 // 4 - 1), (xr + s // 3, cy - s // 2)], fill=MERAH)
    elif nama == 'tumpukan':      # package bertumpuk (dispenser)
        for x0, y0 in ((cx - s, cy), (cx + 1, cy), (cx - s // 2, cy - s)):
            d.rectangle((x0, y0, x0 + s - 1, y0 + s - 1), fill=w, outline=LATAR)
            d.line([(x0 + s // 2, y0), (x0 + s // 2, y0 + s // 3)], fill=LATAR, width=1)
    elif nama == 'resep':
        d.rectangle((cx - s * 3 // 4, cy - s * 3 // 4, cx + s * 3 // 4, cy + s), outline=w, width=lw + 1)
        d.rectangle((cx - s // 3, cy - s, cx + s // 3, cy - s // 2), fill=w)
        for i in (0, 1):
            y = cy - s // 6 + i * s * 2 // 5
            d.line([(cx - s // 3, y), (cx + s // 3, y)], fill=w, width=lw)
    elif nama == 'lainnya':
        for i in (-1, 0, 1):
            x = cx + i * s * 2 // 3
            d.ellipse((x - lw - 1, cy - lw - 1, x + lw + 1, cy + lw + 1), fill=w)
    elif nama == 'lonceng':
        d.pieslice((cx - s * 2 // 3, cy - s, cx + s * 2 // 3, cy + s // 3), 180, 360, fill=w)
        d.rectangle((cx - s * 2 // 3, cy - s // 3, cx + s * 2 // 3, cy + s // 2), fill=w)
        d.rectangle((cx - s, cy + s // 2, cx + s, cy + s * 2 // 3), fill=w)
        d.ellipse((cx - lw - 1, cy + s * 2 // 3, cx + lw + 1, cy + s), fill=w)
    elif nama == 'jeda':
        d.rectangle((cx - s * 2 // 3, cy - s * 3 // 4, cx - s // 5, cy + s * 3 // 4), fill=w)
        d.rectangle((cx + s // 5, cy - s * 3 // 4, cx + s * 2 // 3, cy + s * 3 // 4), fill=w)
    elif nama == 'kunci_pas':
        d.line([(cx - s * 2 // 3, cy + s * 2 // 3), (cx + s // 4, cy - s // 4)], fill=w, width=lw + 3)
        d.ellipse((cx, cy - s, cx + s, cy), outline=w, width=lw + 2)
    elif nama == 'gembok':
        d.arc((cx - s // 2, cy - s, cx + s // 2, cy), 180, 360, fill=w, width=lw + 1)
        d.line([(cx - s // 2, cy - s // 2), (cx - s // 2, cy)], fill=w, width=lw + 1)
        d.line([(cx + s // 2, cy - s // 2), (cx + s // 2, cy)], fill=w, width=lw + 1)
        d.rectangle((cx - s * 3 // 4, cy - s // 6, cx + s * 3 // 4, cy + s), fill=w)


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
    # Penggantian nama (rename) baru aman di kartu SD setelah FOLDER-nya juga di-fsync. Tanpa
    # ini, listrik dicabut beberapa detik setelah SIMPAN bisa mengembalikan config LAMA.
    try:
        fd = os.open(FOLDER, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass                     # Windows: folder tidak bisa di-fsync


def muat_ulang_config():
    """Config baru langsung dipakai mesin produksi berikutnya. Kembalikan pesan error atau None."""
    try:
        globals().update(muat_config(FILE_CONFIG))
        return None
    except SystemExit as e:
        return str(e)


# ============================================================
# HMI SECURA (2026-10-07) -- lapisan tampilan DI ATAS mesin produksi yang sama.
#
# Tidak ada yang berubah di bawahnya: satu Bus (satu pemilik /dev/ttyS3), Pemantau tetap
# satu-satunya pembaca Modbus untuk layar, START/STOP tetap jalankan_produksi() & berhenti.
#
#   Boot      SPLASH -> LOADING -> SYSTEM CHECK -> SYSTEM READY -> menu utama
#   Menu      HOME  AUTO  MANUAL  ALARM  SEGMENT  MORE   (4 terlihat, geser untuk sisanya)
#   MORE      MAINTENANCE  HISTORY  SETTING  SYSTEM
#
#   y   0- 21  header     logo, judul, status mesin, RS485, kamera, alarm, jam
#   y  24-199  isi
#   y 202-239  tab bar
# ============================================================
VERSI_HMI = '2.0'
FOLDER_ASET = os.path.join(FOLDER, 'assets')
FILE_RESEP = os.path.join(FOLDER, 'resep.json')
FILE_PRODUKSI = os.path.join(FOLDER, 'produksi_riwayat.json')

# Kartu tembus pandang (dipakai dengan ImageDraw mode 'RGBA') -- latar JPEG tetap terlihat.
KACA = KARTU + (228,)
KACA_GELAP = LATAR + (235,)


# ============================================================
# ASET JPEG -- dibaca, dirapikan & diskalakan SEKALI saat start, lalu dipakai ulang dari RAM.
# Semua opsional: file yang tidak ada diganti latar polos, layar tetap jalan.
# ============================================================
class AsetGambar:
    FILE = {'splash': '01_splash.jpg', 'loading': '02_loading.jpg', 'cek': '03_system_check.jpg',
            'utama': '04_bg_main.jpg', 'panel': '06_bg_panel.jpg'}   # 05_machine_diagram: diganti ikon

    def __init__(self, folder=FOLDER_ASET):
        self.img, self._rata = {}, {}
        for kunci, nama in self.FILE.items():
            path = os.path.join(folder, nama)
            if not os.path.exists(path):
                log(f"[panel] aset {nama} tidak ada -- pakai latar polos")
                continue
            try:
                im = Image.open(path)
                im.load()
                im = im.convert('RGB')
                if im.size != (LEBAR, TINGGI):
                    im = im.resize((LEBAR, TINGGI), Image.LANCZOS)
                self.img[kunci] = self._rapikan(im)
            except (OSError, ValueError) as e:
                log(f"!! [panel] aset {nama} rusak, tidak dipakai: {e}")
        # Turunan, juga dibuat sekali. Header bawaan JPEG (status & jam yang tercetak MATI)
        # dipotong; header hidup digambar sendiri.
        self.latar_utama = self._tanpa_header('utama', 62)
        self.latar_panel = self._tanpa_header('panel', 66)
        self.logo = None
        if 'splash' in self.img:  # logo IMT di tengah splash
            self.logo = self.img['splash'].crop((128, 6, 202, 92)).resize((17, 20), Image.LANCZOS)

    @staticmethod
    def _rapikan(im):
        """Aset hasil potongan lembar mockup membawa pinggiran putih (judul file lembar itu)
        di tepi atas/kiri. Baris/kolom terang di tepi diganti salinan baris/kolom gelap
        pertama -- tidak terlihat, dan aset yang sudah bersih tidak berubah."""
        abu = im.convert('L')
        lebar, tinggi = im.size

        def terang_baris(y):
            return sum(abu.crop((0, y, lebar, y + 1)).getdata()) / lebar > 150

        def terang_kolom(x):
            return sum(abu.crop((x, 0, x + 1, tinggi)).getdata()) / tinggi > 150
        atas = 0
        while atas < 30 and terang_baris(atas):
            atas += 1
        kiri = 0
        while kiri < 30 and terang_kolom(kiri):
            kiri += 1
        if atas:
            baris = im.crop((0, atas, lebar, atas + 1))
            for y in range(atas):
                im.paste(baris, (0, y))
        if kiri:
            kolom = im.crop((kiri, 0, kiri + 1, tinggi))
            for x in range(kiri):
                im.paste(kolom, (x, 0))
        return im

    def _tanpa_header(self, kunci, y0):
        if kunci not in self.img:
            return None
        latar = Image.new('RGB', (LEBAR, TINGGI), LATAR)
        latar.paste(self.img[kunci].crop((0, y0, LEBAR, TINGGI)).resize((LEBAR, TINGGI - 22), Image.LANCZOS),
                    (0, 22))
        return latar

    def ambil(self, kunci):
        """Salinan untuk digambari (copy = salin memori, bukan decode JPEG)."""
        im = {'utama': self.latar_utama, 'panel': self.latar_panel}.get(kunci, self.img.get(kunci))
        return im.copy() if im is not None else Image.new('RGB', (LEBAR, TINGGI), LATAR)

    def warna_rata(self, kunci, kotak_):
        """Warna rata-rata satu area -- untuk menutup tulisan mati di JPEG secara halus."""
        im = self.img.get(kunci)
        if im is None:
            return LATAR
        k = (kunci, kotak_)
        if k not in self._rata:
            self._rata[k] = im.crop(kotak_).resize((1, 1), Image.BOX).getpixel((0, 0))
        return self._rata[k]


# ============================================================
# FAULT -- nama dari enum FaultCode di include/registers.h tiap firmware. Tindakan = saran
# untuk operator. Kode yang tidak ada di tabel ditampilkan apa adanya, tidak ditebak.
# ============================================================
_FAULT_UMUM = {
    1: ('COMM_TIMEOUT', 'Node tidak menerima perintah Modbus dalam batas waktunya.',
        'Pastikan program Orange Pi berjalan & kabel RS485 A/B tersambung, lalu RESET FAULT.'),
    5: ('ESTOP_ACTIVE', 'Tombol E-STOP node ditekan.',
        'Pastikan area aman, lepaskan E-STOP, lalu RESET FAULT.'),
    20: ('IO_EXPANDER_MISSING', 'Modul IO (MCP23017) tidak menjawab di I2C.',
         'Periksa kabel I2C & daya modul IO, lalu nyalakan ulang node.'),
}
FAULT_INFO = {
    SORTER: {**_FAULT_UMUM, **{
        10: ('HOPPER_JAM', 'Hopper macet -- putaran umpan tidak selesai.',
             'Bersihkan sumbatan hopper, periksa motor hopper, lalu RESET FAULT.')}},
    PICKER: dict(_FAULT_UMUM),
    DISPENSER: {**_FAULT_UMUM, **{
        10: ('STOCK_EMPTY', 'Stok package kosong.', 'Isi ulang magazine package, lalu RESET FAULT.'),
        11: ('BOX_NOT_ARRIVED', 'Package tidak sampai di sensor dalam batas waktu.',
             'Periksa conveyor Dispenser & sensor PROX, singkirkan hambatan.'),
        12: ('PUSH_STUCK', 'Pendorong macet.', 'Periksa hambatan mekanis pada pendorong.'),
        13: ('MIDDLE_PACKAGE_MISSING', 'Package baru tidak terkonfirmasi di sensor tengah.',
             'Periksa servo dispenser, stok package, dan sensor tengah.')}},
    STOCKER: {**_FAULT_UMUM, **{
        10: ('HOMING_FAILED', 'Homing gagal -- limit switch tidak tersentuh dalam 30 s.',
             'Periksa limit switch & motor axis, lalu HOME ALL.'),
        11: ('RACK_IDX_INVALID', 'Nomor rak tidak sah.', 'Periksa urutan rak di SETTING > Rack.'),
        12: ('NOT_HOMED', 'Axis belum homing.', 'Jalankan HOME di MAINTENANCE > STOCKER.'),
        13: ('PUSH_STUCK', 'Pusher Y tidak kembali (limit Y tidak tersentuh).',
             'Periksa hambatan mekanis pusher.')}},
}


def info_alarm(kode, pesan=''):
    """kode alarm (format Riwayat) -> (node, judul, deskripsi, tindakan)."""
    m = re.fullmatch(r'([EWI])-(\d)(\d\d|ES|0|M)', kode or '')
    if kode == 'E-PRD':
        return ('PRODUKSI', 'PRODUKSI DITAHAN', pesan or STATUS.get('alasan', ''),
                'Lihat LOG, perbaiki penyebabnya, lalu START lagi.')
    if not m or int(m.group(2)) not in NAMA:
        return ('-', pesan or kode, pesan, '-')
    sid, akhir = int(m.group(2)), m.group(3)
    node = NAMA[sid]
    if akhir == 'ES':
        return (node, 'E-STOP AKTIF', 'Tombol E-STOP node ditekan.', _FAULT_UMUM[5][2])
    if akhir == '0':
        return (node, 'TIDAK MENJAWAB', 'Node tidak menjawab Modbus.',
                'Periksa daya node, kabel RS485 A/B, dan alamat slave.')
    if akhir == 'M':
        return (node, 'OPERATOR DI MENU', 'Menu LCD node terbuka -- semua perintah diabaikan.',
                'Keluar dari menu LCD node tersebut.')
    nama, desk, aksi = FAULT_INFO[sid].get(int(akhir), (f'FAULT {int(akhir)}', pesan or 'Kode fault firmware.',
                                                        'Lihat LAST_FAULT_CODE di NODE, lalu RESET FAULT.'))
    return (node, nama, desk, aksi)


# ============================================================
# RESEP -- kumpulan nilai sorting.config yang sudah ada (tidak ada parameter baru), plus
# kecepatan conveyor Sorter yang dikirim lewat command SET_CONVEYOR_SPEED saat LOAD.
# ============================================================
RESEP_KUNCI = [   # (kunci resep, bagian config | None, label)
    ('batch_size', 'produksi', 'Batch Size'),
    ('jumlah_siklus', 'produksi', 'Target Cycle'),
    ('pakai_kamera', 'huskylens', 'Camera'),
    ('conveyor_pwm', None, 'Conv. Speed'),
    ('jeda_hopper', 'produksi', 'Hopper Pause'),
    ('delay_picker_s', 'delay', 'Picker Delay'),
    ('rak_random', 'rak', 'Rack Order'),
]


class Resep:
    def __init__(self):
        try:
            with open(FILE_RESEP, encoding='utf-8') as f:
                data = json.load(f)
            self.daftar, self.aktif = data['daftar'], data.get('aktif')
        except (FileNotFoundError, ValueError, KeyError):
            self.daftar, self.aktif = [self.dari_config('STANDAR')], 'STANDAR'
        self.pilih = 0
        for i, r in enumerate(self.daftar):
            if r['nama'] == self.aktif:
                self.pilih = i

    @staticmethod
    def dari_config(nama):
        r = {'nama': nama, 'conveyor_pwm': None}
        for kunci, bagian, _ in RESEP_KUNCI:
            if bagian:
                r[kunci] = nilai_sekarang(kunci)
        return r

    def simpan(self):
        with open(FILE_RESEP + '.tmp', 'w', encoding='utf-8') as f:
            json.dump({'aktif': self.aktif, 'daftar': self.daftar}, f, indent=1)
        os.replace(FILE_RESEP + '.tmp', FILE_RESEP)

    def sekarang(self):
        return self.daftar[self.pilih]


# ============================================================
# RIWAYAT PRODUKSI -- dihitung panel dari counter Sorter yang sudah dibaca Pemantau
# (tidak ada bacaan bus tambahan) dan dari nomor batch STATUS.
# ============================================================
class RiwayatProduksi:
    def __init__(self):
        try:
            with open(FILE_PRODUKSI, encoding='utf-8') as f:
                d = json.load(f)
            self.hari, self.batch = d.get('hari', {}), d.get('batch', [])
        except (FileNotFoundError, ValueError):
            self.hari, self.batch = {}, []
        self._lalu = None              # (ok, reject) bacaan sebelumnya
        self._batch_lalu = STATUS['batch']
        self._t_batch = None
        self._t = time.monotonic()
        self._simpan_berikut = time.monotonic() + 30

    def _hari_ini(self):
        k = time.strftime('%Y-%m-%d')
        return self.hari.setdefault(k, {'ok': 0, 'reject': 0, 'detik_jalan': 0, 'batch': 0})

    def perbarui(self, ok, rej):
        sekarang = time.monotonic()
        h = self._hari_ini()
        if STATUS['fase'] == 'JALAN':
            h['detik_jalan'] += sekarang - self._t
        self._t = sekarang
        if ok is not None and rej is not None:
            if self._lalu is not None:
                for i, kunci in ((0, 'ok'), (1, 'reject')):
                    baru = (ok, rej)[i]
                    lama = self._lalu[i]
                    # counter direset (turun) -> nilai baru seluruhnya objek baru
                    h[kunci] += baru - lama if baru >= lama else baru
            self._lalu = (ok, rej)
        b = STATUS['batch']
        if b != self._batch_lalu:
            if b > self._batch_lalu and b > 0:
                durasi = None if self._t_batch is None else round(sekarang - self._t_batch, 1)
                self.batch.append({'t': time.time(), 'nomor': b, 'rak': STATUS['rak'], 'durasi': durasi})
                self.batch = self.batch[-200:]
                h['batch'] += 1
                self._simpan_berikut = 0
            self._t_batch = sekarang if b > 0 else None
            self._batch_lalu = b
        if sekarang >= self._simpan_berikut:
            self._simpan_berikut = sekarang + 30
            self.simpan()

    def simpan(self):
        hari = dict(sorted(self.hari.items())[-60:])
        try:
            with open(FILE_PRODUKSI + '.tmp', 'w', encoding='utf-8') as f:
                json.dump({'hari': hari, 'batch': self.batch}, f)
            os.replace(FILE_PRODUKSI + '.tmp', FILE_PRODUKSI)
        except OSError:
            pass


# ============================================================
# SETTING -- kategori -> baris. Baris: (bagian, kunci, label) = nilai sorting.config;
# ('lcd', kunci, label) = display.config, HANYA DIBACA (diubah di file); ('aksi', label, nama method).
# ============================================================
KATEGORI_SETTING = [
    ('Production', [('produksi', 'batch_size', 'Objek per batch'),
                    ('produksi', 'jumlah_siklus', 'Siklus (0=semua rak)'),
                    ('produksi', 'jeda_hopper', 'Jeda hopper 0/1/2'),
                    ('produksi', 'lanjut_hopper', 'Hopper lanjut di HOME'),
                    ('produksi', 'stocker_paralel', 'Stocker paralel'),
                    ('delay', 'delay_batch_s', 'Delay batch (s)'),
                    ('delay', 'delay_picker_s', 'Delay picker (s)'),
                    ('delay', 'delay_dispenser_s', 'Delay dispenser (s)'),
                    ('delay', 'delay_stocker_s', 'Delay stocker (s)')]),
    ('Calibration', [('aksi', 'Kalibrasi sentuh', 'mulai_kalibrasi'),
                     ('huskylens', 'pakai_kamera', 'Pakai kamera'),
                     ('huskylens', 'jumlah_sampel', 'Jumlah sampel'),
                     ('huskylens', 'min_dominan', 'Min dominan VALID'),
                     ('huskylens', 'batas_ambil_s', 'Batas ambil (s)'),
                     ('huskylens', 'min_kosong_s', 'Min kosong (s)'),
                     ('huskylens', 'id_valid', 'ID valid'),
                     ('huskylens', 'id_kosong', 'ID kosong')]),
    ('Rack', [('aksi', 'Kelola rak', 'buka_rak'),
              ('rak', 'rak_random', 'Rak acak'),
              ('rak', 'pakai_sensor_rak', 'Pakai sensor rak'),
              ('rak', 'urutan_rak', 'Urutan rak')]),
    ('Display', [('lcd', 'lcd_judul', 'Judul'), ('lcd', 'lcd_spi_hz', 'SPI Hz'),
                 ('lcd', 'lcd_rotasi', 'Rotasi'), ('lcd', 'lcd_bgr', 'BGR'), ('lcd', 'lcd_invert', 'Invert'),
                 ('lcd', 'lcd_pin_dc', 'Pin DC'), ('lcd', 'lcd_pin_cs', 'Pin CS'),
                 ('lcd', 'lcd_pin_rst', 'Pin RESET'), ('lcd', 'lcd_pin_led', 'Pin LED'),
                 ('lcd', 'touch_cs', 'Touch CS'), ('lcd', 'touch_irq', 'Touch IRQ')]),
    ('System', [('aksi', 'Info sistem', 'buka_sistem'),
                ('indikator', 'buzzer_alarm', 'Buzzer alarm'),
                ('indikator', 'alarm_nyala_s', 'Alarm bunyi (s)'),
                ('indikator', 'alarm_mati_s', 'Alarm diam (s)'),
                ('indikator', 'buzzer_valid', 'Bunyi VALID'),
                ('indikator', 'buzzer_invalid', 'Bunyi INVALID'),
                ('indikator', 'buzzer_boot', 'Bunyi boot'),
                ('indikator', 'sw1_tahan_s', 'SW1 tahan STOP (s)'),
                ('port', 'rs485_baud', 'Baud RS485'),
                ('port', 'huskylens_baud', 'Baud HuskyLens'),
                ('port', 'rs485_port', 'Port RS485'),
                ('port', 'huskylens_port', 'Port HuskyLens')]),
    ('Engineering', [('aksi', 'Log program', 'buka_log'),
                     ('batas_waktu', 't_move_package', 'MOVE_PACKAGE (s)'),
                     ('batas_waktu', 't_full_cycle', 'FULL_CYCLE (s)'),
                     ('batas_waktu', 't_package_ujung', 'Ke UJUNG (s)'),
                     ('batas_waktu', 't_dispenser_siap', 'Dispenser siap (s)')]),
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


def teks_nilai(bagian, kunci, v):
    """Nilai untuk DITAMPILKAN. Pilihan 0/1 tampil OFF/ON; yang ditulis ke config tetap 0/1."""
    if bagian == 'lcd':
        satu_nol = kunci in ('lcd_bgr', 'lcd_invert')
    else:
        satu_nol = SKEMA_CONFIG.get(bagian, {}).get(kunci.upper()) is _nol_satu
    if satu_nol and v in (0, 1, True, False):
        return 'ON' if v else 'OFF'
    return tampil_nilai(v)


def tampil_nilai(v):
    if v is None:
        return '-'
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


def detik_pendek(v):
    if v is None:
        return '-'
    return f"{v:.1f} s" if v < 60 else f"{int(v) // 60}:{int(v) % 60:02d} m"


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
        self.putaran = 0                                # jumlah putaran baca selesai (cek awal)
        self.henti = threading.Event()

    def run(self):
        while True:
            if self.bus is not None:
                for sid in NAMA:
                    self.node[sid] = self._baca_node(sid)
                if self.detail_sid is not None:
                    self.detail = self._baca_detail(self.detail_sid)
            self.putaran += 1
            if self.henti.wait(1.0):
                return

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
                    sebab = FAULT_INFO[sid].get(n['fault'], ('',))[0]
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
    hasil.append(('Versi HMI', VERSI_HMI))
    return hasil


def _cek_cpu_memori():
    try:
        with open('/proc/meminfo') as f:
            info = {b.split(':')[0]: int(b.split()[1]) for b in f if ':' in b}
        sisa = info.get('MemAvailable', info.get('MemFree', 0)) // 1024
        beban = os.getloadavg()[0]
        status = 'ok' if sisa >= 50 else ('warn' if sisa >= 20 else 'fail')
        return status, f"RAM {sisa} MB, load {beban:.1f}"
    except (OSError, AttributeError, ValueError):
        return 'warn', 'tidak terbaca'


def _cek_penyimpanan():
    import shutil
    try:
        sisa = shutil.disk_usage(FOLDER).free // 2**20
    except OSError:
        return 'warn', 'tidak terbaca'
    return ('ok' if sisa >= 200 else ('warn' if sisa >= 50 else 'fail')), f"{sisa} MB sisa"


# ============================================================
# APLIKASI
# ============================================================
TAB = [('home', 'HOME', 'rumah'), ('auto', 'AUTO', 'play'), ('manual', 'MANUAL', 'kunci_pas'),
       ('alarm', 'ALARM', 'alarm'), ('recipe', 'SEGMENT', 'resep'), ('more', 'MORE', 'lainnya')]
TAB_TAMPIL = 4
LEBAR_TAB = LEBAR // TAB_TAMPIL
Y_ISI, Y_TAB = 26, 202

# Langkah layar LOADING (teks sama dengan yang tercetak di 02_loading.jpg).
LANGKAH_BOOT = ['Initializing Hardware', 'Loading Configuration', 'Preparing Modules', 'Starting Communication']
# Item SYSTEM CHECK: (kunci, label). Urutan = 2 kolom x 5 baris, sama dengan 03_system_check.jpg.
ITEM_CEK = [('cpu', 'CPU & Memory'), ('disk', 'Storage'), ('rs485', 'RS485 Communication'),
            ('kamera', 'HuskyLens Camera'), ('tft', 'TFT Display'), ('touch', 'Touch Screen'),
            (SORTER, 'Sorter Node'), (PICKER, 'Picker Node'), (DISPENSER, 'Dispenser Node'),
            (STOCKER, 'Stocker Node')]
WARNA_CEK = {'ok': HIJAU, 'cek': KUNING, 'tunggu': ABU_GELAP, 'fail': MERAH, 'warn': ORANYE, 'off': ABU_GELAP}
TEKS_CEK = {'ok': 'OK', 'cek': 'CHECK', 'tunggu': 'WAIT', 'fail': 'FAIL', 'warn': 'WARN', 'off': 'OFF'}

# Field layar detail node (spesifikasi HMI) -> label register Pemantau.
FIELD_DETAIL = {
    SORTER: [('STATE', 'STATE'), ('FAULT', 'FAULT_CODE'), ('MAIN MODE', 'MAIN_MODE_ACTIVE'),
             ('ACTIVITY', 'ACTIVITY_CODE'), ('PASS', 'PASS_COUNT'), ('REJECT', 'REJECT_COUNT'),
             ('MISSED', 'REJECT_MISSED_COUNT'), ('SPEED', 'SPEED_LAST_MM_S'), ('UPTIME', 'UPTIME_SEC'),
             ('I2C ERROR', 'I2C_ERROR_COUNT')],
    PICKER: [('STATE', 'STATE'), ('POSE', 'CURRENT_POSE'), ('MOVEMENT', 'GERAKAN_AKTIF'),
             ('ACTIVITY', 'ACTIVITY_CODE'), ('UPTIME', 'UPTIME_SEC'), ('FAULT', 'FAULT_CODE')],
    DISPENSER: [('STATE', 'STATE'), ('STAGE', 'PIPELINE_STAGE'), ('PKG READY', 'PACKAGE_READY_FLAG'),
                ('MIDDLE PKG', 'MIDDLE_PACKAGE_PRESENT'), ('END PKG', 'UJUNG_PACKAGE_PRESENT'),
                ('STOCK EMPTY', 'STOCK_EMPTY_FLAG'), ('UPTIME', 'UPTIME_SEC'), ('FAULT', 'FAULT_CODE')],
    STOCKER: [('STATE', 'STATE'), ('HOMED', 'ALL_HOMED_FLAG'), ('RACK', 'CURRENT_RACK_IDX'),
              ('OCCUPANCY', 'RACK_OCCUPIED_BITMASK'), ('UPTIME', 'UPTIME_SEC'), ('FAULT', 'FAULT_CODE')],
}
FLAG_YA_TIDAK = ('PACKAGE_READY_FLAG', 'MIDDLE_PACKAGE_PRESENT', 'UJUNG_PACKAGE_PRESENT', 'STOCK_EMPTY_FLAG',
                 'ALL_HOMED_FLAG')

# MANUAL CONTROL -- nilai yang dibaca layar sensor per node: (label, register, hitung selisih?)
SENSOR_NODE = {
    SORTER: [('PASS', 'PASS_COUNT', True), ('REJECT', 'REJECT_COUNT', True),
             ('MISSED', 'REJECT_MISSED_COUNT', True), ('UKUR KEC.', 'SPEED_SAMPLE_COUNT', True),
             ('KECEPATAN', 'SPEED_LAST_MM_S', False), ('WAKTU TEMPUH', 'SPEED_LAST_MS', False)],
    PICKER: [('POSE', 'CURRENT_POSE', False), ('GERAKAN', 'GERAKAN_AKTIF', False),
             ('ADEGAN', 'ADEGAN_KE', False), ('ACTIVITY', 'ACTIVITY_CODE', False)],
    DISPENSER: [('TENGAH', 'MIDDLE_PACKAGE_PRESENT', False), ('UJUNG', 'UJUNG_PACKAGE_PRESENT', False),
                ('PKG READY', 'PACKAGE_READY_FLAG', False), ('STOK KOSONG', 'STOCK_EMPTY_FLAG', False),
                ('TIBA TENGAH', 'MIDDLE_ARRIVAL_COUNT', True), ('TIBA UJUNG', 'UJUNG_ARRIVAL_COUNT', True)],
    STOCKER: [('HOMED', 'ALL_HOMED_FLAG', False), ('RAK', 'CURRENT_RACK_IDX', False),
              ('ISI RAK', 'RACK_OCCUPIED_BITMASK', False), ('ACTIVITY', 'ACTIVITY_CODE', False)],
}

# Command yang ack-nya baru dikirim firmware setelah gerakannya SELESAI (ackPending di firmware).
# Untuk yang lain ack datang segera, jadi T_ACK cukup.
ACK_SAAT_SELESAI = {(STOCKER, 'HOME_ALL'): 'T_STOCKER_HOMING', (STOCKER, 'GOTO_READY'): 'T_FULL_CYCLE',
                    (STOCKER, 'MOVE_TO_RACK'): 'T_FULL_CYCLE', (STOCKER, 'PUSH_BOX'): 'T_FULL_CYCLE',
                    (STOCKER, 'FULL_CYCLE'): 'T_FULL_CYCLE', (PICKER, 'MOVE_PACKAGE'): 'T_MOVE_PACKAGE'}

# Kecepatan conveyor bawaan firmware (cfg.conveyorSpeed) -- tampil sebelum panel mengirim nilai.
KECEPATAN_BAWAAN_PWM = {SORTER: 180, DISPENSER: 160}

GERAKAN_PICKER = [('H>R', 'Home>Ready', 3), ('R>P', 'Ready>Pick', 1), ('P>H', 'Pick>Home', 0),
                  ('H>Pl', 'Home>Place', 2), ('Pl>H', 'Place>Home', 0)]   # (tombol, nama, pose tujuan)


def batas_ack(sid, label):
    k = ACK_SAAT_SELESAI.get((sid, label))
    return max(T_ACK, globals()[k]) if k else T_ACK


def cari_command(sid, label):
    for c in OPCODES[NAMA[sid]]:
        if c[0] == label:
            return c
    return None


def _picker_diam(n):
    return n is not None and not n['r'].get(11) and n['r'].get(17) in (0xFF, None)


def _aktivitas(n, reg):
    return None if n is None else n['r'].get(reg)


class Panel:
    def __init__(self, layar, sentuh, kalibrasi, bus, judul='SECURA', aset=None, lcd=None):
        self.layar, self.sentuh, self.kal, self.bus, self.judul = layar, sentuh, kalibrasi, bus, judul
        self.aset = aset or AsetGambar(folder=os.path.join(FOLDER, '__tidak_ada__'))
        self.lcd = lcd or {}
        self.pemantau = Pemantau(bus)
        self.riwayat = Riwayat()
        self.produksi_riwayat = RiwayatProduksi()
        self.resep = Resep()
        self.produksi = None            # thread mesin produksi
        self.tab, self.sub = 'home', None
        self.halaman = 0
        self.kategori = 0
        self.edit = None                # popup edit nilai
        self.popup = None               # popup konfirmasi
        self.pesan = None               # (teks, warna, sampai)
        self.node_sid = SORTER
        self.cmd = None
        self.kirim_hasil = None         # hasil command terakhir: (jenis, teks)
        self.alarm_pilih = None         # alarm yang dibuka di layar detail
        self.manual_sid, self.manual_sensor, self.manual_siap = SORTER, False, False
        self.sensor_acuan, self.kecepatan, self.durasi_pusher = None, {}, 0.5
        self.urutan_jalan, self.batal, self.acuan = None, threading.Event(), None
        self._hasil_terakhir, self._hasil_t = None, 0.0
        self.alarm_mode = 'aktif'
        self.history_mode = 'harian'
        self.tombol = []
        self.kalibrasi_aktif = False
        self.kal_titik = [(30, 30), (290, 30), (30, 210)]
        self.kal_mentah = []
        self.log_geser = 0
        self.log_kembali = None         # tujuan tombol < di layar LOG (None = MORE > SYSTEM)
        self.kamera_hasil = False       # MANUAL > CAMERA: tampilkan tabel kamera_hasil.csv
        self.hasil_hal = 0
        self.sistem_mode = 'opi'        # halaman MORE > SYSTEM: 'opi' | 'node'
        self.ota_sid = SORTER           # node yang dipilih di layar UPDATE FIRMWARE
        self.ota = None                 # {'sid', 'tahap', 'persen', 'jenis', 'pesan'} proses OTA
        self.ota_jalan = None           # node yang SEDANG di-update (alarm tidak menjawab diredam)
        self.kal_sid, self.kal_pilih = SORTER, 0   # MORE > SYSTEM > KALIBRASI: node & backup terpilih
        self.kal_kerja = None           # {'tahap', 'persen', 'pesan', 't'} backup / restore berjalan
        self.kal_node = {}              # sid -> {'crc'} atau {'galat'} hasil CEK / BACKUP terakhir
        self._kal_cache = (0.0, None)   # (waktu, {sid: daftar backup}) -- file tidak dibaca tiap frame
        self.versi, self.versi_t = {}, None   # hasil versi_node() tiap node, waktu dibaca
        self.tab_awal = 0
        self.geser = None
        self.hitungan = collections.deque()
        self.batch_lalu, self.batch_waktu, self.cycle_time = 0, None, None
        # Boot: state machine, TIDAK memblokir loop layar.
        self.boot = 'splash'
        self.boot_t = time.monotonic()
        self.boot_langkah = [0.0] * len(LANGKAH_BOOT)     # 0..1 per langkah loading
        self.cek = {k: ('tunggu', '') for k, _ in ITEM_CEK}
        self.cek_selesai = False

    # ================= BOOT =================
    def mulai_boot(self):
        threading.Thread(target=self._kerja_boot, daemon=True).start()

    def _kerja_boot(self):
        """Pekerjaan nyata di balik LOADING & SYSTEM CHECK. Thread sendiri: layar tetap hidup."""
        def langkah(i, fungsi, minimal=0.4):
            t0 = time.monotonic()
            self.boot_langkah[i] = 0.3
            log(f"[boot] {i + 1}/{len(LANGKAH_BOOT)} {LANGKAH_BOOT[i]} ...")
            try:
                fungsi()
            except Exception as e:
                log(f"!! [boot] {LANGKAH_BOOT[i]}: {e}")
            sisa = minimal - (time.monotonic() - t0)
            if sisa > 0:
                BERHENTI_PANEL.wait(sisa)
            self.boot_langkah[i] = 1.0
        BERHENTI_PANEL.wait(1.8)            # splash terlihat
        self.boot, self.boot_t = 'loading', time.monotonic()
        langkah(0, lambda: None)            # GPIO, SPI LCD & touch sudah dibuka sebelum panel
        langkah(1, lambda: (muat_ulang_config(), None)[1])
        langkah(2, lambda: None)            # aset JPEG & font sudah dimuat
        langkah(3, lambda: self.pemantau.is_alive() or self.pemantau.start())
        BERHENTI_PANEL.wait(0.3)
        self.boot, self.boot_t = 'cek', time.monotonic()
        log("[boot] SYSTEM CHECK")
        self._system_check()
        for k, label in ITEM_CEK:
            status, ket = self.cek[k]
            log(f"[boot]   {'OK' if status == 'ok' else '!!'} {label:20s} {TEKS_CEK[status]:5s} {ket}")
        if self.bus is not None:
            log("[boot] versi firmware node:")
            self.baca_versi()
            for sid in ALUR:
                v = self.versi.get(sid, {})
                log(f"[boot]   {NAMA[sid]:10s} {v.get('fw', '-')}")
        self.cek_selesai = True
        gagal = [label for k, label in ITEM_CEK if self.cek[k][0] == 'fail']
        log(f"[boot] SYSTEM READY -- {'semua OK' if not gagal else 'GAGAL: ' + ', '.join(gagal)}")
        INDIKATOR.bunyi('boot_error' if gagal else 'boot_ok')
        BERHENTI_PANEL.wait(0.8)
        self.boot, self.boot_t = 'siap', time.monotonic()

    def _system_check(self):
        def set_(k, status, ket=''):
            self.cek[k] = (status, ket)

        def item(k, fungsi):
            set_(k, 'cek')
            BERHENTI_PANEL.wait(0.15)
            try:
                set_(k, *fungsi())
            except Exception as e:
                set_(k, 'fail', str(e))
        item('cpu', _cek_cpu_memori)
        item('disk', _cek_penyimpanan)
        item('rs485', lambda: ('ok', RS485_PORT) if self.bus is not None else ('fail', 'tidak bisa dibuka'))
        if PAKAI_KAMERA:
            item('kamera', lambda: ('ok', HUSKYLENS_PORT) if cek_huskylens() else ('fail', 'lihat LOG'))
        else:
            set_('kamera', 'off', 'pakai_kamera = 0')
        item('tft', lambda: ('ok', f"{LEBAR}x{TINGGI}"))
        item('touch', lambda: ('fail', 'tidak ada') if self.sentuh is None and self.layar is not None
             else (('ok', '') if self.kal.ada() else ('warn', 'perlu kalibrasi')))
        for sid in ALUR:
            set_(sid, 'cek')
        # Node dibaca Pemantau (satu-satunya pembaca bus layar) -- tunggu satu putaran penuh.
        awal, batas = self.pemantau.putaran, time.monotonic() + 4
        while self.pemantau.putaran < awal + 1 and time.monotonic() < batas and not BERHENTI_PANEL.is_set():
            BERHENTI_PANEL.wait(0.1)
        for sid in ALUR:
            n = self.pemantau.node.get(sid)
            if n is None:
                set_(sid, 'fail', 'tidak menjawab')
            elif n['fault'] or n['state'] in (ST_FAULT, ST_ESTOP):
                # E-STOP disebut lebih dulu; kode fault diberi nama (dulu hanya "fault 20" --
                # E-STOP murni malah tampil "fault 0")
                teks = ['E-STOP'] if n['state'] == ST_ESTOP else []
                if n['fault']:
                    teks.append(FAULT_INFO[sid].get(n['fault'], (f"fault {n['fault']}",))[0])
                set_(sid, 'warn', ' + '.join(teks) or 'FAULT')
                log(f"[boot] {NAMA[sid]}: state {STATE_NAMES.get(n['state'], n['state'])}, "
                    f"fault {n['fault']} {' + '.join(teks)}")
            else:
                set_(sid, 'ok', STATE_NAMES.get(n['state'], ''))
            BERHENTI_PANEL.wait(0.1)

    def lewati_boot(self):
        if self.boot == 'siap' or (self.boot == 'cek' and self.cek_selesai):
            self._masuk_menu()

    def _masuk_menu(self):
        if self.boot:
            log("[boot] masuk menu utama")
        self.boot = None
        if not self.kal.ada() and self.sentuh is not None:
            self.mulai_kalibrasi()

    # ================= bantu =================
    def berjalan(self):
        return self.produksi is not None and self.produksi.is_alive()

    def produksi_aktif(self):
        """Produksi sedang/akan menggerakkan mesin -- MAINTENANCE & RESEP dikunci."""
        return self.berjalan() or STATUS['fase'] in ('PERIKSA', 'STARTUP', 'JALAN')

    def ke(self, tab=None, sub=None, halaman=0):
        if tab:
            self.tab = tab
            i = [t[0] for t in TAB].index(tab)
            self.tab_awal = min(max(self.tab_awal, i - TAB_TAMPIL + 1), i)
        self.sub, self.halaman = sub, halaman
        if tab == 'alarm' and sub is None:
            self.riwayat.belum_dilihat = 0
            INDIKATOR.diamkan()          # alarm sudah dilihat operator -> buzzer diam
        if tab and tab != 'manual':
            self.manual_siap = self.manual_sensor = False   # keluar MANUAL = harus diaktifkan ulang
            UJI_KAMERA.stop()
        sid = self.manual_sid if self.tab == 'manual' else (self.node_sid if sub in ('detail', 'raw') else None)
        if sid not in NAMA:
            sid = None
        if sid != self.pemantau.detail_sid:
            self.pemantau.detail = []
        self.pemantau.detail_sid = sid

    def mulai_kalibrasi(self):
        self.kalibrasi_aktif, self.kal_mentah = True, []

    def buka_rak(self):
        self.ke('more', 'rak')

    def buka_sistem(self):
        self.ke('more', 'sistem')

    def buka_log(self, kembali=None):
        """kembali: ke mana tombol < di layar LOG pergi (bawaan: MORE > SYSTEM)."""
        self.log_geser = 0
        self.log_kembali = kembali
        self.ke('more', 'log')

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

    def status_mesin(self):
        """Status mesin untuk header & HOME. Urutan prioritas: ESTOP, FAULT, RUN, STARTING, IDLE."""
        node = self.pemantau.node
        if any(n and n['state'] == ST_ESTOP for n in node.values()):
            return 'ESTOP', MERAH
        fase = STATUS['fase']
        if fase in ('DITAHAN', 'GAGAL CEK', 'GAGAL STARTUP') or any(
                n and (n['fault'] or n['state'] == ST_FAULT) for n in node.values()):
            return 'FAULT', MERAH
        if fase == 'JALAN':
            return 'RUN', HIJAU
        if fase in ('PERIKSA', 'STARTUP'):
            return 'STARTING', KUNING
        return 'IDLE', BIRU

    def status_rs485(self):
        menjawab = sum(n is not None for n in self.pemantau.node.values())
        if self.bus is None or (not menjawab and self.pemantau.putaran):
            return 'ERROR', MERAH
        if menjawab == len(NAMA) or not self.pemantau.putaran:
            return 'OK', HIJAU
        return 'PARTIAL', KUNING

    def status_kamera(self):
        if not PAKAI_KAMERA:
            return 'OFF', ABU_GELAP
        k = STATUS.get('kamera')
        if k == 'OK' or (k is None and self.cek['kamera'][0] == 'ok'):
            return 'OK', HIJAU
        if k == 'GAGAL' or self.cek['kamera'][0] == 'fail':
            return 'ERROR', MERAH
        return 'SIAP', BIRU

    def status_kerja(self, sid):
        """Status singkat chip: apa yang sedang DIKERJAKAN node. Masalah tetap didahulukan."""
        teks, w = self.status_node(sid)
        if teks in ('--', 'MENU', 'ESTOP') or w == MERAH:
            return teks, w
        n = self.pemantau.node[sid]
        r = n['r']
        if sid == SORTER:
            return ('RUN', HIJAU) if n['state'] == ST_RUN else ('IDLE', BIRU)
        if sid == PICKER:
            return ('RUN', HIJAU) if r.get(11) else (NAMA_POSE.get(r.get(10), 'IDLE'), BIRU)
        if sid == DISPENSER:
            tahap = r.get(26)
            if tahap == 15:
                return 'AT END', KUNING
            if tahap == 9:
                return 'READY', BIRU
            return ('RUN', HIJAU) if tahap else ('OFF', ABU_GELAP)
        return ('RUN', HIJAU) if r.get(13) else ('READY', BIRU)

    def status_hopper(self):
        n = self.pemantau.node.get(SORTER)
        if n is None:
            return '--', ABU_GELAP
        if n['state'] != ST_RUN:
            return 'OFF', ABU_GELAP
        return ('PAUSE', KUNING) if hopper_dijeda else ('ON', HIJAU)

    def aktivitas(self, sid):
        """Aktivitas node dalam kata-kata (AUTO > Current Action)."""
        n = self.pemantau.node.get(sid)
        if n is None:
            return 'NO RESPONSE'
        if n['state'] == ST_ESTOP:
            return 'E-STOP'
        if n['fault']:
            return f"FAULT {FAULT_INFO[sid].get(n['fault'], (n['fault'],))[0]}"
        r = n['r']
        if sid == SORTER:
            a = {0: 'IDLE', 1: 'CONVEYOR RUNNING', 2: 'PUSHER ACTIVE', 3: 'PUSHER MANUAL',
                 4: 'HOPPER TEST'}.get(r.get(13), 'IDLE')
            return a + (' / HOPPER PAUSED' if hopper_dijeda and a != 'IDLE' else '')
        if sid == PICKER:
            g = r.get(17)
            if g not in (None, 0xFF):
                return catatan_untuk('PICKER', 'GERAKAN_AKTIF', g, False).strip().upper().replace('>', ' -> ')
            return 'AT ' + NAMA_POSE.get(r.get(10), '?')
        if sid == DISPENSER:
            tahap = r.get(26)
            if tahap is None:
                return '-'
            return {0: 'OFF', 9: 'READY TO FILL', 10: 'MOVING TO END', 15: 'PACKAGE AT END'}.get(
                tahap, PIPELINE_NAMES[tahap].upper() if tahap < len(PIPELINE_NAMES) else '?')
        a = r.get(13)
        if a == 2:
            return f"MOVING TO RACK {STATUS['rak'] or ''}".strip()
        return {0: 'IDLE', 1: 'HOMING', 3: 'PUSHING BOX', 4: 'RETRACT PUSHER', 5: 'RETURNING HOME',
                6: 'MANUAL MOVE'}.get(a, 'IDLE')

    def aksi_berikut(self):
        """(node, apa yang dikerjakan berikutnya, warna)."""
        fase = STATUS['fase']
        if fase in ('DITAHAN', 'GAGAL CEK', 'GAGAL STARTUP'):
            return '', 'Cek ALARM', MERAH
        if fase in ('PERIKSA', 'STARTUP'):
            return '', 'Menyiapkan', KUNING
        if fase != 'JALAN':
            return '', 'Tekan START', PUTIH
        node = self.pemantau.node
        g = node[PICKER]['r'].get(17) if node.get(PICKER) else None
        if g not in (None, 0xFF):
            return 'PICKER', catatan_untuk('PICKER', 'GERAKAN_AKTIF', g, False).strip(), PUTIH
        a = node[STOCKER]['r'].get(13) if node.get(STOCKER) else None
        if a:
            return 'STOCKER', {1: 'Homing', 2: 'Ke rak', 3: 'Dorong box', 4: 'Tarik pusher',
                               5: 'Kembali'}.get(a, 'Bergerak'), PUTIH
        if node.get(DISPENSER) and node[DISPENSER]['r'].get(26) == 15:
            return 'PICKER', 'Ambil package', PUTIH
        return 'SORTER', f"{max(0, BATCH_SIZE - STATUS['pass_batch'])} objek lagi", PUTIH

    def alarm_aktif(self):
        """Alarm yang SEDANG berlaku (bukan riwayat): dibangun ulang tiap gambar dari data node."""
        hasil = []
        if STATUS['fase'] in ('DITAHAN', 'GAGAL CEK', 'GAGAL STARTUP'):
            hasil.append({'tingkat': 'E', 'kode': 'E-PRD', 'pesan': STATUS.get('alasan') or STATUS['fase'],
                          'sid': None})
        for sid in ALUR:
            n = self.pemantau.node.get(sid)
            if sid == self.ota_jalan:
                continue                # sedang update firmware -- memang tidak menjawab / restart
            if n is None:
                if self.pemantau.putaran:
                    hasil.append({'tingkat': 'W', 'kode': f'W-{sid}0', 'pesan': f'{NAMA[sid]} tidak menjawab',
                                  'sid': None})
                continue
            if n['state'] == ST_ESTOP:
                hasil.append({'tingkat': 'E', 'kode': f'E-{sid}ES', 'pesan': f'{NAMA[sid]} E-STOP', 'sid': sid})
            elif n['fault'] or n['state'] == ST_FAULT:
                nama = FAULT_INFO[sid].get(n['fault'], (f"FAULT {n['fault']}",))[0]
                hasil.append({'tingkat': 'E', 'kode': f"E-{sid}{n['fault']:02d}", 'pesan': f'{NAMA[sid]} {nama}',
                              'sid': sid})
            if n['menu']:
                hasil.append({'tingkat': 'W', 'kode': f'I-{sid}M', 'pesan': f'{NAMA[sid]} di menu LCD', 'sid': None})
        return hasil

    def _catat_hitung(self):
        sekarang = time.monotonic()
        ok, rej = self.sorter(10), self.sorter(11)
        self.produksi_riwayat.perbarui(ok, rej)
        if ok is not None and rej is not None:
            h = self.hitungan
            if h and ok + rej < h[-1][1]:
                h.clear()
            if not h or sekarang - h[-1][0] >= 1:
                h.append((sekarang, ok + rej))
            while sekarang - h[0][0] > 60:
                h.popleft()
        b = STATUS['batch']
        if b != self.batch_lalu:
            if b == 0:
                self.cycle_time = None
            elif self.batch_waktu is not None:
                self.cycle_time = sekarang - self.batch_waktu
            self.batch_waktu = None if b == 0 else sekarang
            self.batch_lalu = b

    def throughput(self):
        h = self.hitungan
        if len(h) < 2 or h[-1][0] - h[0][0] < 10:
            return None
        return (h[-1][1] - h[0][1]) * 60 / (h[-1][0] - h[0][0])

    # ================= aksi produksi =================
    def mulai(self, hanya_cek=False):
        UJI_KAMERA.stop()               # produksi memakai port kamera yang sama
        if self.bus is None:
            self.beri_pesan("RS485 tidak bisa dibuka", MERAH)
            return
        err = muat_ulang_config()
        if err:
            self.beri_pesan("Config salah -- lihat LOG", MERAH)
            log(err)
            return
        STATUS['alasan'] = ''
        self.produksi = threading.Thread(target=jalankan_produksi,
                                         args=(self.bus, bool(PAKAI_KAMERA), hanya_cek), daemon=True)
        self.produksi.start()

    def konfirmasi_mulai(self):
        kosong = [r for r in URUTAN_RAK if r not in rak_terisi]
        if not kosong:
            self.beri_pesan("Tidak ada rak kosong -- SETTING > Rack", MERAH)
            return
        siklus = JUMLAH_SIKLUS or len(kosong)
        self.tanya("Mulai produksi?",
                   [f"Segment: {self.resep.aktif or '-'}",
                    f"{BATCH_SIZE} objek/batch, {siklus} siklus",
                    f"Rak kosong: {', '.join(map(str, kosong))}",
                    f"Kamera: {'YA' if PAKAI_KAMERA else 'TIDAK (semua PASS)'}"],
                   self.mulai, 'START', HIJAU, 'play')

    def stop(self):
        berhenti.set()      # mekanisme shutdown aman yang sudah ada
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

    def kirim(self, sid, c, nilai, verifikasi='amati'):
        """Kirim satu command di thread sendiri. ACK BUKAN bukti selesai: sesudah ack, keadaan
        node diamati (STATE/ACTIVITY/tahap) atau FAULT_CODE dibaca ulang (RESET_FAULT)."""
        label, op = c[0], c[1]
        self.kirim_hasil = ('mengirim', f"{label} ...")
        sebelum = self.pemantau.node.get(sid)
        jejak = lambda n: None if n is None else (n['state'], n['r'].get(ALAMAT[NAMA[sid]]['ACTIVITY_CODE']),
                                                 n['r'].get(26), n['r'].get(17), n['r'].get(10))

        def kerja():
            try:
                seq = self.bus.command(sid, op, nilai)
            except Exception as e:
                self.kirim_hasil = ('gagal', f"{label}: gagal kirim ({e})")
                return
            batas = time.monotonic() + batas_ack(sid, label)
            diack = False
            while time.monotonic() < batas and not diack:
                try:
                    diack = self.bus.baca(sid, R_ACK, percobaan=1) == seq
                except Exception:
                    pass
                if not diack:
                    BERHENTI_PANEL.wait(0.1)
            if not diack:
                self.kirim_hasil = ('gagal', f"{label}: TIDAK di-ack -- node mati / di menu LCD")
                return
            log(f"[panel] {NAMA[sid]} <- {label} (arg {nilai}) di-ack")
            if verifikasi == 'ack':      # setting: tidak ada gerakan yang bisa diamati
                self.kirim_hasil = ('ok', f"{label}: diterima node (ack)")
                return
            if verifikasi == 'fault':
                batas = time.monotonic() + 2
                while time.monotonic() < batas:
                    try:
                        if self.bus.baca(sid, R_FAULT, percobaan=1) == 0:
                            self.kirim_hasil = ('ok', f"{label}: FAULT_CODE = 0, fault hilang")
                            return
                    except Exception:
                        pass
                    BERHENTI_PANEL.wait(0.2)
                self.kirim_hasil = ('gagal', f"{label}: di-ack tapi FAULT masih ada -- atasi penyebabnya")
                return
            self.kirim_hasil = ('mengirim', f"{label}: di-ack, mengamati node...")
            awal = jejak(sebelum)
            batas = time.monotonic() + 3
            while time.monotonic() < batas:
                BERHENTI_PANEL.wait(0.3)
                n = self.pemantau.node.get(sid)
                if n and n['fault']:
                    self.kirim_hasil = ('gagal', f"{label}: node FAULT {n['fault']}")
                    return
                if jejak(n) != awal:
                    self.kirim_hasil = ('ok', f"{label}: node bereaksi -- {STATE_NAMES.get(n['state'], '?')}")
                    return
            self.kirim_hasil = ('kuning', f"{label}: di-ack, tidak ada perubahan terbaca -- cek fisik")
        threading.Thread(target=kerja, daemon=True).start()

    def kirim_command(self):
        """Layar PERINTAH (engineering): command apa pun dari tabel OPCODES."""
        c = self.cmd
        nilai = c[2] if isinstance(c[2], int) else (self.edit['nilai'] if self.edit else 0)
        self.kirim(self.node_sid, c, nilai)

    def reset_fault(self, sid):
        c = cari_command(sid, 'RESET_FAULT')
        if self.berjalan():
            self.beri_pesan("Hentikan produksi dulu", ORANYE)
            return
        self.tanya(f"Reset fault {NAMA[sid]}?", ["Pastikan penyebab fault", "sudah diatasi."],
                   lambda: self.kirim(sid, c, 0, verifikasi='fault'), 'RESET', ORANYE)

    # ================= sentuhan =================
    def tekan(self, p):
        if self.boot:
            self.lewati_boot()
            return
        for t in reversed(self.tombol):   # yang digambar terakhir (popup) didahulukan
            if t.kena(p):
                t.aksi()
                return

    def di_tab_bar(self, p):
        return (self.boot is None and not self.kalibrasi_aktif and p[1] >= Y_TAB - 2
                and not self.popup and not self.edit_popup())

    def geser_mulai(self, p):
        self.geser = {'p0': p, 'dx': 0}

    def geser_ke(self, p):
        dx = p[0] - self.geser['p0'][0]
        berubah = abs(dx - self.geser['dx']) >= 4
        self.geser['dx'] = dx
        return berubah

    def geser_selesai(self):
        g, self.geser = self.geser, None
        maks = max(0, len(TAB) - TAB_TAMPIL)
        if abs(g['dx']) >= 15:
            langkah = -round(g['dx'] / LEBAR_TAB) or (-1 if g['dx'] > 0 else 1)
            self.tab_awal = min(maks, max(0, self.tab_awal + langkah))
            return
        x = g['p0'][0]
        if x < 16 and self.tab_awal > 0:
            self.tab_awal -= 1
        elif x >= LEBAR - 16 and self.tab_awal < maks:
            self.tab_awal += 1
        else:
            i = self.tab_awal + int(x // LEBAR_TAB)
            if i < len(TAB):
                self.ke(TAB[i][0])

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

    # ================= gambar =================
    def interval(self):
        """Jarak gambar ulang: cepat saat animasi boot, 0,5 s untuk data biasa."""
        return 0.2 if self.boot else 0.5

    def gambar(self):
        self.tombol = []
        if self.boot:
            return self._gambar_boot()
        self.riwayat.periksa(self.pemantau.node, self.berjalan())
        self._catat_hitung()
        # LED ALARM & buzzer alarm: fault, E-STOP, node tidak menjawab (bukan info "di menu LCD")
        INDIKATOR.set_alarm(a['kode'] for a in self.alarm_aktif() if not a['kode'].startswith('I-'))
        if self.kalibrasi_aktif:
            img = Image.new('RGB', (LEBAR, TINGGI), LATAR)
            self._kalibrasi(kanvas(img))
            return img
        latar = 'panel' if self.tab == 'more' and self.sub else 'utama'
        img = self._img = self.aset.ambil(latar)
        d = kanvas(img, 'RGBA')
        self._header(d)
        getattr(self, '_' + (self.sub or self.tab))(d)
        self._tab_bar(d)
        if self.popup or self.edit_popup():
            img = Image.blend(img, Image.new('RGB', img.size, HITAM), 0.6)
            d = kanvas(img, 'RGBA')
            self.tombol = []
            if self.popup:
                self._popup(d)
            else:
                self._popup_edit(d)
        if self.pesan and time.monotonic() < self.pesan[2]:
            teks = potong(d, self.pesan[0], F_TEBAL, 290)
            w = int(d.textlength(teks, font=F_TEBAL)) + 24
            kotak(d, (160 - w // 2, 172, 160 + w // 2, 196), 12, self.pesan[1])
            teks_tengah(d, 160, 184, teks, F_TEBAL, HITAM if self.pesan[1] == KUNING else PUTIH)
        return img

    def edit_popup(self):
        return self.edit is not None and self.edit.get('popup')

    # ---------- boot ----------
    def _gambar_boot(self):
        sekarang = time.monotonic()
        if self.boot == 'siap' and sekarang - self.boot_t > 1.6:
            self._masuk_menu()
            return self.gambar()
        return getattr(self, '_boot_' + self.boot)(sekarang)

    def _boot_splash(self, sekarang):
        img = self.aset.ambil('splash')
        d = kanvas(img, 'RGBA')
        if 'splash' in self.aset.img:   # tutup "v1.0.0" tercetak, tulis versi sebenarnya
            d.rectangle((282, 222, 319, 239), fill=self.aset.warna_rata('splash', (250, 222, 280, 239)))
        else:
            ikon(d, 'logo', 160, 62, 34)
            teks_tengah(d, 160, 116, 'IMT', _font(26, True), PUTIH)
            teks_tengah(d, 160, 138, 'INOVASI MULTI SOLUSI TEKNOLOGI', F_LABEL, ABU)
            teks_tengah(d, 160, 170, 'SECURA', _font(24, True), PUTIH)
            teks_tengah(d, 160, 192, 'SORTING AUTOMATION SYSTEM', F_KECIL_TEBAL, BIRU_MUDA)
        teks_kanan(d, 316, 228, f"v{VERSI_HMI}", F_LABEL, ABU)
        return img

    def _boot_loading(self, sekarang):
        img = self.aset.ambil('loading')
        d = kanvas(img, 'RGBA')
        ada = 'loading' in self.aset.img
        if ada:   # tutup progress & ikon status yang tercetak di JPEG
            d.rectangle((26, 124, 262, 148), fill=self.aset.warna_rata('loading', (26, 104, 262, 110)))
            d.rectangle((174, 153, 202, 236), fill=self.aset.warna_rata('loading', (150, 156, 172, 236)))
        else:
            teks_tengah(d, 160, 50, 'IMT', _font(26, True), PUTIH)
            d.text((34, 108), 'Starting System...', font=F_JUDUL, fill=PUTIH)
            kotak(d, (30, 152, 206, 236), 6, KACA)
            for i, teks in enumerate(LANGKAH_BOOT):
                d.text((60, 158 + i * 21), teks, font=F_KECIL, fill=PUTIH)
        persen = int(100 * sum(self.boot_langkah) / len(LANGKAH_BOOT))
        kotak(d, (33, 130, 222, 142), 6, (20, 34, 56))
        if persen:
            kotak(d, (33, 130, 33 + int(189 * persen / 100), 142), 6, (40, 170, 240))
        d.text((228, 128), f"{persen}%", font=F_TEBAL, fill=PUTIH)
        for i, nilai in enumerate(self.boot_langkah):
            cy, cx = 164 + i * 21, 186
            if nilai >= 1:
                d.ellipse((cx - 6, cy - 6, cx + 6, cy + 6), fill=HIJAU)
                ikon(d, 'cek', cx, cy, 4)
            elif nilai > 0:
                a = int(sekarang * 360) % 360
                d.arc((cx - 6, cy - 6, cx + 6, cy + 6), a, a + 270, fill=KUNING, width=2)
            else:
                d.ellipse((cx - 6, cy - 6, cx + 6, cy + 6), outline=ABU_GELAP, width=2)
        return img

    def _boot_cek(self, sekarang):
        img = self.aset.ambil('cek')
        d = kanvas(img, 'RGBA')
        ada = 'cek' in self.aset.img
        if not ada:
            d.text((14, 16), 'SYSTEM CHECK', font=_font(20, True), fill=PUTIH)
            kotak(d, (8, 64, 312, 214), 8, KACA)
        selesai = 0
        for i, (k, label) in enumerate(ITEM_CEK):
            kol, baris = i // 5, i % 5
            cy = 85 + baris * 28
            if not ada:
                d.text((14 + kol * 154, cy - 7), label, font=F_KECIL, fill=PUTIH)
            x0 = 114 if kol == 0 else 262
            status, _ = self.cek[k]
            selesai += status not in ('tunggu', 'cek')
            d.rectangle((x0, cy - 10, x0 + 56, cy + 10), fill=self.aset.warna_rata('cek', (x0 - 30, cy - 10, x0 - 22, cy + 10)))
            w = WARNA_CEK[status]
            if status == 'cek':
                a = int(sekarang * 360) % 360
                d.arc((x0 + 2, cy - 6, x0 + 14, cy + 6), a, a + 270, fill=w, width=2)
            else:
                d.ellipse((x0 + 2, cy - 6, x0 + 14, cy + 6), fill=w)
                if status == 'ok':
                    ikon(d, 'cek', x0 + 8, cy, 4)
            d.text((x0 + 18, cy - 6), TEKS_CEK[status], font=F_KECIL_TEBAL, fill={HIJAU: HIJAU_MUDA}.get(w, w if w != ABU_GELAP else ABU))
        persen = int(100 * selesai / len(ITEM_CEK))
        d.rectangle((0, 220, 319, 239), fill=self.aset.warna_rata('cek', (0, 214, 319, 219)))
        kotak(d, (14, 226, 268, 235), 4, (20, 34, 56))
        if persen:
            kotak(d, (14, 226, 14 + int(254 * persen / 100), 235), 4, (40, 220, 160))
        d.text((276, 223), f"{persen}%", font=F_TEBAL, fill=PUTIH)
        return img

    def _boot_siap(self, sekarang):
        img = self._boot_cek(sekarang)     # hasil pemeriksaan tetap terlihat di belakang
        d = kanvas(img, 'RGBA')
        gagal = [label for k, label in ITEM_CEK if self.cek[k][0] == 'fail']
        warn = [label for k, label in ITEM_CEK if self.cek[k][0] == 'warn']
        w = HIJAU if not gagal and not warn else (MERAH if gagal else ORANYE)
        kotak(d, (40, 64, 280, 214), 12, KACA_GELAP, outline=w)
        d.ellipse((140, 74, 180, 114), fill=w)
        ikon(d, 'cek' if not gagal else 'alarm', 160, 94, 12, PUTIH if not gagal else LATAR)
        teks_tengah(d, 160, 130, 'SYSTEM READY', _font(18, True), PUTIH)
        teks_tengah(d, 160, 152, '100%', F_TEBAL, BIRU_MUDA)
        if not gagal and not warn:
            teks_tengah(d, 160, 174, 'Semua sistem OK', F_KECIL, HIJAU_MUDA)
        else:
            teks_tengah(d, 160, 172, potong(d, 'GAGAL: ' + ', '.join(gagal) if gagal else 'PERHATIAN: ' + ', '.join(warn),
                                             F_LABEL, 228), F_LABEL, KUNING if not gagal else MERAH)
            teks_tengah(d, 160, 186, 'Detail di ALARM / LOG', F_LABEL, ABU)
        teks_tengah(d, 160, 203, 'Sentuh untuk lanjut', F_LABEL, ABU)
        return img

    # ---------- header & tab bar ----------
    def _pil(self, d, x, teks, warna):
        w = int(d.textlength(teks, font=F_LABEL)) + 17
        kotak(d, (x, 4, x + w, 17), 6, KARTU2)
        d.ellipse((x + 4, 7, x + 10, 13), fill=warna)
        d.text((x + 13, 5), teks, font=F_LABEL, fill=PUTIH)
        return x + w + 3

    def _header(self, d):
        d.rectangle((0, 0, LEBAR, 21), fill=LATAR)
        d.line([(0, 21), (LEBAR, 21)], fill=(30, 110, 170))
        if self.aset.logo is not None:
            self._img.paste(self.aset.logo, (2, 1))
        else:
            ikon(d, 'logo', 11, 11, 8)
        judul = potong(d, self.judul, F_JUDUL, 62)
        d.text((22, 3), judul, font=F_JUDUL, fill=PUTIH)
        teks, w = self.status_mesin()
        if self.tab == 'manual' and teks == 'IDLE':
            teks, w = 'MANUAL', ORANYE
        x = 24 + int(d.textlength(judul, font=F_JUDUL)) + 6
        x = self._pil(d, x, teks, w)
        x = self._pil(d, x, '485', self.status_rs485()[1])
        x = self._pil(d, x, 'CAM', self.status_kamera()[1])
        n = len(self.alarm_aktif())
        ikon(d, 'lonceng', x + 7, 11, 6, MERAH if n else ABU_GELAP)
        d.text((x + 15, 5), str(n), font=F_KECIL_TEBAL, fill=MERAH if n else ABU)
        self.tombol.append(Tombol(x, 0, 28, 21, '', lambda: (setattr(self, 'alarm_mode', 'aktif'), self.ke('alarm'))))
        teks_kanan(d, 317, 4, time.strftime('%H:%M:%S'), F_TEBAL, PUTIH)

    def _tab_bar(self, d):
        d.rectangle((0, Y_TAB - 2, LEBAR, TINGGI), fill=LATAR)
        d.line([(0, Y_TAB - 2), (LEBAR, Y_TAB - 2)], fill=(30, 110, 170))
        maks = max(0, len(TAB) - TAB_TAMPIL)
        geser = self.tab_awal * LEBAR_TAB - (self.geser['dx'] if self.geser else 0)
        geser = min(maks * LEBAR_TAB + 30, max(-30, geser))
        w = LEBAR_TAB
        for i, (kunci, label, ik) in enumerate(TAB):
            x = i * w - geser
            if x + w <= 0 or x >= LEBAR:
                continue
            aktif = self.tab == kunci
            if aktif:
                kotak(d, (x + 2, Y_TAB, x + w - 3, TINGGI - 4), 6, BIRU)
            warna = PUTIH if aktif else ABU
            ikon(d, ik, x + w // 2, Y_TAB + 12, 7, warna)
            teks_tengah(d, x + w // 2, Y_TAB + 28, label, F_MINI, warna)
            if kunci == 'alarm' and self.riwayat.belum_dilihat:
                n = self.riwayat.belum_dilihat
                cx = x + w // 2 + 13
                d.ellipse((cx - 6, Y_TAB, cx + 6, Y_TAB + 12), fill=MERAH, outline=KARTU)
                teks_tengah(d, cx, Y_TAB + 6, str(n) if n < 10 else '9+', F_LABEL, PUTIH)
        if self.geser is None:
            if self.tab_awal > 0:
                ikon(d, 'kembali', 6, Y_TAB + 17, 4, ABU)
            if self.tab_awal < maks:
                ikon(d, 'maju', LEBAR - 7, Y_TAB + 17, 4, ABU)
        if maks:
            lebar = LEBAR * TAB_TAMPIL // len(TAB)
            x0 = int(min(max(geser, 0), maks * w) * (LEBAR - lebar) / (maks * w))
            kotak(d, (x0 + 30, TINGGI - 3, x0 + lebar - 30, TINGGI - 1), 1, ABU_GELAP)

    def _judul_isi(self, d, ik, teks, kembali=None):
        x = 6
        if kembali:
            t = Tombol(4, Y_ISI, 34, 24, '', kembali, KARTU2)
            t.gambar(d)
            ikon(d, 'kembali', 21, Y_ISI + 12, 5)
            self.tombol.append(t)
            x = 44
        if ik:
            ikon(d, ik, x + 8, Y_ISI + 12, 7)
            x += 20
        d.text((x, Y_ISI + 5), teks, font=F_JUDUL, fill=PUTIH)

    def _tombol(self, d, x, y, w, h, teks, aksi, warna=KARTU2, aktif=True, ikon_=None, font=None):
        t = Tombol(x, y, w, h, teks, aksi, warna, aktif, font=font, ikon_=ikon_)
        t.gambar(d)
        self.tombol.append(t)
        return t

    def _hasil_kirim(self, d, y):
        if not self.kirim_hasil:
            return
        jenis, teks = self.kirim_hasil
        w = {'ok': HIJAU, 'gagal': MERAH, 'kuning': ORANYE}.get(jenis, KUNING)
        kotak(d, (4, y, 316, y + 22), 6, w)
        teks_tengah(d, 160, y + 11, potong(d, teks, F_KECIL_TEBAL, 300), F_KECIL_TEBAL,
                    HITAM if w == KUNING else PUTIH)

    # ---------- HOME ----------
    #   kiri    alur mesin dengan IKON (bukan gambar): baris atas HOPPER > CONVEYOR > CAMERA >
    #           PUSHER, turun ke baris bawah DISPENSER < PICKER < STOCKER (aliran barang).
    #           Bingkai & teks tiap stasiun = status hidup. Di bawahnya TOTAL / OK / REJECT / YIELD.
    #   kanan   status mesin, throughput, cycle time, siklus, batch, rak, aksi berikutnya
    def stasiun(self):
        """(label, ikon, (teks, warna), node) tiap stasiun, urutan aliran barang."""
        n = self.pemantau.node.get(SORTER)
        if n is None:
            conv = palang = ('--', ABU_GELAP)
        else:
            akt = n['r'].get(13)
            conv = ('RUN', HIJAU) if n['state'] == ST_RUN else ('STOP', BIRU)
            palang = ('PUSH', KUNING) if akt in (2, 3) else ('READY', BIRU)
            if n['fault'] or n['state'] in (ST_FAULT, ST_ESTOP):
                conv = palang = self.status_node(SORTER)
        return [('HOPPER', 'hopper', self.status_hopper(), SORTER),
                ('CONVEYOR', 'konveyor', conv, SORTER),
                ('CAMERA', 'kamera', self.status_kamera(), None),
                ('PUSHER', 'diverter', palang, SORTER),
                ('DISPENSER', 'tumpukan', self.status_kerja(DISPENSER), DISPENSER),
                ('PICKER', 'lengan', self.status_kerja(PICKER), PICKER),
                ('STOCKER', 'rak', self.status_kerja(STOCKER), STOCKER)]

    def _alur_ikon(self, d):
        terang = {HIJAU: HIJAU_MUDA, BIRU: BIRU_MUDA, ABU_GELAP: ABU}
        st = self.stasiun()
        # baris atas 4 sel (kiri -> kanan), baris bawah 3 sel (kanan -> kiri)
        sel = [(4 + i * 54, 25, 50) for i in range(4)] + [(148 - i * 72, 95, 68) for i in range(3)]
        for i, ((label, ik, (teks, w), sid), (x, y, lebar)) in enumerate(zip(st, sel)):
            mati = w == ABU_GELAP
            kotak(d, (x, y, x + lebar - 1, y + 61), 6, KACA, outline=w)
            ikon(d, ik, x + lebar // 2, y + 19, 11, ABU if mati else PUTIH)
            teks_tengah(d, x + lebar // 2, y + 40, label, F_LABEL, ABU)
            teks_tengah(d, x + lebar // 2, y + 51, potong(d, teks, F_KECIL_TEBAL, lebar - 4), F_KECIL_TEBAL,
                        terang.get(w, w))
            aksi = (lambda s=sid: self._buka_node(s)) if sid else self._buka_setting_kamera
            self.tombol.append(Tombol(x, y, lebar, 62, '', aksi))
        panah = BIRU_MUDA
        for i in range(3):          # baris atas: >
            x = 4 + i * 54 + 50
            d.polygon([(x, 52), (x, 60), (x + 4, 56)], fill=panah)
        d.polygon([(186, 87), (196, 87), (191, 93)], fill=panah)   # turun: PUSHER -> DISPENSER
        for i in range(2):          # baris bawah: <
            x = 148 - i * 72
            d.polygon([(x - 1, 122), (x - 1, 130), (x - 5, 126)], fill=panah)

    def _home(self, d):
        self._alur_ikon(d)

        ok, rej = self.sorter(10), self.sorter(11)
        total = None if ok is None or rej is None else ok + rej
        yld = '-' if not total else f"{ok * 100 / total:.1f}%"
        kotak_stat = [('TOTAL', '-' if total is None else f"{total:,}", PUTIH, ''),
                      ('OK', '-' if ok is None else f"{ok:,}", HIJAU_MUDA, ''),
                      ('REJECT', '-' if rej is None else f"{rej:,}", MERAH,
                       '' if not total else f"{rej * 100 / total:.1f}%"),
                      ('YIELD', yld, BIRU_MUDA, '')]
        for i, (label, nilai, w, bawah) in enumerate(kotak_stat):
            x = 4 + i * 53
            kotak(d, (x, 160, x + 50, 199), 6, KACA)
            d.text((x + 5, 162), label, font=F_LABEL, fill=ABU)
            d.text((x + 5, 173), potong(d, nilai, F_TEBAL, 44), font=F_TEBAL, fill=w)
            if bawah:
                d.text((x + 5, 188), bawah, font=F_LABEL, fill=w)

        teks, w = self.status_mesin()
        kotak(d, (220, 24, 316, 50), 6, w)
        teks_tengah(d, 268, 37, teks, F_JUDUL, HITAM if w == KUNING else PUTIH)
        self.tombol.append(Tombol(220, 24, 96, 27, '', lambda: self.ke('auto')))
        s = STATUS
        kosong = len([r for r in URUTAN_RAK if r not in rak_terisi])
        target = s['target'] or JUMLAH_SIKLUS or kosong
        tp = self.throughput()
        node_aksi, aksi, w_aksi = self.aksi_berikut()
        kpi = [('Throughput', '-' if tp is None else f"{tp:.0f} pcs/min", BIRU_MUDA),
               ('Cycle Time', detik_pendek(self.cycle_time), PUTIH),
               ('Cycle', f"{s['batch']} / {target}", HIJAU_MUDA),
               ('Batch', f"{s['pass_batch']} / {BATCH_SIZE}", PUTIH),
               ('Rack', f"RAK {s['rak']}" if s['rak'] else '-', BIRU_MUDA),
               ('Action ' + node_aksi, aksi, w_aksi)]
        kotak(d, (220, 53, 316, 199), 6, KACA)
        for i, (label, nilai, wn) in enumerate(kpi):
            y = 54 + i * 24
            d.text((225, y + 1), label, font=F_LABEL, fill=ABU)
            d.text((225, y + 10), potong(d, nilai, F_KECIL_TEBAL, 88), font=F_KECIL_TEBAL, fill=wn)
            if label == 'Batch':
                bar(d, 266, y + 3, 45, 4, s['pass_batch'] / BATCH_SIZE if BATCH_SIZE else 0, HIJAU)
            if i < len(kpi) - 1:
                d.line([(224, y + 23), (312, y + 23)], fill=GARIS)

    def _buka_setting_kamera(self):
        self.kategori = [k[0] for k in KATEGORI_SETTING].index('Calibration')   # isi: pengaturan kamera
        self.ke('more', 'setting')

    def _buka_node(self, sid):
        self.node_sid = sid
        self.ke('more', 'detail')

    # ---------- AUTO ----------
    def _auto(self, d):
        teks, w = self.status_mesin()
        d.ellipse((6, 28, 26, 48), fill=w)
        ikon(d, 'play', 17, 38, 6)
        d.text((32, 30), 'AUTO MODE', font=F_JUDUL, fill=PUTIH)
        teks_kanan(d, 312, 32, f"Segment: {self.resep.aktif or '-'}", F_KECIL, ABU)
        s = STATUS
        kosong = len([r for r in URUTAN_RAK if r not in rak_terisi])
        target = s['target'] or JUMLAH_SIKLUS or kosong
        kotak(d, (4, 52, 140, 140), 8, KACA)
        d.text((10, 55), 'PRODUCTION', font=F_KECIL_TEBAL, fill=PUTIH)
        for y, label, a, b, wb in ((70, 'Cycle', s['batch'], target, BIRU),
                                   (96, 'Batch', s['pass_batch'], BATCH_SIZE, HIJAU)):
            d.text((10, y), label, font=F_KECIL, fill=ABU)
            teks_kanan(d, 134, y, f"{a} / {b}", F_TEBAL, PUTIH)
            bar(d, 10, y + 15, 124, 6, a / b if b else 0, wb)
        d.text((10, 122), f"Rack {s['rak'] or '-'}", font=F_KECIL_TEBAL, fill=BIRU_MUDA)
        teks_kanan(d, 134, 122, f"Cam {'ON' if PAKAI_KAMERA else 'OFF'}", F_KECIL, ABU)

        kotak(d, (144, 52, 316, 140), 8, KACA)
        d.text((150, 55), 'CURRENT ACTION', font=F_KECIL_TEBAL, fill=PUTIH)
        for i, sid in enumerate(ALUR):
            y = 70 + i * 13
            _, wn = self.status_node(sid)
            d.ellipse((150, y + 3, 156, y + 9), fill=wn)
            d.text((160, y), SINGKAT[sid][:4], font=F_LABEL, fill=ABU)
            d.text((186, y), potong(d, self.aktivitas(sid), F_LABEL, 126), font=F_LABEL, fill=PUTIH)
        node_aksi, aksi, wa = self.aksi_berikut()
        d.line([(150, 123), (310, 123)], fill=GARIS)
        d.text((150, 126), 'NEXT', font=F_LABEL, fill=ABU)
        d.text((176, 125), potong(d, f"{node_aksi} {aksi}".strip(), F_KECIL_TEBAL, 136), font=F_KECIL_TEBAL, fill=wa)

        jalan = self.berjalan()
        tombol = [
            ('START', 'play', HIJAU, not jalan and self.bus is not None, self.konfirmasi_mulai),
            ('PAUSE', 'jeda', KUNING, False, None),
            ('STOP', 'stop', MERAH, jalan,
             lambda: self.tanya("Hentikan produksi?", ["Sorter berhenti dulu, lalu", "semua node ke mode TEST."],
                                self.stop, 'STOP')),
            ('RESET', 'ulang', KARTU2, not jalan and self.bus is not None,
             lambda: self.tanya("Reset counter?", ["PASS & REJECT Sorter", "kembali ke 0."], self.reset_counter,
                                'RESET')),
        ]
        for i, (label, ik, wt, aktif, aksi_) in enumerate(tombol):
            t = self._tombol(d, 4 + i * 79, 144, 75, 54, label, aksi_ or (lambda: None), wt if aktif else KARTU,
                             aktif, ikon_=ik)
            if label == 'PAUSE':   # bisa diketuk untuk penjelasan walau nonaktif
                t.aktif = True
                t.aksi = lambda: self.beri_pesan("PAUSE belum ada di mesin produksi -- pakai STOP", ORANYE)

    # ---------- NODE ----------
    def _ringkas_node(self, sid):
        n = self.pemantau.node.get(sid)
        if n is None:
            return ['Tidak menjawab', 'Cek daya & kabel RS485']
        return [self.aktivitas(sid), f"State {STATE_NAMES.get(n['state'], '?')}",
                f"Mode {'MAIN' if n['main'] else 'TEST'}{'  MENU' if n['menu'] else ''}"]

    def _node(self, d):
        for i, sid in enumerate(ALUR):
            x, y = 4 + (i % 2) * 158, 28 + (i // 2) * 86
            kotak(d, (x, y, x + 154, y + 82), 8, KACA)
            ikon(d, IKON_NODE[sid], x + 14, y + 13, 7)
            d.text((x + 26, y + 6), SINGKAT[sid], font=F_TEBAL, fill=PUTIH)
            teks, w = self.status_node(sid)
            kotak(d, (x + 104, y + 5, x + 149, y + 20), 7, w)
            teks_tengah(d, x + 126, y + 12, teks, F_MINI, PUTIH)
            for j, baris in enumerate(self._ringkas_node(sid)):
                d.text((x + 8, y + 27 + j * 16), potong(d, baris, F_KECIL, 140), font=F_KECIL,
                       fill=PUTIH if j == 0 else ABU)
            self.tombol.append(Tombol(x, y, 154, 82, '', lambda s=sid: self._buka_node(s)))

    def _format_field(self, sid, reg_label, v):
        if v in ('-', '?', None):
            return str(v if v is not None else '-'), ABU
        if reg_label == 'STATE':
            return STATE_NAMES.get(v, '?'), {ST_RUN: HIJAU_MUDA, ST_FAULT: MERAH, ST_ESTOP: MERAH}.get(v, PUTIH)
        if reg_label == 'FAULT_CODE':
            return ('0 OK', HIJAU_MUDA) if v == 0 else (f"{v} {FAULT_INFO[sid].get(v, ('',))[0]}".strip(), MERAH)
        if reg_label == 'MAIN_MODE_ACTIVE':
            return ('MAIN', HIJAU_MUDA) if v else ('TEST', BIRU_MUDA)
        if reg_label == 'ACTIVITY_CODE':
            return ACTIVITY_NAMES[NAMA[sid]].get(v, str(v)), PUTIH
        if reg_label == 'UPTIME_SEC':
            return lama_waktu(v), PUTIH
        if reg_label == 'SPEED_LAST_MM_S':
            return f"{v} mm/s", PUTIH
        if reg_label in FLAG_YA_TIDAK:
            return ('YES', HIJAU_MUDA) if v else ('NO', ABU)
        if reg_label == 'STOCK_EMPTY_FLAG':
            return ('YES', MERAH) if v else ('NO', HIJAU_MUDA)
        if reg_label == 'CURRENT_RACK_IDX':
            return ('MOVING', KUNING) if v == 0xFF else (f"RAK {v}", BIRU_MUDA)
        if reg_label in ('CURRENT_POSE', 'GERAKAN_AKTIF', 'PIPELINE_STAGE', 'RACK_OCCUPIED_BITMASK'):
            return catatan_untuk(NAMA[sid], reg_label, v, False).strip() or str(v), PUTIH
        return f"{v:,}" if isinstance(v, int) else str(v), PUTIH

    def _detail(self, d):
        sid = self.node_sid
        teks, w = self.status_node(sid)
        self._judul_isi(d, IKON_NODE[sid], NAMA[sid], lambda: self.ke('more', 'node'))
        kotak(d, (250, 28, 316, 48), 8, w)
        teks_tengah(d, 283, 38, teks, F_TEBAL, PUTIH)
        data = {label: v for label, v, _ in self.pemantau.detail}
        kotak(d, (4, 52, 316, 170), 8, KACA)
        if not data:
            teks_tengah(d, 160, 110, 'membaca...', F_BIASA, ABU)
        field = FIELD_DETAIL[sid]
        for i, (label, reg) in enumerate(field):
            kol, baris = i // 5, i % 5
            x, y = 10 + kol * 156, 57 + baris * 22
            if not data:
                break
            nilai, wn = self._format_field(sid, reg, data.get(reg))
            d.text((x, y), label, font=F_LABEL, fill=ABU)
            d.text((x, y + 9), potong(d, nilai, F_KECIL_TEBAL, 146), font=F_KECIL_TEBAL, fill=wn)
        bebas = not self.berjalan() and self.bus is not None
        n = self.pemantau.node.get(sid)
        ada_fault = bool(n and (n['fault'] or n['state'] in (ST_FAULT, ST_ESTOP)))
        self._tombol(d, 4, 174, 70, 24, 'RAW', lambda: self.ke('more', 'raw'))
        self._tombol(d, 78, 174, 110, 24, 'COMMAND', lambda: self.ke('more', 'perintah'), BIRU, bebas)
        self._tombol(d, 192, 174, 124, 24, 'RESET FAULT', lambda: self.reset_fault(sid), ORANYE,
                     bebas and ada_fault)

    def _raw(self, d):
        nama = NAMA[self.node_sid]
        self._judul_isi(d, None, f"{nama} REGISTER", lambda: self.ke('more', 'detail'))
        isi = self.pemantau.detail or [('membaca...', '', '')]
        per = 6
        hal_maks = (len(isi) - 1) // per
        hal = min(self.halaman, hal_maks)
        teks_kanan(d, 312, Y_ISI + 6, f"{hal + 1}/{hal_maks + 1}", F_KECIL, ABU)
        kotak(d, (4, 52, 316, 172), 8, KACA)
        for i, (label, v, cat) in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 56 + i * 19
            d.text((10, y), potong(d, label, F_KECIL, 130), font=F_KECIL, fill=ABU)
            d.text((145, y - 1), str(v), font=F_TEBAL, fill=PUTIH)
            if cat:
                d.text((195, y), potong(d, cat, F_KECIL, 117), font=F_KECIL, fill=KUNING)
        self._tombol(d, 4, 176, 154, 22, '<', lambda: self.ke('more', 'raw', max(0, hal - 1)), KARTU2, hal > 0)
        self._tombol(d, 162, 176, 154, 22, '>', lambda: self.ke('more', 'raw', min(hal_maks, hal + 1)), KARTU2,
                     hal < hal_maks)

    def _perintah(self, d):
        nama = NAMA[self.node_sid]
        daftar = [c for c in OPCODES[nama] if c[1] is not None]
        per = 5
        hal_maks = (len(daftar) - 1) // per
        self._judul_isi(d, None, f"COMMAND {nama}", lambda: self.ke('more', 'detail'))
        teks_kanan(d, 312, Y_ISI + 6, f"{self.halaman + 1}/{hal_maks + 1}", F_KECIL, ABU)
        n = self.pemantau.node.get(self.node_sid)
        main = bool(n and n['main'])
        for i, c in enumerate(daftar[self.halaman * per:(self.halaman + 1) * per]):
            label, op, arg, gate, kelompok = c
            y = 52 + i * 25
            salah_mode = (gate == 'main' and not main) or (gate == 'test' and main)

            def pilih(c=c):
                self.cmd, self.kirim_hasil = c, None
                if isinstance(c[2], str):
                    self.edit = {'popup': True, 'bagian': '', 'kunci': c[0], 'label': c[0], 'catatan': c[2],
                                 'jenis': 'int', 'pilihan': [1, 10, 100], 'nilai': 0, 'langkah': 0,
                                 'simpan': lambda: self.ke('more', 'kirim')}
                else:
                    self.edit = None
                    self.ke('more', 'kirim')
            self._tombol(d, 4, y, 312, 22, '', pilih, KACA)
            d.text((12, y + 4), label, font=F_TEBAL, fill=ABU if salah_mode else PUTIH)
            tag = {'main': 'MAIN', 'test': 'TEST'}.get(gate, '')
            if tag:
                teks_kanan(d, 308, y + 5, tag, F_KECIL, ORANYE if salah_mode else ABU)
        self._tombol(d, 4, 178, 154, 20, '<', lambda: self.ke('more', 'perintah', max(0, self.halaman - 1)),
                     KARTU2, self.halaman > 0)
        self._tombol(d, 162, 178, 154, 20, '>', lambda: self.ke('more', 'perintah', min(hal_maks, self.halaman + 1)),
                     KARTU2, self.halaman < hal_maks)

    def _kirim(self, d):
        label, op, arg, gate, kelompok = self.cmd
        nama = NAMA[self.node_sid]
        nilai = arg if isinstance(arg, int) else (self.edit['nilai'] if self.edit else 0)
        self._judul_isi(d, None, f"KIRIM KE {nama}", lambda: self.ke('more', 'perintah'))
        n = self.pemantau.node.get(self.node_sid)
        main = bool(n and n['main'])
        kotak(d, (4, 52, 316, 110), 8, KACA)
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
        self._hasil_kirim(d, 118)
        sibuk = bool(self.kirim_hasil and self.kirim_hasil[0] == 'mengirim')
        self._tombol(d, 4, 152, 312, 46, 'KIRIM', self.kirim_command, MERAH, not sibuk and not self.berjalan())

    # ---------- ALARM ----------
    def _alarm(self, d):
        aktif = self.alarm_aktif()
        for i, (mode, label) in enumerate((('aktif', f"ACTIVE ({len(aktif)})"), ('riwayat', 'HISTORY'))):
            self._tombol(d, 4 + i * 104, Y_ISI, 100, 24, label,
                         lambda m=mode: (setattr(self, 'alarm_mode', m), self.ke('alarm')),
                         (MERAH if aktif and mode == 'aktif' else BIRU) if self.alarm_mode == mode else KARTU2)
        isi = aktif if self.alarm_mode == 'aktif' else list(reversed(self.riwayat.isi))
        per = 6
        hal_maks = max(0, (len(isi) - 1) // per)
        hal = min(self.halaman, hal_maks)
        self._tombol(d, 212, Y_ISI, 30, 24, '<', lambda: self.ke('alarm', None, max(0, hal - 1)), KARTU2, hal > 0)
        self._tombol(d, 246, Y_ISI, 30, 24, '>', lambda: self.ke('alarm', None, min(hal_maks, hal + 1)), KARTU2,
                     hal < hal_maks)
        if self.alarm_mode == 'riwayat':
            self._tombol(d, 280, Y_ISI, 36, 24, 'DEL', lambda: self.tanya(
                "Hapus riwayat?", ["Semua catatan alarm dihapus."], self.riwayat.hapus, 'HAPUS'), KARTU2, bool(isi))
        kotak(d, (4, 54, 316, 199), 8, KACA)
        if not isi:
            teks_tengah(d, 160, 125, 'Tidak ada alarm aktif' if self.alarm_mode == 'aktif' else 'Belum ada alarm',
                        F_BIASA, HIJAU_MUDA if self.alarm_mode == 'aktif' else ABU)
        warna = {'E': MERAH, 'W': KUNING, 'I': BIRU}
        simbol = {'E': 'silang', 'W': 'alarm', 'I': 'info'}
        for i, a in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 57 + i * 24
            w = warna.get(a['tingkat'], ABU)
            d.ellipse((9, y + 4, 23, y + 18), fill=w)
            ikon(d, simbol.get(a['tingkat'], 'info'), 16, y + 11, 4, HITAM if w == KUNING else PUTIH)
            if 't' in a:
                d.text((28, y + 5), time.strftime('%d/%m %H:%M', time.localtime(a['t'])), font=F_KECIL, fill=ABU)
                x = 106
            else:
                x = 28
            d.text((x, y + 4), a['kode'], font=F_TEBAL, fill=w)
            d.text((x + 46, y + 5), potong(d, a['pesan'], F_KECIL, 300 - x - 46), font=F_KECIL, fill=PUTIH)
            ikon(d, 'maju', 309, y + 11, 3, ABU)
            if i < per - 1:
                d.line([(10, y + 23), (310, y + 23)], fill=GARIS)
            self.tombol.append(Tombol(4, y, 312, 23, '', lambda a=a: (setattr(self, 'alarm_pilih', a),
                                                                      self.ke('alarm', 'alarm_detail'))))

    def _alarm_detail(self, d):
        a = self.alarm_pilih
        node, judul, desk, tindakan = info_alarm(a['kode'], a['pesan'])
        w = {'E': MERAH, 'W': KUNING, 'I': BIRU}.get(a['tingkat'], ABU)
        self._judul_isi(d, None, 'ALARM DETAIL', lambda: self.ke('alarm'))
        kotak(d, (4, 52, 316, 170), 8, KACA, outline=w)
        d.text((12, 56), a['kode'], font=_font(18, True), fill=w)
        d.text((12, 78), potong(d, judul, F_TEBAL, 296), font=F_TEBAL, fill=PUTIH)
        waktu = time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(a['t'])) if 't' in a else 'AKTIF SEKARANG'
        teks_kanan(d, 310, 58, waktu, F_LABEL, ABU)
        d.text((12, 96), 'NODE', font=F_LABEL, fill=ABU)
        d.text((60, 95), node, font=F_KECIL_TEBAL, fill=BIRU_MUDA)
        d.text((12, 110), potong(d, desk, F_KECIL, 296), font=F_KECIL, fill=PUTIH)
        d.text((12, 128), 'ACTION', font=F_LABEL, fill=ABU)
        kata, baris = tindakan.split(), ['']
        for k in kata:    # bungkus tindakan jadi maks 2 baris
            if d.textlength((baris[-1] + ' ' + k).strip(), font=F_KECIL) > 296 and len(baris) < 2:
                baris.append('')
            baris[-1] = (baris[-1] + ' ' + k).strip()
        for i, b in enumerate(baris):
            d.text((12, 139 + i * 13), potong(d, b, F_KECIL, 296), font=F_KECIL, fill=KUNING)
        m = re.fullmatch(r'E-(\d)', a['kode'][:3])
        sid = int(m.group(1)) if m and int(m.group(1)) in NAMA else None
        n = self.pemantau.node.get(sid) if sid else None
        masih = bool(n and (n['fault'] or n['state'] in (ST_FAULT, ST_ESTOP)))
        self._tombol(d, 4, 174, 154, 24, 'RESET FAULT' if sid else 'KEMBALI',
                     (lambda: self.reset_fault(sid)) if sid else (lambda: self.ke('alarm')),
                     ORANYE if sid else KARTU2, (masih and not self.berjalan()) if sid else True)
        # Shortcut ke LOG -- penyebab rinci (mis. E-PRD "lihat LOG") ada di sana; tombol < di LOG
        # kembali ke detail alarm ini.
        self._tombol(d, 162, 174, 154, 24, 'LIHAT LOG',
                     lambda: self.buka_log(kembali=lambda: self.ke('alarm', 'alarm_detail')), BIRU)
        if self.kirim_hasil and sid:
            self._hasil_kirim(d, 148)

    # ---------- SEGMENT (dulu RECIPE; data tetap di resep.json) ----------
    def _recipe(self, d):
        r = self.resep
        resep = r.sekarang()
        kunci_aktif = self.produksi_aktif()
        self._tombol(d, 4, Y_ISI, 30, 24, '<', lambda: setattr(r, 'pilih', max(0, r.pilih - 1)), KARTU2,
                     r.pilih > 0)
        self._tombol(d, 286, Y_ISI, 30, 24, '>', lambda: setattr(r, 'pilih', min(len(r.daftar) - 1, r.pilih + 1)),
                     KARTU2, r.pilih < len(r.daftar) - 1)
        kotak(d, (38, Y_ISI, 282, Y_ISI + 24), 6, KACA)
        teks_tengah(d, 160, Y_ISI + 12, potong(d, resep['nama'], F_JUDUL, 170), F_JUDUL, PUTIH)
        if resep['nama'] == r.aktif:
            teks_kanan(d, 278, Y_ISI + 8, 'AKTIF', F_LABEL, HIJAU_MUDA)
        teks_kanan(d, 60, Y_ISI + 8, f"{r.pilih + 1}/{len(r.daftar)}", F_LABEL, ABU)
        kotak(d, (4, 54, 316, 164), 8, KACA)
        sel = [('Segment', resep['nama'], None)] + [(lab, resep.get(k), k) for k, _, lab in RESEP_KUNCI]
        for i, (label, nilai, k) in enumerate(sel):
            kol, baris = i // 4, i % 4
            x, y = 10 + kol * 156, 58 + baris * 26
            if k == 'pakai_kamera':
                nilai = 'ON' if nilai else 'OFF'
            elif k == 'rak_random':
                nilai = 'RANDOM' if nilai else 'SEQUENCE'
            elif k == 'jeda_hopper':
                nilai = {0: '0 OFF', 1: '1 REFILL', 2: '2 BATCH'}.get(nilai, nilai)
            elif k == 'delay_picker_s':
                nilai = f"{tampil_nilai(nilai)} s"
            d.text((x, y), label, font=F_LABEL, fill=ABU)
            d.text((x, y + 9), potong(d, tampil_nilai(nilai), F_TEBAL, 140), font=F_TEBAL,
                   fill=ABU if kunci_aktif or k is None else KUNING)
            if k:
                self.tombol.append(Tombol(x - 4, y - 2, 152, 25, '', lambda k=k, lab=label: self._edit_resep(k, lab)))
        tombol = [('LOAD', self._load_resep, BIRU), ('SAVE', self._save_resep, HIJAU),
                  ('NEW', self._new_resep, KARTU2), ('DELETE', self._delete_resep, MERAH)]
        for i, (label, aksi, wt) in enumerate(tombol):
            self._tombol(d, 4 + i * 79, 168, 75, 30, label, aksi, wt if not kunci_aktif else KARTU,
                         not kunci_aktif and (label != 'DELETE' or len(r.daftar) > 1))

    def _edit_resep(self, k, label):
        if self.produksi_aktif():
            self.beri_pesan("Read-only saat produksi berjalan", ORANYE)
            return
        resep = self.resep.sekarang()
        if k == 'conveyor_pwm':
            jenis, pilihan, maks = 'int', [1, 5, 10, 50], 255
            nilai = resep.get(k) if resep.get(k) is not None else 180
        else:
            bagian = dict((kk, b) for kk, b, _ in RESEP_KUNCI)[k]
            jenis, pilihan = jenis_setting(bagian, k)
            nilai, maks = resep.get(k, nilai_sekarang(k)), 65535
        teks = {'pakai_kamera': lambda v: 'ON' if v else 'OFF',
                'rak_random': lambda v: 'RANDOM' if v else 'SEQUENCE',
                'jeda_hopper': lambda v: {0: 'OFF', 1: 'REFILL', 2: 'BATCH'}.get(v, str(v))}.get(k, tampil_nilai)

        def simpan():
            resep[k] = self.edit['nilai']
            self.resep.simpan()
            self.edit = None
            self.beri_pesan("Segment diubah -- LOAD untuk memakai", HIJAU, 2)
        self.edit = {'popup': True, 'bagian': 'resep', 'kunci': k, 'label': label,
                     'catatan': f"segment {resep['nama']}" + (' (PWM 0-255)' if k == 'conveyor_pwm' else ''),
                     'jenis': jenis, 'pilihan': pilihan, 'nilai': nilai, 'langkah': 0, 'maks': maks,
                     'teks': teks, 'simpan': simpan}

    def _load_resep(self):
        resep = self.resep.sekarang()

        def lakukan():
            lama = {k: nilai_sekarang(k) for k, b, _ in RESEP_KUNCI if b}
            try:
                for k, bagian, _ in RESEP_KUNCI:
                    if bagian and k in resep:
                        tulis_config(bagian, k, tampil_nilai(resep[k]))
            except (OSError, ValueError) as e:
                self.beri_pesan(f"Gagal tulis config: {e}", MERAH)
                return
            err = muat_ulang_config()
            if err:
                for k, bagian, _ in RESEP_KUNCI:   # kembalikan semua
                    if bagian:
                        tulis_config(bagian, k, tampil_nilai(lama[k]))
                muat_ulang_config()
                self.beri_pesan("Segment ditolak config -- dibatalkan", MERAH)
                return
            self.resep.aktif = resep['nama']
            self.resep.simpan()
            log(f"[panel] segment {resep['nama']} dimuat ke sorting.config")
            if resep.get('conveyor_pwm') is not None:
                c = cari_command(SORTER, 'SET_CONVEYOR_SPEED')
                self.kirim(SORTER, c, int(resep['conveyor_pwm']), verifikasi='ack')
            self.beri_pesan(f"Segment {resep['nama']} dimuat", HIJAU, 2)
        baris = [f"Batch {resep['batch_size']}, siklus {resep['jumlah_siklus']}",
                 "Nilai ditulis ke sorting.config."]
        if resep.get('conveyor_pwm') is not None:
            baris.append(f"Conveyor Sorter PWM {resep['conveyor_pwm']}")
        self.tanya(f"Load {resep['nama']}?", baris, lakukan, 'LOAD', BIRU)

    def _save_resep(self):
        resep = self.resep.sekarang()

        def lakukan():
            baru = Resep.dari_config(resep['nama'])
            baru['conveyor_pwm'] = resep.get('conveyor_pwm')
            self.resep.daftar[self.resep.pilih] = baru
            self.resep.simpan()
            self.beri_pesan("Segment disimpan dari config", HIJAU, 1.5)
        self.tanya(f"Save {resep['nama']}?", ["Nilai resep ditimpa dengan", "isi sorting.config sekarang."],
                   lakukan, 'SAVE', HIJAU)

    def _new_resep(self):
        nama = [r['nama'] for r in self.resep.daftar]
        i = 1
        while f"SEGMENT {i}" in nama:
            i += 1
        self.resep.daftar.append(Resep.dari_config(f"SEGMENT {i}"))
        self.resep.pilih = len(self.resep.daftar) - 1
        self.resep.simpan()
        self.beri_pesan(f"SEGMENT {i} dibuat dari config (nama: ubah di resep.json)", HIJAU, 3)

    def _delete_resep(self):
        resep = self.resep.sekarang()

        def lakukan():
            del self.resep.daftar[self.resep.pilih]
            if self.resep.aktif == resep['nama']:
                self.resep.aktif = None
            self.resep.pilih = max(0, self.resep.pilih - 1)
            self.resep.simpan()
        self.tanya(f"Hapus {resep['nama']}?", ["Segment dihapus permanen."], lakukan, 'HAPUS')

    # ---------- MORE ----------
    # ---------- NODE STATUS (di MORE) ----------
    def _more(self, d):
        pilihan = [('NODE STATUS', 'chip', 'node', BIRU), ('HISTORY', 'grafik', 'history', BIRU),
                   ('SETTING', 'gear', 'setting', KARTU2), ('SYSTEM', 'info', 'sistem', KARTU2)]
        for i, (label, ik, sub, w) in enumerate(pilihan):
            x, y = 4 + (i % 2) * 158, 28 + (i // 2) * 86
            self._tombol(d, x, y, 154, 82, '', lambda s=sub: self.ke('more', s), KACA)
            d.ellipse((x + 57, y + 10, x + 97, y + 50), fill=w)
            ikon(d, ik, x + 77, y + 30, 11)
            teks_tengah(d, x + 77, y + 66, label, F_TEBAL, PUTIH)

    def _kembali_more(self):
        self.ke('more')

    # ================= MANUAL CONTROL =================
    # Tiap node dijalankan TERPISAH dari produksi. Hanya command yang ada di firmware (OPCODES);
    # urutan uji (CONV + PUSH, 1 OBJEK, FULL CYCLE, ...) = beberapa command itu berurutan, dan
    # setiap langkah diverifikasi dari keadaan node, bukan dari ack saja.
    def sibuk(self):
        return self.urutan_jalan is not None or bool(self.kirim_hasil and self.kirim_hasil[0] == 'mengirim')

    def kunci_manual(self, sid):
        """(alasan, warna) kalau node TIDAK boleh digerakkan dari layar, atau None."""
        n = self.pemantau.node.get(sid)
        if self.produksi_aktif():
            return 'TERKUNCI: produksi berjalan -- STOP dulu', MERAH
        if self.bus is None or n is None:
            return f'{NAMA[sid]} tidak menjawab', MERAH
        if n['menu']:
            return 'Operator di menu LCD node', ORANYE
        if n['state'] == ST_ESTOP:
            return 'E-STOP aktif', MERAH
        if n['fault'] or n['state'] == ST_FAULT:
            nama = FAULT_INFO[sid].get(n['fault'], (f"FAULT {n['fault']}",))[0]
            return f'FAULT {nama} -- RESET FAULT dulu', MERAH
        if not self.manual_siap:
            return 'Mode manual belum diaktifkan', ORANYE
        return None

    def urutan(self, sid, nama, langkah, akhir=()):
        """Jalankan langkah di thread sendiri. Langkah:
             ('cmd', label, arg[, sid])     kirim command, tunggu ack (lama untuk ack-saat-selesai)
             ('tulis', reg, nilai)          tulis register (mis. CLASSIFY_IS_REJECT)
             ('tunggu', detik)
             ('acuan', fungsi(n))           simpan nilai acuan dari bacaan node
             ('sampai', fungsi(n), batas, teks)   tunggu keadaan node (bacaan BARU, bukan lama)
             ('baca', reg, nilai, teks)     baca register langsung dari bus, harus = nilai
           `akhir` SELALU dijalankan (juga saat gagal / STOP ALL) -- untuk mematikan motor."""
        if self.sibuk():
            self.beri_pesan("Tunggu perintah sebelumnya selesai", ORANYE)
            return
        self.batal.clear()
        self.urutan_jalan = nama
        self.kirim_hasil = ('mengirim', f"{nama} ...")

        def kerja():
            try:
                for i, l in enumerate(langkah, 1):
                    if self.batal.is_set():
                        raise Gagal('dibatalkan (STOP ALL)')
                    ket = {'cmd': lambda: l[1], 'sampai': lambda: l[3], 'baca': lambda: l[3],
                           'tunggu': lambda: f"tunggu {l[1]:g} s"}.get(l[0], lambda: '')()
                    self.kirim_hasil = ('mengirim', f"{nama} {i}/{len(langkah)}: {ket}")
                    self._langkah(sid, l)
                self.kirim_hasil = ('ok', f"{nama}: selesai, terverifikasi")
                log(f"[manual] {NAMA[sid]} {nama}: selesai")
            except Exception as e:
                self.kirim_hasil = ('gagal', f"{nama}: {e}")
                log(f"!! [manual] {NAMA[sid]} {nama}: {e}")
            finally:
                for l in akhir:
                    try:
                        self._langkah(sid, l, boleh_batal=False)
                    except Exception as e:
                        log(f"!! [manual] {NAMA[sid]} {nama} -- langkah penutup gagal: {e}")
                        self.kirim_hasil = ('gagal', f"{nama}: penutup gagal ({e}) -- cek mesin!")
                self.urutan_jalan = None
        threading.Thread(target=kerja, daemon=True).start()

    def _langkah(self, sid, l, boleh_batal=True):
        jenis = l[0]
        if jenis == 'cmd':
            tujuan = l[3] if len(l) > 3 else sid
            c = cari_command(tujuan, l[1])
            seq = self.bus.command(tujuan, c[1], l[2])
            batas = time.monotonic() + batas_ack(tujuan, l[1])
            while True:
                try:
                    if self.bus.baca(tujuan, R_ACK, percobaan=1) == seq:
                        return
                except Exception:
                    pass
                if time.monotonic() > batas:
                    raise Gagal(f"{l[1]} tidak di-ack {NAMA[tujuan]}")
                if self.batal.wait(0.1) and boleh_batal:
                    raise Gagal('dibatalkan (STOP ALL)')
        elif jenis == 'tulis':
            self.bus.tulis(sid, l[1], l[2])
        elif jenis == 'tunggu':
            if self.batal.wait(l[1]) and boleh_batal:
                raise Gagal('dibatalkan (STOP ALL)')
        elif jenis == 'acuan':
            awal = self.pemantau.putaran
            while self.pemantau.putaran <= awal:          # bacaan yang diambil SESUDAH langkah ini
                self.batal.wait(0.1)
            self.acuan = l[1](self.pemantau.node.get(sid))
        elif jenis == 'sampai':
            awal, batas = self.pemantau.putaran, time.monotonic() + l[2]
            while True:
                # Hanya bacaan Pemantau yang BARU (sesudah langkah dimulai) yang dipercaya --
                # bacaan lama bisa saja masih menunjukkan keadaan sebelum command.
                if self.pemantau.putaran > awal and l[1](self.pemantau.node.get(sid)):
                    return
                if time.monotonic() > batas:
                    raise Gagal(f"{l[3]} -- tidak terjadi dalam {l[2]:g} s")
                if self.batal.wait(0.2) and boleh_batal:
                    raise Gagal('dibatalkan (STOP ALL)')
        elif jenis == 'baca':
            v = self.bus.baca(sid, l[1], percobaan=2)
            if v != l[2]:
                raise Gagal(f"{l[3]} (register {l[1]} = {v})")

    def stop_semua(self):
        """Hentikan urutan yang berjalan + matikan aktuator manual di semua node. Best-effort,
        tanpa menunggu ack. BUKAN pengganti E-STOP fisik."""
        self.batal.set()
        UJI_KAMERA.stop()
        log("[manual] STOP ALL")

        def kerja():
            for sid, label, arg in ((SORTER, 'SET_MOTOR_A', 0), (SORTER, 'STOP_MAIN', 0),
                                    (DISPENSER, 'CONVEYOR_ON_OFF', 0), (DISPENSER, 'STOP_MAIN', 0),
                                    (PICKER, 'STOP_MAIN', 0), (STOCKER, 'STOP_MAIN', 0)):
                try:
                    self.bus.command(sid, cari_command(sid, label)[1], arg)
                except Exception as e:
                    log(f"!! [manual] STOP ALL {NAMA[sid]} {label}: {e}")
        if self.bus is not None:
            threading.Thread(target=kerja, daemon=True).start()
        self.beri_pesan("STOP ALL dikirim", MERAH, 2)

    def aktifkan_manual(self):
        self.tanya("Aktifkan mode manual?", ["Mesin bisa digerakkan dari layar.",
                                             "Pastikan area mesin aman", "dan tidak ada tangan di dalam."],
                   lambda: setattr(self, 'manual_siap', True), 'AKTIFKAN', ORANYE, 'alarm')

    # ---------- gambar MANUAL ----------
    def _kartu(self, d, x, y, judul, status=None, ws=ABU, w=154, h=58):
        kotak(d, (x, y, x + w, y + h), 6, KACA)
        d.text((x + 6, y + 2), judul, font=F_KECIL_TEBAL, fill=BIRU_MUDA)
        if status:
            teks_kanan(d, x + w - 6, y + 3, status, F_LABEL, ws)

    def _tmb(self, d, x, y, w, h, label, sid, aksi, gate='test', warna=BIRU, nyala=False):
        """Tombol manual: otomatis nonaktif kalau node terkunci, sibuk, atau mode node salah."""
        n = self.pemantau.node.get(sid)
        main = bool(n and n['main'])
        bisa = (self.kunci_manual(sid) is None and not self.sibuk()
                and not (gate == 'test' and main) and not (gate == 'main' and not main))
        self._tombol(d, x, y, w, h, label, aksi, (HIJAU if nyala else warna) if bisa else KARTU, bisa,
                     font=F_KECIL_TEBAL)

    def _atur(self, d, x, y, label, teks, kurang, tambah, sid, gate='netral', warna=PUTIH):
        """Baris pengatur angka: label  [-] nilai [+]."""
        d.text((x, y + 3), label, font=F_LABEL, fill=ABU)
        self._tmb(d, x + 50, y, 26, 18, '-', sid, kurang, gate, KARTU2)
        teks_tengah(d, x + 100, y + 9, teks, F_KECIL_TEBAL, warna)
        self._tmb(d, x + 122, y, 26, 18, '+', sid, tambah, gate, KARTU2)

    def _manual(self, d):
        label = {SORTER: 'SORTER', DISPENSER: 'DISPEN', PICKER: 'PICKER', STOCKER: 'STOCKER', 'cam': 'CAMERA'}
        for i, sid in enumerate(ALUR + ['cam']):
            self._tombol(d, 4 + i * 51, Y_ISI, 49, 22, label[sid], lambda s=sid: self._pilih_manual(s),
                         BIRU if self.manual_sid == sid else KARTU2, font=F_LABEL)
        self._tombol(d, 260, Y_ISI, 56, 22, 'STOP ALL', self.stop_semua, MERAH, self.bus is not None,
                     font=F_LABEL)
        sid = self.manual_sid
        if sid == 'cam':
            (self._kamera_hasil if self.kamera_hasil else self._manual_kamera)(d)
            return
        if self.manual_sensor:
            self._manual_sensor(d, sid)
        else:
            getattr(self, '_manual_' + NAMA[sid].lower())(d, sid)
        self._baris_bawah_manual(d, sid)

    def _pilih_manual(self, sid):
        if sid != 'cam':
            UJI_KAMERA.stop()            # port kamera hanya dibuka selama tab CAMERA dipakai
        self.manual_sid, self.manual_sensor, self.kirim_hasil = sid, False, None
        self.kamera_hasil = False
        self.pemantau.detail, self.pemantau.detail_sid = [], (sid if sid in NAMA else None)

    def _baris_bawah_manual(self, d, sid):
        kunci = self.kunci_manual(sid)
        n = self.pemantau.node.get(sid)
        if self.kirim_hasil is not self._hasil_terakhir:      # hasil baru: tampil 8 s
            self._hasil_terakhir, self._hasil_t = self.kirim_hasil, time.monotonic()
        if self.kirim_hasil and (self.sibuk() or time.monotonic() - self._hasil_t < 8):
            self._hasil_kirim(d, 176)
        elif kunci and kunci[0] == 'Mode manual belum diaktifkan' and not self.produksi_aktif():
            self._tombol(d, 4, 176, 312, 22, 'AKTIFKAN MODE MANUAL', self.aktifkan_manual, ORANYE,
                         font=F_KECIL_TEBAL)
        elif kunci:
            kotak(d, (4, 176, 316 if 'FAULT' not in kunci[0] else 206, 198), 6, kunci[1])
            ikon(d, 'gembok', 14, 187, 5)
            d.text((24, 181), potong(d, kunci[0], F_KECIL_TEBAL, 178 if 'FAULT' in kunci[0] else 288),
                   font=F_KECIL_TEBAL, fill=PUTIH)
            if 'FAULT' in kunci[0] and not self.produksi_aktif():
                self._tombol(d, 210, 176, 106, 22, 'RESET FAULT', lambda: self.reset_fault(sid), ORANYE,
                             font=F_KECIL_TEBAL)
        else:
            main = bool(n and n['main'])
            kotak(d, (4, 176, 316, 198), 6, KACA)
            d.text((10, 181), f"MODE {'MAIN' if main else 'TEST'}", font=F_KECIL_TEBAL,
                   fill=KUNING if main else HIJAU_MUDA)
            d.text((72, 181), potong(d, self.aktivitas(sid), F_KECIL, 150), font=F_KECIL, fill=PUTIH)
            if main:   # perintah uji ditolak selama MAIN
                self._tombol(d, 232, 177, 82, 20, 'KE TEST', lambda: self.urutan(
                    sid, 'KE TEST', [('cmd', 'STOP_MAIN', 0), ('sampai', lambda n: n and not n['main'], 4,
                                                                'node pindah ke TEST')]), BIRU, font=F_LABEL)

    # --- SORTER ---
    def _manual_sorter(self, d, sid):
        n = self.pemantau.node.get(sid)
        r = n['r'] if n else {}
        jalan = bool(n and n['state'] == ST_RUN)
        kec, kec_dikirim = self.persen_kecepatan(SORTER)
        # CONVEYOR -- di firmware Sorter conveyor hanya berjalan dalam mode MAIN (START).
        self._kartu(d, 4, 52, 'CONVEYOR', 'RUN' if jalan else 'STOP', HIJAU_MUDA if jalan else ABU)
        self._tmb(d, 10, 68, 70, 20, 'RUN', sid, lambda: self.tanya(
            "Jalankan conveyor Sorter?", ["Sorter ke mode MAIN, umpan hopper", "langsung dijeda.",
                                          "Uji palang/hopper nonaktif selama jalan."],
            lambda: self.urutan(sid, 'CONVEYOR RUN', [
                ('cmd', 'START_MAIN', 0), ('cmd', 'SET_HOPPER_JEDA', 1),
                ('sampai', lambda n: n and n['state'] == ST_RUN, 4, 'conveyor jalan')]), 'RUN', HIJAU, 'play'),
            'netral', BIRU, jalan)
        self._tmb(d, 84, 68, 66, 20, 'STOP', sid, lambda: self.urutan(sid, 'CONVEYOR STOP', [
            ('cmd', 'STOP_MAIN', 0), ('sampai', lambda n: n and n['state'] != ST_RUN, 4, 'conveyor berhenti')]),
            'netral', MERAH)
        self._atur(d, 10, 91, 'Speed' if kec_dikirim else 'Speed fw', f"{kec}%",
                   lambda: self._kirim_kecepatan(SORTER, -10), lambda: self._kirim_kecepatan(SORTER, 10), sid,
                   warna=PUTIH if kec_dikirim else ABU)

        # PUSHER (palang) -- Motor A. SET_MOTOR_A = arah MENTAH, tanpa batas waktu di firmware:
        # panel mematikannya sendiri setelah `durasi`.
        akt = r.get(13)
        self._kartu(d, 162, 52, 'PUSHER', {2: 'PUSH', 3: 'MANUAL'}.get(akt, 'DIAM'),
                    KUNING if akt in (2, 3) else ABU)
        dur = self.durasi_pusher
        for i, (label, arah) in enumerate((('MAJU', 1), ('MUNDUR', 2))):
            self._tmb(d, 168 + i * 50, 68, 47, 20, label, sid, lambda a=arah, lb=label: self.urutan(
                sid, f'PUSHER {lb} {dur:g}s', [('cmd', 'SET_MOTOR_A', a), ('tunggu', dur)],
                akhir=[('cmd', 'SET_MOTOR_A', 0),
                       ('sampai', lambda n: n and n['r'].get(13) != 3, 4, 'motor palang berhenti')]))
        self._tmb(d, 268, 68, 42, 20, '1x', sid, lambda: self.urutan(sid, 'PUSH 1x', [
            ('acuan', lambda n: n['r'].get(11, 0)), ('cmd', 'TRIGGER_PALANG', 0),
            ('sampai', lambda n: n and n['r'].get(11, 0) != self.acuan, 5, 'palang mendorong (REJECT_COUNT naik)')]))
        self._atur(d, 168, 91, 'Durasi', f"{dur:.1f} s",
                   lambda: setattr(self, 'durasi_pusher', max(0.1, round(dur - 0.1, 1))),
                   lambda: setattr(self, 'durasi_pusher', min(3.0, round(dur + 0.1, 1))), sid, 'test')

        # HOPPER -- 1 CYCLE hanya di TEST; FEED/PAUSE (SET_HOPPER_JEDA) saat conveyor jalan (MAIN).
        self._kartu(d, 4, 114, 'HOPPER', 'PAUSE' if hopper_dijeda else '', ABU)
        self._tmb(d, 10, 130, 140, 20, '1 CYCLE', sid, lambda: self.urutan(sid, 'HOPPER 1 CYCLE', [
            ('cmd', 'HOPPER_CYCLE', 0), ('sampai', lambda n: n and n['r'].get(13) != 4, 30, 'hopper selesai')]))
        self._tmb(d, 10, 152, 68, 18, 'FEED', sid, lambda: self.urutan(sid, 'HOPPER FEED', [
            ('cmd', 'SET_HOPPER_JEDA', 0), ('baca', R_SORTER_HOPPER_DIJEDA, 0, 'hopper masih dijeda')]), 'main')
        self._tmb(d, 82, 152, 68, 18, 'PAUSE', sid, lambda: self.urutan(sid, 'HOPPER PAUSE', [
            ('cmd', 'SET_HOPPER_JEDA', 1), ('baca', R_SORTER_HOPPER_DIJEDA, 1, 'hopper tidak dijeda')]), 'main')

        # TEST SEQUENCE
        self._kartu(d, 162, 114, 'TEST SEQUENCE')
        self._tmb(d, 168, 130, 70, 20, 'CONV+PUSH', sid, lambda: self.tanya(
            "Uji conveyor + pusher?", ["Conveyor jalan, satu reject disimulasikan,", "palang mendorong, lalu STOP."],
            lambda: self.urutan(sid, 'CONV+PUSH', [
                ('cmd', 'START_MAIN', 0), ('cmd', 'SET_HOPPER_JEDA', 1),
                ('sampai', lambda n: n and n['state'] == ST_RUN, 4, 'conveyor jalan'),
                ('acuan', lambda n: n['r'].get(11, 0)), ('tulis', R_SORTER_CLASSIFY, 1),
                ('sampai', lambda n: n and n['r'].get(11, 0) != self.acuan, 6, 'palang mendorong (REJECT_COUNT naik)'),
                ('tunggu', 1.0)],
                akhir=[('cmd', 'STOP_MAIN', 0)]), 'UJI', ORANYE, 'alarm'), 'netral', ORANYE)
        self._tmb(d, 242, 130, 68, 20, 'SENSOR', sid, self._buka_sensor, 'netral', KARTU2)
        self._tmb(d, 168, 152, 142, 18, 'AUTO CYCLE (1 objek)', sid, lambda: self.tanya(
            "Satu objek lewat?", ["Conveyor & hopper jalan sampai", "1 objek terhitung PASS/REJECT, lalu STOP."],
            lambda: self.urutan(sid, 'AUTO CYCLE', [
                ('cmd', 'START_MAIN', 0),
                ('sampai', lambda n: n and n['state'] == ST_RUN, 4, 'conveyor jalan'),
                ('acuan', lambda n: n['r'].get(10, 0) + n['r'].get(11, 0)),
                ('sampai', lambda n: n and n['r'].get(10, 0) + n['r'].get(11, 0) != self.acuan, 30,
                 'objek terhitung'),
                ('cmd', 'SET_HOPPER_JEDA', 1), ('tunggu', 2.0)],
                akhir=[('cmd', 'STOP_MAIN', 0)]), 'JALANKAN', ORANYE, 'alarm'), 'netral', ORANYE)

    def persen_kecepatan(self, sid):
        """(persen, sudah_dikirim). Firmware tidak punya register untuk membaca kecepatan conveyor,
        jadi sebelum panel mengirim apa pun yang tampil = nilai BAWAAN firmware (abu-abu)."""
        if sid in self.kecepatan:
            return self.kecepatan[sid], True
        return round(KECEPATAN_BAWAAN_PWM[sid] * 100 / 255), False

    def _manual_kamera(self, d):
        u = UJI_KAMERA
        aktif = u.aktif()
        terkunci = self.produksi_aktif()
        jps = u.jawaban_per_detik()
        # KONEKSI
        self._kartu(d, 4, 52, 'KONEKSI', 'aktif' if aktif else 'port tertutup', HIJAU_MUDA if aktif else ABU)
        d.text((10, 68), potong(d, f"{HUSKYLENS_PORT} @ {HUSKYLENS_BAUD}", F_KECIL, 140), font=F_KECIL, fill=PUTIH)
        if aktif and u.mode in ('live', 'klasifikasi'):
            ok = jps > 0
            d.text((10, 84), f"{'MENJAWAB' if ok else 'TIDAK MENJAWAB'}", font=F_KECIL_TEBAL,
                   fill=HIJAU_MUDA if ok else MERAH)
            teks_kanan(d, 152, 86, f"{jps:.0f} jawaban/s", F_LABEL, ABU)
        else:
            d.text((10, 86), f"pakai_kamera = {PAKAI_KAMERA}", font=F_LABEL, fill=ABU)
        # DETEKSI
        baru = u.id_terakhir is not None and time.monotonic() - u.t_id < 0.5
        oid = u.id_terakhir if baru else None
        if oid is None:
            teks, w = ('kosong' if aktif else '-'), ABU
        elif oid == ID_VALID:
            teks, w = 'VALID', HIJAU_MUDA
        elif oid in ID_INVALID:
            teks, w = 'INVALID', MERAH
        elif oid == ID_KOSONG:
            teks, w = 'ID kosong', ABU
        else:
            teks, w = 'ID lain', KUNING
        self._kartu(d, 162, 52, 'DETEKSI', f"ID valid {ID_VALID}", ABU)
        teks_tengah(d, 200, 85, '-' if oid is None else str(oid), F_SEDANG, w)
        teks_tengah(d, 268, 85, teks, F_KECIL_TEBAL, w)
        # KLASIFIKASI
        self._kartu(d, 4, 114, 'KLASIFIKASI', f"V {u.hitung['VALID']}  I {u.hitung['INVALID']}", ABU)
        if aktif and u.mode == 'klasifikasi':
            self._tombol(d, 10, 130, 140, 20, 'STOP KLASIFIKASI', u.stop, MERAH, font=F_KECIL_TEBAL)
        else:
            self._tombol(d, 10, 130, 140, 20, 'KLASIFIKASI + LIVE', lambda: u.mulai('klasifikasi', self.bus),
                         ORANYE, not terkunci and not aktif, font=F_KECIL_TEBAL)
        # boleh diubah saat klasifikasi berjalan -- dibaca ulang tiap objek
        self._tombol(d, 10, 152, 140, 18, ('[x]' if u.kirim_palang else '[ ]') + ' INVALID ke palang',
                     lambda: setattr(u, 'kirim_palang', not u.kirim_palang), KARTU2, font=F_LABEL)
        # TES
        self._kartu(d, 162, 114, 'TES')
        if aktif and u.mode == 'live':
            self._tombol(d, 168, 130, 70, 20, 'STOP', u.stop, MERAH, font=F_KECIL_TEBAL)
        else:
            self._tombol(d, 168, 130, 70, 20, 'LIVE', lambda: u.mulai('live'), BIRU,
                         not terkunci and not aktif, font=F_KECIL_TEBAL)
        self._tombol(d, 240, 130, 70, 20, f"HASIL {len(u.hasil)}" if u.hasil else 'HASIL',
                     lambda: (setattr(self, 'kamera_hasil', True), setattr(self, 'hasil_hal', 0)),
                     KARTU2, font=F_KECIL_TEBAL)
        # CONVEYOR Sorter (hopper dijeda, sama dengan RUN di tab SORTER): objek ditaruh di conveyor
        # dan lewat kamera dengan kecepatan produksi. Tetap jalan kalau pindah tab -- STOP di sini,
        # di tab SORTER, atau STOP ALL.
        n = self.pemantau.node.get(SORTER)
        conv = bool(n and n['state'] == ST_RUN)
        kunci = self.kunci_manual(SORTER)
        if conv:
            self._tmb(d, 168, 152, 70, 18, 'CONV OFF', SORTER, lambda: self.urutan(SORTER, 'CONVEYOR STOP', [
                ('cmd', 'STOP_MAIN', 0), ('sampai', lambda n: n and n['state'] != ST_RUN, 4, 'conveyor berhenti')]),
                'netral', MERAH)
        elif kunci and kunci[0] == 'Mode manual belum diaktifkan':
            self._tombol(d, 168, 152, 70, 18, 'CONVEYOR', self.aktifkan_manual, BIRU, font=F_KECIL_TEBAL)
        else:
            self._tmb(d, 168, 152, 70, 18, 'CONVEYOR', SORTER, lambda: self.tanya(
                "Jalankan conveyor Sorter?", ["Sorter ke mode MAIN, hopper dijeda.",
                                              "Taruh objek di conveyor -- lewat kamera",
                                              "dengan kecepatan produksi."],
                lambda: self.urutan(SORTER, 'CONVEYOR RUN', [
                    ('cmd', 'START_MAIN', 0), ('cmd', 'SET_HOPPER_JEDA', 1),
                    ('sampai', lambda n: n and n['state'] == ST_RUN, 4, 'conveyor jalan')]), 'RUN', HIJAU, 'play'),
                'netral', BIRU)
        self._tombol(d, 240, 152, 70, 18, 'DIAGNOSA', lambda: u.mulai('diagnosa'), KARTU2,
                     not terkunci and not aktif, font=F_KECIL_TEBAL)
        # baris bawah
        if self.kirim_hasil is not self._hasil_terakhir:      # hasil perintah conveyor: tampil 4 s
            self._hasil_terakhir, self._hasil_t = self.kirim_hasil, time.monotonic()
        if terkunci:
            kotak(d, (4, 176, 316, 198), 6, MERAH)
            d.text((10, 181), 'TERKUNCI: produksi memakai kamera -- STOP dulu', font=F_KECIL_TEBAL, fill=PUTIH)
        elif self.kirim_hasil and (self.sibuk() or time.monotonic() - self._hasil_t < 4):
            self._hasil_kirim(d, 176)
        elif u.pesan:
            teks_p, w = u.pesan
            kotak(d, (4, 176, 316, 198), 6, w)
            teks_tengah(d, 160, 187, potong(d, teks_p, F_KECIL_TEBAL, 300), F_KECIL_TEBAL,
                        HITAM if w == KUNING else PUTIH)
        else:
            kotak(d, (4, 176, 316, 198), 6, KACA)
            d.text((10, 181), potong(d, 'CONVEYOR lalu KLASIFIKASI: sama dengan produksi', F_KECIL, 300),
                   font=F_KECIL, fill=ABU)

    def _kamera_hasil(self, d):
        """Tabel kamera_hasil.csv, terbaru di atas. Uji yang sedang berjalan tetap jalan --
        baris baru muncul langsung di halaman pertama."""
        isi = list(reversed(UJI_KAMERA.daftar_hasil()))
        per = 8
        hal_maks = max(0, (len(isi) - 1) // per)
        hal = min(self.hasil_hal, hal_maks)
        kotak(d, (4, 52, 316, 174), 6, KACA)
        for x, judul in ((10, 'NO'), (54, 'WAKTU'), (158, 'SAMPEL'), (250, 'HASIL')):
            d.text((x, 55), judul, font=F_LABEL, fill=ABU)
        if not isi:
            teks_tengah(d, 160, 112, 'Belum ada hasil', F_BIASA, ABU)
        for i, (no, waktu, sampel, hasil) in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 67 + i * 13
            d.text((10, y), no, font=F_KECIL, fill=PUTIH)
            d.text((54, y), f"{waktu[8:10]}/{waktu[5:7]}{waktu[10:]}", font=F_KECIL, fill=PUTIH)
            d.text((158, y), potong(d, sampel, F_KECIL, 88), font=F_KECIL, fill=PUTIH)
            d.text((250, y), hasil, font=F_KECIL_TEBAL, fill=HIJAU_MUDA if hasil == 'VALID' else MERAH)
        self._tombol(d, 4, 178, 96, 20, '< KEMBALI', lambda: setattr(self, 'kamera_hasil', False), KARTU2,
                     font=F_KECIL_TEBAL)
        teks_tengah(d, 160, 188, f"{hal + 1}/{hal_maks + 1}", F_KECIL, ABU)
        self._tombol(d, 210, 178, 52, 20, '<', lambda: setattr(self, 'hasil_hal', max(0, hal - 1)), KARTU2,
                     hal > 0)
        self._tombol(d, 264, 178, 52, 20, '>', lambda: setattr(self, 'hasil_hal', min(hal_maks, hal + 1)),
                     KARTU2, hal < hal_maks)

    def _kirim_kecepatan(self, sid, langkah):
        baru = max(10, min(100, self.persen_kecepatan(sid)[0] + langkah))
        self.kecepatan[sid] = baru
        self.urutan(sid, f'SPEED {baru}%', [('cmd', 'SET_CONVEYOR_SPEED', round(baru * 255 / 100))])

    # --- PICKER ---
    def _manual_picker(self, d, sid):
        n = self.pemantau.node.get(sid)
        r = n['r'] if n else {}
        pose = NAMA_POSE.get(r.get(10), '?')

        def ke_pose(label, cmd, arg, p):
            return lambda: self.urutan(sid, f'KE {label}', [
                ('cmd', cmd, arg), ('tunggu', 0.5),
                ('sampai', _picker_diam, 40, 'lengan berhenti'),
                ('sampai', lambda n: n and n['r'].get(10) == p, 3, f'pose {label}')])
        self._kartu(d, 4, 52, 'POSE', f"di {pose}", BIRU_MUDA)
        for i, (label, cmd, arg, p) in enumerate((('HOME', 'GOTO_HOME', 0, 0), ('READY', 'GOTO_READY', 0, 3),
                                                  ('PICKUP', 'GOTO_POSE_N', 1, 1), ('LIFT', 'GOTO_POSE_N', 2, 2))):
            self._tmb(d, 10 + (i % 2) * 72, 68 + (i // 2) * 22, 68, 20, label, sid, ke_pose(label, cmd, arg, p))

        g = r.get(17)
        self._kartu(d, 162, 52, 'GERAKAN', '' if g in (None, 0xFF) else GERAKAN_PICKER[g][1] if g < 5 else '',
                    KUNING)
        for i, (lb, nama, tujuan) in enumerate(GERAKAN_PICKER):
            x, y = 168 + (i % 3) * 48, 68 + (i // 3) * 22
            self._tmb(d, x, y, 45, 20, lb, sid, lambda i=i, nama=nama, t=tujuan: self.urutan(sid, nama, [
                ('cmd', 'RUN_GERAKAN', i), ('tunggu', 0.5), ('sampai', _picker_diam, 60, 'gerakan selesai'),
                ('sampai', lambda n: n and n['r'].get(10) == t, 3, f'pose {NAMA_POSE[t]}')]))

        self._kartu(d, 4, 114, 'PICK / PLACE')
        for i, label in enumerate(('PICK', 'PLACE')):
            self._tmb(d, 10 + i * 72, 130, 68, 22, label, sid, lambda lb=label: self.urutan(sid, lb, [
                ('cmd', lb, 0), ('tunggu', 0.5), ('sampai', _picker_diam, 60, f'{lb} selesai')]))
        d.text((10, 156), potong(d, f"Akt: {self.aktivitas(sid)}", F_LABEL, 140), font=F_LABEL, fill=ABU)

        self._kartu(d, 162, 114, 'TEST SEQUENCE')
        langkah = [('cmd', 'GOTO_HOME', 0), ('tunggu', 0.5), ('sampai', _picker_diam, 40, 'lengan di HOME')]
        for i, (_, nama, tujuan) in enumerate(GERAKAN_PICKER + [GERAKAN_PICKER[0]]):
            i = i % len(GERAKAN_PICKER)
            langkah += [('cmd', 'RUN_GERAKAN', i), ('tunggu', 0.5), ('sampai', _picker_diam, 60, f'{nama} selesai'),
                        ('sampai', lambda n, t=tujuan: n and n['r'].get(10) == t, 3, f'pose {NAMA_POSE[tujuan]}')]
        self._tmb(d, 168, 130, 142, 20, 'FULL CYCLE (1x)', sid, lambda: self.tanya(
            "Picker full cycle?", ["HOME, lalu Home>Ready > Ready>Pick >", "Pick>Home > Home>Place > Place>Home",
                                   "> Home>Ready. Tanpa package."],
            lambda: self.urutan(sid, 'FULL CYCLE', langkah), 'JALANKAN', ORANYE, 'alarm'), 'test', ORANYE)
        self._tmb(d, 168, 152, 142, 18, 'STATUS / SENSOR', sid, self._buka_sensor, 'netral', KARTU2)

    # --- DISPENSER ---
    def _manual_dispenser(self, d, sid):
        n = self.pemantau.node.get(sid)
        r = n['r'] if n else {}
        kec, kec_dikirim = self.persen_kecepatan(DISPENSER)
        tahap = r.get(26)
        self._kartu(d, 4, 52, 'CONVEYOR', PIPELINE_NAMES[tahap] if tahap is not None and tahap < len(PIPELINE_NAMES)
                    else '', ABU)
        self._tmb(d, 10, 68, 70, 20, 'ON', sid, lambda: self.urutan(sid, 'CONVEYOR ON', [
            ('cmd', 'CONVEYOR_ON_OFF', 1)]))
        self._tmb(d, 84, 68, 66, 20, 'OFF', sid, lambda: self.urutan(sid, 'CONVEYOR OFF', [
            ('cmd', 'CONVEYOR_ON_OFF', 0)]), 'test', MERAH)
        self._atur(d, 10, 91, 'Speed' if kec_dikirim else 'Speed fw', f"{kec}%",
                   lambda: self._kirim_kecepatan(DISPENSER, -10), lambda: self._kirim_kecepatan(DISPENSER, 10), sid,
                   warna=PUTIH if kec_dikirim else ABU)
        for k, (x, y, no) in enumerate(((162, 52, 1), (4, 114, 2))):
            self._kartu(d, x, y, f'SERVO {no} (GATE)')
            for i, (label, cmd, arg) in enumerate((('AWAL', f'MOVE_SERVO{no}_TO', 0), ('AKHIR', f'MOVE_SERVO{no}_TO', 1),
                                                   ('CYCLE', f'SERVO{no}_CYCLE', 0))):
                self._tmb(d, x + 6 + i * 48, y + 16, 45, 22, label, sid, lambda lb=label, c=cmd, a=arg, no=no:
                          self.urutan(sid, f'SERVO {no} {lb}', [('cmd', c, a)]))
            d.text((x + 6, y + 42), 'titik awal/akhir = kalibrasi node', font=F_LABEL, fill=ABU)
        self._kartu(d, 162, 114, 'TEST SEQUENCE',
                    f"T:{'ADA' if r.get(16) else '--'} U:{'ADA' if r.get(17) else '--'}", BIRU_MUDA)
        self._tmb(d, 168, 130, 142, 20, 'PACKAGE KE UJUNG', sid, lambda: self.urutan(sid, 'KE UJUNG', [
            ('cmd', 'CONVEYOR_ON_OFF', 1),
            ('sampai', lambda n: n and n['r'].get(17) == 1, 15, 'package sampai sensor UJUNG')],
            akhir=[('cmd', 'CONVEYOR_ON_OFF', 0)]), 'test', ORANYE)
        self._tmb(d, 168, 152, 142, 18, 'SENSOR TEST', sid, self._buka_sensor, 'netral', KARTU2)

    # --- STOCKER ---
    def _manual_stocker(self, d, sid):
        n = self.pemantau.node.get(sid)
        r = n['r'] if n else {}
        homed = bool(r.get(11))
        idx = r.get(10)
        self._kartu(d, 4, 52, 'AXIS', 'HOMED' if homed else 'BELUM HOMING', HIJAU_MUDA if homed else KUNING)
        self._tmb(d, 10, 68, 70, 22, 'HOME ALL', sid, lambda: self.urutan(sid, 'HOME ALL', [
            ('cmd', 'HOME_ALL', 0), ('sampai', lambda n: n and n['r'].get(11) == 1, 5, 'ALL_HOMED_FLAG = 1')]),
            'netral')
        self._tmb(d, 84, 68, 66, 22, 'READY', sid, lambda: self.urutan(sid, 'READY', [('cmd', 'GOTO_READY', 0)]),
                  'netral')
        d.text((10, 94), potong(d, f"Akt: {self.aktivitas(sid)}", F_LABEL, 140), font=F_LABEL, fill=ABU)

        self._kartu(d, 162, 52, 'RAK', '-' if idx in (None, 0xFF) else f"di RAK {idx}", BIRU_MUDA)

        def ke_rak(no):
            return [('cmd', 'MOVE_TO_RACK', no), ('sampai', lambda n: n and n['r'].get(10) == no, 5, f'di RAK {no}')]
        for i in range(4):
            self._tmb(d, 168 + i * 36, 68, 33, 22, str(i + 1), sid,
                      lambda no=i + 1: self.urutan(sid, f'KE RAK {no}', ke_rak(no)), 'test', BIRU, idx == i + 1)
        d.text((168, 94), 'gerak ke rak (MOVE_TO_RACK)', font=F_LABEL, fill=ABU)

        self._kartu(d, 4, 114, 'PUSHER')
        self._tmb(d, 10, 130, 140, 22, 'PUSH BOX', sid, lambda: self.tanya(
            "Dorong box?", ["Pusher Stocker mendorong sekali", "di posisi sekarang."],
            lambda: self.urutan(sid, 'PUSH BOX', [('cmd', 'PUSH_BOX', 0)]), 'PUSH', ORANYE, 'alarm'), 'test')

        self._kartu(d, 162, 114, 'TEST SEQUENCE')
        langkah = sum((ke_rak(no) for no in (1, 2, 3, 4)), []) + [('cmd', 'GOTO_READY', 0)]
        self._tmb(d, 168, 130, 142, 20, 'RACK TEST 1-4', sid, lambda: self.tanya(
            "Uji semua rak?", ["Stocker ke RAK 1, 2, 3, 4,", "lalu kembali ke READY. Tanpa dorong."],
            lambda: self.urutan(sid, 'RACK TEST', langkah), 'JALANKAN', ORANYE, 'alarm'), 'test', ORANYE)
        self._tmb(d, 168, 152, 142, 18, 'STATUS / SENSOR', sid, self._buka_sensor, 'netral', KARTU2)

    # --- layar sensor (semua node) ---
    def _buka_sensor(self):
        self.manual_sensor, self.sensor_acuan = True, None

    def _manual_sensor(self, d, sid):
        data = {label: v for label, v, _ in self.pemantau.detail}
        if self.sensor_acuan is None and data:
            self.sensor_acuan = {reg: data.get(reg) for _, reg, hitung in SENSOR_NODE[sid] if hitung}
        self._tombol(d, 4, 52, 70, 18, 'KEMBALI', lambda: setattr(self, 'manual_sensor', False), KARTU2,
                     font=F_LABEL)
        self._tombol(d, 246, 52, 70, 18, 'NOL-KAN +', lambda: setattr(self, 'sensor_acuan', None), KARTU2,
                     font=F_LABEL)
        teks_tengah(d, 160, 61, 'SENSOR / STATUS (live)', F_KECIL_TEBAL, BIRU_MUDA)
        kotak(d, (4, 72, 316, 172), 6, KACA)
        if not data:
            teks_tengah(d, 160, 120, 'membaca...', F_BIASA, ABU)
        for i, (label, reg, hitung) in enumerate(SENSOR_NODE[sid]):
            if not data:
                break
            kol, baris = i % 2, i // 2
            x, y = 10 + kol * 156, 76 + baris * 32
            nilai, w = self._format_field(sid, reg, data.get(reg))
            d.text((x, y), label, font=F_LABEL, fill=ABU)
            d.text((x, y + 10), potong(d, nilai, F_TEBAL, 96), font=F_TEBAL, fill=w)
            acuan = (self.sensor_acuan or {}).get(reg)
            if hitung and isinstance(data.get(reg), int) and isinstance(acuan, int):
                teks_kanan(d, x + 146, y + 12, f"+{(data[reg] - acuan) % 65536}", F_KECIL_TEBAL, HIJAU_MUDA)

    # ---------- HISTORY ----------
    def _history(self, d):
        self._judul_isi(d, None, '', self._kembali_more)
        for i, (mode, label) in enumerate((('harian', 'DAILY'), ('batch', 'BATCH'))):
            self._tombol(d, 42 + i * 90, Y_ISI, 86, 24, label,
                         lambda m=mode: (setattr(self, 'history_mode', m), setattr(self, 'halaman', 0)),
                         BIRU if self.history_mode == mode else KARTU2)
        pr = self.produksi_riwayat
        if self.history_mode == 'harian':
            h = pr._hari_ini()
            total = h['ok'] + h['reject']
            menit = h['detik_jalan'] / 60
            dur = [b['durasi'] for b in pr.batch if b.get('durasi') and
                   time.strftime('%Y-%m-%d', time.localtime(b['t'])) == time.strftime('%Y-%m-%d')]
            kotak(d, (4, 54, 316, 112), 8, KACA)
            d.text((10, 57), 'HARI INI ' + time.strftime('%d/%m/%Y'), font=F_LABEL, fill=ABU)
            sel = [('Total', f"{total:,}", PUTIH), ('OK', f"{h['ok']:,}", HIJAU_MUDA),
                   ('Reject', f"{h['reject']:,}", MERAH),
                   ('Yield', f"{h['ok'] * 100 / total:.1f}%" if total else '-', BIRU_MUDA),
                   ('Thru', f"{total / menit:.0f}/min" if menit >= 1 else '-', BIRU_MUDA),
                   ('Cycle', detik_pendek(sum(dur) / len(dur)) if dur else '-', PUTIH)]
            for i, (label, nilai, w) in enumerate(sel):
                x = 10 + i * 51
                d.text((x, 70), label, font=F_LABEL, fill=ABU)
                d.text((x, 82), potong(d, nilai, F_KECIL_TEBAL, 49), font=F_KECIL_TEBAL, fill=w)
            d.text((10, 98), f"Batch {h['batch']}   jalan {lama_waktu(h['detik_jalan'])}", font=F_LABEL, fill=ABU)
            kotak(d, (4, 116, 316, 199), 8, KACA)
            hari = sorted(pr.hari.items(), reverse=True)[:5]
            d.text((10, 119), 'TANGGAL', font=F_LABEL, fill=ABU)
            for x, judul in ((110, 'TOTAL'), (170, 'OK'), (226, 'REJECT'), (280, 'YIELD')):
                d.text((x, 119), judul, font=F_LABEL, fill=ABU)
            for i, (tgl, v) in enumerate(hari):
                y = 131 + i * 13
                t = v['ok'] + v['reject']
                d.text((10, y), tgl, font=F_KECIL, fill=PUTIH)
                for x, nilai in ((110, f"{t:,}"), (170, f"{v['ok']:,}"), (226, f"{v['reject']:,}"),
                                 (280, f"{v['ok'] * 100 / t:.0f}%" if t else '-')):
                    d.text((x, y), nilai, font=F_KECIL, fill=PUTIH)
            return
        isi = list(reversed(pr.batch))
        per = 7
        hal_maks = max(0, (len(isi) - 1) // per)
        hal = min(self.halaman, hal_maks)
        self._tombol(d, 246, Y_ISI, 34, 24, '<', lambda: setattr(self, 'halaman', max(0, hal - 1)), KARTU2, hal > 0)
        self._tombol(d, 282, Y_ISI, 34, 24, '>', lambda: setattr(self, 'halaman', min(hal_maks, hal + 1)), KARTU2,
                     hal < hal_maks)
        kotak(d, (4, 54, 316, 199), 8, KACA)
        for x, judul in ((10, 'WAKTU'), (110, 'BATCH'), (170, 'RAK'), (230, 'DURASI')):
            d.text((x, 57), judul, font=F_LABEL, fill=ABU)
        if not isi:
            teks_tengah(d, 160, 125, 'Belum ada batch', F_BIASA, ABU)
        for i, b in enumerate(isi[hal * per:(hal + 1) * per]):
            y = 70 + i * 18
            d.text((10, y), time.strftime('%d/%m %H:%M', time.localtime(b['t'])), font=F_KECIL, fill=PUTIH)
            d.text((110, y), f"#{b['nomor']}", font=F_KECIL_TEBAL, fill=BIRU_MUDA)
            d.text((170, y), str(b.get('rak') or '-'), font=F_KECIL, fill=PUTIH)
            d.text((230, y), detik_pendek(b.get('durasi')), font=F_KECIL, fill=PUTIH)

    # ---------- SETTING ----------
    def _setting(self, d):
        self._judul_isi(d, None, '', self._kembali_more)
        for i, (nama, _) in enumerate(KATEGORI_SETTING):
            y = 54 + i * 24
            self._tombol(d, 4, y, 90, 22, nama, lambda i=i: (setattr(self, 'kategori', i), setattr(self, 'halaman', 0)),
                         BIRU if i == self.kategori else KACA, font=F_KECIL_TEBAL)
        nama, daftar = KATEGORI_SETTING[self.kategori]
        per = 5
        hal_maks = max(0, (len(daftar) - 1) // per)
        hal = min(self.halaman, hal_maks)
        d.text((44, Y_ISI + 5), nama.upper(), font=F_JUDUL, fill=PUTIH)
        if hal_maks:
            self._tombol(d, 236, Y_ISI, 38, 24, '<', lambda: setattr(self, 'halaman', max(0, hal - 1)), KARTU2,
                         hal > 0)
            self._tombol(d, 278, Y_ISI, 38, 24, '>', lambda: setattr(self, 'halaman', min(hal_maks, hal + 1)),
                         KARTU2, hal < hal_maks)
        kotak(d, (98, 54, 316, 199), 8, KACA)
        for i, baris in enumerate(daftar[hal * per:(hal + 1) * per]):
            y = 58 + i * 28
            if baris[0] == 'aksi':
                self._tombol(d, 104, y, 206, 24, baris[1], getattr(self, baris[2]), BIRU)
                continue
            bagian, kunci, label = baris
            if bagian == 'lcd':
                jenis, nilai = 'baca', teks_nilai('lcd', kunci, self.lcd.get(kunci))
            else:
                jenis, _ = jenis_setting(bagian, kunci)
                nilai = teks_nilai(bagian, kunci, nilai_sekarang(kunci))
            d.text((104, y + 6), potong(d, label, F_KECIL, 126), font=F_KECIL, fill=ABU if jenis == 'baca' else PUTIH)
            kotak(d, (236, y + 2, 310, y + 22), 5, KARTU2 if jenis != 'baca' else KARTU,
                  outline=GARIS if jenis != 'baca' else None)
            teks_tengah(d, 273, y + 12, potong(d, nilai, F_TEBAL, 70), F_TEBAL, KUNING if jenis != 'baca' else ABU)
            if bagian == 'lcd':
                self.tombol.append(Tombol(100, y, 214, 26, '', lambda: self.beri_pesan(
                    "Ubah di display.config, lalu restart", ORANYE)))
            else:
                self.tombol.append(Tombol(100, y, 214, 26, '', lambda b=bagian, k=kunci, l=label: self._buka_edit(b, k, l)))

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
                     'teks': lambda v: teks_nilai(bagian, kunci, v), 'simpan': self._simpan_setting}

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
            tulis_config(e['bagian'], e['kunci'], tampil_nilai(lama))
            muat_ulang_config()
            self.beri_pesan("Nilai ditolak config -- dibatalkan", MERAH)
            return
        log(f"[panel] {e['kunci']} = {tampil_nilai(e['nilai'])} (dari LCD)")
        self.edit = None
        self.beri_pesan("Tersimpan", HIJAU, 1.5)

    def _rak(self, d):
        self._judul_isi(d, 'rak', 'KELOLA RAK', lambda: self.ke('more', 'setting'))
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
            self._tombol(d, x, 70, 75, 88, '', ubah, ORANYE if terisi else HIJAU)
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
        self._tombol(d, 4, 164, 312, 34, 'SEMUA KOSONG', semua, KARTU2)

    # ---------- SYSTEM ----------
    def baca_versi(self):
        """Baca versi firmware semua node (dipanggil dari thread, bukan thread layar)."""
        for sid in ALUR:
            self.versi[sid] = versi_node(self.bus, sid)
        self.versi_t = time.time()

    def _baca_versi_latar(self):
        if self.bus is None:
            self.beri_pesan("RS485 tidak terbuka", MERAH)
            return
        self.versi_t = None
        threading.Thread(target=self.baca_versi, daemon=True).start()

    def _sistem(self, d):
        self._judul_isi(d, None, '', self._kembali_more)
        for i, (mode, label) in enumerate((('opi', 'ORANGE PI'), ('node', 'NODE'))):
            def pilih(m=mode):
                self.sistem_mode = m
                if m == 'node' and not self.versi:
                    self._baca_versi_latar()
            self._tombol(d, 42 + i * 110, Y_ISI, 106, 24, label, pilih,
                         BIRU if self.sistem_mode == mode else KARTU2, font=F_KECIL_TEBAL)
        kotak(d, (4, 54, 316, 172), 8, KACA)
        if self.sistem_mode == 'opi':
            for i, (k, v) in enumerate(info_orangepi()):
                y = 57 + i * 12
                d.text((10, y), k, font=F_LABEL, fill=ABU)
                d.text((70, y - 1), potong(d, v, F_MINI, 240), font=F_MINI, fill=PUTIH)
            self._tombol(d, 4, 176, 154, 22, 'LOG', self.buka_log, KARTU2, font=F_KECIL_TEBAL)
            self._tombol(d, 162, 176, 154, 22, 'KALIBRASI SENTUH', self.mulai_kalibrasi, BIRU, font=F_KECIL_TEBAL)
            return
        d.text((10, 57), 'NODE', font=F_LABEL, fill=ABU)
        d.text((96, 57), 'FIRMWARE (waktu build)', font=F_LABEL, fill=ABU)
        teks_kanan(d, 310, 57, 'UPTIME  I2C  FAULT', F_LABEL, ABU)
        if self.versi_t is None and not self.versi:
            teks_tengah(d, 160, 112, 'membaca...', F_BIASA, ABU)
        for i, sid in enumerate(ALUR):
            y = 70 + i * 25
            v = self.versi.get(sid)
            _, w = self.status_node(sid)
            d.ellipse((10, y + 4, 17, y + 11), fill=w)
            d.text((22, y + 1), SINGKAT[sid][:9], font=F_KECIL_TEBAL, fill=PUTIH)
            if v:
                lama = not v['baru']
                d.text((96, y + 1), potong(d, v['fw'], F_KECIL_TEBAL, 214), font=F_KECIL_TEBAL,
                       fill=ORANYE if lama else HIJAU_MUDA)
                wifi = (f"{v['ip']}  {v['rssi']} dBm" if v['ip'] else
                        ('WiFi tidak tersambung' if v['baru'] else ''))
                d.text((22, y + 13), wifi, font=F_LABEL, fill=ABU if v['ip'] else ORANYE)
                diag = (f"{lama_waktu(v['uptime']) if v['uptime'] is not None else '-'}  "
                        f"{v['i2c'] if v['i2c'] is not None else '-'}  "
                        f"{v['last_fault'] if v['last_fault'] is not None else '-'}")
                teks_kanan(d, 310, y + 13, diag, F_LABEL, ABU)
            if i < 3:
                d.line([(10, y + 23), (310, y + 23)], fill=GARIS)
        if self.versi_t:     # jam pembacaan di kanan baris judul (bukan di bawah -- menimpa baris STOCKER)
            teks_kanan(d, 314, Y_ISI + 8, time.strftime('%H:%M:%S', time.localtime(self.versi_t)), F_LABEL, ABU)
        for i, (label, aksi, w, aktif) in enumerate((
                ('BACA ULANG', self._baca_versi_latar, BIRU, self.bus is not None),
                ('UPDATE FW', lambda: self.ke('more', 'ota'), ORANYE, True),
                ('KALIBRASI', self.buka_kal, HIJAU, True),
                ('LOG', self.buka_log, KARTU2, True))):
            self._tombol(d, 4 + i * 78, 176, 76, 22, label, aksi, w, aktif, font=F_LABEL)

    # ---------- KALIBRASI NODE (backup / restore) ----------
    def _kal(self, d):
        """Backup kalibrasi tiap node ke Orange Pi dan restore ke ESP32 pengganti."""
        self._judul_isi(d, None, 'KALIBRASI NODE', lambda: self.ke('more', 'sistem'))
        k = self.kal_kerja
        sibuk = k is not None and k['tahap'] not in ('selesai', 'gagal')
        t, backup = self._kal_cache
        if backup is None or time.monotonic() - t > 5:
            backup = {sid: kalibrasi_daftar(NAMA[sid]) for sid in ALUR}
            self._kal_cache = (time.monotonic(), backup)
        kotak(d, (4, 54, 316, 144), 8, KACA)
        d.text((10, 56), 'NODE', font=F_LABEL, fill=ABU)
        d.text((96, 56), 'BACKUP TERAKHIR', font=F_LABEL, fill=ABU)
        teks_kanan(d, 310, 56, 'vs NODE', F_LABEL, ABU)
        for i, sid in enumerate(ALUR):
            y = 68 + i * 19
            daftar, info = backup[sid], self.kal_node.get(sid)
            if sid == self.kal_sid:
                kotak(d, (6, y - 1, 314, y + 16), 4, BIRU + (110,))
            self.tombol.append(Tombol(6, y - 1, 308, 18, '', lambda s=sid: (setattr(self, 'kal_sid', s),
                                                                          setattr(self, 'kal_pilih', 0)),
                                      KARTU2, not sibuk))
            d.text((12, y + 1), SINGKAT[sid][:9], font=F_KECIL_TEBAL, fill=PUTIH)
            d.text((96, y + 1), daftar[0]['waktu'][:16] if daftar else 'belum ada', font=F_KECIL,
                   fill=PUTIH if daftar else ORANYE)
            # bandingkan CRC node (CEK / BACKUP) dengan backup terakhir
            if info is None:
                status, w = '-', ABU
            elif info.get('galat'):
                status, w = info['galat'], MERAH
            elif not daftar:
                status, w = 'belum backup', ORANYE
            elif info['crc'] == daftar[0]['crc']:
                status, w = 'SAMA', HIJAU_MUDA
            else:
                status, w = 'BERUBAH', ORANYE
            teks_kanan(d, 310, y + 1, potong(d, status, F_KECIL_TEBAL, 90), F_KECIL_TEBAL, w)
        # baris keterangan: backup yang akan di-RESTORE ke node terpilih, atau progres kerja
        sid = self.kal_sid
        daftar = backup[sid]
        pilih = min(self.kal_pilih, max(0, len(daftar) - 1))
        if k is not None and (sibuk or time.monotonic() - k['t'] < 6):
            w = {'selesai': HIJAU, 'gagal': MERAH}.get(k['tahap'], KUNING)
            kotak(d, (4, 148, 316, 172), 6, w)
            if sibuk:
                bar(d, 10, 166, 300, 3, k['persen'], HITAM)
            teks_tengah(d, 160, 158, potong(d, k['pesan'], F_KECIL_TEBAL, 300), F_KECIL_TEBAL,
                        HITAM if w == KUNING else PUTIH)
        elif daftar:
            r = daftar[pilih]
            d.text((8, 149), f"Restore {SINGKAT[sid]} dari backup {pilih + 1}/{len(daftar)}:", font=F_LABEL, fill=ABU)
            d.text((8, 160), potong(d, f"{r['waktu'][:16]}  {r['fw'].split()[0]}  {r['ukuran']} B", F_KECIL, 200),
                   font=F_KECIL, fill=PUTIH)
            self._tombol(d, 214, 150, 50, 22, '<', lambda: setattr(self, 'kal_pilih', min(len(daftar) - 1, pilih + 1)),
                         KARTU2, pilih < len(daftar) - 1 and not sibuk)
            self._tombol(d, 266, 150, 50, 22, '>', lambda: setattr(self, 'kal_pilih', max(0, pilih - 1)),
                         KARTU2, pilih > 0 and not sibuk)
        else:
            d.text((8, 154), potong(d, f"{SINGKAT[sid]}: belum ada backup -- tekan BACKUP SEMUA", F_KECIL, 304),
                   font=F_KECIL, fill=ORANYE)
        boleh = self.bus is not None and not sibuk and not self.produksi_aktif()
        self._tombol(d, 4, 176, 70, 22, 'CEK', lambda: self._mulai_kal('cek'), BIRU, boleh, font=F_KECIL_TEBAL)
        self._tombol(d, 78, 176, 118, 22, 'BACKUP SEMUA', lambda: self._mulai_kal('backup'), HIJAU, boleh,
                     font=F_KECIL_TEBAL)
        self._tombol(d, 200, 176, 116, 22, 'RESTORE', lambda: self._konfirmasi_restore(sid, daftar[pilih]),
                     ORANYE, boleh and bool(daftar), font=F_KECIL_TEBAL)

    def buka_kal(self):
        self.ke('more', 'kal')
        if self.bus is not None and not self.produksi_aktif() and (
                self.kal_kerja is None or self.kal_kerja['tahap'] in ('selesai', 'gagal')):
            self._mulai_kal('cek')        # langsung bandingkan node dengan backup terakhir

    def _konfirmasi_restore(self, sid, rekam):
        self.tanya(f"Restore {NAMA[sid]}?", [f"Backup {rekam['waktu'][:16]} ({rekam['fw'].split()[0]})",
                                             "Kalibrasi di node DITIMPA,", "node restart sekitar 5 detik."],
                   lambda: self._mulai_kal('restore', sid, rekam), 'RESTORE', ORANYE, 'alarm')

    def _mulai_kal(self, jenis, sid=None, rekam=None):
        """jenis: 'cek' (bandingkan node vs backup), 'backup' (semua node), 'restore' (satu node)."""
        if self.produksi_aktif():
            self.beri_pesan("Hentikan produksi dulu", ORANYE)
            return
        self.kal_kerja = {'tahap': 'kerja', 'persen': 0.0, 'pesan': 'mulai...', 't': time.monotonic()}
        k = self.kal_kerja

        def pesan(teks, persen=None):
            k.update(pesan=teks, t=time.monotonic(), **({} if persen is None else {'persen': persen}))

        def kerja():
            gagal = []
            try:
                if jenis == 'restore':
                    self.ota_jalan = sid           # alarm "tidak menjawab" diredam selama node restart
                    log(f"[kalibrasi] RESTORE {NAMA[sid]} dari backup {rekam['waktu']} ({rekam['fw']})")
                    baru = kalibrasi_tulis(self.bus, sid, rekam,
                                           lambda f: pesan(f"restore {NAMA[sid]} {int(f * 100)}%", f),
                                           lambda: (pesan(f"{NAMA[sid]} restart, menunggu...", 0.85),
                                                    BERHENTI_PANEL.wait(4)))
                    self.kal_node[sid] = {'crc': baru['crc']}
                    log(f"[kalibrasi] RESTORE {NAMA[sid]} BERHASIL (CRC {baru['crc']:04X})")
                    k.update(tahap='selesai', pesan=f"RESTORE {NAMA[sid]} BERHASIL -- terverifikasi", t=time.monotonic())
                    return
                for i, s in enumerate(ALUR):
                    awal = i / len(ALUR)
                    try:
                        if jenis == 'cek':
                            pesan(f"cek {NAMA[s]}...", awal)
                            _cal_perintah(self.bus, s, OP_CAL_BACA, 0, 'CAL_BACA')
                            self.kal_node[s] = {'crc': self.bus.baca(s, R_CAL[s] + 2)}
                        else:
                            r = kalibrasi_baca(self.bus, s, lambda f: pesan(f"backup {NAMA[s]} {int(f * 100)}%",
                                                                            awal + f / len(ALUR)))
                            path = kalibrasi_simpan(r)
                            self._kal_cache = (0.0, None)
                            self.kal_node[s] = {'crc': r['crc']}
                            log(f"[kalibrasi] backup {NAMA[s]} -> {path} ({r['ukuran']} byte)")
                    except RegisterTidakAda:
                        self.kal_node[s] = {'galat': 'FW < v2.01'}
                        gagal.append(NAMA[s])
                    except Exception as e:
                        self.kal_node[s] = {'galat': 'GAGAL'}
                        gagal.append(NAMA[s])
                        log(f"!! [kalibrasi] {jenis} {NAMA[s]}: {e}")
                judul = 'CEK' if jenis == 'cek' else 'BACKUP'
                if gagal:
                    k.update(tahap='gagal', pesan=f"{judul}: gagal di {', '.join(gagal)} -- lihat LOG", t=time.monotonic())
                else:
                    k.update(tahap='selesai', persen=1.0, t=time.monotonic(),
                             pesan='BACKUP semua node tersimpan' if jenis == 'backup' else 'CEK selesai')
            except Exception as e:
                log(f"!! [kalibrasi] {jenis}: {e}")
                k.update(tahap='gagal', pesan=f"GAGAL: {e}", t=time.monotonic())
            finally:
                if jenis == 'restore':
                    self.ota_jalan = None
        threading.Thread(target=kerja, daemon=True).start()

    def _ota(self, d):
        """UPDATE FIRMWARE lewat WiFi (OTA). IP diambil dari register node, tidak diketik."""
        self._judul_isi(d, None, '', lambda: self.ke('more', 'sistem'))
        sibuk = self.ota is not None and self.ota['tahap'] not in ('selesai', 'gagal')
        for i, sid in enumerate(ALUR):
            self._tombol(d, 42 + i * 69, Y_ISI, 66, 24, SINGKAT[sid][:7],
                         lambda s=sid: setattr(self, 'ota_sid', s),
                         BIRU if self.ota_sid == sid else KARTU2, not sibuk, font=F_KECIL_TEBAL)
        sid = self.ota_sid
        v = self.versi.get(sid) or {}
        path, ket_file, galat_file = file_firmware(sid)
        pw = password_ota()
        kotak(d, (4, 54, 316, 150), 8, KACA)
        baris = [('Sekarang', v.get('fw', '- (BACA ULANG dulu)'), HIJAU_MUDA if v.get('baru') else ORANYE),
                 ('WiFi', f"{v['ip']}  ({v['rssi']} dBm)" if v.get('ip') else 'IP belum diketahui / tidak tersambung',
                  PUTIH if v.get('ip') else ORANYE),
                 ('File', f"{FOLDER_FIRMWARE}/{NAMA_FILE_FIRMWARE[sid]}  {ket_file}", PUTIH if not galat_file else ORANYE),
                 ('Password', os.path.basename(FILE_OTA_PASSWORD) + (' ada' if pw else ' BELUM ADA'),
                  PUTIH if pw else ORANYE)]
        for i, (k, teks, w) in enumerate(baris):
            y = 58 + i * 17
            d.text((10, y + 1), k, font=F_LABEL, fill=ABU)
            d.text((66, y), potong(d, teks, F_KECIL, 244), font=F_KECIL, fill=w)
        o = self.ota if self.ota and self.ota['sid'] == sid else None
        if o:
            bar(d, 10, 130, 300, 8, o['persen'] / 100, {'gagal': MERAH, 'selesai': HIJAU}.get(o['tahap'], BIRU))
            teks_kanan(d, 310, 139, f"{o['persen']}%", F_LABEL, ABU)
        alasan = None
        if self.produksi_aktif():
            alasan = 'Terkunci: produksi berjalan -- STOP dulu'
        elif not v.get('ip'):
            alasan = 'IP node belum diketahui -- BACA ULANG (butuh firmware v2.00+ & WiFi)'
        elif galat_file:
            alasan = galat_file
        elif not pw:
            alasan = f"Buat file {os.path.basename(FILE_OTA_PASSWORD)} berisi password OTA"
        if o:
            w = {'selesai': HIJAU if o['jenis'] == 'ok' else ORANYE, 'gagal': MERAH}.get(o['tahap'], KUNING)
            kotak(d, (4, 152, 316, 172), 6, w)
            teks_tengah(d, 160, 162, potong(d, o['pesan'], F_KECIL_TEBAL, 300), F_KECIL_TEBAL,
                        HITAM if w == KUNING else PUTIH)
        elif alasan:
            d.text((8, 156), potong(d, alasan, F_KECIL, 304), font=F_KECIL, fill=ORANYE)
        self._tombol(d, 4, 176, 200, 22, 'KIRIM UPDATE', lambda: self._konfirmasi_ota(sid, path, ket_file),
                     ORANYE, alasan is None and not sibuk, font=F_KECIL_TEBAL)
        self._tombol(d, 208, 176, 108, 22, 'BACA ULANG', self._baca_versi_latar, BIRU,
                     self.bus is not None and not sibuk, font=F_KECIL_TEBAL)

    def _konfirmasi_ota(self, sid, path, ket_file):
        v = self.versi.get(sid, {})
        self.tanya(f"Update {NAMA[sid]}?", [f"Sekarang: {v.get('fw', '-')}", f"File: {ket_file}",
                                            f"Ke {v.get('ip')} -- node berhenti &", "restart sekitar 30 detik."],
                   lambda: self._mulai_ota(sid, path), 'UPDATE', ORANYE, 'alarm')

    def _mulai_ota(self, sid, path):
        v_awal = dict(self.versi.get(sid, {}))
        self.ota = {'sid': sid, 'tahap': 'kirim', 'persen': 0, 'jenis': None, 'pesan': 'menghubungi node...'}
        self.ota_jalan = sid

        def progres(f):
            self.ota.update(persen=int(f * 100), pesan=f"mengirim firmware {int(f * 100)}%")

        def kerja():
            try:
                log(f"[OTA] {NAMA[sid]} {v_awal.get('ip')}: kirim {path} (sekarang {v_awal.get('fw')})")
                ota_kirim(v_awal['ip'], path, password_ota(), progres)
                log(f"[OTA] {NAMA[sid]}: firmware diterima & diverifikasi node, node restart")
                self.ota.update(tahap='restart', persen=100, pesan='node restart, menunggu menjawab Modbus...')
                v_baru, batas = None, time.monotonic() + 60
                BERHENTI_PANEL.wait(5)
                while time.monotonic() < batas and not BERHENTI_PANEL.is_set():
                    v_baru = versi_node(self.bus, sid)
                    if v_baru['jawab']:
                        break
                    BERHENTI_PANEL.wait(2)
                if not v_baru or not v_baru['jawab']:
                    raise Gagal("firmware terkirim, tapi node tidak menjawab Modbus 60 s setelah restart")
                self.versi[sid] = v_baru
                if v_baru['fw'] != v_awal.get('fw'):
                    self.ota.update(tahap='selesai', jenis='ok', pesan=f"BERHASIL -- sekarang {v_baru['fw']}")
                else:
                    self.ota.update(tahap='selesai', jenis='sama',
                                    pesan=f"terkirim, tapi versi sama ({v_baru['fw']}) -- file lama?")
                log(f"[OTA] {NAMA[sid]}: {self.ota['pesan']}")
            except Exception as e:
                self.ota.update(tahap='gagal', pesan=f"GAGAL: {e}")
                log(f"!! [OTA] {NAMA[sid]}: {e}")
            finally:
                self.ota_jalan = None
        threading.Thread(target=kerja, daemon=True).start()


    def _log(self, d):
        self._judul_isi(d, 'daftar', 'LOG', self.log_kembali or (lambda: self.ke('more', 'sistem')))
        semua = list(LOG_BARU)
        per = 11
        akhir = max(0, len(semua) - self.log_geser)
        kotak(d, (4, 54, 316, 199), 8, KACA_GELAP)
        for i, b in enumerate(semua[max(0, akhir - per):akhir]):
            d.text((8, 56 + i * 13), potong(d, b[9:], F_MINI, 304), font=F_MINI, fill=MERAH if '!!' in b else PUTIH)

        def naik():
            self.log_geser = min(self.log_geser + per, max(0, len(semua) - per))

        def turun():
            self.log_geser = max(0, self.log_geser - per)
        for i, (teks_t, aksi) in enumerate((('NAIK', naik), ('TURUN', turun))):
            self._tombol(d, 178 + i * 70, Y_ISI, 66, 24, teks_t, aksi, KARTU2)

    # ---------- POPUP ----------
    def _popup(self, d):
        p = self.popup
        kotak(d, (34, 22, 286, 218), 12, KARTU, outline=GARIS)
        ikon(d, p['ikon'], 160, 44, 13, KUNING if p['ikon'] in ('tanya', 'info', 'alarm') else PUTIH)
        teks_tengah(d, 160, 72, potong(d, p['judul'], F_JUDUL, 240), F_JUDUL, PUTIH)
        for i, b in enumerate(p['baris'][:4]):
            teks_tengah(d, 160, 93 + i * 16, potong(d, b, F_KECIL, 240), F_KECIL, ABU)

        def ya():
            self.popup = None
            p['aksi']()

        def batal():
            self.popup = None
        for x, teks_t, aksi, w in ((44, 'BATAL', batal, ABU_GELAP), (164, p['ya'], ya, p['warna'])):
            self._tombol(d, x, 168, 112, 40, teks_t, aksi, w)

    def _popup_edit(self, d):
        e = self.edit
        kotak(d, (14, 18, 306, 222), 12, KARTU, outline=GARIS)
        teks_tengah(d, 160, 32, potong(d, e['label'], F_JUDUL, 280), F_JUDUL, PUTIH)
        teks_tengah(d, 160, 48, potong(d, e['catatan'], F_KECIL, 280), F_KECIL, ABU)
        teks = e.get('teks', tampil_nilai)
        teks_tengah(d, 160, 80, teks(e['nilai']), F_BESAR, KUNING)
        if e['jenis'] == 'pilih':
            pil = e['pilihan']
            lebar = (276 - 4 * (len(pil) - 1)) // len(pil)
            for i, v in enumerate(pil):
                self._tombol(d, 22 + i * (lebar + 4), 108, lebar, 40, teks(v), lambda v=v: e.update(nilai=v),
                             BIRU if v == e['nilai'] else KARTU2)
        else:
            langkah = e['pilihan'][e['langkah']]
            minimum = 1 if e['kunci'] == 'batch_size' else 0

            def ubah(arah):
                v = e['nilai'] + arah * langkah
                v = round(v, 2) if e['jenis'] == 'float' else int(v)
                e['nilai'] = min(e.get('maks', 65535), max(minimum, v))
            for i, (teks_t, aksi) in enumerate(((f"- {langkah:g}", lambda: ubah(-1)),
                                                (f"langkah {langkah:g}",
                                                 lambda: e.update(langkah=(e['langkah'] + 1) % len(e['pilihan']))),
                                                (f"+ {langkah:g}", lambda: ubah(1)))):
                self._tombol(d, 22 + i * 94, 108, 90, 40, teks_t, aksi, KARTU2)

        def batal():
            self.edit = None

        def simpan():
            e['popup'] = False
            e['simpan']()
        self._tombol(d, 22, 166, 134, 46, 'BATAL', batal, ABU_GELAP)
        self._tombol(d, 164, 166, 134, 46, 'SIMPAN' if e.get('bagian') else 'LANJUT', simpan, HIJAU)

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
# LOOP UTAMA -- tanpa sleep() untuk penjadwalan: setiap putaran menghitung kapan sentuhan
# berikutnya dipindai & kapan layar perlu digambar, lalu menunggu event BERHENTI_PANEL
# selama sisa waktu itu (bisa langsung bangun saat program dihentikan).
# ============================================================
SCAN_SENTUH_S = 0.02
SEHAT_S = 60          # tiap sekian detik: catat beban panel ke log (diagnosa hang)


def _baca_angka(path, ubah=float):
    try:
        with open(path) as f:
            return ubah(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def catat_sehat(waktu_gambar):
    """Satu baris log: lama gambar, memori proses & sistem, suhu, beban CPU, jumlah thread.
    Kalau Orange Pi hang, baris terakhir sebelum hang menunjukkan apa yang naik."""
    try:
        with open('/proc/self/statm') as f:
            rss = int(f.read().split()[1]) * os.sysconf('SC_PAGE_SIZE') // 2**20
    except (OSError, ValueError, AttributeError):
        rss = None
    bebas = None
    try:
        with open('/proc/meminfo') as f:
            info = {b.split(':')[0]: int(b.split()[1]) for b in f if ':' in b}
        bebas = info.get('MemAvailable', 0) // 1024
    except (OSError, ValueError):
        pass
    suhu = _baca_angka('/sys/class/thermal/thermal_zone0/temp')
    beban = _baca_angka('/proc/loadavg')
    rata = sum(waktu_gambar) / len(waktu_gambar) if waktu_gambar else 0
    log(f"[sehat] gambar {rata * 1000:.0f} ms rata / {max(waktu_gambar or [0]) * 1000:.0f} ms maks "
        f"({len(waktu_gambar)} frame), RAM proses {rss if rss is not None else '?'} MB, "
        f"RAM bebas {bebas if bebas is not None else '?'} MB, "
        f"suhu {suhu / 1000 if suhu else '?'} C, load {beban if beban is not None else '?'}, "
        f"thread {threading.active_count()}")


def jalankan(panel, layar, sentuh):
    ditekan_sejak = None
    kandidat = None           # sentuhan pertama -- baru dipakai kalau masih ditekan di scan berikutnya
    lepas = 0
    terakhir_gambar = 0.0
    perlu = True
    waktu_gambar, sehat_berikut = [], time.monotonic() + 5
    while not BERHENTI_PANEL.is_set():
        sekarang = time.monotonic()
        INDIKATOR.detak()                  # heartbeat LED OPR: loop layar masih berputar
        if sekarang >= sehat_berikut:
            catat_sehat(waktu_gambar)
            waktu_gambar, sehat_berikut = [], sekarang + SEHAT_S
        mentah = sentuh.mentah() if sentuh else None
        if mentah is not None:
            lepas = 0
            if ditekan_sejak is None:
                if kandidat is None:
                    kandidat = sekarang          # tunggu satu scan: tekanan stabil dulu
                elif sekarang - kandidat >= 0.02:
                    ditekan_sejak, kandidat = sekarang, None
                    if panel.kalibrasi_aktif:
                        panel.tekan_kalibrasi(mentah)
                    elif panel.kal.ada():
                        p = panel.kal.ke_layar(mentah)
                        if panel.di_tab_bar(p):
                            panel.geser_mulai(p)
                        else:
                            panel.tekan(p)
                    elif panel.boot:
                        panel.lewati_boot()
                    perlu = True
            elif panel.geser is not None:
                if panel.geser_ke(panel.kal.ke_layar(mentah)):
                    perlu = True
            elif sekarang - ditekan_sejak > 5 and not panel.kalibrasi_aktif and not panel.boot:
                panel.mulai_kalibrasi()          # jalan keluar kalau kalibrasi rusak
                perlu = True
        else:
            kandidat = None
            if ditekan_sejak is not None:
                lepas += 1
                if lepas >= 3:                   # ~3 scan kosong = jari benar-benar diangkat
                    ditekan_sejak = None
                    if panel.geser is not None:
                        panel.geser_selesai()
                        perlu = True
        if perlu or sekarang - terakhir_gambar >= panel.interval():
            t0 = time.monotonic()
            layar.tampilkan(gambar_aman(panel))
            terakhir_gambar, perlu = time.monotonic(), False
            waktu_gambar.append(terakhir_gambar - t0)
        tunggu = min(SCAN_SENTUH_S, max(0.0, terakhir_gambar + panel.interval() - time.monotonic()))
        BERHENTI_PANEL.wait(tunggu if sentuh else max(tunggu, 0.05))


BERHENTI_PANEL = threading.Event()


def gambar_aman(panel):
    """Error di satu layar TIDAK boleh mematikan loop layar (tampilan membeku, sentuhan mati,
    produksi tetap jalan tanpa terlihat). Error dicatat lengkap ke log, panel kembali ke HOME."""
    try:
        return panel.gambar()
    except Exception:
        import traceback
        log("!! [panel] error saat menggambar layar "
            f"{panel.tab}/{panel.sub} -- kembali ke HOME:\n" + traceback.format_exc())
        panel.popup = panel.edit = None
        panel.tab, panel.sub, panel.boot, panel.kalibrasi_aktif = 'home', None, None, False
        try:
            return panel.gambar()
        except Exception:
            img = Image.new('RGB', (LEBAR, TINGGI), LATAR)
            teks_tengah(kanvas(img), LEBAR // 2, TINGGI // 2, "Error layar -- lihat LOG", F_TEBAL, MERAH)
            return img


def jalankan_panel(kalibrasi=False):
    """Mode utama: panel LCD sentuh. Produksi dimulai/dihentikan dari layar."""
    if not ADA_PIL:
        sys.exit("ERROR: panel butuh Pillow & numpy:  apt install python3-pil python3-numpy fonts-dejavu-core")
    log_info_awal('panel LCD')
    k = muat_config_lcd()
    log(f"[boot] layar ILI9341 SPI {k['lcd_spi_bus']}.{k['lcd_spi_device']} @ {k['lcd_spi_hz'] // 1000000} MHz, "
        f"rotasi {k['lcd_rotasi']}, touch IRQ {k['touch_irq'] if k['touch_irq'] is not None else 'none (tekanan)'}")
    INDIKATOR.sumber_detak = 'panel'
    if INDIKATOR.sw1_ditekan():
        log("[SW1] ditahan saat program mulai -- kalibrasi sentuh")
        kalibrasi = True
    gpio = Gpio()
    layar = LayarILI9341(k, gpio)
    sentuh = SentuhXPT2046(k, gpio, layar.spi)
    INDIKATOR.mulai()           # SETELAH LCD & touch siap -- tidak ada akses GPIO yang tumpang tindih
    log(f"[boot] indikator: LED OPR {LED_OPR}, RUN {LED_RUN}, ALARM {LED_ALARM}, buzzer {BUZZER}, SW1 {SW1} "
        f"(wPi, none = tidak dipakai)")
    kal = Kalibrasi(None) if kalibrasi else Kalibrasi.muat()
    aset = AsetGambar()
    log(f"[boot] layar & touch siap, aset JPEG dimuat: {', '.join(sorted(aset.img)) or 'tidak ada'}")
    try:
        bus = Bus()
        log(f"[boot] RS485 {RS485_PORT} @ {RS485_BAUD} terbuka")
    except Exception as e:
        log(f"!! RS485 {RS485_PORT} tidak bisa dibuka: {e}")
        bus = None
    panel = Panel(layar, sentuh, kal, bus, k.get('lcd_judul') or 'SECURA', aset, k)
    INDIKATOR.on_pendek = lambda: setattr(panel.riwayat, 'belum_dilihat', 0)
    if kalibrasi:
        panel.boot = None
        panel.pemantau.start()
        panel.mulai_kalibrasi()
    else:
        panel.mulai_boot()

    def keluar(*_):
        BERHENTI_PANEL.set()
    signal.signal(signal.SIGTERM, keluar)
    signal.signal(signal.SIGINT, keluar)
    try:
        jalankan(panel, layar, sentuh)
    finally:
        INDIKATOR.berhenti()
        panel.produksi_riwayat.simpan()
        if panel.berjalan():
            log("[panel] program ditutup -- produksi dihentikan")
            berhenti.set()
            panel.produksi.join(timeout=60)
        img = Image.new('RGB', (LEBAR, TINGGI), HITAM)
        teks_tengah(kanvas(img), LEBAR // 2, TINGGI // 2, "Panel berhenti", F_TEBAL, ABU)
        layar.tampilkan(img)
