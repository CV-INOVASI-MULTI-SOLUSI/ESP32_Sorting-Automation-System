#!/usr/bin/env python3
"""
============================================================
 ORANGE PI ORCHESTRATOR -- program produksi (2026-09-30)
 HuskyLens + Sorter + Dispenser + Picker + Stocker dalam SATU program
============================================================

Menggantikan DUA program sekaligus:
  - orangepi-orchestrator-batch.py   (urutan batch package)
  - script HuskyLens produksi         (klasifikasi objek -> palang)

KENAPA DIGABUNG. Keduanya memakai /dev/ttyS3. Dua program di satu bus RS485 saling
merusak pembacaan -- gejalanya mirip kabel rusak. Di sini seluruh akses Modbus lewat SATU
objek `Bus` dengan kunci (lock), dan setiap command 3-langkah (ARG -> SEQ -> CMD) ditulis
utuh di bawah kunci itu, jadi tidak pernah bisa disisipi tulisan lain di tengahnya.

TIGA THREAD:
  kamera     HuskyLens -> keputusan -> CLASSIFY_IS_REJECT. Tidak pernah menunggu batch.
             (Kalau digabung secara berurutan, satu batch yang makan puluhan detik akan
             membuat semua objek lewat tanpa diklasifikasi -- reject lolos semua.)
  indikator  LED & buzzer. Terpisah supaya bunyi tidak menunda kamera.
  keepalive  membaca STATE tiap node berkala, supaya watchdog komunikasi node tidak
             menganggap master mati saat sedang menunggu gerakan panjang.
  (utama)    startup -> pantau PASS_COUNT -> batch -> shutdown.

YANG DIPERBAIKI DARI DUA PROGRAM LAMA
  1. Klasifikasi ditulis SEBELUM LED/buzzer. Script HuskyLens lama membunyikan buzzer dulu
     (~0,9 s) baru menulis REJECT. Firmware mencatat waktu scan saat tulisan TIBA, jadi
     palang mendorong 0,9 s terlambat dari seharusnya.
  2. Satu objek = satu klasifikasi. Script lama langsung mengambil sampel lagi begitu satu
     keputusan selesai; objek yang masih di depan kamera dihitung DUA kali, palang mendorong
     dua kali, dan antrian cepat penuh. Sekarang kamera harus melihat bidang kosong dulu
     (ID kosong / tidak ada objek) selama MIN_KOSONG_S sebelum objek berikutnya diterima.
  3. Startup Dispenser tidak lagi mengirim FORCE_MIDDLE_REFILL. START_MAIN sudah menjalankan
     pipeline yang menjatuhkan package pertama sendiri; FORCE_MIDDLE_REFILL di atasnya
     membuat dua mekanisme menggerakkan servo & conveyor yang sama. Sekarang: START_MAIN,
     lalu tunggu DISPENSER_READY = 1.
  4. REQUEST_REFILL hanya dikirim setelah DISPENSER_READY = 1 (firmware menolaknya di tahap
     lain, dan penolakan tetap di-ack -- dulu batch berikutnya gagal tanpa sebab terlihat).
  5. Batas waktu mengikuti firmware: package ke UJUNG 30 s (firmware 20 s), MOVE_PACKAGE
     180 s (ack Picker baru dikirim setelah 8 langkah lengan selesai).
  6. PASS_COUNT tidak lagi di-reset tiap batch. Objek yang lewat SELAMA batch dulu ikut
     terhapus, sehingga package berikutnya berisi lebih dari BATCH_SIZE. Sekarang dihitung
     dari titik acuan yang maju BATCH_SIZE per batch.
  7. Batch gagal = BERHENTI, bukan diulang. Program lama mengulang batch tiap ~5 detik karena
     PASS_COUNT tetap di atas batas -- berlawanan dengan komentarnya sendiri.
  8. Pemeriksaan awal menolak jalan kalau ada node dengan firmware lama, sedang FAULT,
     E-stop, atau operatornya sedang di menu LCD.

SEBELUM PRODUKSI -- tidak bisa diperiksa dari sini:
  - SEMUA node di-flash dengan firmware >= 2026-09-30. Firmware lebih tua belum menghitung
    request baca sebagai tanda master hidup, jadi Sorter jatuh FAULT COMM_TIMEOUT 30 detik
    setelah START.
  - Pose Picker 1 (PACKAGE_PICKUP) dan 2 (LIFT_LOAD) sudah dikalibrasi, dan POST_PLACE
    OFFSET tidak lagi nol (tanpa itu sendok menyeret package setelah diletakkan).
  - Load Position Stocker sejajar dengan pose 2 Picker.
  - Protocol Type HuskyLens = UART dengan baud = HUSKYLENS_BAUD.

Jalankan:
    python3 orangepi-orchestrator.py --cek          # periksa saja, tidak menggerakkan apa pun
    python3 orangepi-orchestrator.py                # produksi
    python3 orangepi-orchestrator.py --tanpa-kamera # produksi tanpa HuskyLens (semua PASS)

Ctrl+C (atau SIGTERM dari systemd) = shutdown rapi: Sorter berhenti mengumpan dulu, lalu
semua node kembali ke TEST mode.
"""

import argparse
import queue
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

# ============================================================
# KONFIGURASI
# ============================================================
RS485_PORT = '/dev/ttyS3'
RS485_BAUD = 19200

HUSKYLENS_PORT = '/dev/ttyS1'
HUSKYLENS_BAUD = 9600          # samakan dengan Protocol Type di menu HuskyLens (115200 lebih cepat)

BATCH_SIZE = 20                # objek PASS per package
POLL_S = 0.5                   # jeda pemantauan Sorter
KEEPALIVE_S = 5.0              # watchdog node 30 s -- jauh di bawahnya

# HuskyLens -- sama dengan script produksi lama
JUMLAH_SAMPEL = 3
INTERVAL_SAMPEL_S = 0.05
BATAS_AMBIL_S = 0.7
ID_KOSONG = 1
ID_VALID = 3
ID_INVALID = (2, 4, 5)
MIN_KOSONG_S = 0.3             # bidang pandang harus kosong selama ini sebelum objek berikutnya

# Batas waktu (detik)
T_ACK = 5.0
T_PICKER_HOME = 30.0
T_STOCKER_HOMING = 120.0
T_STOCKER_GERAK = 60.0
T_DISPENSER_SIAP_AWAL = 180.0  # urutan awal: servo1, servo2, lalu package jalan ke PROX_2
T_DISPENSER_SIAP = 90.0
T_PACKAGE_UJUNG = 30.0         # firmware sendiri memberi 20 s
T_MOVE_PACKAGE = 180.0
T_FULL_CYCLE = 120.0
# Ack yang datang lebih cepat dari ini mustahil berarti gerakan selesai -- berarti DITOLAK.
MIN_DURASI_MOVE_PACKAGE_S = 3.0
MIN_DURASI_FULL_CYCLE_S = 2.0

# LED & buzzer (nomor wPi, sama dengan script HuskyLens lama)
LED_RED, LED_BLUE, BUZZER = 5, 13, 10

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
R_SORTER_CLASSIFY = 12
R_PICKER_POSE, R_PICKER_ACTIVITY = 10, 11
R_DISP_PACKAGE_READY, R_DISP_READY, R_DISP_STAGE = 15, 25, 26
R_STOCKER_HOMED, R_STOCKER_RAK = 11, 12

OP_START_MAIN = {SORTER: 1, PICKER: 11, DISPENSER: 14, STOCKER: 9}   # Sorter: START
OP_STOP_MAIN = {SORTER: 2, PICKER: 12, DISPENSER: 15, STOCKER: 10}   # Sorter: STOP
OP_SORTER_RESET_COUNTERS = 7
OP_PICKER_GOTO_HOME, OP_PICKER_MOVE_PACKAGE = 2, 8
OP_DISP_REQUEST_REFILL, OP_DISP_ACK_TAKEN = 1, 5
OP_STOCKER_HOME_ALL, OP_STOCKER_FULL_CYCLE, OP_STOCKER_GOTO_LOAD = 1, 2, 6

PIPELINE_NAMES = [
    'MATI', 'INIT servo1 buka', 'INIT servo1 tahan', 'INIT servo1 tutup',
    'INIT servo2 buka', 'INIT servo2 tahan', 'INIT servo2 tutup', 'INIT tunggu PROX_2',
    'buka gerbang', 'SIAP ISI (ready)', 'maju ke ujung', 'refill servo1 tutup',
    'refill servo2 buka', 'refill servo2 tahan', 'refill servo2 tutup', 'tunggu diambil arm',
]

berhenti = threading.Event()      # Ctrl+C / SIGTERM
produksi = threading.Event()      # set = kamera boleh menulis klasifikasi


def log(teks):
    print(time.strftime('%H:%M:%S ') + teks, flush=True)


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

    try:
        kosong = [r for r in range(1, 5) if not (bus.baca(STOCKER, R_STOCKER_RAK) & (1 << r))]
        if kosong:
            log(f"  OK rak kosong: {', '.join(map(str, kosong))}")
        else:
            log("  !! SEMUA RAK PENUH")
            ok = False
    except (Gagal, RegisterTidakAda):
        pass   # sudah dilaporkan di atas

    if pakai_kamera:
        try:
            with serial.Serial(HUSKYLENS_PORT, HUSKYLENS_BAUD, timeout=0.1) as hl:
                hl.reset_input_buffer()
                hl.write(REQUEST_BLOCKS)
                data, batas = b'', time.monotonic() + 0.8
                while time.monotonic() < batas:
                    data += hl.read(64)
            if b'\x55\xAA\x11' in data:
                log(f"  OK HuskyLens menjawab di {HUSKYLENS_PORT}")
            else:
                log(f"  !! HuskyLens diam di {HUSKYLENS_PORT} ({len(data)} byte) -- cek kabel & Protocol Type")
                ok = False
        except Exception as e:
            log(f"  !! {HUSKYLENS_PORT} tidak bisa dibuka: {e}")
            ok = False
    else:
        log("  -- kamera DIMATIKAN (--tanpa-kamera): semua objek dianggap PASS, palang tidak bekerja")

    log("  -- tidak bisa diperiksa dari sini: firmware >= 2026-09-30 di SEMUA node, kalibrasi")
    log("     pose Picker 1/2 & POST_PLACE, Load Position Stocker. Lihat docstring.")
    return ok


# ============================================================
# HUSKYLENS
# ============================================================
REQUEST_BLOCKS = bytes([0x55, 0xAA, 0x11, 0x00, 0x20, 0x30])


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
        for pin in (LED_RED, LED_BLUE, BUZZER):
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
            led, bunyi = POLA[jenis]
            wiringpi.digitalWrite(led, 1)
            for on_s, off_s in bunyi:
                wiringpi.digitalWrite(BUZZER, 1)
                time.sleep(on_s)
                wiringpi.digitalWrite(BUZZER, 0)
                time.sleep(off_s)
            time.sleep(0.2)
            wiringpi.digitalWrite(led, 0)
    finally:
        for pin in (LED_RED, LED_BLUE, BUZZER):
            wiringpi.digitalWrite(pin, 0)


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

    log("[1] Picker -> HOME (masih TEST mode; GOTO_HOME ditolak setelah MAIN)")
    kirim(bus, PICKER, OP_PICKER_GOTO_HOME, "GOTO_HOME")
    # GOTO_HOME di-ack SEKETIKA, gerakannya baru mulai -- tunggu sampai benar-benar diam di pose 0.
    # Jeda sebentar dulu: tepat setelah ack, antrian gerak belum tentu diambil, sehingga
    # ACTIVITY masih DIAM dan CURRENT_POSE masih nilai lama -- terlihat "sudah sampai".
    time.sleep(0.5)
    tunggu(bus, PICKER,
           lambda: bus.baca(PICKER, R_PICKER_ACTIVITY) == 0 and bus.baca(PICKER, R_PICKER_POSE) == 0,
           T_PICKER_HOME, "Picker diam di HOME")

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

    log("[4] Stocker -> homing (kalau perlu), Load Position, START_MAIN")
    if bus.baca(STOCKER, R_STOCKER_HOMED) != 1:
        kirim(bus, STOCKER, OP_STOCKER_HOME_ALL, "HOME_ALL", timeout=T_STOCKER_HOMING)
        if bus.baca(STOCKER, R_STOCKER_HOMED) != 1:
            raise Gagal("STOCKER: HOME_ALL di-ack tapi ALL_HOMED_FLAG masih 0")
    kirim(bus, STOCKER, OP_STOCKER_GOTO_LOAD, "GOTO_LOAD_POSITION", timeout=T_STOCKER_GERAK)
    kirim(bus, STOCKER, OP_START_MAIN[STOCKER], "START_MAIN")
    pastikan_main(bus, STOCKER, True)

    log("[5] Sorter -> RESET_COUNTERS, START")
    kirim(bus, SORTER, OP_SORTER_RESET_COUNTERS, "RESET_COUNTERS")
    kirim(bus, SORTER, OP_START_MAIN[SORTER], "START")
    pastikan_main(bus, SORTER, True)
    log("== STARTUP SELESAI -- produksi berjalan ==")


def cari_rak_kosong(bus):
    bitmask = bus.baca(STOCKER, R_STOCKER_RAK)
    for rak in range(1, 5):          # bit N = Rak N (firmware >= 2026-09-20)
        if not bitmask & (1 << rak):
            return rak
    return None


def batch(bus, nomor):
    log(f"== BATCH {nomor} ==")
    for sid in NAMA:
        state, fault = bus.baca(sid, R_STATE, 2)
        if fault or state in (ST_FAULT, ST_ESTOP):
            raise Gagal(f"{NAMA[sid]} FAULT (code={fault}) sebelum batch dimulai")

    rak = cari_rak_kosong(bus)
    if rak is None:
        raise Gagal("SEMUA RAK PENUH -- kosongkan rak dulu")

    log("[1] tunggu Dispenser SIAP ISI")
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_READY) == 1,
           T_DISPENSER_SIAP, "DISPENSER_READY")

    log("[2] Dispenser -> REQUEST_REFILL, tunggu package sampai UJUNG")
    kirim(bus, DISPENSER, OP_DISP_REQUEST_REFILL, "REQUEST_REFILL")
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_PACKAGE_READY) == 1,
           T_PACKAGE_UJUNG, "PACKAGE_READY_FLAG")

    log("[3] Stocker -> Load Position")
    kirim(bus, STOCKER, OP_STOCKER_GOTO_LOAD, "GOTO_LOAD_POSITION", timeout=T_STOCKER_GERAK)

    log("[4] Picker -> MOVE_PACKAGE (ack baru datang setelah siklus lengan selesai)")
    durasi = kirim(bus, PICKER, OP_PICKER_MOVE_PACKAGE, "MOVE_PACKAGE", timeout=T_MOVE_PACKAGE)
    if durasi < MIN_DURASI_MOVE_PACKAGE_S:
        raise Gagal(f"PICKER: MOVE_PACKAGE di-ack dalam {durasi:.1f} s -- mustahil selesai, berarti DITOLAK")
    log(f"    siklus lengan {durasi:.1f} s")

    log("[5] Dispenser -> ACK_PACKAGE_TAKEN")
    kirim(bus, DISPENSER, OP_DISP_ACK_TAKEN, "ACK_PACKAGE_TAKEN")
    # Firmware membersihkan flag pada putaran pipeline BERIKUTNYA, bukan saat command
    # diterima -- dibaca seketika bisa masih 1. Karena itu ditunggu, bukan dibaca sekali.
    tunggu(bus, DISPENSER, lambda: bus.baca(DISPENSER, R_DISP_PACKAGE_READY) == 0,
           5.0, "PACKAGE_READY_FLAG kembali 0")

    log(f"[6] Stocker -> simpan ke Rak {rak}")
    durasi = kirim(bus, STOCKER, OP_STOCKER_FULL_CYCLE, "RUN_FULL_CYCLE", arg=rak, timeout=T_FULL_CYCLE)
    if durasi < MIN_DURASI_FULL_CYCLE_S:
        raise Gagal(f"STOCKER: RUN_FULL_CYCLE di-ack dalam {durasi:.1f} s -- berarti DITOLAK")
    log(f"== BATCH {nomor} SELESAI -> Rak {rak} ==")


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
    while not berhenti.is_set():
        state, fault = bus.baca(SORTER, R_STATE, 2)
        if fault or state != ST_RUN:
            raise Gagal(f"SORTER berhenti (state={state}, fault={fault})")
        blok = bus.baca(SORTER, R_SORTER_BLOK, 10)
        passc, rejectc, missed = blok[0], blok[1], blok[9]
        if missed != missed_awal:
            log(f"!! REJECT_MISSED_COUNT naik ke {missed} -- ada reject yang lolos tanpa didorong")
            missed_awal = missed
        if (passc - acuan) % 65536 >= BATCH_SIZE:
            nomor += 1
            batch(bus, nomor)
            acuan = (acuan + BATCH_SIZE) % 65536
        if time.monotonic() >= cetak_berikut:
            cetak_berikut = time.monotonic() + 60
            log(f"[status] pass={passc} reject={rejectc} missed={missed} batch={nomor} "
                f"kamera valid={statistik['valid']} invalid={statistik['invalid']} "
                f"gagal_tulis={statistik['gagal_tulis']}")
        time.sleep(POLL_S)


def tahan(bus, alasan):
    """Batch gagal: hentikan umpan Sorter (package tidak kebanjiran), lalu TUNGGU operator.
    Tidak ada pengulangan otomatis -- bisa tabrakan mekanis."""
    produksi.clear()
    log(f"!!!! PRODUKSI DITAHAN: {alasan}")
    try:
        bus.command(SORTER, OP_STOP_MAIN[SORTER])
        log("     Sorter dihentikan. Node lain dibiarkan apa adanya -- periksa panelnya.")
    except Exception as e:
        log(f"     gagal menghentikan Sorter: {e}")
    log("     Ctrl+C untuk shutdown penuh setelah diperiksa.")
    berhenti.wait()


# ============================================================
def main():
    ap = argparse.ArgumentParser(description="Orkestrator produksi sorting automation")
    ap.add_argument('--cek', action='store_true', help='pemeriksaan awal saja, tidak menggerakkan apa pun')
    ap.add_argument('--tanpa-kamera', action='store_true', help='jalan tanpa HuskyLens (semua PASS)')
    args = ap.parse_args()
    pakai_kamera = not args.tanpa_kamera

    signal.signal(signal.SIGTERM, lambda *_: berhenti.set())
    signal.signal(signal.SIGINT, lambda *_: berhenti.set())

    bus = Bus()
    if not periksa(bus, pakai_kamera):
        log("!! Pemeriksaan awal GAGAL -- tidak ada yang digerakkan.")
        sys.exit(1)
    if args.cek:
        log("Pemeriksaan awal OK. (--cek: berhenti di sini)")
        return

    antrian = queue.Queue(maxsize=20)
    threads = [threading.Thread(target=thread_keepalive, args=(bus,), daemon=True),
               threading.Thread(target=thread_indikator, args=(antrian,), daemon=True)]
    if pakai_kamera:
        threads.append(threading.Thread(target=thread_kamera, args=(bus, antrian), daemon=True))
    for t in threads:
        t.start()

    sudah_jalan = False
    try:
        startup(bus)
        sudah_jalan = True
        produksi.set()
        pantau(bus)
    except Gagal as e:
        if berhenti.is_set():
            log(f"dihentikan: {e}")
        elif sudah_jalan:
            tahan(bus, str(e))
        else:
            log(f"!! STARTUP GAGAL: {e}")
    finally:
        produksi.clear()
        berhenti.set()
        for t in threads:
            t.join(timeout=2)
        shutdown(bus)


if __name__ == '__main__':
    main()
