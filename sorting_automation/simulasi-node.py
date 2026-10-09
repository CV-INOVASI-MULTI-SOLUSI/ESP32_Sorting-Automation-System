#!/usr/bin/env python3
"""
============================================================
 SIMULASI 4 NODE -- Modbus RTU slave, dengan TIMING firmware (2026-10-07)
============================================================

Script ini MENJAWAB sebagai keempat ESP32 (slave 1 SORTER, 2 PICKER, 3 DISPENSER,
4 STOCKER), seakan-akan semua node tersambung. Firmware asli TIDAK diubah atau dipakai --
perilakunya ditiru dari src/main.cpp & include/registers.h tiap node.

Semua durasi dihitung dari nilai default firmware (lihat simulasi-waktu.config):
servo/hopper dari step_us & step_interval_ms, lengan Picker dari trajStepUs/IntervalMs,
stepper Stocker dari jumlah langkah x stepIntervalUs, objek di conveyor dari jarak mm dan
kecepatan conveyor (PWM). Command SET_* dari master ikut mengubah durasi tersebut.

SAMBUNGAN (mode adaptor)
  Laptop/PC --USB-- adaptor USB-RS485 --A/B-- modul RS485 Orange Pi (/dev/ttyS3)
  A ke A, B ke B (kalau tidak ada jawaban sama sekali, coba tukar A/B).

JALANKAN
  python simulasi-node.py --port COM5                  (Windows)
  python3 simulasi-node.py --port /dev/ttyUSB0         (Linux)
  Tanpa adaptor sama sekali: sorting-automation.py --simulasi (memakai file ini di dalam program)

  --waktu FILE        file timing (bawaan: simulasi-waktu.config di folder ini)
  --cepat 2           semua durasi 2x lebih cepat (menimpa [umum] cepat)
  --objek 1.5         paksa hopper menjatuhkan 1 objek tiap 1,5 detik
  --baud 19200        harus sama dengan rs485_baud di sorting.config
  --sensor-rak        Stocker menandai rak terisi di RACK_OCCUPIED_BITMASK (seolah ada sensor)
  --mati 4            node 4 (STOCKER) tidak menjawab sejak awal
  --tanpa-watchdog    matikan FAULT COMM_TIMEOUT
  --echo              adaptor memantulkan data yang dikirimnya sendiri (jarang)

PERINTAH SAAT BERJALAN (ketik lalu Enter)
  s              tabel status keempat node
  w              ringkasan durasi yang sedang dipakai
  f <id> <kode>  jadikan node FAULT, mis. "f 3 11" = Dispenser BOX_NOT_ARRIVED
  r <id>         RESET_FAULT dari panel node
  e <id>         tekan / lepas E-STOP
  m <id>         operator masuk / keluar menu LCD (command diabaikan TANPA ack)
  x <id>         node mati / hidup lagi (tidak menjawab Modbus)
  n <id>         node RESTART (register kembali awal, MAIN mati, 3 detik tidak menjawab)
  b              satu objek langsung muncul di depan kamera
  q              keluar

YANG DITIRU
  - peta register sama persis; alamat yang tidak ada dijawab ILLEGAL DATA ADDRESS (02)
  - command 3 langkah ARG -> SEQ -> CMD, ack = CMD_ACK_SEQ; seq yang sama diabaikan;
    command yang ditolak (salah mode / sibuk) tetap di-ack tanpa melakukan apa pun
  - ack TERTUNDA sampai selesai: Picker MOVE_PACKAGE; Stocker HOME_ALL, RUN_FULL_CYCLE,
    GOTO_LOAD (READY), MOVE_TO_RACK, PUSH_BOX
  - menu LCD aktif: command diabaikan dan TIDAK di-ack; watchdog COMM_TIMEOUT
  - SORTER: siklus hopper (dorong-tahan-tarik-jeda), objek berjalan sesuai kecepatan PWM,
    CLASSIFY_IS_REJECT -> palang mendorong setelah TOF (jarak scan-palang / kecepatan),
    REJECT_COUNT naik saat palang mulai dorong, objek yang lewat palang terdorong keluar,
    sisanya terhitung PASS di PROX_2 + uji kecepatan PROX_1->PROX_2; SET_MOTOR_A (palang
    manual), TEST_TRIGGER_PALANG, TEST_HOPPER_CYCLE, SET_HOPPER_JEDA
  - DISPENSER: pipeline START_MAIN -> SIAP ISI -> REQUEST_REFILL -> ke UJUNG -> servo ->
    tunggu diambil -> ACK_PACKAGE_TAKEN; stok package habis -> MIDDLE_PACKAGE_MISSING;
    TEST: conveyor ON/OFF (berhenti sendiri di UJUNG), MOVE_SERVO, SERVO CYCLE
  - PICKER: antrian gerakan seperti firmware (GOTO_POSE, gerakan = adegan lalu pose tujuan),
    CURRENT_POSE ditulis saat gerakan SELESAI, GERAKAN_AKTIF/ADEGAN_KE selama gerakan
  - STOCKER: homing 3 axis, gerak ke rak, dorong/tarik pusher, kembali READY
"""

import argparse
import configparser
import os
import random
import sys
import threading
import time

try:
    import serial
except ImportError:
    serial = None   # mode --simulasi di dalam program tidak butuh pyserial

SORTER, PICKER, DISPENSER, STOCKER = 1, 2, 3, 4
NAMA = {SORTER: 'SORTER', PICKER: 'PICKER', DISPENSER: 'DISPENSER', STOCKER: 'STOCKER'}
IDLE, RUN, FAULT, ESTOP = 1, 2, 3, 4
FOLDER = os.path.dirname(os.path.abspath(__file__))

# Register yang ADA di firmware tiap node (selain 0-6 universal). Nilai awal.
REGISTER = {
    SORTER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0, 19: 0,
             20: 0, 21: 0, 22: 0, 23: 0, 24: 0},
    PICKER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0xFF, 18: 0},
    DISPENSER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0, 19: 0, 20: 0,
                21: 0, 22: 0, 23: 0, 24: 0, 25: 0, 26: 0},
    STOCKER: {10: 0xFF, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0},
}
# Waktu "build" firmware tiruan (FW_TAHUN, FW_BULAN_HARI, FW_JAM_MENIT) = saat simulator dimulai.
_t = time.localtime()
for _sid, _a in ((SORTER, 25), (PICKER, 19), (DISPENSER, 27), (STOCKER, 19)):
    REGISTER[_sid].update({_a: _t.tm_year, _a + 1: _t.tm_mon * 100 + _t.tm_mday, _a + 2: _t.tm_hour * 100 + _t.tm_min,
                           _a + 3: 201, _a + 4: (127 << 8) | 0, _a + 5: (0 << 8) | 1, _a + 6: (-55) & 0xFFFF})
# BARU (2026-10-09): backup / restore kalibrasi (firmware v2.01, include/kalibrasi_modbus.h).
# FORMAT, UKURAN, CRC, OFFSET, HASIL, lalu 32 register DATA. Ukuran gambar tiruan ~ ukuran asli.
R_CAL = {SORTER: 32, PICKER: 26, DISPENSER: 34, STOCKER: 26}
UKURAN_CAL = {SORTER: 36, PICKER: 508, DISPENSER: 64, STOCKER: 73}
for _sid, _a in R_CAL.items():
    REGISTER[_sid].update({_a: 1, _a + 1: UKURAN_CAL[_sid], _a + 2: 0, _a + 3: 0, _a + 4: 0})
    REGISTER[_sid].update({_a + 5 + i: 0 for i in range(32)})
# "NVS" tiruan -- bertahan walau node tiruan restart (selama simulator hidup)
NVS_TIRUAN = {sid: bytes((sid * 37 + i * 11) & 0xFF for i in range(UKURAN_CAL[sid])) for sid in R_CAL}
R_MAIN = {SORTER: 17, PICKER: 15, DISPENSER: 18, STOCKER: 17}
R_MENU = {SORTER: 18, PICKER: 16, DISPENSER: 20, STOCKER: 18}
R_UPTIME = {SORTER: 16, PICKER: 14, DISPENSER: 14, STOCKER: 16}
R_LASTFAULT = {SORTER: 15, PICKER: 13, DISPENSER: 13, STOCKER: 15}
R_AKTIVITAS = {SORTER: 13, PICKER: 11, DISPENSER: 11, STOCKER: 13}
BISA_DITULIS = {2, 3, 4, 6}                       # CMD, ARG, SEQ, HEARTBEAT
BISA_DITULIS_KHUSUS = {SORTER: {12}}              # CLASSIFY_IS_REJECT
for _sid, _a in R_CAL.items():                     # jendela data kalibrasi
    BISA_DITULIS_KHUSUS.setdefault(_sid, set()).update(range(_a + 5, _a + 37))

NAMA_CMD = {
    SORTER: {1: 'START', 2: 'STOP', 3: 'RESET_FAULT', 4: 'SET_HOPPER_INTERVAL', 5: 'SET_CONVEYOR_SPEED',
             6: 'SET_CONVEYOR_DIR', 7: 'RESET_COUNTERS', 8: 'SET_MOTOR_A', 9: 'SET_PALANG_SPEED',
             10: 'SET_HOPPER_STEP', 11: 'SET_HOPPER_JEDA', 97: 'TEST_HOPPER_CYCLE', 98: 'TEST_TRIGGER_PALANG'},
    PICKER: {2: 'GOTO_HOME', 5: 'PICK', 6: 'PLACE', 7: 'RESET_FAULT', 8: 'MOVE_PACKAGE', 9: 'SET_TRAJ_STEP',
             10: 'SET_TRAJ_STEP_INTERVAL', 11: 'START_MAIN', 12: 'STOP_MAIN', 13: 'GOTO_POSE_N',
             14: 'RUN_GERAKAN', 15: 'GOTO_READY'},
    DISPENSER: {1: 'REQUEST_REFILL', 2: 'RESET_FAULT', 3: 'SET_CONVEYOR_SPEED', 4: 'SET_CONVEYOR_DIR',
                5: 'ACK_PACKAGE_TAKEN', 6: 'FORCE_MIDDLE_REFILL', 7: 'SET_SERVO1_STEP',
                8: 'SET_SERVO1_STEP_INTERVAL', 9: 'SET_SERVO2_STEP', 10: 'SET_SERVO2_STEP_INTERVAL',
                11: 'SET_CONVEYOR_ON_OFF', 12: 'MOVE_SERVO1_TO', 13: 'MOVE_SERVO2_TO', 14: 'START_MAIN',
                15: 'STOP_MAIN', 97: 'TEST_SERVO1_CYCLE', 98: 'TEST_SERVO2_CYCLE'},
    STOCKER: {1: 'HOME_ALL', 2: 'RUN_FULL_CYCLE', 3: 'MOVE_TO_RACK', 4: 'PUSH_BOX', 5: 'RESET_FAULT',
              6: 'GOTO_READY(LOAD)', 7: 'SET_STEP_INTERVAL', 8: 'SET_HOMING_STEP_INTERVAL', 9: 'START_MAIN',
              10: 'STOP_MAIN'},
}
for _n in NAMA_CMD.values():
    _n.update({60: 'CAL_BACA', 61: 'CAL_TULIS', 62: 'CAL_TERAPKAN'})
NAMA_POSE = {0: 'HOME', 1: 'PICKUP', 2: 'LIFT', 3: 'READY'}
NAMA_TAHAP = ['MATI', 'INIT servo1 buka', 'INIT servo1 tahan', 'INIT servo1 tutup', 'INIT servo2 buka',
              'INIT servo2 tahan', 'INIT servo2 tutup', 'INIT tunggu PROX_2', 'buka gerbang', 'SIAP ISI',
              'maju ke ujung', 'refill servo1 tutup', 'refill servo2 buka', 'refill servo2 tahan',
              'refill servo2 tutup', 'tunggu diambil arm']
# Picker: gerakan i berangkat dari pose ASAL, berakhir di pose TUJUAN (GERAKAN_ASAL/_TUJUAN firmware)
GERAKAN_ASAL = [0, 3, 1, 0, 2]
GERAKAN_TUJUAN = [3, 1, 0, 2, 0]
AKT_POSE = {0: 1, 1: 10, 2: 11, 3: 13}        # Picker ACTIVITY saat menuju pose

AWALAN = ''   # '[SIM] ' saat dijalankan di dalam sorting-automation.py --simulasi


def log(teks):
    print(time.strftime('%H:%M:%S ') + AWALAN + teks, flush=True)


def crc_ccitt(data):
    """CRC16-CCITT init 0xFFFF -- sama dengan calCrc16() firmware."""
    import binascii
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


# ============================================================
# WAKTU -- simulasi-waktu.config (semua opsional; bawaan = default firmware)
# ============================================================
BAWAAN = {
    'umum': {'cepat': 1.0, 'watchdog_s': 30},
    'sorter': {'conveyor_pwm': 180, 'mm_per_s_pwm_max': 300, 'jarak_scan_palang_mm': 150,
               'jarak_hopper_scan_mm': 250, 'jarak_palang_pass_mm': 200, 'jarak_uji_kecepatan_mm': 200,
               'palang_push_ms': 300, 'palang_retract_ms': 300, 'hopper_start_us': 1000,
               'hopper_push_us': 2000, 'hopper_step_us': 20, 'hopper_step_interval_ms': 20,
               'hopper_hold_ms': 300, 'hopper_jeda_siklus_ms': 1000, 'peluang_objek': 1.0},
    'picker': {'traj_step_us': 20, 'traj_step_interval_ms': 20, 'jarak_pose_us': 800,
               'adegan_per_gerakan': 3, 'jarak_adegan_us': 300, 'pick_offset_us': 600, 'place_offset_us': 600},
    'dispenser': {'servo_start_us': 1000, 'servo_end_us': 2000, 'servo_step_us': 20,
                  'servo_step_interval_ms': 20, 'servo_hold_ms': 500, 'conveyor_ke_tengah_s': 2.0,
                  'conveyor_ke_ujung_s': 3.0, 'batas_box_tiba_s': 8.0, 'stok_package': 50},
    'stocker': {'step_interval_us': 600, 'homing_step_interval_us': 150, 'langkah_homing_per_axis': 6000,
                'langkah_antar_rak': 3000, 'langkah_ready_ke_rak1': 2000, 'push_extend_steps': 1000},
}


def muat_waktu(path=None):
    w = {b: dict(isi) for b, isi in BAWAAN.items()}
    path = path or os.path.join(FOLDER, 'simulasi-waktu.config')
    if os.path.exists(path):
        cp = configparser.ConfigParser(inline_comment_prefixes=('#',))
        cp.read(path, encoding='utf-8-sig')
        for bagian, isi in w.items():
            for kunci in isi:
                if cp.has_option(bagian, kunci):
                    try:
                        isi[kunci] = float(cp.get(bagian, kunci))
                    except ValueError:
                        sys.exit(f"ERROR di {path}: [{bagian}] {kunci} bukan angka")
    return w


# ============================================================
# NODE -- dasar bersama: register, fault, antrian langkah berwaktu
# ============================================================
class Node:
    def __init__(self, sid, dunia):
        self.sid, self.dunia = sid, dunia
        self.w = dict(dunia.waktu[NAMA[sid].lower()])     # disalin: SET_* mengubah salinan node ini
        self.reg = {0: IDLE, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        self.reg.update(REGISTER[sid])
        self.mati = False
        self.terakhir_rx = time.monotonic()
        self.ack_tertunda = None
        self.langkah = []             # [(detik, fungsi_saat_selesai)]
        self.langkah_sejak = None
        self.mulai = dunia.mulai
        self.boot_sampai = 0
        self.awal()

    def awal(self):
        pass

    @property
    def main(self):
        return self.reg[R_MAIN[self.sid]] == 1

    def set_main(self, nyala):
        self.reg[R_MAIN[self.sid]] = 1 if nyala else 0

    def boleh_gerak(self):
        return self.reg[0] not in (FAULT, ESTOP) and self.reg[1] == 0

    def sibuk(self):
        return bool(self.langkah)

    def akt(self, kode):
        self.reg[R_AKTIVITAS[self.sid]] = kode

    def fault(self, kode, alasan=''):
        self.reg[1] = kode
        self.reg[R_LASTFAULT[self.sid]] = kode
        self.reg[0] = FAULT
        self.langkah, self.ack_tertunda = [], None
        self.akt(90)
        self.saat_fault()
        log(f"   !! {NAMA[self.sid]} FAULT {kode} {alasan}")

    def saat_fault(self):
        pass

    def reset_fault(self):
        if self.reg[0] == ESTOP:
            return
        self.reg[1] = 0
        self.reg[0] = IDLE
        self.akt(0)

    def antre(self, rencana, ack_seq=None, tambah=False):
        """rencana: [(detik, fungsi_saat_selesai)]. Durasi dibagi faktor cepat.
        tambah=True: disambung di belakang antrian yang sedang jalan (antrian Picker)."""
        c = self.dunia.cepat
        baru = [(d / c, f) for d, f in rencana]
        if tambah and self.langkah:
            self.langkah += baru
        else:
            self.langkah = baru
            self.langkah_sejak = time.monotonic()
        if ack_seq is not None:
            self.ack_tertunda = ack_seq
        if self.langkah and self.reg[0] == IDLE:
            self.reg[0] = RUN

    def restart(self):
        self.reg = {0: IDLE, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        self.reg.update(REGISTER[self.sid])
        self.langkah, self.ack_tertunda = [], None
        self.w = dict(self.dunia.waktu[NAMA[self.sid].lower()])
        self.mulai = time.monotonic()
        self.boot_sampai = self.mulai + 3
        self.awal()
        log(f"   !! {NAMA[self.sid]} RESTART (boot 3 detik)")

    def tick(self, sekarang):
        if getattr(self, 'restart_pada', None) and sekarang >= self.restart_pada:
            self.restart_pada = None
            self.restart()
        self.reg[R_UPTIME[self.sid]] = int(sekarang - self.mulai) & 0xFFFF
        if (self.dunia.watchdog and self.reg[0] == RUN and self.main and self.reg[1] == 0
                and sekarang - self.terakhir_rx > self.dunia.waktu['umum']['watchdog_s']):
            self.fault(1, 'COMM_TIMEOUT')
        while self.langkah and self.boleh_gerak():
            durasi, fungsi = self.langkah[0]
            if sekarang - self.langkah_sejak < durasi:
                break
            self.langkah.pop(0)
            self.langkah_sejak += durasi
            fungsi()
            if not self.langkah:
                self.langkah_sejak = None
                if self.ack_tertunda is not None:
                    self.reg[5] = self.ack_tertunda
                    log(f"   {NAMA[self.sid]} selesai -> ack seq {self.ack_tertunda}")
                    self.ack_tertunda = None
                self.selesai_semua()
        self.tick_khusus(sekarang)

    def selesai_semua(self):
        if self.reg[0] == RUN:
            self.reg[0] = IDLE
        self.akt(0)

    def tick_khusus(self, sekarang):
        pass

    def command(self):
        op, arg, seq = self.reg[2], self.reg[3], self.reg[4]
        if self.reg[R_MENU[self.sid]]:
            log(f"   {NAMA[self.sid]}: operator di menu LCD -- command DIABAIKAN tanpa ack")
            return
        if seq == self.reg[5]:
            return
        nama = NAMA_CMD[self.sid].get(op, f'opcode {op}')
        if op in (60, 61, 62):
            self.kalibrasi(op, arg)
            log(f"<- {NAMA[self.sid]:9s} {nama} arg={arg}  hasil {self.reg[R_CAL[self.sid] + 4]}")
            self.reg[5] = seq
            return
        tunda = self.jalankan(op, arg, seq)
        log(f"<- {NAMA[self.sid]:9s} {nama}" + (f" arg={arg}" if arg else '') +
            ("  (ack setelah selesai)" if tunda else ''))
        if not tunda:
            self.reg[5] = seq

    def jalankan(self, op, arg, seq):
        return False

    def kalibrasi(self, op, arg):
        """Tiruan KalibrasiModbus (firmware v2.01) -- kode hasil sama dengan CalHasil."""
        a, uk = R_CAL[self.sid], UKURAN_CAL[self.sid]
        hasil = lambda h: self.reg.__setitem__(a + 4, h)
        if self.main or self.sibuk():
            return hasil(10)
        if op == 60:                                   # CAL_BACA
            if arg == 0:
                self.cal_buf = NVS_TIRUAN[self.sid]
                self.reg[a + 2] = crc_ccitt(self.cal_buf)
            elif getattr(self, 'cal_buf', None) is None:
                return hasil(11)
            if arg >= uk:
                return hasil(11)
            p = self.cal_buf[arg:arg + 64].ljust(64, b'\0')
            for i in range(32):
                self.reg[a + 5 + i] = p[2 * i] << 8 | p[2 * i + 1]
            self.reg[a + 3] = arg
            return hasil(1)
        if op == 61:                                   # CAL_TULIS
            if arg == 0:
                self.cal_tulis, self.cal_terisi = bytearray(uk), 0
            elif getattr(self, 'cal_tulis', None) is None:
                return hasil(11)
            if arg % 64 or arg > self.cal_terisi or arg >= uk:
                return hasil(11)
            n = min(64, uk - arg)
            for i in range(n):
                w = self.reg[a + 5 + i // 2]
                self.cal_tulis[arg + i] = (w >> 8) if i % 2 == 0 else (w & 0xFF)
            self.cal_terisi = max(self.cal_terisi, arg + n)
            self.reg[a + 3] = arg
            return hasil(2)
        if getattr(self, 'cal_tulis', None) is None:   # CAL_TERAPKAN
            return hasil(11)
        if self.cal_terisi < uk:
            return hasil(12)
        if crc_ccitt(self.cal_tulis) != arg:
            return hasil(13)
        NVS_TIRUAN[self.sid] = bytes(self.cal_tulis)
        self.cal_tulis = None
        self.restart_pada = time.monotonic() + 0.5
        log(f"   {NAMA[self.sid]}: kalibrasi disimpan ke NVS tiruan, restart 0,5 s")
        return hasil(3)

    def tolak(self, alasan):
        log(f"   {NAMA[self.sid]}: DITOLAK -- {alasan} (tetap di-ack)")
        return False


# ============================================================
# SORTER -- conveyor, hopper, palang, objek
# ============================================================
class Sorter(Node):
    def awal(self):
        self.objek = []             # [{'mm', 'prox1_t'}]
        self.hopper_fase, self.hopper_sejak = None, 0.0   # None | 'dorong' | 'tahan' | 'tarik' | 'jeda'
        self.hopper_uji = False
        self.palang_antri = []      # waktu dorong terjadwal
        self.palang_fase, self.palang_sejak = None, 0.0
        self.motor_a = 0
        self.reg[23] = int(self.w['mm_per_s_pwm_max'])

    # --- besaran turunan (ikut berubah oleh SET_*) ---
    def v_mm_s(self):
        return self.w['mm_per_s_pwm_max'] * self.w['conveyor_pwm'] / 255

    def t_hopper_gerak(self):
        selisih = abs(self.w['hopper_push_us'] - self.w['hopper_start_us'])
        return selisih / max(1, self.w['hopper_step_us']) * self.w['hopper_step_interval_ms'] / 1000

    def tof(self):
        return self.w['jarak_scan_palang_mm'] / max(1.0, self.v_mm_s())

    def posisi(self):
        scan = self.w['jarak_hopper_scan_mm']
        palang = scan + self.w['jarak_scan_palang_mm']
        prox2 = palang + self.w['jarak_palang_pass_mm']
        return scan, palang, prox2 - self.w['jarak_uji_kecepatan_mm'], prox2

    def conveyor_jalan(self):
        return self.reg[0] == RUN and self.main and self.reg[1] == 0

    def saat_fault(self):
        self.hopper_fase, self.motor_a, self.palang_fase = None, 0, None

    def jalankan(self, op, arg, seq):
        r = self.reg
        if op == 1:                                            # START
            if not self.boleh_gerak():
                return self.tolak('masih FAULT/ESTOPPED')
            r[0] = RUN
            self.set_main(True)
            r[24] = 0
            if self.hopper_fase is None:
                self.hopper_fase, self.hopper_sejak = 'dorong', time.monotonic()
        elif op == 2:                                          # STOP
            r[0], r[24] = IDLE, 0
            self.set_main(False)
            self.hopper_fase = None
        elif op == 3:
            self.reset_fault()
        elif op == 4:
            self.w['hopper_step_interval_ms'] = max(0, min(500, arg))
        elif op == 5:
            self.w['conveyor_pwm'] = max(0, min(255, arg))
        elif op == 7:
            r[10] = r[11] = r[19] = 0
        elif op == 8:                                          # SET_MOTOR_A
            if self.main:
                return self.tolak('MAIN aktif')
            if self.palang_fase or self.palang_antri:
                return self.tolak('palang (reject) sedang memakai Motor A')
            self.motor_a = max(0, min(2, arg))
        elif op == 10:
            self.w['hopper_step_us'] = max(1, min(2500, arg))
        elif op == 11:                                         # SET_HOPPER_JEDA
            r[24] = 1 if arg else 0
        elif op == 97:                                         # TEST_HOPPER_CYCLE
            if self.main:
                return self.tolak('MAIN aktif')
            if self.hopper_fase is not None:
                return self.tolak('hopper sedang bergerak')
            self.hopper_uji = True
            self.hopper_fase, self.hopper_sejak = 'dorong', time.monotonic()
        elif op == 98:                                         # TEST_TRIGGER_PALANG
            if self.main:
                return self.tolak('MAIN aktif')
            self.klasifikasi(1)
        return False

    def klasifikasi(self, reject):
        """CLASSIFY_IS_REJECT = 1: palang mendorong TOF detik kemudian (enqueueClassification)."""
        if reject:
            self.palang_antri.append(time.monotonic() + self.tof() / self.dunia.cepat)

    def tick_khusus(self, sekarang):
        r, c = self.reg, self.dunia.cepat
        jalan = self.conveyor_jalan()
        # --- hopper: dorong -> tahan -> tarik -> jeda (jeda hanya saat produksi) ---
        if self.hopper_fase and self.boleh_gerak():
            durasi = {'dorong': self.t_hopper_gerak(), 'tahan': self.w['hopper_hold_ms'] / 1000,
                      'tarik': self.t_hopper_gerak(), 'jeda': self.w['hopper_jeda_siklus_ms'] / 1000}
            if self.dunia.objek_paksa and self.hopper_fase == 'jeda':
                durasi['jeda'] = max(0.0, self.dunia.objek_paksa - durasi['dorong'] * 2 - durasi['tahan'])
            if sekarang - self.hopper_sejak >= durasi[self.hopper_fase] / c:
                self.hopper_sejak = sekarang
                if self.hopper_fase == 'dorong':
                    if random.random() < self.w['peluang_objek']:
                        self.objek.append({'mm': 0.0, 'prox1_t': None})
                    self.hopper_fase = 'tahan'
                elif self.hopper_fase == 'tahan':
                    self.hopper_fase = 'tarik'
                elif self.hopper_fase == 'tarik':
                    if self.hopper_uji:
                        self.hopper_fase, self.hopper_uji = None, False
                    else:
                        self.hopper_fase = 'jeda'
                elif self.hopper_fase == 'jeda':
                    if jalan and not r[24]:
                        self.hopper_fase = 'dorong'
                    else:
                        self.hopper_sejak = sekarang      # dijeda / berhenti: tunggu di posisi awal
            if not jalan and not self.hopper_uji and self.hopper_fase == 'jeda':
                self.hopper_fase = None
        # --- objek bergerak hanya saat conveyor jalan ---
        if jalan:
            dt = getattr(self, '_t_lalu', sekarang)
            maju = self.v_mm_s() * (sekarang - dt) * c
            _, palang, prox1, prox2 = self.posisi()
            sisa = []
            for o in self.objek:
                o['mm'] += maju
                if o['prox1_t'] is None and o['mm'] >= prox1:
                    o['prox1_t'] = sekarang
                if o['mm'] >= prox2:
                    r[10] = (r[10] + 1) & 0xFFFF
                    if o['prox1_t'] is not None:          # uji kecepatan PROX_1 -> PROX_2
                        ms = max(1, int((sekarang - o['prox1_t']) * 1000 * c))
                        r[21], r[20] = ms, int(self.w['jarak_uji_kecepatan_mm'] * 1000 / ms)
                        r[22] = (r[22] + 1) & 0xFFFF
                    log(f"   SORTER: objek lewat sensor PASS (PASS_COUNT={r[10]})")
                else:
                    sisa.append(o)
            self.objek = sisa
        self._t_lalu = sekarang
        # --- palang: antrian klasifikasi reject -> dorong -> tarik ---
        if self.palang_fase is None and self.palang_antri and sekarang >= self.palang_antri[0]:
            terlambat = sekarang - self.palang_antri.pop(0)
            if terlambat > 1.0:
                r[19] = (r[19] + 1) & 0xFFFF
                log(f"   SORTER: !! REJECT dilewati, giliran dorong lewat {terlambat:.1f} s")
            elif self.boleh_gerak():
                self.palang_fase, self.palang_sejak = 'dorong', sekarang
                r[11] = (r[11] + 1) & 0xFFFF
                _, palang, _, _ = self.posisi()
                sebelum = len(self.objek)
                self.objek = [o for o in self.objek if abs(o['mm'] - palang) > 25]
                log(f"   SORTER: palang DORONG (REJECT_COUNT={r[11]})"
                    + ('' if len(self.objek) < sebelum else ' -- tidak ada objek di depan palang'))
        if self.palang_fase == 'dorong' and sekarang - self.palang_sejak >= self.w['palang_push_ms'] / 1000 / c:
            self.palang_fase, self.palang_sejak = 'tarik', sekarang
        elif self.palang_fase == 'tarik' and sekarang - self.palang_sejak >= self.w['palang_retract_ms'] / 1000 / c:
            self.palang_fase = None
        # --- ACTIVITY_CODE seperti activityCode() firmware ---
        if r[0] == FAULT:
            self.akt(90)
        elif r[0] == ESTOP:
            self.akt(91)
        elif self.palang_fase:
            self.akt(2)
        elif self.motor_a:
            self.akt(3)
        elif self.hopper_uji:
            self.akt(4)
        else:
            self.akt(1 if jalan else 0)


# ============================================================
# PICKER -- antrian gerakan, waktu dari trajStepUs / trajStepIntervalMs
# ============================================================
class Picker(Node):
    def t_us(self, us):
        laju = max(1, self.w['traj_step_us']) / max(1, self.w['traj_step_interval_ms'])   # us per ms
        return us / laju / 1000

    def ke_pose(self, p):
        r = self.reg

        def mulai():
            r[11] = AKT_POSE.get(p, 8)

        def selesai():
            r[10] = p          # CURRENT_POSE ditulis saat gerakan SELESAI
        return [(0, mulai), (self.t_us(self.w['jarak_pose_us']), selesai)]

    def gerakan(self, i, saat=None):
        """antrekanUjiGerakan: ke pose ASAL dulu, adegan ON satu per satu, lalu pose TUJUAN."""
        r = self.reg
        rencana = self.ke_pose(GERAKAN_ASAL[i])

        def mulai():
            r[17], r[18], r[11] = i, 0, 12
        rencana.append((0, mulai))
        for k in range(int(self.w['adegan_per_gerakan'])):
            rencana.append((self.t_us(self.w['jarak_adegan_us']), lambda k=k: r.__setitem__(18, k + 1)))
            if saat and k in saat:
                rencana.append((0, saat[k]))

        def selesai_adegan():
            r[17], r[18] = 0xFF, 0
        rencana.append((0, selesai_adegan))
        return rencana + self.ke_pose(GERAKAN_TUJUAN[i])

    def ke_ready(self):
        if self.reg[10] == 3 and not self.sibuk():
            return []
        return self.ke_pose(0) + self.gerakan(0)[len(self.ke_pose(0)):]

    def jalankan(self, op, arg, seq):
        r = self.reg
        if op == 11:
            if self.boleh_gerak():
                self.set_main(True)
        elif op == 12:
            self.set_main(False)
        elif op == 7:
            self.reset_fault()
        elif op == 9:
            self.w['traj_step_us'] = max(1, min(500, arg))
        elif op == 10:
            self.w['traj_step_interval_ms'] = max(5, min(200, arg))
        elif op in (2, 13, 15, 5, 6, 14):                      # gerakan manual -- TEST saja
            if self.main:
                return self.tolak('MAIN aktif')
            if not self.boleh_gerak():
                return self.tolak('FAULT')
            if op == 2:
                rencana = self.ke_pose(0)
            elif op == 15:
                rencana = self.ke_ready()
            elif op == 13:
                if arg > 3:
                    return self.tolak(f'pose {arg} tidak ada')
                rencana = self.ke_pose(arg)
            elif op == 14:
                if arg > 4:
                    return self.tolak(f'gerakan {arg} tidak ada')
                rencana = self.gerakan(arg)
            else:
                us = self.w['pick_offset_us' if op == 5 else 'place_offset_us']
                rencana = [(0, lambda: r.__setitem__(11, 5 if op == 5 else 6)), (self.t_us(us), lambda: None)]
            self.antre(rencana, tambah=True)
        elif op == 8:                                          # MOVE_PACKAGE -- MAIN saja
            if not self.main or not self.boleh_gerak():
                return self.tolak('butuh MAIN')

            def angkat():
                self.dunia.node[DISPENSER].reg[17] = 0
                log("   PICKER mengangkat package dari ujung Dispenser")

            def taruh():
                log("   PICKER meletakkan package di Stocker")
            # Ready>Pick, Pick>Home (package terangkat di adegan pertama), Home>Place, Place>Home, Home>Ready
            rencana = (self.ke_ready() + self.gerakan(1)[len(self.ke_pose(3)):]
                       + self.gerakan(2, saat={0: angkat})[len(self.ke_pose(1)):]
                       + self.gerakan(3)[len(self.ke_pose(0)):] + [(0, taruh)]
                       + self.gerakan(4)[len(self.ke_pose(2)):]
                       + self.gerakan(0)[len(self.ke_pose(0)):])
            self.antre(rencana, ack_seq=seq, tambah=True)
            return True
        return False


# ============================================================
# DISPENSER -- pipeline produksi + uji TEST
# ============================================================
class Dispenser(Node):
    def awal(self):
        self.stok = int(self.w['stok_package'])
        self.servo = {1: self.w['servo_start_us'], 2: self.w['servo_start_us']}
        self.conveyor_uji = False
        self.uji_sejak = 0.0

    def t_servo(self, dari, ke):
        return abs(ke - dari) / max(1, self.w['servo_step_us']) * self.w['servo_step_interval_ms'] / 1000

    def servo_gerak(self, no, buka, akt_gerak):
        """Langkah servo ke titik akhir (buka) / awal (tutup), durasi dari jarak sekarang."""
        tujuan = self.w['servo_end_us'] if buka else self.w['servo_start_us']
        durasi = self.t_servo(self.servo[no], tujuan)

        def mulai():
            self.reg[11] = akt_gerak

        def selesai():
            self.servo[no] = tujuan
        return [(0, mulai), (durasi, selesai)]

    def siklus_servo(self, no, tahap=None):
        """buka -> tahan -> tutup. tahap = (t_buka, t_tahan, t_tutup) PIPELINE_STAGE untuk produksi."""
        a = 2 if no == 1 else 5
        rencana = self.servo_gerak(no, True, a)
        if tahap:
            rencana.insert(0, (0, lambda: self._tahap(tahap[0])))
        rencana += [(0, lambda: (self.reg.__setitem__(11, a + 1), tahap and self._tahap(tahap[1]))),
                    (self.w['servo_hold_ms'] / 1000, lambda: None),
                    (0, lambda: tahap and self._tahap(tahap[2]))]
        return rencana + self.servo_gerak(no, False, a + 2)

    def _tahap(self, t):
        r = self.reg
        r[26] = t
        r[25] = 1 if t == 9 else 0

    def jatuhkan_package(self):
        """Gerbang bawah melepas satu package kosong -> berjalan ke sensor TENGAH."""
        r = self.reg

        def tiba():
            if self.stok <= 0:
                r[10] = 1
                self.fault(13, 'MIDDLE_PACKAGE_MISSING (stok package habis)')
                return
            self.stok -= 1
            r[10] = 1 if self.stok == 0 else 0
            r[16] = 1
            r[19] = (r[19] + 1) & 0xFFFF
        return [(0, lambda: r.__setitem__(11, 9)), (self.w['conveyor_ke_tengah_s'], tiba)]

    def ke_tengah(self):
        """Package kosong ke TENGAH, lalu gerbang (servo 1) dibuka -> SIAP ISI."""
        return ([(0, lambda: self._tahap(7))] + self.jatuhkan_package() + self.buka_gerbang())

    def buka_gerbang(self):
        return ([(0, lambda: self._tahap(8))] + self.servo_gerak(1, True, 2)
                + [(0, lambda: (self._tahap(9), self.reg.__setitem__(11, 8)))])

    def jalankan(self, op, arg, seq):
        r = self.reg
        if op == 14:                                           # START_MAIN
            if not self.boleh_gerak():
                return self.tolak('FAULT')
            self.set_main(True)
            r[15] = 0
            if r[16]:
                self.antre(self.buka_gerbang())
            else:
                awal = (self.siklus_servo(1, (1, 2, 3)) + self.siklus_servo(2, (4, 5, 6)))
                self.antre(awal + self.ke_tengah())
        elif op == 15:                                         # STOP_MAIN
            self.set_main(False)
            self.langkah = []
            self._tahap(0)
            r[15], r[0], r[11] = 0, IDLE, 0
            self.conveyor_uji = False
        elif op == 2:
            self.reset_fault()
        elif op == 3:
            pass                                               # SET_CONVEYOR_SPEED -- disimpan firmware
        elif op == 1:                                          # REQUEST_REFILL
            if not self.main or not self.boleh_gerak() or r[15] or r[26] != 9:
                return self.tolak(f'REQUEST_REFILL di tahap {NAMA_TAHAP[r[26]]}')

            def sampai_ujung():
                r[16], r[17], r[15] = 0, 1, 1
                r[24] = (r[24] + 1) & 0xFFFF
                r[23] = (r[23] + 1) & 0xFFFF
                log("   DISPENSER: package penuh sampai UJUNG -> PACKAGE_READY_FLAG = 1")
            self.antre([(0, lambda: (self._tahap(10), r.__setitem__(11, 1))),
                        (self.w['conveyor_ke_ujung_s'], sampai_ujung)]
                       + [(0, lambda: self._tahap(11))] + self.servo_gerak(1, False, 4)
                       + self.siklus_servo(2, (12, 13, 14))
                       + [(0, lambda: (self._tahap(15), r.__setitem__(11, 8)))])
        elif op == 5:                                          # ACK_PACKAGE_TAKEN
            r[15] = 0
            if r[26] == 15:
                self.antre(self.ke_tengah())
        elif op in (11, 12, 13, 97, 98):                       # uji -- TEST saja
            if self.main:
                return self.tolak('MAIN aktif')
            if not self.boleh_gerak():
                return self.tolak('FAULT')
            if op == 11:
                self.conveyor_uji, self.uji_sejak = bool(arg), time.monotonic()
                r[11] = 1 if arg else 0
            elif op in (12, 13):
                no = 1 if op == 12 else 2
                a = 2 if no == 1 else 5
                self.antre(self.servo_gerak(no, bool(arg), a + (0 if arg else 2)))
            else:
                no = 1 if op == 97 else 2
                rencana = self.siklus_servo(no)
                if no == 1 and not r[16]:                      # gerbang bawah melepas package
                    rencana += self.jatuhkan_package()
                self.antre(rencana)
        return False

    def tick_khusus(self, sekarang):
        r = self.reg
        if self.conveyor_uji and self.boleh_gerak() and r[16]:
            if sekarang - self.uji_sejak >= self.w['conveyor_ke_ujung_s'] / self.dunia.cepat:
                r[16], r[17] = 0, 1
                r[24] = (r[24] + 1) & 0xFFFF
                r[23] = (r[23] + 1) & 0xFFFF                   # conveyor berhenti sendiri di UJUNG
                self.conveyor_uji, r[11] = False, 0
                log("   DISPENSER (uji): package sampai UJUNG, conveyor berhenti otomatis")
        elif self.conveyor_uji and not r[16]:
            self.uji_sejak = sekarang

    def selesai_semua(self):
        if self.reg[0] == RUN:
            self.reg[0] = IDLE
        if not self.conveyor_uji:
            self.reg[11] = 0


# ============================================================
# STOCKER -- stepper: langkah x stepIntervalUs
# ============================================================
class Stocker(Node):
    def awal(self):
        self.pos = 0.0          # posisi X/Z dalam langkah (0 = READY / home)

    def t_langkah(self, n, homing=False):
        return n * self.w['homing_step_interval_us' if homing else 'step_interval_us'] / 1e6

    def pos_rak(self, no):
        return self.w['langkah_ready_ke_rak1'] + (no - 1) * self.w['langkah_antar_rak']

    def gerak_ke(self, tujuan, akt, idx_akhir):
        r = self.reg

        def mulai():
            r[13], r[10] = akt, 0xFF

        def selesai():
            self.pos, r[10] = tujuan, idx_akhir
        return [(0, mulai), (self.t_langkah(abs(tujuan - self.pos)), selesai)]

    def dorong(self, rak=None):
        r = self.reg

        def terdorong():
            if rak and self.dunia.sensor_rak:
                r[12] |= 1 << rak
            log(f"   STOCKER: package didorong{' ke Rak ' + str(rak) if rak else ''}")
        n = self.w['push_extend_steps']
        return [(0, lambda: r.__setitem__(13, 3)), (self.t_langkah(n), terdorong),
                (0, lambda: r.__setitem__(13, 4)), (self.t_langkah(n), lambda: None)]

    def jalankan(self, op, arg, seq):
        r = self.reg
        if op == 9:
            if self.boleh_gerak():
                self.set_main(True)
        elif op == 10:
            self.set_main(False)
        elif op == 5:
            self.reset_fault()
        elif op == 7:
            self.w['step_interval_us'] = max(0, min(5000, arg))
        elif op == 8:
            self.w['homing_step_interval_us'] = max(20, min(5000, arg))
        elif op in (1, 2, 3, 4, 6):
            if not self.boleh_gerak():
                return self.tolak('FAULT')
            if self.sibuk():
                return self.tolak('masih bergerak')
            if op == 1:                                        # HOME_ALL: 3 axis berurutan
                t = self.t_langkah(self.w['langkah_homing_per_axis'], homing=True)

                def homed():
                    self.pos, r[11], r[10] = 0.0, 1, 0xFF
                self.antre([(0, lambda: r.__setitem__(13, 1)), (t * 3, homed)], seq)
                return True
            if not r[11]:
                self.fault(12, 'NOT_HOMED')
                return False
            if op == 6:                                        # GOTO_LOAD (READY)
                self.antre(self.gerak_ke(0.0, 5, 0xFF), seq)
                return True
            if op == 3:                                        # MOVE_TO_RACK -- TEST
                if self.main:
                    return self.tolak('MAIN aktif')
                if not 1 <= arg <= 6:
                    self.fault(11, 'RACK_IDX_INVALID')
                    return False
                self.antre(self.gerak_ke(self.pos_rak(arg), 2, arg), seq)
                return True
            if op == 4:                                        # PUSH_BOX -- TEST
                if self.main:
                    return self.tolak('MAIN aktif')
                self.antre(self.dorong(), seq)
                return True
            if op == 2:                                        # RUN_FULL_CYCLE -- MAIN
                if not self.main or not 1 <= arg <= 4:
                    return self.tolak('butuh MAIN & rak 1-4')
                self.antre(self.gerak_ke(self.pos_rak(arg), 2, arg) + self.dorong(arg)
                           + self.gerak_ke(0.0, 5, 0xFF), seq)
                return True
        return False


# ============================================================
# DUNIA -- keempat node
# ============================================================
class Dunia:
    def __init__(self, waktu, cepat=None, objek=None, sensor_rak=False, watchdog=True):
        self.waktu = waktu
        self.cepat = cepat or waktu['umum']['cepat']
        self.objek_paksa = objek          # detik antar objek dari hopper (None = dari siklus hopper)
        self.sensor_rak, self.watchdog = sensor_rak, watchdog
        self.mulai = time.monotonic()
        kelas = {SORTER: Sorter, PICKER: Picker, DISPENSER: Dispenser, STOCKER: Stocker}
        self.node = {sid: kelas[sid](sid, self) for sid in NAMA}
        self.kunci = threading.Lock()

    def tick(self):
        sekarang = time.monotonic()
        for n in self.node.values():
            n.tick(sekarang)

    def ringkasan(self):
        s, p, d, k = (self.node[i] for i in (SORTER, PICKER, DISPENSER, STOCKER))
        scan, palang, _, prox2 = s.posisi()
        siklus = s.t_hopper_gerak() * 2 + s.w['hopper_hold_ms'] / 1000 + s.w['hopper_jeda_siklus_ms'] / 1000
        baris = [
            f"faktor cepat x{self.cepat:g} (durasi di bawah = waktu NYATA, dibagi faktor ini)",
            f"SORTER    conveyor {s.v_mm_s():.0f} mm/s (PWM {s.w['conveyor_pwm']:.0f}), hopper siklus "
            f"{self.objek_paksa or siklus:.2f} s, objek hopper->kamera {scan / s.v_mm_s():.2f} s, "
            f"TOF kamera->palang {s.tof():.2f} s, hopper->PASS {prox2 / s.v_mm_s():.2f} s, "
            f"palang {s.w['palang_push_ms']:.0f}+{s.w['palang_retract_ms']:.0f} ms",
            f"PICKER    pindah pose {p.t_us(p.w['jarak_pose_us']):.2f} s, adegan "
            f"{p.t_us(p.w['jarak_adegan_us']):.2f} s x {p.w['adegan_per_gerakan']:.0f}",
            f"DISPENSER servo buka/tutup {d.t_servo(d.w['servo_start_us'], d.w['servo_end_us']):.2f} s, "
            f"tahan {d.w['servo_hold_ms'] / 1000:.2f} s, ke tengah {d.w['conveyor_ke_tengah_s']:g} s, "
            f"ke ujung {d.w['conveyor_ke_ujung_s']:g} s, stok {d.stok}",
            f"STOCKER   homing {k.t_langkah(k.w['langkah_homing_per_axis'], True) * 3:.2f} s, READY->Rak 1 "
            f"{k.t_langkah(k.w['langkah_ready_ke_rak1']):.2f} s, antar rak {k.t_langkah(k.w['langkah_antar_rak']):.2f} s, "
            f"dorong+tarik {k.t_langkah(k.w['push_extend_steps']) * 2:.2f} s",
        ]
        for b in baris:
            log('   ' + b)


# ============================================================
# MODBUS RTU SLAVE
# ============================================================
class Slave:
    def __init__(self, ser, dunia, echo):
        self.ser, self.dunia, self.echo = ser, dunia, echo
        self.buf = b''
        self.jumlah = {'req': 0, 'crc': 0}

    def _panjang(self, b):
        if len(b) < 2:
            return None
        fc = b[1]
        if fc in (3, 4, 6):
            return 8
        if fc == 16:
            return 9 + b[6] if len(b) >= 7 else None
        return 8

    def proses(self, data):
        self.buf += data
        while len(self.buf) >= 4:
            if self.buf[0] not in NAMA:
                self.buf = self.buf[1:]
                continue
            n = self._panjang(self.buf)
            if n is None or len(self.buf) < n:
                return
            frame, sisa = self.buf[:n], self.buf[n:]
            if crc16(frame[:-2]) != frame[-2:]:
                self.jumlah['crc'] += 1
                self.buf = self.buf[1:]
                continue
            self.buf = sisa
            jawaban = self.jawab(frame)
            if jawaban:
                self.ser.write(jawaban)
                self.ser.flush()
                if self.echo:
                    self.ser.read(len(jawaban))

    def jawab(self, f):
        sid, fc = f[0], f[1]
        node = self.dunia.node[sid]
        if node.mati or time.monotonic() < node.boot_sampai:
            return None
        node.terakhir_rx = time.monotonic()
        self.jumlah['req'] += 1
        salah = lambda kode: bytes([sid, fc | 0x80, kode])
        with self.dunia.kunci:
            if fc == 3:
                alamat, jumlah = int.from_bytes(f[2:4], 'big'), int.from_bytes(f[4:6], 'big')
                if any(a not in node.reg for a in range(alamat, alamat + jumlah)) or not 1 <= jumlah <= 125:
                    isi = salah(2)
                else:
                    data = b''.join((int(node.reg[a]) & 0xFFFF).to_bytes(2, 'big') for a in range(alamat, alamat + jumlah))
                    isi = bytes([sid, 3, len(data)]) + data
            elif fc in (6, 16):
                alamat = int.from_bytes(f[2:4], 'big')
                if fc == 6:
                    nilai = [int.from_bytes(f[4:6], 'big')]
                else:
                    jumlah = int.from_bytes(f[4:6], 'big')
                    nilai = [int.from_bytes(f[7 + 2 * i:9 + 2 * i], 'big') for i in range(jumlah)]
                tulis = BISA_DITULIS | BISA_DITULIS_KHUSUS.get(sid, set())
                if any(a not in tulis for a in range(alamat, alamat + len(nilai))):
                    isi = salah(2)
                else:
                    for i, v in enumerate(nilai):
                        a = alamat + i
                        node.reg[a] = v
                        if a == 2:
                            node.command()
                        elif a == 12 and sid == SORTER:
                            node.klasifikasi(v)
                    isi = f[:6] if fc == 16 else f[:-2]
            else:
                isi = salah(1)
        return isi + crc16(isi)


# ============================================================
# LAYAR TERMINAL
# ============================================================
def tabel(dunia):
    st = {IDLE: 'IDLE', RUN: 'RUN', FAULT: 'FAULT', ESTOP: 'ESTOP', 0: 'INIT'}
    for sid, n in dunia.node.items():
        r = n.reg
        teks = {SORTER: lambda: f"pass={r[10]} reject={r[11]} hopper={'JEDA' if r[24] else (n.hopper_fase or '-')} "
                                f"objek di conveyor={len(n.objek)} akt={r[13]}",
                PICKER: lambda: f"pose={NAMA_POSE.get(r[10], r[10])} gerakan={r[17] if r[17] != 0xFF else '-'} "
                                f"adegan={r[18]} akt={r[11]} antri={len(n.langkah)}",
                DISPENSER: lambda: f"tahap={NAMA_TAHAP[r[26]]} tengah={r[16]} ujung={r[17]} flag={r[15]} "
                                   f"stok={n.stok} akt={r[11]}",
                STOCKER: lambda: f"homed={r[11]} rak={r[10] if r[10] != 0xFF else '-'} bitmask={r[12]:#06b} "
                                 f"akt={r[13]}"}[sid]()
        status = 'MATI' if n.mati else st.get(r[0], r[0])
        print(f"  {sid} {NAMA[sid]:9s} {status:5s} fault={r[1]:<3d} main={r[R_MAIN[sid]]} "
              f"menu={r[R_MENU[sid]]}  {teks}")


def konsol(dunia, berhenti):
    while not berhenti.is_set():
        try:
            baris = input().strip().split()
        except EOFError:
            return
        if not baris:
            continue
        k, a = baris[0].lower(), baris[1:]
        try:
            with dunia.kunci:
                if k == 'q':
                    berhenti.set()
                elif k == 's':
                    tabel(dunia)
                elif k == 'w':
                    dunia.ringkasan()
                elif k == 'b':
                    s = dunia.node[SORTER]
                    s.objek.append({'mm': s.posisi()[0], 'prox1_t': None})
                elif k == 'n':
                    dunia.node[int(a[0])].restart()
                elif k in ('f', 'r', 'e', 'm', 'x'):
                    n = dunia.node[int(a[0])]
                    if k == 'f':
                        n.fault(int(a[1]), '(dari konsol)')
                    elif k == 'r':
                        n.reset_fault()
                        log(f"   {NAMA[n.sid]} RESET_FAULT dari panel")
                    elif k == 'e':
                        if n.reg[0] == ESTOP:
                            n.reg[0] = FAULT
                            log(f"   {NAMA[n.sid]} E-STOP dilepas (masih FAULT 5 -- perlu RESET_FAULT)")
                        else:
                            n.fault(5, 'E-STOP ditekan')
                            n.reg[0] = ESTOP
                    elif k == 'm':
                        n.reg[R_MENU[n.sid]] ^= 1
                        log(f"   {NAMA[n.sid]} menu LCD {'DIBUKA' if n.reg[R_MENU[n.sid]] else 'ditutup'}")
                    elif k == 'x':
                        n.mati = not n.mati
                        log(f"   {NAMA[n.sid]} {'MATI (tidak menjawab)' if n.mati else 'hidup lagi'}")
                else:
                    print("  perintah: s | w | f <id> <kode> | r <id> | e <id> | m <id> | x <id> | n <id> | b | q")
        except (IndexError, ValueError, KeyError):
            print("  format salah -- contoh: f 3 11   (node 3 FAULT kode 11)")


# ============================================================
# PORT VIRTUAL -- simulasi DI DALAM program master (sorting-automation.py --simulasi)
# ============================================================
class PortVirtual:
    def __init__(self):
        self.masuk, self.cv, self.lawan = bytearray(), threading.Condition(), None
        self.timeout, self.write_timeout, self.is_open, self.port = 0.5, 2, True, 'SIMULASI'
        self.baudrate, self.bytesize, self.parity, self.stopbits = 19200, 8, 'N', 1

    def write(self, data):
        with self.lawan.cv:
            self.lawan.masuk += data
            self.lawan.cv.notify_all()
        return len(data)

    def read(self, n=1):
        batas = time.monotonic() + (self.timeout or 0)
        with self.cv:
            while len(self.masuk) < n:
                sisa = batas - time.monotonic()
                if sisa <= 0:
                    break
                self.cv.wait(sisa)
            data = bytes(self.masuk[:n])
            del self.masuk[:n]
            return data

    def reset_input_buffer(self):
        with self.cv:
            self.masuk.clear()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False


def _putaran(dunia, berhenti):
    while not berhenti.is_set():
        with dunia.kunci:
            dunia.tick()
        time.sleep(0.01)


def mulai_virtual(cepat=None, objek=None, tempuh=None, sensor_rak=False, konsol_aktif=True, waktu=None):
    """Jalankan keempat node di thread latar. Kembalikan (port_master, dunia).
    cepat/objek None = dari simulasi-waktu.config. `tempuh` diabaikan (dulu: detik objek di
    conveyor; sekarang dihitung dari jarak & kecepatan conveyor)."""
    global AWALAN
    AWALAN = '[SIM] '
    master, ujung = PortVirtual(), PortVirtual()
    master.lawan, ujung.lawan = ujung, master
    ujung.timeout = 0.005
    dunia = Dunia(muat_waktu(waktu), cepat, objek, sensor_rak, True)
    slave = Slave(ujung, dunia, False)
    berhenti = threading.Event()

    def bus():
        while not berhenti.is_set():
            data = ujung.read(256)
            if data:
                slave.proses(data)
    threading.Thread(target=bus, daemon=True).start()
    threading.Thread(target=_putaran, args=(dunia, berhenti), daemon=True).start()
    if konsol_aktif:
        threading.Thread(target=konsol, args=(dunia, berhenti), daemon=True).start()
    dunia.ringkasan()
    return master, dunia


def main():
    ap = argparse.ArgumentParser(description="Simulasi 4 node ESP32 (Modbus RTU slave) dengan timing firmware")
    ap.add_argument('--port', required=True, help='mis. COM5 (Windows) atau /dev/ttyUSB0 (Linux)')
    ap.add_argument('--baud', type=int, default=19200)
    ap.add_argument('--waktu', help='file timing (bawaan: simulasi-waktu.config)')
    ap.add_argument('--cepat', type=float, help='faktor percepatan (menimpa [umum] cepat)')
    ap.add_argument('--objek', type=float, help='paksa detik antar objek dari hopper')
    ap.add_argument('--sensor-rak', action='store_true', help='Stocker menandai rak terisi di bitmask')
    ap.add_argument('--tanpa-watchdog', action='store_true', help='matikan FAULT COMM_TIMEOUT')
    ap.add_argument('--mati', type=int, nargs='*', default=[], metavar='ID', help='node yang tidak menjawab')
    ap.add_argument('--echo', action='store_true', help='buang pantulan data kirim (adaptor tertentu)')
    args = ap.parse_args()
    if serial is None:
        sys.exit("ERROR: butuh pyserial:  pip install pyserial")

    dunia = Dunia(muat_waktu(args.waktu), args.cepat, args.objek, args.sensor_rak, not args.tanpa_watchdog)
    for sid in args.mati:
        dunia.node[sid].mati = True
    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8, parity='N', stopbits=1, timeout=0.005)
    except serial.SerialException as e:
        sys.exit(f"ERROR: {args.port} tidak bisa dibuka: {e}")
    slave = Slave(ser, dunia, args.echo)
    berhenti = threading.Event()
    threading.Thread(target=_putaran, args=(dunia, berhenti), daemon=True).start()
    threading.Thread(target=konsol, args=(dunia, berhenti), daemon=True).start()

    log(f"Simulasi 4 node di {args.port} @ {args.baud}")
    dunia.ringkasan()
    log("Ketik 's' + Enter untuk status, 'w' untuk timing, 'q' untuk keluar. Menunggu master ...")
    terakhir_lapor, req_lalu = time.monotonic(), 0
    try:
        while not berhenti.is_set():
            data = ser.read(256)
            if data:
                slave.proses(data)
                slave._buf_sejak = time.monotonic()
            elif slave.buf and time.monotonic() - getattr(slave, '_buf_sejak', 0) > 0.05:
                slave.buf = b''        # sisa frame terpotong -- buang setelah jeda
            if time.monotonic() - terakhir_lapor > 30:
                terakhir_lapor = time.monotonic()
                baru = slave.jumlah['req'] - req_lalu
                req_lalu = slave.jumlah['req']
                log(f"   [bus] {baru} request dijawab dalam 30 s, CRC salah {slave.jumlah['crc']}"
                    + ("  -- TIDAK ADA REQUEST: cek kabel A/B & baud" if baru == 0 else ''))
    except KeyboardInterrupt:
        pass
    finally:
        berhenti.set()
        ser.close()
        log("Simulasi berhenti.")


if __name__ == '__main__':
    main()
