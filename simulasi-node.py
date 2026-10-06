#!/usr/bin/env python3
"""
============================================================
 SIMULASI 4 NODE -- Modbus RTU slave lewat adaptor USB-RS485 (2026-10-07)
============================================================

Script ini MENJAWAB sebagai keempat ESP32 (slave 1 SORTER, 2 PICKER, 3 DISPENSER,
4 STOCKER), supaya sorting-automation.py di Orange Pi bisa diuji tanpa mesin.

SAMBUNGAN
  Laptop/PC --USB-- adaptor USB-RS485 --A/B-- modul RS485 Orange Pi (/dev/ttyS3)
  A ke A, B ke B (kalau tidak ada jawaban sama sekali, coba tukar A/B). GND bersama
  disarankan kalau kabel panjang.

  Bisa juga dijalankan di Orange Pi KEDUA / komputer Linux mana pun yang punya
  adaptor USB-RS485 (/dev/ttyUSB0).

JALANKAN
  pip install pyserial
  python simulasi-node.py --port COM5                  (Windows)
  python3 simulasi-node.py --port /dev/ttyUSB0         (Linux)

  --baud 19200        harus sama dengan rs485_baud di sorting.config
  --cepat 2           semua gerakan 2x lebih cepat dari perkiraan waktu nyata
  --objek 1.5         hopper melepas 1 objek tiap 1,5 detik
  --sensor-rak        Stocker menandai rak terisi di RACK_OCCUPIED_BITMASK (seolah ada sensor)
  --mati 4            node 4 (STOCKER) tidak menjawab sejak awal
  --echo              adaptor memantulkan data yang dikirimnya sendiri (jarang)

PERINTAH SAAT BERJALAN (ketik lalu Enter)
  s              tabel status keempat node
  f <id> <kode>  jadikan node FAULT, mis. "f 3 11" = Dispenser BOX_NOT_ARRIVED
  r <id>         RESET_FAULT dari panel node
  e <id>         tekan / lepas E-STOP
  m <id>         operator masuk / keluar menu LCD (command diabaikan TANPA ack)
  x <id>         node mati / hidup lagi (tidak menjawab Modbus)
  n <id>         node RESTART (register kembali awal, MAIN mati, 3 detik tidak menjawab)
  b              tombol uji: satu objek lewat sensor PASS sekarang juga
  q              keluar

YANG DITIRU DARI FIRMWARE (register.h tiap node, 2026-10-07)
  - peta register sama persis; alamat yang tidak ada dijawab ILLEGAL DATA ADDRESS (02),
    persis seperti firmware, jadi pemeriksaan "firmware lama" di master ikut teruji
  - command 3 langkah ARG -> SEQ -> CMD, ack = CMD_ACK_SEQ; seq yang sama diabaikan
  - ack TERTUNDA: Picker MOVE_PACKAGE, Stocker HOME_ALL / RUN_FULL_CYCLE / GOTO_LOAD /
    MOVE_TO_RACK / PUSH_BOX (ack baru dikirim setelah gerakan selesai)
  - command yang ditolak (salah mode) tetap di-ack tanpa melakukan apa pun
  - menu LCD aktif: command diabaikan dan TIDAK di-ack
  - watchdog COMM_TIMEOUT: node RUNNING tanpa request 30 detik -> FAULT 1
  - Sorter: hopper melepas objek, objek menempuh conveyor lalu terhitung PASS (atau
    REJECT kalau master menulis CLASSIFY_IS_REJECT = 1); jeda hopper (SET_HOPPER_JEDA)
  - Dispenser: pipeline START_MAIN -> SIAP ISI -> REQUEST_REFILL -> maju ke ujung ->
    refill servo -> tunggu diambil -> ACK_PACKAGE_TAKEN -> package kosong ke tengah
  - Picker: MOVE_PACKAGE = Ready>Pick, Pick>Home, Home>Place, Place>Home, Home>Ready dengan
    CURRENT_POSE & GERAKAN_AKTIF berubah seperti aslinya; package di ujung Dispenser
    terangkat saat lengan meninggalkan PICKUP
"""

import argparse
import random
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("ERROR: butuh pyserial:  pip install pyserial")

SORTER, PICKER, DISPENSER, STOCKER = 1, 2, 3, 4
NAMA = {SORTER: 'SORTER', PICKER: 'PICKER', DISPENSER: 'DISPENSER', STOCKER: 'STOCKER'}
IDLE, RUN, FAULT, ESTOP = 1, 2, 3, 4

# Register yang ADA di firmware tiap node (selain 0-6 universal). Nilai awal.
REGISTER = {
    SORTER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0, 19: 0,
             20: 0, 21: 0, 22: 0, 23: 0, 24: 0},
    PICKER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0xFF, 18: 0},
    DISPENSER: {10: 0, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0, 19: 0, 20: 0,
                21: 0, 22: 0, 23: 0, 24: 0, 25: 0, 26: 0},
    STOCKER: {10: 0xFF, 11: 0, 12: 0, 13: 0, 14: 0, 15: 0, 16: 0, 17: 0, 18: 0},
}
R_MAIN = {SORTER: 17, PICKER: 15, DISPENSER: 18, STOCKER: 17}
R_MENU = {SORTER: 18, PICKER: 16, DISPENSER: 20, STOCKER: 18}
R_UPTIME = {SORTER: 16, PICKER: 14, DISPENSER: 14, STOCKER: 16}
R_LASTFAULT = {SORTER: 15, PICKER: 13, DISPENSER: 13, STOCKER: 15}
R_AKTIVITAS = {SORTER: 13, PICKER: 11, DISPENSER: 11, STOCKER: 13}
BISA_DITULIS = {2, 3, 4, 6}                       # CMD, ARG, SEQ, HEARTBEAT
BISA_DITULIS_KHUSUS = {SORTER: {12}}              # CLASSIFY_IS_REJECT

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
NAMA_POSE = {0: 'HOME', 1: 'PICKUP', 2: 'LIFT', 3: 'READY'}
NAMA_TAHAP = ['MATI', 'INIT servo1 buka', 'INIT servo1 tahan', 'INIT servo1 tutup', 'INIT servo2 buka',
              'INIT servo2 tahan', 'INIT servo2 tutup', 'INIT tunggu PROX_2', 'buka gerbang', 'SIAP ISI',
              'maju ke ujung', 'refill servo1 tutup', 'refill servo2 buka', 'refill servo2 tahan',
              'refill servo2 tutup', 'tunggu diambil arm']


AWALAN = ''   # '[SIM] ' saat dijalankan di dalam sorting-automation.py --simulasi


def log(teks):
    print(time.strftime('%H:%M:%S ') + AWALAN + teks, flush=True)


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


# ============================================================
# NODE
# ============================================================
class Node:
    def __init__(self, sid, dunia):
        self.sid, self.dunia = sid, dunia
        self.reg = {0: IDLE, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        self.reg.update(REGISTER[sid])
        self.mati = False
        self.terakhir_rx = time.monotonic()
        self.ack_tertunda = None      # seq yang di-ack SETELAH gerakan selesai
        self.langkah = []             # antrian (detik, fungsi) untuk gerakan / tahap
        self.langkah_sejak = None
        self.mulai = dunia.mulai      # waktu 'menyala' -- direset saat node restart
        self.boot_sampai = 0

    # --- bantu ---
    @property
    def main(self):
        return self.reg[R_MAIN[self.sid]] == 1

    def set_main(self, nyala):
        self.reg[R_MAIN[self.sid]] = 1 if nyala else 0

    def boleh_gerak(self):
        return self.reg[0] not in (FAULT, ESTOP) and self.reg[1] == 0

    def fault(self, kode, alasan=''):
        self.reg[1] = kode
        self.reg[R_LASTFAULT[self.sid]] = kode
        self.reg[0] = FAULT
        self.langkah, self.ack_tertunda = [], None
        log(f"   !! {NAMA[self.sid]} FAULT {kode} {alasan}")

    def reset_fault(self):
        if self.reg[0] == ESTOP:
            return
        self.reg[1] = 0
        self.reg[0] = RUN if (self.sid == SORTER and self.main) else IDLE

    def antre(self, rencana, ack_seq=None):
        """rencana: [(detik, fungsi_saat_selesai), ...]. Durasi dibagi faktor --cepat."""
        self.langkah = [(d / self.dunia.cepat, f) for d, f in rencana]
        self.langkah_sejak = time.monotonic()
        self.ack_tertunda = ack_seq
        if self.sid != SORTER:
            self.reg[0] = RUN

    def restart(self):
        """Seperti ESP32 yang menyala ulang (mis. tegangan turun): semua register kembali ke
        nilai awal, MAIN mati, Stocker belum homing, tidak menjawab selama boot ~3 detik."""
        self.reg = {0: IDLE, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        self.reg.update(REGISTER[self.sid])
        self.langkah, self.ack_tertunda = [], None
        self.mulai = time.monotonic()
        self.boot_sampai = self.mulai + 3
        log(f"   !! {NAMA[self.sid]} RESTART (boot 3 detik)")

    def tick(self, sekarang):
        self.reg[R_UPTIME[self.sid]] = int(sekarang - self.mulai) & 0xFFFF
        self.reg[6] = self.reg[R_UPTIME[self.sid]]
        # watchdog komunikasi -- sama dengan firmware (30 s, hanya saat RUNNING)
        if (self.dunia.watchdog and self.reg[0] == RUN and self.main
                and sekarang - self.terakhir_rx > 30 and self.reg[1] == 0):
            self.fault(1, 'COMM_TIMEOUT (tidak ada request 30 s)')
        if self.langkah and self.boleh_gerak():
            durasi, fungsi = self.langkah[0]
            if sekarang - self.langkah_sejak >= durasi:
                self.langkah.pop(0)
                self.langkah_sejak = sekarang
                fungsi()
                if not self.langkah:
                    if self.ack_tertunda is not None:
                        self.reg[5] = self.ack_tertunda
                        log(f"   {NAMA[self.sid]} selesai -> ack seq {self.ack_tertunda}")
                        self.ack_tertunda = None
                    if self.sid != SORTER and self.reg[0] == RUN:
                        self.reg[0] = IDLE

    # --- command ---
    def command(self):
        op, arg, seq = self.reg[2], self.reg[3], self.reg[4]
        if self.reg[R_MENU[self.sid]]:
            log(f"   {NAMA[self.sid]}: operator di menu LCD -- command DIABAIKAN tanpa ack")
            return
        if seq == self.reg[5]:
            return                    # seq yang sama sudah diproses
        nama = NAMA_CMD[self.sid].get(op, f'opcode {op}')
        tunda = self.jalankan(op, arg, seq)
        log(f"<- {NAMA[self.sid]:9s} {nama}" + (f" arg={arg}" if arg else '') +
            (f"  (ack setelah selesai)" if tunda else ''))
        if not tunda:
            self.reg[5] = seq

    def jalankan(self, op, arg, seq):
        """True = ack ditunda sampai gerakan selesai."""
        return getattr(self, f'_cmd_{NAMA[self.sid].lower()}')(op, arg, seq)

    # ---------- SORTER ----------
    def _cmd_sorter(self, op, arg, seq):
        r = self.reg
        if op == 1:                                            # START
            if self.boleh_gerak():
                r[0], r[13] = RUN, 1
                self.set_main(True)
                r[24] = 0
        elif op == 2:                                          # STOP
            r[0], r[13], r[24] = IDLE, 0, 0
            self.set_main(False)
        elif op == 3:
            self.reset_fault()
        elif op == 7:                                          # RESET_COUNTERS
            r[10] = r[11] = r[19] = 0
        elif op == 11:                                         # SET_HOPPER_JEDA
            r[24] = 1 if arg else 0
        elif op == 97 and not self.main and self.boleh_gerak():
            self.dunia.objek_lepas()
        return False

    # ---------- PICKER ----------
    def _gerak_pose(self, pose, detik=1.5, gerakan=None):
        r = self.reg

        def mulai():
            r[11] = 8
            if gerakan is not None:
                r[17], r[18] = gerakan, 1

        def selesai():
            r[10] = pose
            r[17], r[18], r[11] = 0xFF, 0, 0
        return [(0, mulai), (detik, selesai)]

    def _ke_ready(self):
        if self.reg[10] == 3:
            return []
        return self._gerak_pose(0, 1.5) + self._gerak_pose(3, 2.0, gerakan=0)

    def _cmd_picker(self, op, arg, seq):
        r = self.reg
        if op == 11:
            if self.boleh_gerak():
                self.set_main(True)
        elif op == 12:
            self.set_main(False)
        elif op == 7:
            self.reset_fault()
        elif op in (2, 13, 15, 5, 6, 14):                      # gerakan manual -- TEST saja
            if self.main or not self.boleh_gerak():
                return False
            if op == 2:
                self.antre(self._gerak_pose(0))
            elif op == 15:
                self.antre(self._ke_ready())
            elif op == 13 and arg <= 3:
                self.antre(self._gerak_pose(arg))
            elif op in (5, 6, 14):
                self.antre([(1.0, lambda: None)])
        elif op == 8:                                          # MOVE_PACKAGE -- MAIN saja
            if not self.main or not self.boleh_gerak():
                return False

            def angkat():                                       # package lepas dari ujung Dispenser
                self.dunia.node[DISPENSER].reg[17] = 0
                log("   PICKER mengangkat package dari ujung Dispenser")

            def taruh():
                log("   PICKER meletakkan package di Stocker")
            rencana = (self._ke_ready() + self._gerak_pose(1, 2.5, gerakan=1)
                       + [(0, lambda: (r.__setitem__(17, 2), r.__setitem__(11, 12))), (0.8, angkat)]
                       + self._gerak_pose(0, 1.7, gerakan=2)
                       + self._gerak_pose(2, 2.5, gerakan=3) + [(0, taruh)]
                       + self._gerak_pose(0, 2.5, gerakan=4)
                       + self._gerak_pose(3, 2.0, gerakan=0))
            self.antre(rencana, ack_seq=seq)
            return True
        return False

    # ---------- DISPENSER ----------
    def _tahap(self, t):
        r = self.reg
        r[26] = t
        r[25] = 1 if t == 9 else 0
        r[11] = {0: 0, 7: 1, 10: 1}.get(t, 2 if t else 0)

    def _ke_tengah(self):
        """Package kosong berikutnya berjalan ke TENGAH, lalu gerbang dibuka -> SIAP ISI."""
        r = self.reg

        def tiba():
            r[16] = 1
            r[19] = (r[19] + 1) & 0xFFFF
            r[23] = (r[23] + 1) & 0xFFFF
        return [(0, lambda: self._tahap(7)), (2.0, tiba), (0, lambda: self._tahap(8)), (0.8, lambda: self._tahap(9))]

    def _cmd_dispenser(self, op, arg, seq):
        r = self.reg
        if op == 14:                                           # START_MAIN
            if not self.boleh_gerak():
                return False
            self.set_main(True)
            r[15] = 0
            awal = [(0.6, (lambda t=t: self._tahap(t))) for t in range(1, 7)]
            if r[16]:
                self.antre([(0, lambda: self._tahap(8)), (0.8, lambda: self._tahap(9))])
            else:
                self.antre(awal + self._ke_tengah())
        elif op == 15:                                         # STOP_MAIN
            self.set_main(False)
            self.langkah = []
            self._tahap(0)
            r[15], r[0] = 0, IDLE
        elif op == 2:
            self.reset_fault()
        elif op == 1:                                          # REQUEST_REFILL
            if not self.main or not self.boleh_gerak() or r[15] or r[26] != 9:
                log(f"   DISPENSER: REQUEST_REFILL DITOLAK (tahap {NAMA_TAHAP[r[26]]})")
                return False

            def sampai_ujung():
                r[16], r[17], r[15] = 0, 1, 1
                r[24] = (r[24] + 1) & 0xFFFF
                r[23] = (r[23] + 1) & 0xFFFF
                log("   DISPENSER: package penuh sampai UJUNG -> PACKAGE_READY_FLAG = 1")
            self.antre([(0, lambda: self._tahap(10)), (3.0, sampai_ujung)]
                       + [(0.5, (lambda t=t: self._tahap(t))) for t in (11, 12, 13, 14)]
                       + [(0.5, lambda: self._tahap(15))])
        elif op == 5:                                          # ACK_PACKAGE_TAKEN
            r[15] = 0
            if r[26] == 15:
                self.antre(self._ke_tengah())
            else:
                log(f"   DISPENSER: ACK_PACKAGE_TAKEN di tahap {NAMA_TAHAP[r[26]]} -- flag dibersihkan saja")
        return False

    # ---------- STOCKER ----------
    def _cmd_stocker(self, op, arg, seq):
        r = self.reg
        if op == 9:
            if self.boleh_gerak():
                self.set_main(True)
        elif op == 10:
            self.set_main(False)
        elif op == 5:
            self.reset_fault()
        elif op == 1:                                          # HOME_ALL
            if not self.boleh_gerak():
                return False
            self.antre([(0, lambda: r.__setitem__(13, 1)), (4.0, lambda: (r.__setitem__(11, 1),
                                                                           r.__setitem__(13, 0)))], seq)
            return True
        elif op == 6:                                          # GOTO_LOAD (READY)
            if not self.boleh_gerak():
                return False
            if not r[11]:
                self.fault(12, 'NOT_HOMED')
                return False
            self.antre([(0, lambda: r.__setitem__(13, 2)), (1.5, lambda: (r.__setitem__(10, 0xFF),
                                                                           r.__setitem__(13, 0)))], seq)
            return True
        elif op == 2:                                          # RUN_FULL_CYCLE
            if not self.main or not self.boleh_gerak() or not 1 <= arg <= 4:
                return False
            if not r[11]:
                self.fault(12, 'NOT_HOMED')
                return False

            def di_rak():
                r[10] = arg

            def dorong():
                if self.dunia.sensor_rak:
                    r[12] |= 1 << arg
                log(f"   STOCKER: package didorong ke Rak {arg}")
            self.antre([(0, lambda: r.__setitem__(13, 2)), (2.0, di_rak), (0, lambda: r.__setitem__(13, 3)),
                        (1.5, dorong), (0, lambda: r.__setitem__(13, 4)), (1.0, lambda: None),
                        (0, lambda: r.__setitem__(13, 5)),
                        (2.0, lambda: (r.__setitem__(10, 0xFF), r.__setitem__(13, 0)))], seq)
            return True
        elif op in (3, 4):                                     # MOVE_TO_RACK / PUSH_BOX -- TEST
            if self.main or not self.boleh_gerak():
                return False
            self.antre([(2.0, lambda: r.__setitem__(10, arg if op == 3 else r[10]))], seq)
            return True
        return False


# ============================================================
# DUNIA -- hubungan antar node (objek, package)
# ============================================================
class Dunia:
    def __init__(self, cepat, jeda_objek, tempuh, sensor_rak, watchdog):
        self.cepat, self.jeda_objek, self.tempuh = cepat, jeda_objek, tempuh
        self.sensor_rak, self.watchdog = sensor_rak, watchdog
        self.mulai = time.monotonic()
        self.node = {sid: Node(sid, self) for sid in NAMA}
        self.objek = []               # [waktu_tiba_di_sensor, reject]
        self.objek_terakhir = 0
        self.kunci = threading.Lock()

    def objek_lepas(self):
        self.objek.append([time.monotonic() + self.tempuh / self.cepat, False])

    def tick(self):
        sekarang = time.monotonic()
        s = self.node[SORTER]
        # hopper melepas objek selama Sorter RUNNING dan tidak dijeda
        if (s.reg[0] == RUN and s.boleh_gerak() and not s.reg[24]
                and sekarang - self.objek_terakhir >= self.jeda_objek / self.cepat):
            self.objek_terakhir = sekarang
            self.objek_lepas()
        # objek yang sudah menempuh conveyor sampai di sensor PASS / didorong palang
        sisa = []
        for o in self.objek:
            if sekarang < o[0]:
                sisa.append(o)
            elif s.reg[0] == RUN or not s.main:
                if o[1]:
                    s.reg[11] = (s.reg[11] + 1) & 0xFFFF
                else:
                    s.reg[10] = (s.reg[10] + 1) & 0xFFFF
                    log(f"   SORTER: objek lewat sensor PASS (PASS_COUNT={s.reg[10]})")
            else:
                sisa.append(o)        # conveyor berhenti (FAULT) -- objek diam di tempat
        self.objek = sisa
        for n in self.node.values():
            n.tick(sekarang)

    def klasifikasi(self, reject):
        """CLASSIFY_IS_REJECT ditulis master: berlaku untuk objek terdepan yang belum dinilai."""
        for o in self.objek:
            if not o[1] and o[0] - time.monotonic() > 0:
                o[1] = bool(reject)
                return


# ============================================================
# MODBUS RTU SLAVE
# ============================================================
class Slave:
    def __init__(self, ser, dunia, echo):
        self.ser, self.dunia, self.echo = ser, dunia, echo
        self.buf = b''
        self.jumlah = {'req': 0, 'crc': 0}

    def _panjang(self, b):
        """Panjang frame request yang diharapkan, atau None kalau belum bisa ditentukan."""
        if len(b) < 2:
            return None
        fc = b[1]
        if fc in (3, 4, 6):
            return 8
        if fc == 16:
            return 9 + b[6] if len(b) >= 7 else None
        return 8   # fungsi lain: anggap 8 lalu dijawab ILLEGAL FUNCTION

    def proses(self, data):
        self.buf += data
        while len(self.buf) >= 4:
            if self.buf[0] not in NAMA:
                self.buf = self.buf[1:]                       # bukan untuk node mana pun
                continue
            n = self._panjang(self.buf)
            if n is None or len(self.buf) < n:
                return
            frame, sisa = self.buf[:n], self.buf[n:]
            if crc16(frame[:-2]) != frame[-2:]:
                self.jumlah['crc'] += 1
                self.buf = self.buf[1:]                       # salah sinkron -- geser satu byte
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
                    data = b''.join((node.reg[a] & 0xFFFF).to_bytes(2, 'big') for a in range(alamat, alamat + jumlah))
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
                            self.dunia.klasifikasi(v)
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
        teks = {SORTER: lambda: f"pass={r[10]} reject={r[11]} hopper={'JEDA' if r[24] else 'jalan'} "
                                f"objek di conveyor={len(dunia.objek)}",
                PICKER: lambda: f"pose={NAMA_POSE.get(r[10], r[10])} gerakan={r[17] if r[17] != 0xFF else '-'}",
                DISPENSER: lambda: f"tahap={NAMA_TAHAP[r[26]]} tengah={r[16]} ujung={r[17]} flag={r[15]}",
                STOCKER: lambda: f"homed={r[11]} rak={r[10] if r[10] != 0xFF else '-'} bitmask={r[12]:#06b}"}[sid]()
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
                elif k == 'b':
                    dunia.objek.append([time.monotonic(), False])
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
                    print("  perintah: s | f <id> <kode> | r <id> | e <id> | m <id> | x <id> | n <id> | b | q")
        except (IndexError, ValueError, KeyError):
            print("  format salah -- contoh: f 3 11   (node 3 FAULT kode 11)")


# ============================================================
# PORT VIRTUAL -- simulasi DI DALAM program master (sorting-automation.py --simulasi),
# tanpa adaptor & kabel. Dua ujung serial yang saling tersambung di memori.
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


def mulai_virtual(cepat=1.0, objek=2.0, tempuh=3.0, sensor_rak=False, konsol_aktif=True):
    """Jalankan keempat node simulasi di thread latar. Kembalikan (port_master, dunia):
    port_master dipasang sebagai port serial minimalmodbus di program master."""
    global AWALAN
    AWALAN = '[SIM] '
    master, ujung = PortVirtual(), PortVirtual()
    master.lawan, ujung.lawan = ujung, master
    ujung.timeout = 0.005
    dunia = Dunia(cepat, objek, tempuh, sensor_rak, True)
    slave = Slave(ujung, dunia, False)
    berhenti = threading.Event()

    def bus():
        while not berhenti.is_set():
            data = ujung.read(256)
            if data:
                slave.proses(data)

    def putaran():
        while not berhenti.is_set():
            with dunia.kunci:
                dunia.tick()
            time.sleep(0.02)
    threading.Thread(target=bus, daemon=True).start()
    threading.Thread(target=putaran, daemon=True).start()
    if konsol_aktif:
        threading.Thread(target=konsol, args=(dunia, berhenti), daemon=True).start()
    return master, dunia


def main():
    ap = argparse.ArgumentParser(description="Simulasi 4 node ESP32 (Modbus RTU slave) lewat USB-RS485")
    ap.add_argument('--port', required=True, help='mis. COM5 (Windows) atau /dev/ttyUSB0 (Linux)')
    ap.add_argument('--baud', type=int, default=19200)
    ap.add_argument('--cepat', type=float, default=1.0, help='faktor percepatan semua gerakan (2 = 2x lebih cepat)')
    ap.add_argument('--objek', type=float, default=2.0, help='detik antar objek dari hopper')
    ap.add_argument('--tempuh', type=float, default=3.0, help='detik objek menempuh conveyor Sorter')
    ap.add_argument('--sensor-rak', action='store_true', help='Stocker menandai rak terisi di bitmask')
    ap.add_argument('--tanpa-watchdog', action='store_true', help='matikan FAULT COMM_TIMEOUT 30 s')
    ap.add_argument('--mati', type=int, nargs='*', default=[], metavar='ID', help='node yang tidak menjawab')
    ap.add_argument('--echo', action='store_true', help='buang pantulan data kirim (adaptor tertentu)')
    args = ap.parse_args()

    dunia = Dunia(args.cepat, args.objek, args.tempuh, args.sensor_rak, not args.tanpa_watchdog)
    for sid in args.mati:
        dunia.node[sid].mati = True
    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8, parity='N', stopbits=1, timeout=0.005)
    except serial.SerialException as e:
        sys.exit(f"ERROR: {args.port} tidak bisa dibuka: {e}")
    slave = Slave(ser, dunia, args.echo)
    berhenti = threading.Event()

    def putaran():
        while not berhenti.is_set():
            with dunia.kunci:
                dunia.tick()
            time.sleep(0.02)
    threading.Thread(target=putaran, daemon=True).start()
    threading.Thread(target=konsol, args=(dunia, berhenti), daemon=True).start()

    log(f"Simulasi 4 node di {args.port} @ {args.baud} -- cepat x{args.cepat:g}, objek tiap {args.objek:g} s")
    log("Ketik 's' + Enter untuk status, 'q' untuk keluar. Menunggu master ...")
    terakhir_lapor, req_lalu = time.monotonic(), 0
    try:
        while not berhenti.is_set():
            data = ser.read(256)
            if data:
                slave.proses(data)
            elif slave.buf and time.monotonic() - getattr(slave, '_buf_sejak', 0) > 0.05:
                slave.buf = b''        # sisa frame terpotong -- buang setelah jeda (3,5 karakter)
            if data:
                slave._buf_sejak = time.monotonic()
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
