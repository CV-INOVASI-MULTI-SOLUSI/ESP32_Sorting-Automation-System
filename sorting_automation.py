#!/usr/bin/env python3
"""
============================================================
 SORTING AUTOMATION SYSTEM -- TEST TOOL (diperbarui 2026-09-30)
 Untuk Orange Pi sebagai Modbus RTU Master (One; sebelumnya PC Plus)
============================================================

FILE ACUAN. Urutan dan nomor command di sini SAMA dengan menu Test Command di
panel LCD keempat node, dan SAMA antar node:

  Nomor tetap di SEMUA node
     1  START_MAIN     masuk mode produksi
     2  STOP_MAIN      kembali ke mode test
     3  RESET_FAULT

  Lalu, dengan urutan kelompok yang juga sama di semua node
     Produksi   command yang hanya diterima saat MAIN
     Aksi       gerakan/aksi yang tidak dibatasi mode
     Uji        command yang ditolak selama MAIN

  Nomor 1 sampai item terakhir kelompok Uji = persis urutan di panel
  (panel memakai huruf: a = 1, b = 2, c = 3, dan seterusnya).

  Sesudahnya: command yang HANYA ada lewat Modbus, tidak ada di panel --
     Uji tambahan   jog mentah yang sengaja tidak dipasang di panel
     Setting        tuning angka (kecepatan dsb.), tidak menggerakkan apa pun

Label yang sama dengan panel dipakai apa adanya (maks 15 karakter, batas LCD),
dan nomor opcode Modbus aslinya ditampilkan di kolom "op".

SORTER: firmware menamai opcode 1 dan 2 "START" dan "STOP", bukan START_MAIN /
STOP_MAIN. Perilakunya persis sama -- START menyalakan MAIN, STOP mematikannya --
jadi di sini dan di panel ditampilkan dengan nama yang sama seperti node lain.

TIGA HAL YANG MEMBUAT PERINTAH DIABAIKAN TANPA GEJALA JELAS
  1. Operator sedang di menu kalibrasi LCD node itu. Semua command diabaikan
     DAN tidak di-ack, jadi dari sini kelihatan persis seperti node mati.
     Register MENU_ACTIVE ada supaya dua kondisi itu bisa dibedakan.
  2. Salah mode. Command produksi ditolak selama TEST, command uji ditolak
     selama MAIN. Tool ini memperingatkan SEBELUM mengirim.
  3. FAULT / E-stop. Sebagian besar command gerak ditolak sampai RESET_FAULT.

ACK BUKAN BERARTI BERHASIL. Firmware menulis CMD_ACK_SEQ tanpa syarat, termasuk
untuk command yang baru saja DITOLAKNYA. Ack hanya berarti "sudah sampai dan
sudah diproses". Berhasil atau tidaknya dibaca dari STATE / FAULT_CODE /
ACTIVITY_CODE, atau dari efek fisiknya.

Peta register & opcode diambil dari `include/registers.h` tiap node per
2026-09-30. Kalau firmware berubah, file ini ikut diperbarui -- jangan menebak.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial

Jalankan:
    python3 sorting_automation.py

SATU program Modbus saja yang boleh memegang /dev/ttyS3 pada satu waktu.
Hentikan dulu orchestrator atau script uji lain sebelum menjalankan ini.
"""

import sys
import time

try:
    import minimalmodbus
    import serial
except ImportError:
    print("ERROR: library belum terinstal. Jalankan dulu:")
    print("    pip3 install minimalmodbus pyserial")
    sys.exit(1)

# ============================================================
# KONFIGURASI
# ============================================================
SERIAL_PORT = '/dev/ttyS3'
BAUDRATE = 19200
ACK_TIMEOUT_S = 5.0

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
        ('PICK',              5,  None, 'test',   'uji'),
        ('PLACE',             6,  None, 'test',   'uji'),
        ('GOTO_POSE_N',       13, '0=HOME, 1=PICKUP, 2=LIFT', 'test', 'uji+'),
        ('RUN_GERAKAN',       14, '0=Home>Pick 1=Pick>Home 2=Home>Place 3=Place>Home', 'test', 'uji+'),
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
        ('GOTO_LOAD',         6,  None, 'netral', 'aksi'),
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
               10: 'MENUJU_PACKAGE_PICKUP', 11: 'MENUJU_LIFT_LOAD', 12: 'ADEGAN_GERAK',
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

_cmd_seq_counter = 0
_instruments = {}


class RegisterTidakAda(Exception):
    """Register memang tidak ada di firmware -- beda dari node yang tidak menjawab."""


def get_instrument(slave_id):
    """Satu objek per slave, dipakai ulang.

    Membuat objek baru (dan membuka port lagi) tiap pembacaan register berarti belasan
    kali buka-tutup port pada layar status -- sumber kegagalan yang muncul sebagai
    'node tidak menjawab' padahal nodenya baik-baik saja.
    """
    if slave_id not in _instruments:
        instr = minimalmodbus.Instrument(SERIAL_PORT, slave_id)
        instr.serial.baudrate = BAUDRATE
        instr.serial.bytesize = 8
        instr.serial.parity = serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.serial.timeout = 0.5
        instr.mode = minimalmodbus.MODE_RTU
        instr.clear_buffers_before_each_transaction = True
        _instruments[slave_id] = instr
    return _instruments[slave_id]


def baca(slave_id, reg, percobaan=3):
    """Bedakan tiga kegagalan yang artinya berbeda jauh:

      NoResponseError     node tidak menjawab sama sekali -- mati, atau kabel
                          RS485 A/B lepas/tertukar
      IllegalRequestError node menjawab, tapi register itu tidak ada -- firmware
                          belum di-flash ulang
      lain-lain           meleset sesaat, gangguan bus -- diulang

    Menyamakan yang pertama dengan yang kedua pernah menghasilkan kesimpulan
    'firmware masih lama' yang terdengar yakin padahal nodenya tidak ada di bus.
    """
    instr = get_instrument(slave_id)
    terakhir = None
    for _ in range(percobaan):
        try:
            return instr.read_register(reg, functioncode=3)
        except minimalmodbus.IllegalRequestError:
            raise RegisterTidakAda(f"register {reg} tidak ada di firmware node ini")
        except Exception as e:
            terakhir = e
            time.sleep(0.05)
    raise ConnectionError(str(terakhir))


def send_command(slave_id, opcode, arg=0, tunggu_ack=True):
    """Urutan WAJIB: CMD_ARG -> CMD_SEQ -> CMD (opcode ditulis PALING TERAKHIR)."""
    global _cmd_seq_counter
    _cmd_seq_counter = (_cmd_seq_counter % 65534) + 1
    seq = _cmd_seq_counter
    instr = get_instrument(slave_id)
    try:
        instr.write_register(REG_CMD_ARG, arg, functioncode=6)
        instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
        instr.write_register(REG_CMD, opcode, functioncode=6)
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
                print("     atau dari efek fisiknya. Node juga mencetak alasan penolakan ke")
                print("     Serial USB-nya.")
                return True
        except Exception:
            pass
        time.sleep(0.1)

    print(f"  !! TIDAK di-ack dalam {ACK_TIMEOUT_S}s.")
    print("     Dua sebab yang paling sering, dan keduanya terlihat sama dari sini:")
    print("       - node mati / kabel RS485 lepas")
    print("       - operator sedang di menu kalibrasi LCD node itu (command diabaikan")
    print("         TANPA ack). Baca status: MENU_ACTIVE akan bernilai 1.")
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
        return '  ' + {0: 'Home>Pick', 1: 'Pick>Home', 2: 'Home>Place', 3: 'Place>Home',
                       0xFF: '(tidak ada)'}.get(val, '?')
    if label == 'CURRENT_POSE':
        return '  ' + {0: 'HOME', 1: 'PICKUP', 2: 'LIFT'}.get(val, '?')
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
        print("     Modbus lain yang sedang memegang " + SERIAL_PORT + ".")
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


def main_menu():
    while True:
        print("\n============================================")
        print(" SORTING AUTOMATION SYSTEM -- TEST TOOL")
        print(f" Port: {SERIAL_PORT}  Baud: {BAUDRATE}")
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


if __name__ == '__main__':
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\nDihentikan (Ctrl+C).")
    finally:
        for instr in _instruments.values():
            try:
                instr.serial.close()
            except Exception:
                pass
