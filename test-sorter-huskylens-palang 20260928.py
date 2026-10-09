#!/usr/bin/env python3
"""TEST SORTER — HuskyLens + palang, bertahap (versi 2026-09-28).

Dijalankan di Orange Pi. Menguji rantai penuh jalur REJECT:

    HuskyLens (UART /dev/ttyS1)  ->  Orange Pi  ->  Modbus /dev/ttyS3
    ->  Sorter CLASSIFY_IS_REJECT  ->  antrian klasifikasi  ->  palang mendorong

Bedanya dengan script produksi HuskyLens yang sudah ada: script produksi mengirim
klasifikasi lalu langsung membaca status. Itu tidak pernah bisa membuktikan palang
bekerja, karena firmware SENGAJA MENUNDA dorongan sejauh waktu tempuh objek dari
titik scan ke palang (TOF = jarak / kecepatan conveyor, bawaannya sekitar 700 ms).
Pembacaan status yang diambil beberapa milidetik setelah write selalu terjadi
SEBELUM palang bergerak, jadi hasilnya selalu terlihat "tidak ada apa-apa".

Script ini memisahkan rantai itu menjadi tiga tahap supaya kalau gagal, ketahuan
gagal di mana:

  TAHAP A — HuskyLens saja. Tidak satu pun register Modbus ditulis, palang tidak
            mungkin bergerak. Membuktikan kamera terbaca, baud benar, dan pemetaan
            ID sesuai. Menampilkan sebaran ID mentah, bukan cuma kesimpulannya,
            supaya ID yang goyah (objek di batas pandang) kelihatan.

  TAHAP B — Palang saja, tanpa HuskyLens. TEST_TRIGGER_PALANG beberapa kali dan
            JEDA dari command sampai palang bergerak DIUKUR. Angka inilah TOF
            nyata pada kalibrasi yang sekarang terpasang; tahap C memakainya untuk
            menentukan lebar jendela pengamatannya, jadi tidak ada lagi angka
            tebakan di script ini.

  TAHAP C — Gabungan. HuskyLens memutuskan, hasilnya dikirim sebagai klasifikasi
            sungguhan, lalu palang diverifikasi per objek. Ringkasan di akhir
            menghitung objek terdeteksi, verdict, dorongan yang terjadi, dan
            reject yang HILANG (REJECT_MISSED_COUNT).

CATATAN PENTING TENTANG ACTIVITY_CODE
    Dokumen dan script lain di repo ini menyuruh menunggu ACTIVITY_CODE == 2
    (CONVEYOR_JALAN_PALANG_AKTIF) sebagai tanda palang mendorong. Pada firmware
    per 2026-09-20 itu TIDAK AKAN PERNAH MUNCUL: siklus palang ikut menyetel
    motorAState = 1, dan activityCode() memeriksa motorAState LEBIH DULU, jadi
    yang terbaca 3 (MOTOR_A_JALAN). Karena itu script ini menerima 2 MAUPUN 3
    sebagai "palang bergerak", dan tetap benar baik sebelum maupun sesudah
    firmware diperbaiki. Bukti yang paling tidak ambigu tetap REJECT_COUNT naik.

Instalasi (sekali saja):
    pip3 install minimalmodbus pyserial

Jalankan:
    python3 'test-sorter-huskylens-palang 20260928.py'

SATU program Modbus saja yang boleh memegang /dev/ttyS3 pada satu waktu. Hentikan
dulu orchestrator atau script HuskyLens produksi sebelum menjalankan ini, kalau
tidak kedua sisi saling merusak pembacaan dan gejalanya mirip kerusakan bus.
"""

import select
import signal
import sys
import time

import minimalmodbus
import serial

# ============================================================
# KONFIGURASI — HUSKYLENS
# ============================================================
HUSKYLENS_PORT = '/dev/ttyS1'
HUSKYLENS_BAUD = 9600

JUMLAH_SAMPEL = 3          # sampel per objek sebelum diputuskan
INTERVAL_SAMPEL_S = 0.05
BATAS_AMBIL_S = 0.7        # berhenti mengumpulkan sampel setelah sekian detik

ID_KOSONG = 1              # tidak dihitung objek
ID_VALID = 3
ID_INVALID = (2, 4, 5)

# ============================================================
# KONFIGURASI — MODBUS SORTER
# ============================================================
SERIAL_PORT = '/dev/ttyS3'
BAUDRATE = 19200
SORTER_SLAVE_ID = 1

REG_STATE                = 0
REG_FAULT_CODE           = 1
REG_CMD                  = 2
REG_CMD_ARG              = 3
REG_CMD_SEQ              = 4
REG_CMD_ACK_SEQ          = 5
REG_PASS_COUNT           = 10
REG_REJECT_COUNT         = 11
REG_CLASSIFY_IS_REJECT   = 12
REG_ACTIVITY_CODE        = 13
REG_MAIN_MODE_ACTIVE     = 17
REG_MENU_ACTIVE          = 18
REG_REJECT_MISSED_COUNT  = 19
REG_SPEED_MM_S_AT_MAX    = 23

CMD_START               = 1
CMD_STOP                = 2
CMD_RESET_FAULT         = 3
CMD_SET_PALANG_SPEED    = 9
CMD_TEST_TRIGGER_PALANG = 98

STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "RUNNING", 3: "FAULT", 4: "ESTOPPED"}
ACTIVITY_NAMES = {
    0: "DIAM",
    1: "CONVEYOR_JALAN",
    2: "CONVEYOR_JALAN_PALANG_AKTIF",
    3: "MOTOR_A_JALAN",
    4: "TEST_HOPPER_AKTIF",
    90: "FAULT_AKTIF",
    91: "ESTOP_AKTIF",
}

# Kedua kode ini sama-sama berarti Motor A sedang berputar, dan siklus palang
# adalah satu-satunya hal yang memutarnya di tahap ini (jog manual tidak dipakai).
ACTIVITY_PALANG = (2, 3)

# ============================================================
# PILIHAN JALANNYA TEST
# ============================================================
TAHAP_A = True             # HuskyLens saja
TAHAP_B = True             # palang saja
TAHAP_C = True             # gabungan

JUMLAH_OBJEK_TAHAP_A = 5   # berapa objek diamati di tahap A
JUMLAH_TRIGGER_TAHAP_B = 3
JUMLAH_OBJEK_TAHAP_C = 10

# Conveyor pada Sorter TIDAK punya opcode nyala/mati sendiri: ia berjalan selama
# node RUNNING, jadi satu-satunya cara menjalankannya dari Modbus adalah START —
# dan START berarti MAIN, yang juga menjalankan hopper menjatuhkan objek.
#
#   False -> conveyor DIAM. Rantai listrik dan logika terbukti, tapi TOF tidak ada
#            artinya: palang mendorong ~700 ms kemudian ke objek yang tidak pindah.
#            Pakai ini untuk memeriksa kabel, ID, dan bahwa palang benar bergerak.
#   True  -> START dikirim, conveyor DAN hopper jalan. Inilah kondisi produksi
#            sesungguhnya, satu-satunya cara menilai apakah palang mengenai objek
#            yang benar. Pastikan area palang bebas tangan sebelum memilih ini.
TAHAP_C_JALANKAN_CONVEYOR = False

# Berapa lama menunggu objek berikutnya di tahap A/C sebelum menyerah
BATAS_TUNGGU_OBJEK_S = 120.0

# Jendela pengamatan palang. Kalau tahap B berhasil mengukur, angka terukur itulah
# yang dipakai (ditambah margin di bawah) dan nilai bawaan ini tidak terpakai.
JENDELA_PALANG_BAWAAN_S = 3.0
MARGIN_JENDELA_S = 1.5
POLL_PALANG_S = 0.04

ACK_TIMEOUT_S = 5.0

# LED + buzzer panel Orange Pi. Sepenuhnya opsional — kalau wiringpi tidak ada
# (misalnya script dijalankan di laptop untuk diperiksa), test tetap berjalan
# tanpa indikator, bukan gagal di baris import.
LED_RED = 5
LED_BLUE = 13
BUZZER = 10

try:
    import wiringpi
    wiringpi.wiringPiSetup()
    for _pin in (LED_RED, LED_BLUE, BUZZER):
        wiringpi.pinMode(_pin, 1)
        wiringpi.digitalWrite(_pin, 0)
    INDIKATOR_ADA = True
except Exception as _e:
    wiringpi = None
    INDIKATOR_ADA = False
    print(f".. indikator LED/buzzer dilewati ({_e})")

KEYBOARD_ADA = sys.stdin is not None and sys.stdin.isatty()

_berhenti = False


def minta_berhenti(signum, frame):
    global _berhenti
    _berhenti = True
    print("\n.. sinyal berhenti diterima, menutup dengan rapi")


signal.signal(signal.SIGINT, minta_berhenti)
signal.signal(signal.SIGTERM, minta_berhenti)


# ============================================================
# INDIKATOR
# ============================================================
def indikator(pin, nyala):
    if INDIKATOR_ADA:
        wiringpi.digitalWrite(pin, 1 if nyala else 0)


def bunyi(pola):
    """pola = deret (durasi_on, durasi_off) dalam detik."""
    if not INDIKATOR_ADA:
        return
    for on_s, off_s in pola:
        wiringpi.digitalWrite(BUZZER, 1)
        time.sleep(on_s)
        wiringpi.digitalWrite(BUZZER, 0)
        time.sleep(off_s)


def tanda_valid():
    indikator(LED_RED, False)
    indikator(LED_BLUE, True)
    bunyi([(0.08, 0.12), (0.08, 0.12)])
    time.sleep(0.2)
    indikator(LED_BLUE, False)


def tanda_invalid():
    indikator(LED_BLUE, False)
    indikator(LED_RED, True)
    bunyi([(0.30, 0.12), (0.08, 0.12), (0.08, 0.0)])
    time.sleep(0.2)
    indikator(LED_RED, False)


def matikan_indikator():
    indikator(LED_RED, False)
    indikator(LED_BLUE, False)
    indikator(BUZZER, False)


# ============================================================
# HUSKYLENS
# ============================================================
REQUEST_BLOCKS = bytes([0x55, 0xAA, 0x11, 0x00, 0x20, 0x30])


class HuskyLens:
    """Pembaca frame HuskyLens.

    Satu perbedaan yang menentukan dari versi sebelumnya: buffer byte disimpan di
    objek, tidak dibuat ulang tiap pembacaan. Versi lama memakai buffer lokal yang
    dibuang setiap kali fungsi selesai, jadi frame yang kebetulan terpotong di
    tengah (hal normal pada serial 9600) HILANG seluruhnya, bukan tersambung dengan
    sisa byte yang datang berikutnya. Akibatnya deteksi kadang meleset tanpa sebab
    yang kelihatan, dan itu paling sering terjadi justru saat objek datang cepat.
    """

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def baca_id(self, batas_s=0.05):
        """Kirim satu permintaan, kembalikan ID objek pertama yang terbaca.

        None berarti tidak ada frame objek dalam jendela ini — belum tentu tidak
        ada objek, bisa juga frame-nya belum lengkap. Pemanggil mengulangnya.
        """
        self.ser.write(REQUEST_BLOCKS)

        batas = time.monotonic() + batas_s
        while time.monotonic() < batas:
            data = self.ser.read(64)
            if data:
                self.buf += data

            obj_id = self._ambil_frame()
            if obj_id is not None:
                return obj_id

        return None

    def _ambil_frame(self):
        """Proses isi buffer. Kembalikan ID kalau ada frame objek yang lengkap."""
        while True:
            # Buang byte sampai header ketemu. Buffer dipotong, bukan digeser satu
            # per satu, supaya byte sampah tidak menumpuk selamanya.
            awal = self.buf.find(b'\x55\xAA\x11')
            if awal < 0:
                # Sisakan 2 byte terakhir: header bisa terbelah antar pembacaan.
                if len(self.buf) > 2:
                    del self.buf[:-2]
                return None
            if awal > 0:
                del self.buf[:awal]

            if len(self.buf) < 4:
                return None

            panjang = self.buf[3]
            total = 4 + panjang + 2
            if len(self.buf) < total:
                return None

            paket = bytes(self.buf[:total])
            del self.buf[:total]

            # 0x2A = COMMAND_RETURN_BLOCK, payload 10 byte: x,y,w,h,id (little endian)
            if panjang == 0x0A and paket[4] == 0x2A:
                return paket[13] | (paket[14] << 8)

            # Frame jenis lain (misalnya COMMAND_RETURN_INFO 0x29) dibuang, lanjut cari.

    def ambil_sampel(self, jumlah=JUMLAH_SAMPEL, batas_s=BATAS_AMBIL_S):
        """Kumpulkan beberapa sampel ID untuk SATU objek.

        ID_KOSONG dilewati dan tidak mengakhiri pengumpulan — itu berarti bidang
        pandang kosong sesaat, bukan objek dengan kelas 'kosong'.
        """
        sampel = []
        mulai = time.monotonic()
        while len(sampel) < jumlah:
            if _berhenti or time.monotonic() - mulai >= batas_s:
                break
            obj_id = self.baca_id()
            if obj_id is None:
                continue
            if obj_id == ID_KOSONG:
                time.sleep(INTERVAL_SAMPEL_S)
                continue
            if obj_id == ID_VALID or obj_id in ID_INVALID:
                sampel.append(obj_id)
            time.sleep(INTERVAL_SAMPEL_S)
        return sampel


def putuskan(sampel):
    """Kembalikan ('KOSONG'|'VALID'|'INVALID', jumlah_valid, jumlah_invalid)."""
    if not sampel:
        return "KOSONG", 0, 0
    valid = sampel.count(ID_VALID)
    invalid = sum(sampel.count(i) for i in ID_INVALID)
    return ("VALID" if valid > invalid else "INVALID"), valid, invalid


# ============================================================
# MODBUS
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


class RegisterTidakAda(Exception):
    """Register memang tidak ada di firmware — beda dari sekadar gagal baca sesaat."""


def baca(instr, reg, nama, percobaan=3, diam=False):
    """Bedakan dengan sengaja: 'register tidak ada' (IllegalRequestError, firmware
    belum di-flash ulang) versus 'meleset sesaat' (gangguan bus), dan keduanya lagi
    dari 'node tidak menjawab sama sekali' (NoResponseError, node mati atau kabel
    RS485 lepas). Menyamakan ketiganya pernah menghasilkan kesimpulan yakin tapi
    salah bahwa firmware-nya ketinggalan versi."""
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


def kirim_command(instr, nama, opcode, arg=0, timeout=ACK_TIMEOUT_S):
    """Protokol 3 langkah: CMD_ARG, lalu CMD_SEQ, lalu CMD paling akhir.

    ACK BUKAN BERARTI BERHASIL. Firmware menulis CMD_ACK_SEQ tanpa syarat,
    termasuk untuk command yang baru saja DITOLAKNYA. Keberhasilan dinilai dari
    STATE / FAULT_CODE / ACTIVITY_CODE atau dari efek fisiknya.
    """
    global _seq
    _seq += 1
    seq = _seq
    print(f"  -> {nama} (opcode={opcode} arg={arg} seq={seq})")
    try:
        instr.write_register(REG_CMD_ARG, arg, functioncode=6)
        instr.write_register(REG_CMD_SEQ, seq, functioncode=6)
        instr.write_register(REG_CMD, opcode, functioncode=6)
    except Exception as e:
        print(f"  !! gagal menulis command {nama}: {e}")
        return False

    batas = time.monotonic() + timeout
    while time.monotonic() < batas:
        if baca(instr, REG_CMD_ACK_SEQ, "CMD_ACK_SEQ", percobaan=1, diam=True) == seq:
            return True
        time.sleep(0.1)

    print(f"  !! {nama} tidak di-ack dalam {timeout}s "
          f"(node mati, atau operator masuk menu kalibrasi setelah test dimulai)")
    return False


def status(instr, judul=""):
    st = baca(instr, REG_STATE, "STATE", diam=True)
    act = baca(instr, REG_ACTIVITY_CODE, "ACTIVITY_CODE", diam=True)
    print(f"     STATE={st}({STATE_NAMES.get(st, '?')}) "
          f"FAULT={baca(instr, REG_FAULT_CODE, 'FAULT_CODE', diam=True)} "
          f"ACTIVITY={act}({ACTIVITY_NAMES.get(act, '?')}) "
          f"PASS={baca(instr, REG_PASS_COUNT, 'PASS_COUNT', diam=True)} "
          f"REJECT={baca(instr, REG_REJECT_COUNT, 'REJECT_COUNT', diam=True)}"
          f"{('  <- ' + judul) if judul else ''}")


def cek_prasyarat(instr):
    """Empat hal yang diam-diam membuat seluruh test ini tidak berarti."""
    st = baca(instr, REG_STATE, "STATE")
    if st is None:
        print("!! Sorter tidak menjawab Modbus sama sekali.")
        print("   Cek daya node, kabel RS485 A/B, dan pastikan tidak ada program")
        print("   Modbus lain yang sedang memegang /dev/ttyS3.")
        return False

    # (1) Menu kalibrasi LCD. Selama operator di menu, command Modbus diabaikan TANPA
    #     ack, DAN sejak 2026-09-20 klasifikasi (CLASSIFY_IS_REJECT) ikut diabaikan.
    #     Itu justru yang paling menyesatkan di sini: HuskyLens terlihat bekerja,
    #     write-nya sukses, tapi palang tidak akan pernah bergerak.
    menu = baca(instr, REG_MENU_ACTIVE, "MENU_ACTIVE", diam=True)
    if menu == 1:
        print("!! Operator sedang di menu kalibrasi LCD Sorter.")
        print("   Semua command DAN semua klasifikasi diabaikan. Keluar dulu dari menu")
        print("   (tombol D sampai kembali ke layar utama).")
        return False
    if menu is None:
        print(".. MENU_ACTIVE tidak terbaca — firmware mungkin belum di-flash ulang.")
        print("   Test tetap bisa jalan, tapi kondisi 'operator di menu' tidak")
        print("   terdeteksi dan akan terlihat seperti palang yang tidak bekerja.")

    # (2) FAULT/ESTOP. handlePalangQueue() TIDAK dipanggil sama sekali saat FAULT atau
    #     ESTOPPED, jadi klasifikasi apa pun akan menumpuk di antrian tanpa efek.
    fault = baca(instr, REG_FAULT_CODE, "FAULT_CODE")
    if fault:
        print(f"!! Sorter FAULT (code={fault}). Palang tidak akan bergerak.")
        print("   Kirim RESET_FAULT, atau lewat menu LCD: Setting Kalibrasi -> Reset Fault.")
        return False
    if st == 4:
        print("!! Sorter ESTOPPED. Lepaskan E-stop dulu.")
        return False

    # (3) Mode. Hanya memengaruhi tahap B (TEST_TRIGGER_PALANG ditolak saat MAIN).
    #     Klasifikasi produksi sendiri tidak dibatasi mode.
    mode = "MAIN (produksi)" if baca(instr, REG_MAIN_MODE_ACTIVE, "MAIN_MODE_ACTIVE",
                                     diam=True) == 1 else "TEST"
    print(f"  .. mode Sorter sekarang: {mode}")

    # (4) Kecepatan hasil ukur, kalau uji kecepatan objek pernah dijalankan. Nilai ini
    #     yang menentukan TOF, jadi ia menentukan pula kapan palang mendorong.
    mmps = baca(instr, REG_SPEED_MM_S_AT_MAX, "SPEED_MM_S_AT_MAX_PWM", diam=True)
    if mmps:
        print(f"  .. kecepatan terukur @PWM penuh: {mmps} mm/detik "
              f"(saran untuk kalibrasi 'Mm/s Max')")
    else:
        print("  .. belum ada hasil uji kecepatan objek. TOF memakai angka kalibrasi")
        print("     'Mm/s Max' yang mungkin masih tebakan — lihat menu Uji Kecepatan.")
    return True


# ============================================================
# VERIFIKASI PALANG
# ============================================================
def pantau_palang(instr, jendela_s):
    """Amati satu dorongan palang. Kembalikan dict hasil.

    Dipanggil SETELAH klasifikasi/trigger dikirim. Dua bukti dikumpulkan:
      - REJECT_COUNT naik  -> bukti paling tidak ambigu, dicatat firmware saat PUSH mulai
      - ACTIVITY_CODE 2/3  -> palang sedang bergerak saat itu diamati

    Keduanya dikumpulkan karena masing-masing bisa terlewat: counter bisa naik dan
    turun dari pandangan kalau polling terlalu lambat untuk menangkap ACTIVITY,
    sedangkan ACTIVITY bisa saja tertangkap tanpa counter sempat terbaca berubah.
    """
    reject_awal = baca(instr, REG_REJECT_COUNT, "REJECT_COUNT", diam=True)
    missed_awal = baca(instr, REG_REJECT_MISSED_COUNT, "REJECT_MISSED_COUNT", diam=True)

    t0 = time.monotonic()
    batas = t0 + jendela_s
    jeda_ms = None
    aktivitas = set()
    reject_akhir = reject_awal

    while time.monotonic() < batas:
        act = baca(instr, REG_ACTIVITY_CODE, "ACTIVITY_CODE", percobaan=1, diam=True)
        if act is not None:
            aktivitas.add(act)
            if act in ACTIVITY_PALANG and jeda_ms is None:
                jeda_ms = int((time.monotonic() - t0) * 1000)

        rc = baca(instr, REG_REJECT_COUNT, "REJECT_COUNT", percobaan=1, diam=True)
        if rc is not None:
            reject_akhir = rc
            if reject_awal is not None and rc > reject_awal and jeda_ms is None:
                jeda_ms = int((time.monotonic() - t0) * 1000)

        # Palang sudah bergerak DAN sudah selesai -> tidak perlu menunggu sisa jendela.
        if jeda_ms is not None and act is not None and act not in ACTIVITY_PALANG \
                and time.monotonic() - t0 > (jeda_ms / 1000.0) + 0.2:
            break

        time.sleep(POLL_PALANG_S)

    missed_akhir = baca(instr, REG_REJECT_MISSED_COUNT, "REJECT_MISSED_COUNT", diam=True)

    naik = (reject_awal is not None and reject_akhir is not None
            and reject_akhir > reject_awal)
    terlewat = 0
    if missed_awal is not None and missed_akhir is not None:
        terlewat = missed_akhir - missed_awal

    return {
        'dorong': naik or any(a in ACTIVITY_PALANG for a in aktivitas),
        'reject_naik': naik,
        'jeda_ms': jeda_ms,
        'aktivitas': sorted(aktivitas),
        'terlewat': terlewat,
        'reject_awal': reject_awal,
        'reject_akhir': reject_akhir,
    }


def laporkan_palang(hasil):
    act_teks = ", ".join(f"{a}({ACTIVITY_NAMES.get(a, '?')})" for a in hasil['aktivitas'])
    print(f"     ACTIVITY terlihat: {act_teks or '(tidak ada)'}")
    print(f"     REJECT_COUNT {hasil['reject_awal']} -> {hasil['reject_akhir']}")
    if hasil['dorong']:
        jeda = f"{hasil['jeda_ms']} ms" if hasil['jeda_ms'] is not None else "?"
        print(f"     PALANG BEKERJA, jeda dari command {jeda}")
    else:
        print("     Palang TIDAK terlihat bekerja dalam jendela pengamatan.")
        print("     Kemungkinan berurutan dari yang paling sering:")
        print("       - TOF lebih lama dari jendela (perbesar JENDELA_PALANG_BAWAAN_S)")
        print("       - command/klasifikasi ditolak (lihat Serial USB Sorter)")
        print("       - palang macet secara mekanis, atau kabel Motor A lepas")
    if hasil['terlewat']:
        print(f"     !! REJECT_MISSED_COUNT naik {hasil['terlewat']} — ada reject yang")
        print("        dibuang firmware: antrian penuh, atau giliran dorongnya sudah basi.")


# ============================================================
# TAHAP A — HUSKYLENS SAJA
# ============================================================
def tahap_a(husky):
    """Tidak menulis satu pun register. Palang tidak mungkin bergerak di tahap ini."""
    print("\n" + "=" * 62)
    print("TAHAP A — HuskyLens saja (Modbus tidak disentuh)")
    print("=" * 62)
    print(f"Lewatkan {JUMLAH_OBJEK_TAHAP_A} objek di depan kamera satu per satu.")
    print(f"ID{ID_VALID} = VALID, ID{'/'.join(str(i) for i in ID_INVALID)} = INVALID, "
          f"ID{ID_KOSONG} = kosong (dilewati).\n")

    hasil = []
    for n in range(1, JUMLAH_OBJEK_TAHAP_A + 1):
        if _berhenti:
            break
        print(f"  [objek {n}/{JUMLAH_OBJEK_TAHAP_A}] menunggu...")
        sampel = tunggu_objek(husky)
        if sampel is None:
            print("  .. tidak ada objek sampai batas waktu, tahap A dihentikan")
            break

        verdict, valid, invalid = putuskan(sampel)
        print(f"     sampel mentah = {sampel}  (valid={valid} invalid={invalid})")
        print(f"     verdict = {verdict}")
        if len(set(sampel)) > 1:
            print("     .. ID tidak seragam. Kalau ini sering terjadi, objek mungkin")
            print("        terbaca di batas bidang pandang, atau pencahayaan berubah.")
        hasil.append(verdict)

        if verdict == "VALID":
            tanda_valid()
        else:
            tanda_invalid()

    print(f"\n  Ringkasan tahap A: {len(hasil)} objek, "
          f"VALID={hasil.count('VALID')} INVALID={hasil.count('INVALID')}")
    return len(hasil) > 0


def tunggu_objek(husky, timeout=BATAS_TUNGGU_OBJEK_S):
    """Kembalikan daftar sampel objek berikutnya, atau None kalau kehabisan waktu."""
    batas = time.monotonic() + timeout
    while time.monotonic() < batas:
        if _berhenti:
            return None
        sampel = husky.ambil_sampel()
        if sampel:
            return sampel
    return None


# ============================================================
# TAHAP B — PALANG SAJA
# ============================================================
def tahap_b(instr):
    """TEST_TRIGGER_PALANG beberapa kali, jeda command->dorong diukur.

    Kembalikan jeda terlama (detik) atau None. Angka itu dipakai tahap C untuk
    menentukan lebar jendela pengamatannya — jadi jendela di tahap C berdasar
    ukuran nyata, bukan tebakan.
    """
    print("\n" + "=" * 62)
    print("TAHAP B — palang saja (HuskyLens tidak dipakai)")
    print("=" * 62)
    print("Pastikan tidak ada tangan di area palang. Palang akan mendorong")
    print(f"{JUMLAH_TRIGGER_TAHAP_B} kali, masing-masing ~700 ms setelah command.\n")

    if baca(instr, REG_MAIN_MODE_ACTIVE, "MAIN_MODE_ACTIVE", diam=True) == 1:
        print("  .. MAIN aktif. TEST_TRIGGER_PALANG ditolak selama produksi berjalan,")
        print("     jadi STOP dikirim dulu.")
        kirim_command(instr, "STOP", CMD_STOP)
        time.sleep(0.3)

    jeda_terukur = []
    for n in range(1, JUMLAH_TRIGGER_TAHAP_B + 1):
        if _berhenti:
            break
        print(f"  [trigger {n}/{JUMLAH_TRIGGER_TAHAP_B}]")
        if not kirim_command(instr, "TEST_TRIGGER_PALANG", CMD_TEST_TRIGGER_PALANG):
            print("     command tidak di-ack, trigger ini dilewati")
            continue

        hasil = pantau_palang(instr, JENDELA_PALANG_BAWAAN_S)
        laporkan_palang(hasil)
        if hasil['jeda_ms'] is not None:
            jeda_terukur.append(hasil['jeda_ms'])

        # Satu siklus palang ~600 ms (push + retract). Jangan menumpuk trigger di
        # atasnya: antrian yang menunggu palang selesai bisa jadi basi lalu dibuang.
        time.sleep(1.0)

    if not jeda_terukur:
        print("\n  Tidak ada jeda yang berhasil diukur. Tahap C akan memakai jendela")
        print(f"  bawaan {JENDELA_PALANG_BAWAAN_S}s.")
        return None

    terlama = max(jeda_terukur)
    print(f"\n  Ringkasan tahap B: {len(jeda_terukur)} dorongan terukur, "
          f"jeda {min(jeda_terukur)}-{terlama} ms")
    print("  Jeda inilah TOF nyata pada kalibrasi yang sekarang terpasang.")
    print("  Kalau angkanya jauh dari (jarak scan->palang / kecepatan conveyor),")
    print("  salah satu dari dua angka kalibrasi itu tidak sesuai kenyataan.")
    return terlama / 1000.0


# ============================================================
# TAHAP C — GABUNGAN
# ============================================================
def tahap_c(instr, husky, jendela_s):
    print("\n" + "=" * 62)
    print("TAHAP C — HuskyLens memutuskan, palang diverifikasi")
    print("=" * 62)
    print(f"Jendela pengamatan palang: {jendela_s:.1f}s per objek INVALID.")

    conveyor_dinyalakan = False
    if TAHAP_C_JALANKAN_CONVEYOR:
        print("\n  PERINGATAN: START akan dikirim. Conveyor DAN hopper akan berjalan,")
        print("  dan palang akan mendorong benda nyata. Pastikan area palang bebas.")
        if KEYBOARD_ADA:
            print("  Tekan Enter untuk lanjut, Ctrl+C untuk batal.")
            try:
                sys.stdin.readline()
            except Exception:
                pass
        if _berhenti:
            return
        conveyor_dinyalakan = kirim_command(instr, "START", CMD_START)
        time.sleep(0.5)
        status(instr, "setelah START")
    else:
        print("\n  Conveyor DIAM (TAHAP_C_JALANKAN_CONVEYOR=False). Rantai logika diuji,")
        print("  tapi objek tidak bergerak — jadi tahap ini TIDAK menilai apakah palang")
        print("  mengenai objek yang benar, hanya bahwa palang bergerak saat diminta.")

    print()
    ringkas = {'objek': 0, 'valid': 0, 'invalid': 0, 'dorong': 0, 'gagal': 0, 'terlewat': 0}

    try:
        for n in range(1, JUMLAH_OBJEK_TAHAP_C + 1):
            if _berhenti:
                break
            print(f"  [objek {n}/{JUMLAH_OBJEK_TAHAP_C}] menunggu...")
            sampel = tunggu_objek(husky)
            if sampel is None:
                print("  .. tidak ada objek sampai batas waktu, tahap C dihentikan")
                break

            verdict, valid, invalid = putuskan(sampel)
            ringkas['objek'] += 1
            print(f"     sampel = {sampel} -> {verdict} (valid={valid} invalid={invalid})")

            if verdict == "VALID":
                ringkas['valid'] += 1
                tanda_valid()
                # Klasifikasi pass tetap dikirim, tidak dilewati. Firmware memang tidak
                # melakukan apa-apa untuknya, tapi mengirimnya membuat urutan objek di
                # antrian firmware sama dengan urutan objek sungguhan — dan kalau
                # PASS_COUNT tidak ikut naik, itu petunjuk sendiri.
                try:
                    instr.write_register(REG_CLASSIFY_IS_REJECT, 0, functioncode=6)
                    print("     -> CLASSIFY_IS_REJECT = 0 (pass)")
                except Exception as e:
                    print(f"     !! gagal kirim klasifikasi pass: {e}")
                continue

            ringkas['invalid'] += 1
            tanda_invalid()

            # Jalur produksi sesungguhnya: SATU write, tanpa protokol 3 langkah dan
            # tanpa ack. Register ini sengaja dibuat semurah mungkin karena ditulis
            # sekali per objek. Konsekuensinya keberhasilan HANYA bisa dinilai dari
            # efeknya, yaitu apa yang dipantau di bawah.
            try:
                instr.write_register(REG_CLASSIFY_IS_REJECT, 1, functioncode=6)
                print("     -> CLASSIFY_IS_REJECT = 1 (reject, tanpa ack)")
            except Exception as e:
                print(f"     !! gagal kirim klasifikasi reject: {e}")
                ringkas['gagal'] += 1
                continue

            hasil = pantau_palang(instr, jendela_s)
            laporkan_palang(hasil)
            if hasil['dorong']:
                ringkas['dorong'] += 1
            else:
                ringkas['gagal'] += 1
            ringkas['terlewat'] += hasil['terlewat']

    finally:
        if conveyor_dinyalakan:
            print("\n  Mengembalikan Sorter ke TEST mode (STOP).")
            kirim_command(instr, "STOP", CMD_STOP)

    print("\n" + "-" * 62)
    print(f"  Ringkasan tahap C: {ringkas['objek']} objek terdeteksi")
    print(f"    VALID   : {ringkas['valid']}")
    print(f"    INVALID : {ringkas['invalid']}")
    print(f"    palang bekerja        : {ringkas['dorong']}")
    print(f"    palang tidak terlihat : {ringkas['gagal']}")
    print(f"    REJECT hilang (firmware membuangnya) : {ringkas['terlewat']}")
    if ringkas['invalid'] and ringkas['dorong'] == ringkas['invalid'] \
            and ringkas['terlewat'] == 0:
        print("\n  Rantai HuskyLens -> Orange Pi -> Sorter -> palang UTUH.")
    elif ringkas['invalid']:
        print("\n  Ada objek INVALID yang tidak menghasilkan dorongan. Kalau")
        print("  REJECT hilang > 0, sebabnya di firmware (antrian penuh atau giliran")
        print("  basi) — objek datang lebih cepat dari satu siklus palang (~600 ms).")


# ============================================================
if __name__ == '__main__':
    print("=" * 62)
    print("TEST SORTER — HuskyLens + palang (slave 1)")
    print("=" * 62)

    husky = None
    sorter = None
    try:
        sorter = connect(SORTER_SLAVE_ID)
        status(sorter, "kondisi awal")
        if not cek_prasyarat(sorter):
            raise SystemExit(1)

        husky = HuskyLens(HUSKYLENS_PORT, HUSKYLENS_BAUD)

        if TAHAP_A and not _berhenti:
            if not tahap_a(husky):
                print("\n!! Tahap A tidak mendeteksi objek apa pun. Tahap C dilewati —")
                print("   menguji rantai gabungan dengan kamera yang belum terbukti")
                print("   hanya menghasilkan kegagalan yang tidak bisa ditelusuri.")
                if TAHAP_B and not _berhenti:
                    tahap_b(sorter)
                raise SystemExit(1)

        jendela = JENDELA_PALANG_BAWAAN_S
        if TAHAP_B and not _berhenti:
            terukur = tahap_b(sorter)
            if terukur:
                jendela = terukur + MARGIN_JENDELA_S

        if TAHAP_C and not _berhenti:
            tahap_c(sorter, husky, jendela)

        print("\nSelesai.")
        status(sorter, "kondisi akhir")

    except RegisterTidakAda as e:
        print(f"\n!! {e}")
        print("   Sorter belum di-flash dengan firmware terbaru. Register MENU_ACTIVE(18),")
        print("   REJECT_MISSED_COUNT(19) dan SPEED_*(20-23) baru ada setelah flash ulang.")
    except SystemExit:
        raise
    except Exception as e:
        print(f"\n!! Test berhenti karena kesalahan: {e}")
    finally:
        # Palang TIDAK dihentikan di sini dengan sengaja. Siklusnya push+retract, dan
        # memotongnya di tengah meninggalkan palang menjulur di atas conveyor. Biarkan
        # firmware menyelesaikan siklusnya sendiri.
        matikan_indikator()
        if husky:
            husky.close()
        if sorter:
            try:
                sorter.serial.close()
            except Exception:
                pass
        print("LED dan buzzer dimatikan, port ditutup.")
