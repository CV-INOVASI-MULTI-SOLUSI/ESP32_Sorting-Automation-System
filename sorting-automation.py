#!/usr/bin/env python3
"""
============================================================
 SORTING AUTOMATION -- program utama (2026-10-07)
============================================================

Semua fungsi ada di applib.py. File ini hanya memilih MODE dari argumen.

  python3 sorting-automation.py                  panel LCD sentuh (mode utama)
  python3 sorting-automation.py --kalibrasi      panel, mulai dengan kalibrasi sentuh
  python3 sorting-automation.py --tanpa-lcd      produksi dari terminal (tanpa layar)
  python3 sorting-automation.py --cek            pemeriksaan awal saja, tidak menggerakkan apa pun
  python3 sorting-automation.py --uji            menu uji node di terminal
  python3 sorting-automation.py --kosongkan-rak  rak sudah dikosongkan fisik -> hapus catatannya
                                     (atau --kosongkan-rak 2,3 untuk rak tertentu)

  Tambahan untuk --tanpa-lcd / --cek:
     --tanpa-kamera   semua objek PASS (sama dengan pakai_kamera = 0)
     --batch N        objek per package, menimpa sorting.config sekali jalan
     --siklus N       berhenti rapi setelah N siklus (0 = sampai rak kosong habis)

File satu folder: applib.py, sorting.config (produksi), display.config (layar).
Hanya SATU program yang boleh memegang /dev/ttyS3. Kalau service sorting-automation
sedang jalan, hentikan dulu (systemctl stop sorting-automation) sebelum mode terminal.

Ctrl+C (atau SIGTERM dari systemd) = shutdown rapi: Sorter berhenti mengumpan dulu, lalu
semua node kembali ke TEST mode.
"""

import argparse
import os
import signal
import sys

FOLDER = os.path.dirname(os.path.abspath(__file__))
_kurang = [n for n in ('applib.py', 'sorting.config') if not os.path.exists(os.path.join(FOLDER, n))]
if _kurang:
    sys.exit(f"ERROR: file berikut belum ada di {FOLDER}:\n  " + "\n  ".join(_kurang) +
             "\nSalin ke folder yang sama dengan sorting-automation.py.")
sys.path.insert(0, FOLDER)
import applib as A   # noqa: E402  (memuat & memeriksa sorting.config)


def main():
    ap = argparse.ArgumentParser(description="Sorting Automation -- panel LCD, produksi, dan uji node")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--tanpa-lcd', action='store_true', help='produksi dari terminal, tanpa layar')
    mode.add_argument('--cek', action='store_true', help='pemeriksaan awal saja, tidak menggerakkan apa pun')
    mode.add_argument('--uji', action='store_true', help='menu uji node di terminal')
    mode.add_argument('--kosongkan-rak', nargs='?', const='1,2,3,4', metavar='RAK',
                      help='hapus catatan rak terisi SETELAH rak dikosongkan fisik (semua, atau mis. 2,3)')
    mode.add_argument('--kalibrasi', action='store_true', help='panel, mulai dengan kalibrasi sentuh')
    ap.add_argument('--tanpa-kamera', action='store_true',
                    help='semua objek PASS -- sama dengan pakai_kamera = 0 di sorting.config')
    ap.add_argument('--batch', type=int, default=A.BATCH_SIZE, metavar='N',
                    help=f'objek PASS per package (sorting.config: {A.BATCH_SIZE})')
    ap.add_argument('--siklus', type=int, default=A.JUMLAH_SIKLUS, metavar='N',
                    help=f'berhenti rapi setelah N siklus (sorting.config: {A.JUMLAH_SIKLUS}; 0 = semua rak kosong)')
    ap.add_argument('--simulasi', action='store_true',
                    help='keempat node DITIRU di dalam program -- tanpa RS485, tanpa ESP32 (butuh simulasi-node.py)')
    ap.add_argument('--sim-cepat', type=float, default=1.0, metavar='X',
                    help='simulasi: gerakan X kali lebih cepat (bawaan 1)')
    ap.add_argument('--sim-objek', type=float, default=2.0, metavar='DETIK',
                    help='simulasi: hopper melepas 1 objek tiap DETIK (bawaan 2)')
    args = ap.parse_args()
    if args.simulasi:
        siapkan_simulasi(args)

    if args.kosongkan_rak:
        try:
            dikosongkan = set(A._urutan_rak(args.kosongkan_rak))
        except ValueError:
            ap.error('--kosongkan-rak: nomor rak 1-4 dipisah koma, mis. 2,3')
        A.rak_terisi.difference_update(dikosongkan)
        A.simpan_rak_terisi()
        A.log(f"Catatan rak dikosongkan: {sorted(dikosongkan)}. "
              f"Rak terisi sekarang: {sorted(A.rak_terisi) or 'tidak ada'}")
        return

    if args.uji:
        try:
            A.menu_uji(A.Bus())
        except KeyboardInterrupt:
            print("\nDihentikan (Ctrl+C).")
        return

    if args.tanpa_lcd or args.cek:
        if args.batch < 1:
            ap.error('--batch minimal 1')
        if args.siklus < 0:
            ap.error('--siklus minimal 0')
        A.BATCH_SIZE, A.JUMLAH_SIKLUS = args.batch, args.siklus
        signal.signal(signal.SIGTERM, lambda *_: A.berhenti.set())
        signal.signal(signal.SIGINT, lambda *_: A.berhenti.set())
        kamera = A.PAKAI_KAMERA and not args.tanpa_kamera
        if not A.jalankan_produksi(A.Bus(), kamera, hanya_cek=args.cek):
            sys.exit(1)
        return

    A.jalankan_panel(kalibrasi=args.kalibrasi)


def siapkan_simulasi(args):
    """--simulasi: keempat node ditiru simulasi-node.py di DALAM program ini, tersambung lewat
    port virtual. Seluruh alur (pemeriksaan, startup, batch, rak, shutdown) berjalan seperti
    aslinya, tanpa adaptor RS485 dan tanpa ESP32.

    Catatan rak & alarm memakai file TERSENDIRI (..._simulasi), dan catatan rak dikosongkan
    tiap simulasi dimulai -- catatan mesin yang asli tidak pernah tersentuh."""
    import importlib.util
    path = os.path.join(FOLDER, 'simulasi-node.py')
    if not os.path.exists(path):
        sys.exit(f"ERROR: --simulasi butuh simulasi-node.py di {FOLDER}")
    spec = importlib.util.spec_from_file_location('simulasi_node', path)
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    port, _ = sim.mulai_virtual(cepat=args.sim_cepat, objek=args.sim_objek, konsol_aktif=not args.uji)
    import minimalmodbus
    minimalmodbus.serial.Serial = lambda *a, **k: port     # semua Instrument memakai port virtual
    A.FILE_RAK = os.path.join(FOLDER, 'rak_terisi_simulasi.txt')
    A.rak_terisi.clear()
    A.simpan_rak_terisi()
    A.FILE_ALARM = os.path.join(FOLDER, 'alarm_riwayat_simulasi.json')
    A.PAKAI_KAMERA = 0                                      # HuskyLens tidak ikut disimulasikan
    A.log("=" * 70)
    A.log(f"MODE SIMULASI -- 4 node ditiru di dalam program (gerakan x{args.sim_cepat:g}, "
          f"objek tiap {args.sim_objek:g} s). TIDAK ada yang dikirim ke RS485.")
    if not args.uji:
        A.log("Perintah simulasi (ketik + Enter): s status | f <id> <kode> fault | r <id> reset | "
              "e <id> e-stop | m <id> menu | x <id> mati/hidup | n <id> restart | b objek")
    A.log("=" * 70)


if __name__ == '__main__':
    main()
