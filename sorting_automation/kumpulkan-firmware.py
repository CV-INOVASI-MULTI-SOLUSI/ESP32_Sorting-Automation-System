#!/usr/bin/env python3
"""
Kumpulkan hasil build firmware keempat node ke folder firmware/ dengan nama yang dipakai
menu UPDATE FW di Orange Pi (MORE > SYSTEM > NODE > UPDATE FW).

DI PC, setelah build:
    pio run                       (di tiap folder ESP32_SortingAutomation_*)
    python kumpulkan-firmware.py  (di folder sorting_automation/)
hasilnya ada di sorting_automation/firmware/, lalu salin ke Orange Pi:
    scp -r firmware root@<IP-OrangePi>:/root/sorting_automation/

Hanya file yang ada yang disalin. Nomor rilis (FIRMWARE_VERSI di include/registers.h) ikut
ditampilkan -- naikkan nomor itu sebelum build kalau ingin rilis baru mudah dibedakan.
"""
import os
import re
import shutil
import time

FOLDER = os.path.dirname(os.path.abspath(__file__))          # sorting_automation/
REPO = os.path.dirname(FOLDER)                                 # folder proyek ESP32_SortingAutomation_*
NODE = {'Sorter': 'sorter.bin', 'Picker': 'picker.bin', 'Dispenser': 'dispenser.bin', 'Lifter': 'stocker.bin'}

tujuan = os.path.join(FOLDER, 'firmware')
os.makedirs(tujuan, exist_ok=True)
for proyek, nama in NODE.items():
    asal = os.path.join(REPO, f'ESP32_SortingAutomation_{proyek}', '.pio', 'build', 'esp32dev', 'firmware.bin')
    if not os.path.exists(asal):
        print(f"  -- {proyek:10s} belum di-build ({asal})")
        continue
    versi = '?'
    try:
        with open(os.path.join(REPO, f'ESP32_SortingAutomation_{proyek}', 'include', 'registers.h'),
                  encoding='utf-8') as f:
            m = re.search(r'FIRMWARE_VERSI\s*=\s*(\d+)', f.read())
        if m:
            v = int(m.group(1))
            versi = f"v{v // 100}.{v % 100:02d}"
    except OSError:
        pass
    shutil.copy2(asal, os.path.join(tujuan, nama))
    waktu = time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(asal)))
    print(f"  OK {proyek:10s} -> firmware/{nama:14s} {versi}  build {waktu}  {os.path.getsize(asal) // 1024} KB")
print(f"\nSalin ke Orange Pi:  scp -r firmware root@<IP-OrangePi>:/root/sorting_automation/")
