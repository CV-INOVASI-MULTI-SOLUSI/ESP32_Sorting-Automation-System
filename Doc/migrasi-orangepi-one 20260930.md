# Migrasi master: Orange Pi PC Plus → Orange Pi One

Tanggal: 2026-09-30.

Script Python di repo ini **tidak terikat ke board**. Semuanya hanya memakai `/dev/ttyS3`
untuk RS485/Modbus dan `/dev/ttyS1` untuk HuskyLens. Karena itu migrasi ini bukan menulis ulang
kode, melainkan menyiapkan board baru supaya dua port itu muncul lagi dengan nama yang sama.

File pendukung:

| File | Dijalankan di | Gunanya |
|---|---|---|
| `setup-orangepi-one.sh` | Orange Pi One, sekali | aktifkan UART1 + UART3, pasang Python & library, (opsional) wiringOP |
| `cek-orangepi.py` | Orange Pi One, setelah reboot | periksa port, library, keempat node, HuskyLens — **tidak mengirim command** |
| `sorting_automation.py` | Orange Pi One | tool utama, tidak berubah |

---

## 1. Yang berbeda dan berpengaruh

| | PC Plus | One | Dampak |
|---|---|---|---|
| SoC | Allwinner H3 | Allwinner H3 | **sama** — UART dan pin header sama |
| RAM | 1 GB | 512 MB | cukup untuk script Python ini |
| Jaringan | Ethernet + **WiFi** | Ethernet 100M saja, **tanpa WiFi** | wajib kabel LAN; IP hampir pasti berubah |
| Penyimpanan | eMMC 8 GB + microSD | microSD saja | OS & script ada di kartu — simpan cadangan image |
| USB host | 3 | 1 | kalau suatu saat pakai adapter USB-RS485, hanya satu slot |
| Daya | 5V DC | 5V DC (jack barrel), 2A | pakai adaptor 5V 2A, jangan dari port USB laptop |

**Soal WiFi.** Orange Pi One tidak punya WiFi. Kalau jaringan WiFi yang dipakai ESP32 untuk OTA
ternyata dipancarkan oleh PC Plus sebagai hotspot, Orange Pi One **tidak bisa menggantikannya**,
dan jaringan itu perlu disediakan router lain. Kalau jaringannya dari router biasa, OTA ESP32
tidak terpengaruh, karena OTA dikirim dari laptop, bukan dari Orange Pi.

## 2. Sambungan kabel

Kedua board memakai H3 dengan header 40-pin yang pinoutnya sama, jadi kabel bisa dipindah ke
nomor pin fisik yang sama. **Tetap cocokkan dulu** dengan cetakan di board atau `gpio readall`
sebelum menyalakan. Kalau TX/RX tertukar, hasilnya node terlihat "tidak menjawab", bukan kerusakan.

| Fungsi | UART | Device | TX board (pin fisik) | RX board (pin fisik) |
|---|---|---|---|---|
| RS485 / Modbus | UART3 | `/dev/ttyS3` | 8 (PA13) → DI modul RS485 | 10 (PA14) ← RO modul RS485 |
| HuskyLens | UART1 | `/dev/ttyS1` | 38 (PG6) → RX HuskyLens | 40 (PG7) ← TX HuskyLens |
| GND | | | pin 6 / 9 / 14 / 20 / 25 / 30 / 34 / 39 | |

**Jangan** memakai UART0 (`/dev/ttyS0`). Itu konsol debug.

**Kenapa HuskyLens di UART1, bukan UART2.** Header 40-pin H3 membawa tiga UART: UART1 (pin
38/40), UART2 (pin 11/13), UART3 (pin 8/10, dipakai RS485). UART2 sebenarnya bisa, tapi pada
tabel wiringOP yang umum untuk board H3, **wPi 5 = pin fisik 11 = UART2 TX** — pin yang sama
dengan LED merah script HuskyLens. Memakai UART2 berarti LED merah harus dipindah dulu.
UART1 tidak bentrok dengan LED/buzzer, dan sama dengan setup PC Plus sebelumnya, jadi tidak
ada script yang perlu diubah. Pastikan dengan `gpio readall` di board itu sendiri.

**UART, bukan I2C, untuk HuskyLens.** HuskyLens juga bisa I2C (alamat 0x32), tapi di sistem ini
kurang disarankan: I2C dirancang untuk jarak sependek di dalam satu papan, sedangkan kamera ada
di conveyor, dekat motor — kombinasi yang di proyek ini sudah berkali-kali menghasilkan
gangguan I2C (`I2C_ERROR_COUNT` di keempat node ada karena itu). Seluruh pembaca HuskyLens yang
ada juga berbasis UART. Kalau yang dicari kecepatan, naikkan baud HuskyLens ke 115200 (menu
HuskyLens → General Settings → Protocol Type) lalu samakan `HUSKYLENS_BAUD` di script — itu
memangkas waktu tiap pembacaan sekitar 12 kali tanpa mengganti jalur. Set Protocol Type secara
eksplisit, bukan "Auto Detect".

LED merah, LED biru, dan buzzer di script HuskyLens produksi memakai nomor **wPi** 5, 13, dan 10,
bukan nomor pin fisik. Tabel wiringOP untuk board H3 40-pin sama antara PC Plus dan One, jadi
nomornya seharusnya jatuh ke pin fisik yang sama. `setup-orangepi-one.sh --wiringop` mencetak
`gpio readall` di akhir; pakai tabel itu untuk memastikan sebelum memindah kabel LED/buzzer.

## 3. Langkah

### 3.0 Sebelum PC Plus dicabut — kalau masih bisa dinyalakan

Ada yang **tidak ada di repo** dan akan hilang bersama PC Plus:

- **Script HuskyLens produksi** yang menyalakan LED/buzzer dan mengirim `CLASSIFY_IS_REJECT`.
  Script itu tidak ada di `/root/sorting` maupun di repo.
- Service systemd, crontab, atau `rc.local` yang menjalankan script otomatis saat boot, kalau ada.
- `/boot/armbianEnv.txt` atau `/boot/orangepiEnv.txt`, sebagai pembanding overlay.

Di PC Plus:

```bash
crontab -l > /root/crontab-lama.txt 2>/dev/null
```

```bash
tar czf /root/cadangan-pcplus.tgz /root/sorting /root/*.py /root/crontab-lama.txt /etc/systemd/system/*.service /etc/rc.local /boot/*Env.txt 2>/dev/null
```

Lalu ambil `/root/cadangan-pcplus.tgz` ke laptop (WinSCP, atau `scp`).

### 3.1 Siapkan Orange Pi One

1. Flash OS ke microSD: Armbian untuk **Orange Pi One**, atau image resmi Orange Pi untuk One.
   Image untuk PC Plus **tidak** bisa dipakai, karena device tree-nya berbeda.
2. Colok kabel LAN, nyalakan, lalu cari IP-nya di daftar DHCP router.
   **Sangat disarankan:** reservasi DHCP atau IP statis. Kalau diberi `192.168.3.61` seperti PC
   Plus, catatan dan alat yang sudah ada langsung berlaku tanpa diubah.
3. Salin dan jalankan setup (dari laptop, di folder repo):

   ```bash
   scp setup-orangepi-one.sh cek-orangepi.py root@<IP-BARU>:/root/
   ```

   Di Orange Pi One:

   ```bash
   sudo bash /root/setup-orangepi-one.sh --wiringop
   ```

   Tanpa `--wiringop` kalau tidak butuh LED/buzzer script HuskyLens.
4. `reboot` — overlay UART baru aktif setelah ini.

### 3.2 Pindahkan script

```bash
scp sorting_automation.py cek-orangepi.py test-*.py contoh-*.py orangepi-orchestrator.py root@<IP-BARU>:/root/sorting/
```

Ditambah script HuskyLens produksi dari cadangan 3.0.

### 3.3 Periksa

Di Orange Pi One, **dengan kabel RS485 dan HuskyLens sudah tersambung**:

```bash
python3 /root/sorting/cek-orangepi.py
```

Script ini memeriksa dari yang paling dasar: port ada → library ada → port tidak dipegang proses
lain → keempat node menjawab → HuskyLens menjawab. Masalah paling atas biasanya penyebab masalah
di bawahnya, jadi selesaikan berurutan.

Setelah semua OK:

```bash
python3 /root/sorting/sorting_automation.py
```

Pilih `f` (cek firmware), lalu `m` (status keempat node).

## 4. Kalau gagal

| Gejala | Penyebab paling mungkin |
|---|---|
| pip: `certificate is not yet valid` / apt: `Release file ... is not valid yet` | **jam sistem mundur** — Orange Pi One tanpa RTC. `timedatectl set-ntp true`, atau `date -s "YYYY-MM-DD HH:MM:SS"`. Terulang tiap boot kalau NTP (UDP 123) diblok jaringan |
| `/dev/ttyS3` tidak ada | overlay `uart3` belum aktif, atau belum reboot. Cek `grep overlays /boot/*Env.txt` |
| Port ada, keempat node "tidak menjawab" | TX/RX UART3 tertukar, GND tidak tersambung, atau A/B RS485 tertukar |
| Sebagian node menjawab | masalah di node itu (daya/kabel cabang), bukan di Orange Pi |
| HuskyLens diam | TX/RX UART1 tertukar, atau protokol HuskyLens belum diset ke UART 9600 |
| `import wiringpi` gagal | binding wiringOP belum terbangun — ikuti README `orangepi-xunlong/wiringOP-Python` |
| `$'\r': command not found` | `setup-orangepi-one.sh` ber-akhiran baris CRLF (dari Windows). Perbaiki: `sed -i 's/\r$//' setup-orangepi-one.sh`. Repo sudah punya `.gitattributes` untuk mencegahnya |

PC Plus jangan dikosongkan dulu sampai Orange Pi One lolos 3.3 dan satu siklus uji penuh.
Selama itu, PC Plus adalah jalan kembali.
