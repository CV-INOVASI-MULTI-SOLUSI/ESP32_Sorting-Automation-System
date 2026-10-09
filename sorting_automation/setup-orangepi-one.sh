#!/usr/bin/env bash
# ============================================================
#  SETUP ORANGE PI ONE -- master Modbus sorting automation (2026-09-30)
# ============================================================
#  Jalankan SEKALI di board baru, sebagai root:
#      sudo bash setup-orangepi-one.sh              # dasar: UART + Python
#      sudo bash setup-orangepi-one.sh --wiringop   # + wiringOP (LED/buzzer HuskyLens)
#      sudo bash setup-orangepi-one.sh --panel      # + LCD sentuh ILI9341 (SPI, Pillow) -- termasuk wiringOP
#
#  Yang dikerjakan:
#    1. cek model board
#    2. aktifkan overlay UART3 (RS485, /dev/ttyS3). UART1 TIDAK lagi: HuskyLens lewat USB-TTL
#       (/dev/ttyUSB0) dan pin 38/40 (UART1) dipakai LED OPR & LED RUN.
#    3. pasang python3, minimalmodbus, pyserial, fuser
#    4. siapkan folder /root/sorting
#    5. (opsional) wiringOP + binding Python, dipakai script HuskyLens untuk LED & buzzer
#
#  Aman dijalankan ulang: overlay tidak digandakan, file env dicadangkan sekali saja.
#  Overlay baru aktif SETELAH REBOOT.
# ============================================================
set -euo pipefail

PASANG_WIRINGOP=0
PASANG_PANEL=0
for a in "$@"; do
  [[ "$a" == "--wiringop" ]] && PASANG_WIRINGOP=1
  [[ "$a" == "--panel" ]] && { PASANG_PANEL=1; PASANG_WIRINGOP=1; }   # panel butuh GPIO wiringOP
done

if [[ $EUID -ne 0 ]]; then
  echo "Jalankan sebagai root:  sudo bash $0 ${1:-}"
  exit 1
fi

echo "== 0. Jam sistem"
# Orange Pi One TIDAK punya baterai RTC: tiap boot jamnya mulai dari tanggal lama sampai NTP
# menyinkronkannya. Selama jam mundur, pip gagal dengan "certificate is not yet valid" dan apt
# gagal dengan "Release file ... is not valid yet" -- dua-duanya terlihat seperti masalah
# internet, padahal penyebabnya jam. Karena itu dicek PALING AWAL, sebelum apa pun diunduh.
timedatectl set-ntp true 2>/dev/null || true
for _ in $(seq 1 30); do
  [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]] && break
  sleep 1
done
echo "   sekarang: $(date '+%Y-%m-%d %H:%M:%S %Z')"
# Batas bawah = tanggal script ini ditulis. Jam yang lebih awal dari itu PASTI salah.
if (( $(date +%s) < $(date -d 2026-09-30 +%s) )); then
  echo "   !! Jam jelas salah (lebih awal dari 2026-09-30) dan NTP belum menyinkronkannya."
  echo "      Set manual (isi tanggal & jam SEKARANG), lalu jalankan ulang script ini:"
  echo "        timedatectl set-timezone Asia/Jakarta"
  echo "        date -s \"YYYY-MM-DD HH:MM:SS\""
  exit 1
fi
if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]]; then
  echo "   .. NTP belum tersinkron, tapi tanggalnya masuk akal -- lanjut."
  echo "      Setelah reboot cek lagi 'timedatectl': tanpa RTC, jam mundur tiap boot"
  echo "      kalau jaringan memblok NTP (UDP 123)."
fi

echo
echo "== 1. Board"
MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "tidak terbaca")
echo "   $MODEL"
if [[ "$MODEL" != *"One"* ]]; then
  echo "   !! Bukan Orange Pi One menurut device tree. Lanjut, tapi periksa lagi"
  echo "      pinout header 40-pin board ini sebelum menyambung RS485 & HuskyLens."
fi

echo
echo "== 2. Overlay UART  (uart3 = RS485 -> /dev/ttyS3; HuskyLens lewat USB, uart1 tidak dipakai)"
ENV=""
for f in /boot/armbianEnv.txt /boot/orangepiEnv.txt; do
  if [[ -f "$f" ]]; then ENV="$f"; break; fi
done
if [[ -z "$ENV" ]]; then
  echo "   !! /boot/armbianEnv.txt maupun /boot/orangepiEnv.txt tidak ada."
  echo "      Aktifkan UART3 manual lewat armbian-config / orangepi-config"
  echo "      (System -> Hardware), lalu reboot."
else
  [[ -f "$ENV.sebelum-sorting" ]] || cp "$ENV" "$ENV.sebelum-sorting"
  if grep -q '^overlays=' "$ENV"; then
    for o in uart3; do
      if ! grep -qE "^overlays=(.*[[:space:]])?$o([[:space:]]|$)" "$ENV"; then
        sed -i "s/^overlays=\(.*\)$/overlays=\1 $o/" "$ENV"
      fi
    done
    sed -i 's/^overlays=[[:space:]]*/overlays=/' "$ENV"
  else
    echo "overlays=uart3" >> "$ENV"
  fi
  if [[ $PASANG_PANEL -eq 1 ]]; then
    # Panel LCD sorting-automation.py: SPI0 sebagai /dev/spidev0.0. Batas frekuensi bawaan overlay
    # hanya 1 MHz -- satu layar penuh butuh >1 detik. 32 MHz memberi ruang untuk 24 MHz.
    if ! grep -qE "^overlays=(.*[[:space:]])?spi-spidev([[:space:]]|$)" "$ENV"; then
      sed -i "s/^overlays=\(.*\)$/overlays=\1 spi-spidev/" "$ENV"
    fi
    for p in param_spidev_spi_bus=0 param_spidev_max_freq=32000000; do
      k="${p%%=*}"
      if grep -q "^$k=" "$ENV"; then sed -i "s/^$k=.*/$p/" "$ENV"; else echo "$p" >> "$ENV"; fi
    done
  fi
  echo "   $ENV  ->  $(grep '^overlays=' "$ENV")"
  echo "   cadangan asli: $ENV.sebelum-sorting"
fi

echo
echo "== 3. Paket Python"
apt-get update -qq
apt-get install -y -qq python3 python3-pip python3-serial psmisc
# Debian/Ubuntu baru menolak pip ke sistem (PEP 668); coba biasa dulu, lalu paksa.
if ! pip3 install -q minimalmodbus pyserial 2>/dev/null; then
  pip3 install -q --break-system-packages minimalmodbus pyserial
fi
python3 -c "import minimalmodbus, serial; print('   minimalmodbus', minimalmodbus.__version__, '| pyserial', serial.VERSION)"
if [[ $PASANG_PANEL -eq 1 ]]; then
  apt-get install -y -qq python3-pil python3-numpy python3-spidev fonts-dejavu-core
  python3 -c "import PIL, numpy, spidev; print('   Pillow', PIL.__version__, '| numpy', numpy.__version__, '| spidev OK')"
fi

echo
echo "== 4. Folder script"
mkdir -p /root/sorting
echo "   /root/sorting siap -- salin semua script ke sini (lihat panduan migrasi)."

if [[ $PASANG_WIRINGOP -eq 1 ]]; then
  echo
  echo "== 5. wiringOP (opsional -- LED & buzzer script HuskyLens)"
  # Bagian ini TIDAK menghentikan setup kalau gagal: LED/buzzer bersifat opsional,
  # sedangkan UART & Python di atas yang wajib.
  set +e
  if command -v gpio >/dev/null 2>&1; then
    echo "   gpio sudah ada (image resmi Orange Pi biasanya sudah membawanya)."
  else
    apt-get install -y -qq git build-essential
    git clone --depth 1 https://github.com/orangepi-xunlong/wiringOP.git /opt/wiringOP \
      && (cd /opt/wiringOP && ./build clean && ./build)
  fi
  if python3 -c "import wiringpi" 2>/dev/null; then
    echo "   binding Python wiringpi sudah terpasang."
  else
    apt-get install -y -qq git build-essential python3-dev swig
    git clone --recursive https://github.com/orangepi-xunlong/wiringOP-Python.git /opt/wiringOP-Python \
      && (cd /opt/wiringOP-Python \
          && git submodule update --init --remote \
          && python3 generate-bindings.py > bindings.i \
          && python3 setup.py install)
  fi
  if python3 -c "import wiringpi" 2>/dev/null; then
    echo "   OK: import wiringpi berhasil."
    echo "   Cocokkan nomor wPi 5 (LED merah), 13 (LED biru), 10 (buzzer) ke pin fisik:"
    gpio readall || true
  else
    echo "   !! wiringpi belum bisa di-import. Ikuti README repo wiringOP-Python --"
    echo "      langkah build-nya bisa berubah antar versi. Script uji tetap jalan tanpa"
    echo "      LED/buzzer; script HuskyLens produksi butuh modul ini."
  fi
  set -e
fi

echo
echo "============================================================"
echo " SELESAI. Langkah berikutnya:"
echo "   1. reboot                       (overlay UART baru aktif setelah ini)"
echo "   2. ls -l /dev/ttyS1 /dev/ttyS3  (dua-duanya harus ada)"
echo "   3. salin script ke /root/sorting, lalu:"
echo "      python3 /root/sorting_automation/cek-orangepi.py"
if [[ $PASANG_PANEL -eq 1 ]]; then
  echo "   4. panel LCD: ls -l /dev/spidev0.0, cocokkan [lcd] di display.config dengan"
  echo "      'gpio readall', lalu: python3 sorting-automation.py  (lihat sorting-automation.service)"
fi
echo "============================================================"
