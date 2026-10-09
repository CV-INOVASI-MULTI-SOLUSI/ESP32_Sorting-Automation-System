#pragma once
// ============================================================
// BACKUP / RESTORE KALIBRASI LEWAT MODBUS (2026-10-09)
//
// File ini SAMA PERSIS di keempat node (include/kalibrasi_modbus.h di tiap proyek). Kalau
// diubah, ubah keempatnya -- Orange Pi (applib.py) memakai protokol yang sama untuk semua node.
//
// Tujuan: kalibrasi tiap node (isi NVS) bisa disimpan ke Orange Pi dan dimuat ke ESP32
// pengganti, tanpa kalibrasi ulang.
//
// "Gambar kalibrasi" = byte variabel kalibrasi (daftar CalSeg di main.cpp) berurutan, dengan
// nilai dari NVS -- BUKAN nilai RAM yang mungkin sedang diubah sementara lewat Modbus (mis.
// SET_CONVEYOR_SPEED yang tidak disimpan). Layout byte = layout struct di firmware ini, jadi
// restore hanya diterima kalau CAL_FORMAT dan CAL_UKURAN sama (dicek Orange Pi).
//
// Register (alamat awal = Reg::CAL_FORMAT di registers.h tiap node):
//   +0      CAL_FORMAT  R   nomor format gambar -- NAIKKAN kalau daftar/isi variabel berubah
//   +1      CAL_UKURAN  R   panjang gambar (byte)
//   +2      CAL_CRC     R   CRC16-CCITT (init 0xFFFF, poly 0x1021) gambar, diisi CAL_BACA offset 0
//   +3      CAL_OFFSET  R   offset jendela data yang terakhir diisi / diterima
//   +4      CAL_HASIL   R   kode hasil perintah CAL terakhir (CalHasil di bawah)
//   +5..+36 CAL_DATA    RW  jendela 64 byte, 2 byte per register (byte pertama = 8 bit atas)
//
// Command (opcode SAMA di semua node, ack langsung):
//   CAL_BACA     60  arg = offset   offset 0: gambar dibuat dari NVS + CRC. Isi jendela data.
//   CAL_TULIS    61  arg = offset   salin jendela data ke buffer -- berurutan mulai 0, kelipatan 64
//   CAL_TERAPKAN 62  arg = CRC      cek panjang + CRC + validasi node -> tulis RAM & NVS -> restart
//
// Beban node: NOL saat produksi. loop() hanya membandingkan dua angka; buffer dialokasikan
// selama backup/restore saja dan dibebaskan setelah selesai atau 60 s tidak dipakai. Semua
// perintah CAL ditolak selama MAIN aktif atau node bergerak.
// ============================================================
#include <Arduino.h>
#include <ModbusRTU.h>

struct CalSeg { void* p; uint16_t n; };

enum class CalHasil : uint16_t {
  KOSONG = 0, BACA_OK = 1, TULIS_OK = 2, DITERAPKAN = 3,
  DITOLAK_SIBUK = 10,      // MAIN aktif / node bergerak
  OFFSET_SALAH = 11,       // urutan potongan salah, atau CAL_BACA/TULIS offset 0 belum dikirim
  UKURAN_KURANG = 12,      // CAL_TERAPKAN sebelum semua byte diterima
  CRC_SALAH = 13,
  DATA_TIDAK_VALID = 14,   // ditolak validasi khusus node (mis. data gerakan Picker)
  MEMORI_HABIS = 15
};

inline uint16_t calCrc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

class KalibrasiModbus {
public:
  static constexpr uint8_t JUMLAH_DATA = 32;            // register jendela data
  static constexpr uint16_t BYTE_JENDELA = JUMLAH_DATA * 2;

  // muatNvs: isi variabel kalibrasi dari NVS (fungsi load yang dipakai saat boot).
  // simpanNvs: tulis SEMUA variabel kalibrasi ke NVS.
  // valid: opsional, cek gambar sebelum diterapkan (true = boleh).
  void begin(ModbusRTU& mb, uint16_t base, uint16_t format, const CalSeg* seg, uint8_t jumlahSeg,
             void (*muatNvs)(), void (*simpanNvs)(), bool (*valid)(const uint8_t*) = nullptr) {
    _mb = &mb; _base = base; _seg = seg; _nSeg = jumlahSeg;
    _muat = muatNvs; _simpan = simpanNvs; _valid = valid;
    _ukuran = 0;
    for (uint8_t i = 0; i < jumlahSeg; i++) _ukuran += seg[i].n;
    mb.addHreg(base, format);
    mb.addHreg(base + 1, _ukuran);
    mb.addHreg(base + 2, 0);
    mb.addHreg(base + 3, 0);
    mb.addHreg(base + 4, (uint16_t)CalHasil::KOSONG);
    mb.addHreg(base + 5, 0, JUMLAH_DATA);
    Serial.printf("[KAL] gambar kalibrasi %u byte (format %u)\n", _ukuran, format);
  }

  // boleh = false kalau MAIN aktif / node bergerak (diputuskan main.cpp)
  bool baca(uint16_t offset, bool boleh) {
    if (!boleh) return gagal(CalHasil::DITOLAK_SIBUK);
    if (offset == 0) {
      if (!siapkan(Mode::BACA)) return gagal(CalHasil::MEMORI_HABIS);
      uint8_t* ram = (uint8_t*)malloc(_ukuran);
      if (!ram) { bebas(); return gagal(CalHasil::MEMORI_HABIS); }
      salinKe(ram);        // simpan nilai RAM sekarang
      _muat();             // RAM <- NVS
      salinKe(_buf);       // gambar = isi NVS
      salinDari(ram);      // RAM dikembalikan persis seperti sebelumnya
      free(ram);
      _mb->Hreg(_base + 2, calCrc16(_buf, _ukuran));
    } else if (_mode != Mode::BACA || !_buf) {
      return gagal(CalHasil::OFFSET_SALAH);
    }
    if (offset >= _ukuran) return gagal(CalHasil::OFFSET_SALAH);
    for (uint8_t i = 0; i < JUMLAH_DATA; i++) {
      uint32_t a = (uint32_t)offset + i * 2;
      uint16_t hi = a < _ukuran ? _buf[a] : 0, lo = a + 1 < _ukuran ? _buf[a + 1] : 0;
      _mb->Hreg(_base + 5 + i, (uint16_t)(hi << 8 | lo));
    }
    _mb->Hreg(_base + 3, offset);
    _tSentuh = millis();
    if ((uint32_t)offset + BYTE_JENDELA >= _ukuran) bebas();   // jendela terakhir sudah di register
    return berhasil(CalHasil::BACA_OK);
  }

  bool tulis(uint16_t offset, bool boleh) {
    if (!boleh) return gagal(CalHasil::DITOLAK_SIBUK);
    if (offset == 0) {
      if (!siapkan(Mode::TULIS)) return gagal(CalHasil::MEMORI_HABIS);
      _terisi = 0;
    } else if (_mode != Mode::TULIS || !_buf) {
      return gagal(CalHasil::OFFSET_SALAH);
    }
    // berurutan; potongan yang sama boleh dikirim ulang (ack sebelumnya hilang di jalan)
    if (offset % BYTE_JENDELA != 0 || offset > _terisi || offset >= _ukuran) return gagal(CalHasil::OFFSET_SALAH);
    uint16_t n = min<uint32_t>(BYTE_JENDELA, _ukuran - offset);
    for (uint16_t i = 0; i < n; i++) {
      uint16_t w = _mb->Hreg(_base + 5 + i / 2);
      _buf[offset + i] = (i % 2 == 0) ? (uint8_t)(w >> 8) : (uint8_t)(w & 0xFF);
    }
    if (offset + n > _terisi) _terisi = offset + n;
    _mb->Hreg(_base + 3, offset);
    _tSentuh = millis();
    return berhasil(CalHasil::TULIS_OK);
  }

  bool terapkan(uint16_t crc, bool boleh) {
    if (!boleh) return gagal(CalHasil::DITOLAK_SIBUK);
    if (_mode != Mode::TULIS || !_buf) return gagal(CalHasil::OFFSET_SALAH);
    if (_terisi < _ukuran) return gagal(CalHasil::UKURAN_KURANG);
    if (calCrc16(_buf, _ukuran) != crc) return gagal(CalHasil::CRC_SALAH);
    if (_valid && !_valid(_buf)) return gagal(CalHasil::DATA_TIDAK_VALID);
    salinDari(_buf);
    _simpan();
    bebas();
    Serial.println("[KAL] kalibrasi dari Orange Pi DISIMPAN ke NVS -- restart dalam 0,5 s");
    _restartPada = millis() + 500;
    _restartAktif = true;
    return berhasil(CalHasil::DITERAPKAN);
  }

  // Panggil dari loop(): bebaskan buffer yang ditinggal, dan restart setelah restore --
  // node boot ulang memuat kalibrasi lewat jalur yang sama dengan boot biasa.
  void loop() {
    if (_buf && millis() - _tSentuh > 60000) { bebas(); Serial.println("[KAL] buffer dibebaskan (60 s tidak dipakai)"); }
    if (_restartAktif && (int32_t)(millis() - _restartPada) >= 0) ESP.restart();
  }

private:
  enum class Mode : uint8_t { TIDAK, BACA, TULIS };
  ModbusRTU* _mb = nullptr;
  uint16_t _base = 0, _ukuran = 0, _terisi = 0;
  const CalSeg* _seg = nullptr;
  uint8_t _nSeg = 0;
  void (*_muat)() = nullptr;
  void (*_simpan)() = nullptr;
  bool (*_valid)(const uint8_t*) = nullptr;
  uint8_t* _buf = nullptr;
  Mode _mode = Mode::TIDAK;
  uint32_t _tSentuh = 0, _restartPada = 0;
  bool _restartAktif = false;

  bool siapkan(Mode m) {
    bebas();
    _buf = (uint8_t*)malloc(_ukuran);
    if (!_buf) return false;
    _mode = m;
    _tSentuh = millis();
    return true;
  }
  void bebas() { free(_buf); _buf = nullptr; _mode = Mode::TIDAK; }
  void salinKe(uint8_t* b) const {
    for (uint8_t i = 0; i < _nSeg; i++) { memcpy(b, _seg[i].p, _seg[i].n); b += _seg[i].n; }
  }
  void salinDari(const uint8_t* b) const {
    for (uint8_t i = 0; i < _nSeg; i++) { memcpy(_seg[i].p, b, _seg[i].n); b += _seg[i].n; }
  }
  bool gagal(CalHasil h) {
    _mb->Hreg(_base + 4, (uint16_t)h);
    Serial.printf("[KAL] ditolak, kode %u\n", (uint16_t)h);
    return false;
  }
  bool berhasil(CalHasil h) { _mb->Hreg(_base + 4, (uint16_t)h); return true; }
};
