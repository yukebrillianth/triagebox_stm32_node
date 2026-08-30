# Permintaan perubahan hardware: bus I²C ke board display

Status: **belum dikerjakan**. Semua di sini soal kabel dan komponen pasif, bukan
firmware. Diukur pada 2026-08-30 dengan Black Pill F411CEU6 disambung langsung ke
Waveshare ESP32-S3-Touch-LCD-4 rev 3.0.

## Ringkasan

Bus I²C antara STM32 (slave 0x42, I2C2: PB10=SCL, PB3=SDA) dan ESP32-S3 (master)
jalan 15-20 detik lalu berhenti, dan tidak pulih sendiri. Bus yang sama dipakai
GT911 (touch), TCA9554 (expander), dan SW6106 (PMIC), jadi bus yang wedge
membuat panel ikut mati — itu gejala "layar hang" yang dilaporkan.

Sisi firmware sudah diperbaiki sejauh yang bisa diperbaiki dari firmware (lihat
`Core/Src/tb_slave.c`). Yang tersisa butuh perubahan fisik.

## 1. Pull-up I²C — WAJIB

**Pasang 2.2k–4.7k dari SDA ke 3V3, dan 2.2k–4.7k dari SCL ke 3V3.**

Black Pill tidak punya pull-up I²C sama sekali di PB3/PB10 — resistor di board
itu hanya untuk USB dan NRST. Jadi seluruh bus bergantung pada pull-up di board
display, yang dipasang untuk 3 chip di satu PCB, bukan untuk 3 chip plus kabel
jumper plus board kedua.

Terukur: dengan board ESP32 tidak bertenaga, kedua pad di sisi STM32 terbaca LOW
(GPIOB IDR bit 10 dan bit 3 keduanya 0). Tidak ada apa pun di sisi Black Pill yang
menarik jalur itu naik.

Akibatnya rise time keluar dari batas 1000 ns yang diizinkan standar I²C
Fast-mode. Tepi yang lambat terbaca sebagai level yang salah, dan itu sumber
START salah tempat, BERR, dan flag BUSY yang nyangkut. Erratum STM32F411 2.8.7
("I2C analog filter may provide wrong value, locking BUSY flag") persisnya soal
filter analog salah membaca tepi.

Sampai pull-up terpasang, `tb_link_i2c.c` di repo ESP32 diturunkan ke **50 kHz**
sebagai penyangga — bukan perbaikan, hanya memberi jalur waktu dua kali lebih lama
untuk naik sebelum bit-nya di-sample. Setelah pull-up ada, kembalikan ke 100000.

## 2. Ground khusus antar board — WAJIB

**Satu kabel GND pendek langsung dari header GND Black Pill ke GND board display.**

Return current I²C harus punya jalur pendek. Tanpa itu, ground bergeser dan
threshold logic ikut bergeser.

Catatan yang relevan untuk debugging: saat ST-Link tertancap, laptop menyuplai
jalur ground kedua. Itu membuat gejalanya berubah antara "ST-Link tertancap" dan
"tidak" tanpa satu baris kode pun berubah. Jangan simpulkan apa pun dari
perbandingan dua kondisi itu sampai GND khusus terpasang.

## 3. Panjang kabel SDA/SCL — sependek mungkin

Setiap sentimeter menambah kapasitansi, dan kapasitansi itulah yang melawan
pull-up. Kalau bisa di bawah 10 cm.

## 4. AD8232: cek supply, SDN, dan kabel output ke PA1 — WAJIB

Output AD8232 pernah **ditahan di 1 count (0.8 mV)** selama ~2 menit penuh, lalu
kembali normal ke ~2040 count sendirinya. Terukur 2026-08-30 lewat SWD:

```
29 window berturut-turut (4.02 s tiap window):
  mon_ecg_mean = 1        <- normalnya 2040 (mid-rail AD8232)
  mon_ecg_rms  = 0.2-0.3  <- normalnya 256-344
  0 beat, 0% publish

lalu 30 window berikutnya, tanpa satu baris kode berubah:
  mon_ecg_mean = 2010-2205
  mon_ecg_rms  = 256-344
  bpm 78-115, 30/30 publish, cocok dengan PR MAX30102 (85/86, 93/92, 94/95)
```

Yang sudah dipastikan BUKAN penyebabnya:

- **Bukan pin mengapung.** Pin ADC yang mengapung terbaca acak dan berisik. Ini
  stabil di 1 count dengan rms 0.2 sepanjang 2000 sampel — ada yang menarik jalur
  itu ke ground secara aktif.
- **Bukan ADC atau DMA.** Kedua rank tetap hidup selama kejadian: PA1 dan PA2
  sama-sama bergerak, `mon_adc_samples` tetap 498/detik.
- **Bukan channel ketuker.** PA1 = rank 1 = `analog_data[0]` = `ECG_ADC_INDEX`,
  diverifikasi dari `adc.c`, `main.h` dan `.ioc` — semuanya sepakat.
- **Bukan DSP.** Detektor firmware di-replay pada window yang ditarik dari RAM:
  6 beat, 5 interval, 93.3 bpm, spread 0.07 — lolos semua gate.

Transisinya mendadak, bukan sinyal yang melemah: window 601 masih rms 87, window
602 langsung 0.3 dan bertahan. Dan sebaliknya juga terlihat dalam satu run —
window 257-265 rata di 1 count, 266-271 hidup (mean ~2020, rms ~150), 272-275 rata
lagi. Jadi output-nya menyala dan mati sendiri dalam skala puluhan detik.

**PENTING, dan ini menurunkan prioritas item ini:** semua pengukuran di atas
diambil saat **clamp EKG tidak terpasang**. AD8232 punya deteksi lead-off sendiri,
dan pada kondisi lead-off output-nya memang bisa dijatuhkan ke rail. Jadi "rata di
1 count" kemungkinan besar **perilaku normal AD8232 saat elektroda lepas**, bukan
modul kehilangan supply.

Jadi jangan bongkar supply dulu. Yang perlu dilakukan berurutan:

1. Ulangi pengamatan dengan **clamp terpasang di badan**. Kalau `mon_ecg_mean`
   stabil di ~2000-2300 dan tidak pernah jatuh ke 1, item ini selesai dan bukan
   masalah hardware — cukup kerjakan item 5.
2. Kalau masih jatuh ke 1 count **dengan clamp terpasang**, baru periksa 3V3 ke
   modul, pin SDN, dan kabel output ke PA1.

Bukti bahwa jalur EKG memang sehat saat elektroda terpasang, supaya tidak ada yang
mengejar hantu: 30 window berturut-turut publish bpm 78-115 dan cocok dengan PR
MAX30102 dalam 3% (85/86, 93/92, 94/95). Dua sensor independen sepakat.

## 5. Sambungkan pin LO AD8232 ke GPIO — INI YANG PALING PENTING

Pin LO (leads-off) AD8232 sekarang **tidak tersambung ke mana pun** — tidak ada
di `.ioc`, tidak dibaca di kode. Akibatnya tiga kondisi ini tidak bisa dibedakan
tanpa menempel SWD:

1. elektroda lepas dari kulit
2. modul AD8232 mati / kehilangan supply (item 4 di atas)
3. sinyal ada tapi ditolak DSP

Dan konsekuensinya bukan cuma diagnostik. **Terukur dengan clamp tidak terpasang,
firmware mempublikasikan detak jantung palsu: 214, 218, 136, dan 55 bpm** di
window-window berturutan, sementara PPG di jari stabil 82-100. Semua lolos gate
karena `DSP_HR_MAX_BPM` = 220, dan gate terakhir `RateIsRegular()` justru tidak
bisa menolongnya — **hum 50 Hz lebih teratur daripada jantung**, jadi tes
keteraturan malah lebih menyukai artefak daripada sinyal asli.

Sudah dipasang penambal di firmware (`Dsp_PickRate()`: EKG dibuang kalau beda >25%
dari PR PPG), tapi itu bergantung pada adanya jari di MAX30102. Kalau hanya
elektroda EKG yang dipakai, tidak ada pembanding, dan angka palsu lolos.

Satu GPIO input bebas cukup. Sebutkan pin mana yang dipakai supaya bisa
ditambahkan ke `.ioc` dan `Core/Inc/main.h`.

## Yang SUDAH diperiksa dan BUKAN penyebabnya

**Bukan kekurangan power di STM32.** Diukur 45 detik dengan PVD (programmable
voltage detector) dipasang di trip point tertinggi 2.9 V, dan semua flag reset
di RCC_CSR dibersihkan lebih dulu:

- reset STM32: **0**
- PVDO high (VDD di bawah 2.9 V): **0 dari 9107 sampel**
- RCC_CSR setelah run: `0x00000000` — bit BOR dan bit POR keduanya bersih,
  artinya tidak ada brownout reset

Caveat yang harus dipegang: pengukuran itu dilakukan dengan ST-Link tertancap,
yang mungkin ikut menyuplai 3V3. Pengukuran ini **belum diulang tanpa ST-Link**.
Kalau mau memastikan, ukur 3V3 Black Pill dengan multimeter saat MAX30102 dan
LoRa aktif, tanpa ST-Link.

**Bukan pin PB3 yang dipakai debugger.** DBGMCU_CR bit TRACE_IOEN terbaca 0, jadi
PB3 tidak diklaim sebagai TRACESWO. AF PB3 = 9, mode = AF, open-drain — benar
untuk I2C2.

**Bukan STM32 yang menahan jalur, pada kondisi mati yang terakhir diukur.** Dump
register saat link mati, setelah semua perbaikan firmware:

```
CR1=0x0401   PE=1, ACK=1        siap
OAR1=0x4084  alamat 0x42        benar
CR2=0x0332   ITEVTEN=1          armed
SR1=0x0000   tidak ada error
SR2=0x0000   BUSY bersih
FLTR=0x0011  filter digital     workaround erratum aktif
IDR          kedua jalur HIGH   bus dilepas
```

STM32-nya siap menjawab dan tidak dipanggil. ESP32 melaporkan poll gagal tanpa
satu pun error dari driver I²C-nya, yang berarti NACK di fase alamat. Sisa masalah
ini di sisi master, dan pull-up tidak akan menutupnya — tapi pull-up tetap harus
dipasang karena tanpa itu tidak ada pengukuran di bus ini yang bisa dipercaya.
