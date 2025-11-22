# **SOP: Sistem Akuisisi Data Isyarat ESP32-S3 (4 MPU-6050 & 10 Flex Sensor)**

Dokumen ini menjelaskan konfigurasi, fungsionalitas, dan prosedur kalibrasi yang harus diikuti untuk sistem akuisisi data waktu nyata pada ESP32-S3, beroperasi pada laju sampel 100 Hz.

## **1. Konfigurasi Perangkat Keras**

Sistem ini menggunakan arsitektur dual-bus I²C pada ESP32-S3 untuk mendukung empat MPU-6050.

### **1.1 Pinout I²C MPU-6050 (Dual Bus)**

Sistem ini menggunakan dua bus I²C perangkat keras terpisah untuk mengatasi batasan alamat MPU-6050 (`0x68` dan `0x69`).

| Bus ID | MPU ID | Alamat I²C (AD0) | Pin SDA (Data) | Pin SCL (Clock) | Penempatan Konseptual |
|--------|---------|------------------|----------------|-----------------|------------------------|
| **I²C0 (Wire)** | MPU11 | 0x68 (AD0 → GND) | **39** | **40** | Tangan Kanan (Punggung) |
| | MPU12 | 0x69 (AD0 → VCC) | **39** | **40** | Tangan Kanan (Siku) |
| **I²C1 (Wire1)** | MPU21 | 0x68 (AD0 → GND) | **41** | **42** | Tangan Kiri (Punggung) |
| | MPU22 | 0x69 (AD0 → VCC) | **41** | **42** | Tangan Kiri (Siku) |

### **1.2 Pin Flex Sensor (Analog)**

Flex sensor terhubung melalui pembagi tegangan (resistor `47 KΩ`) ke pin `ADC` ESP32-S3.

| Sensor ID | Tangan | Pin ADC (Input) | Sensor ID | Tangan | Pin ADC (Input) |
|-----------|---------|-----------------|-----------|---------|-----------------|
| **F11** | Kanan (Jempol) | 3 | **F21** | Kiri (Jempol) | 8 |
| **F12** | Kanan (Telunjuk) | 4 | **F22** | Kiri (Telunjuk) | 9 |
| **F13** | Kanan (Tengah) | 5 | **F23** | Kiri (Tengah) | 10 |
| **F14** | Kanan (Manis) | 6 | **F24** | Kiri (Manis) | 11 |
| **F15** | Kanan (Kelingking) | 7 | **F25** | Kiri (Kelingking) | 12 |

## **2. Prosedur Operasi Standar (SOP)**

### **2.1 Fase I: Persiapan dan Mode Kalibrasi (Saluran USB-CDC Tunggal)**

1. **Koneksi**: Hubungkan ESP32-S3 ke Mini PC melalui **USB bawaan**. Buka **Serial Monitor** atau program logging data pada **921,600** baud.
2. **Verifikasi MPU**: Periksa log MPU Initialization. Pastikan keempat MPU menampilkan status **OK**. Jika ada yang GAGAL, perbaiki kabel sebelum melanjutkan.
3. **Masuk Mode Kalibrasi**: Kirim karakter **'c'** (huruf kecil) melalui input Serial Monitor dan tekan `Enter`.
   - Monitor Serial akan menampilkan: `=== CALIBRATION MODE ACTIVE (10s) ===`
   - **Catatan**: Selama mode kalibrasi/perintah, aliran data `100 Hz` mungkin terganggu oleh pesan status.

### **2.2 Fase II: Gerakan Kalibrasi (Selama 10 Detik)**

Tujuan: Memberi sistem kesempatan untuk mencatat batas **tertinggi (Max)** dan **terendah (Min)** `ADC` yang mungkin dari setiap flex sensor.

1. **Posisi Awal (Maksimum `ADC`)**: Segera setelah mengirim 'c', rentangkan kedua tangan Anda dengan jari-jari **lurus, rata, dan terpisah sejauh mungkin**. Tahan posisi ini selama **2 detik**.
2. **Posisi Akhir (Minimum `ADC`)**: Secara perlahan dan bersamaan, **kepalkan kedua tangan Anda menjadi tinju yang ketat dan padat**. Tahan posisi ini selama **2 detik**.
3. **Gerakan Dinamis**: Habiskan sisa waktu (**6 detik**) dengan **berulang kali membuka dan menutup kepalan** Anda secara perlahan dan penuh.
4. **Akhir Otomatis**: Kalibrasi akan berakhir secara otomatis setelah 10 detik. Anda juga dapat mengetik **'n'** untuk mengakhirinya secara manual.

### **2.3 Fase III: Perekaman Data**

Data `100 Hz` dikirim melalui saluran USB-CDC tunggal.

## **3. Format Output Data**

### **3.1 Struktur Data dan Kecepatan**

- **Kecepatan Output**: **100 Hz**
- **Baud Rate (Port Data & Kontrol)**: **921,600**
- **Total Kolom**: **34 kolom** (24 MPU + 10 Flex)

| Bagian | Kolom | Keterangan |
|--------|-------|-------------|
| **MPU Data** | 1 hingga 24 | (4 MPUs x 6 DOF). **4 MPU** × **(Accel X,Y,Z + Gyro X,Y,Z)**. |
| **Flex Data** | 25 hingga 34 | (10 Flex Sensors). **F11, F12, F13, F14, F15, F21, F22, F23, F24, F25**. |

### **3.2 Detail Data Frame**

| Tipe Data | Unit | Keterangan |
|-----------|------|-------------|
| **Akselerometer** (`AX, AY, AZ`) | `m/s²` (Float) | Perubahan linier gerak, dipengaruhi gravitasi. |
| **Giroscope** (`GX, GY, GZ`) | `rad/s` (Float) | Kecepatan sudut rotasi, penting untuk rotasi lengan. |
| **Flex Sensor** (`F11` hingga `F25`) | `0 hingga 1000` (Integer) | **0** = Jari Lurus (Min Tekukan); **1000** = Jari Bengkok Penuh (Max Tekukan). |

## **4. Analisis Kode Teknikal (gesture_data_logger.ino)**

### **4.1 Strategi Komunikasi Saluran Tunggal (USB-CDC)**

- **Port Tunggal**: Serial.begin(921600) digunakan untuk semua I/O.
- **Perintah**: Fungsi checkSerialCommands() memeriksa Serial.available() untuk input perintah.
- **Status**: Semua pesan status/error (termasuk hasil inisialisasi dan kalibrasi) dicetak ke Serial, yang akan bercampur dengan aliran data `100 Hz`.

### **4.2 Akuisisi Data MPU**

| Fungsi | Deskripsi Teknis |
|--------|------------------|
| readMPU() | Menggunakan fungsi mpu->getEvent(). Jika pembacaan gagal, `MPU` tersebut ditandai non-fungsional (*ok = false), dan pesan error dicetak ke Serial. |
| setupMPU() | Menetapkan konfigurasi sensor yang seragam (Accel `±8 g`, Gyro `±500 deg/s`, `DLPF 94 Hz`). |

### **4.3 Logika Kalibrasi Flex Sensor**

| Variabel/Fungsi | Deskripsi Teknikal |
|-----------------|---------------------|
| flex_raw_max / flex_raw_min | Melacak rentang `ADC` ekstrem selama calibration_mode. |
| map(raw, min, max, 1000, 0) | **Normalisasi Invers**: Nilai `ADC` yang **rendah** (min `ADC`, bent) dipetakan ke `1000`, dan nilai `ADC` yang **tinggi** (max `ADC`, straight) dipetakan ke `0`. |

## **5. Pertimbangan Komunikasi Masa Depan**

### **5.1 Strategi Komunikasi Dual-Channel**

Untuk mengisolasi aliran data `100 Hz` dari pesan status dan perintah, disarankan untuk mengimplementasikan Dual-Channel Communication:

1. **Port Data (High Speed)**: Tetap menggunakan Serial (USB-CDC) pada **921,600 bps** **hanya untuk data** **34 kolom**.
2. **Port Kontrol (Status/CMD)**: Menggunakan Serial1 (Hardware UART 1, `GPIO17/GPIO18`) pada **115,200 bps** untuk semua input perintah ('c'/'n') dan output pesan error.

### **5.2 Peningkatan Efisiensi Data (Pengemasan Biner)**

Format CSV saat ini memiliki overhead (komma dan konversi float ke string). Untuk peningkatan throughput dan pengurangan latency, data **34 kolom** harus dikirim dalam format **Biner** (struct C++), bukan sebagai string.

**Struktur Data Biner yang Direkomendasikan:**

```cpp
#pragma pack(push, 1)
struct BinaryDataFrame {
    uint32_t timestamp_ms;        // 4 bytes - Timestamp
    uint16_t frame_counter;       // 2 bytes - Counter frame
    uint8_t mpu_count;           // 1 byte  - Jumlah MPU aktif
    
    // Data MPU (4 × 6 float = 24 values × 4 bytes = 96 bytes)
    float mpu_data[24];          // 96 bytes - MPU data
    
    // Data Flex (10 × 2 bytes = 20 bytes)
    uint16_t flex_data[10];      // 20 bytes - Flex sensor data
    
    uint8_t checksum;            // 1 byte  - Checksum sederhana
};
#pragma pack(pop)
// Total: 4+2+1+96+20+1 = 124 bytes/frame
```
**Keunggulan Format Biner:**

- **Efisiensi**: 124 bytes/frame vs ~250-300 bytes/frame CSV
- **Kecepatan**: Tidak perlu konversi float-string
- **Integritas**: Checksum untuk deteksi error
- **Struktur**: Header dengan timestamp dan frame counter

**Implementasi Pengiriman:**
```cpp
void sendBinaryData() {
    BinaryDataFrame frame;
    frame.timestamp_ms = millis();
    frame.frame_counter = frame_count++;
    frame.mpu_count = active_mpu_count;
    
    // Copy MPU data
    memcpy(frame.mpu_data, mpu_scaled, sizeof(mpu_scaled));
    
    // Copy flex data
    for(int i = 0; i < 10; i++) {
        frame.flex_data[i] = flex_calibrated[i];
    }
    
    // Calculate checksum
    frame.checksum = calculateChecksum((uint8_t*)&frame, sizeof(frame)-1);
    
    // Send binary frame
    Serial.write((uint8_t*)&frame, sizeof(frame));
}
```
### **5.3 Opsi Nirkabel (Wi-Fi/Bluetooth)**

**Wi-Fi (UDP/TCP):**

- **Throughput**: ~10-20 Mbps (cukup untuk 100 Hz × 124 bytes = 99.2 Kbps)
- **Latensi**: 5-20 ms (baik untuk real-time)
- **Implementasi**: UDP untuk latency rendah, TCP untuk reliability

**Bluetooth Low Energy (BLE):**

- **Throughput**: ~1-2 Mbps (membutuhkan kompresi atau reduksi data)
- **Latensi**: 20-50 ms (masih acceptable untuk 100 Hz)
- **Implementasi**: GATT Characteristic dengan notifikasi

**Komparasi Protokol Komunikasi untuk Sistem Akuisisi Data:**

| Protokol | Throughput (Maks) | Latensi Khas | Konsumsi Daya | Kompleksitas Implementasi | Keterangan |
|----------|-------------------|--------------|---------------|---------------------------|------------|
| **USB-CDC (Serial)** | ~1 Mbps | 1-5 ms | Sedang | Rendah | Solusi saat ini, stabil, plug & play |
| **Wi-Fi (TCP)** | ~10-20 Mbps | 10-50 ms | Tinggi | Sedang | Reliable, throughput tinggi, cocok untuk streaming data kontinu |
| **Wi-Fi (UDP)** | ~10-20 Mbps | 5-20 ms | Tinggi | Sedang | Latensi lebih rendah, potensi packet loss |
| **Bluetooth Classic (SPP)** | ~2-3 Mbps | 20-100 ms | Sedang-Tinggi | Sedang | Kompatibilitas luas, throughput memadai |
| **BLE (GATT)** | ~0.1-0.5 Mbps | 20-50 ms | Rendah | Tinggi | Ideal untuk aplikasi daya rendah, throughput terbatas |
| **ESP-NOW** | ~5 Mbps | 10-30 ms | Sedang | Sedang | Komunikasi peer-to-peer, latensi konsisten |
| **Serial over Bluetooth** | ~1-2 Mbps | 30-80 ms | Sedang | Rendah-Sedang | Emulasi serial wirelessly, mudah diintegrasikan |