// VERİCİ KODU 
#include <WiFi.h>

#define TRIG_PIN 5
#define ECHO_PIN 18

// Alıcı ESP32'nin MAC adresi (kendi alıcına göre değiştir)
uint8_t receiverAddress[] = {0xA0, 0xA3, 0xB3, 0x28, 0x42, 0x68};

// Mesaj yapısı
typedef struct struct_message {
  uint16_t encoded_data[2];  // 2 adet 15-bitlik veri
  float original_distance;   // Debug için orijinal mesafe
} struct_message;

struct_message dataToSend;

// Hamming (15,11) kodlama fonksiyonu
uint16_t hammingEncode(uint16_t data) {
  data &= 0x7FF; // Sadece ilk 11 bit (0b0111_1111_1111)

  // Parity bitlerini hesapla (pozisyonlar: 1,2,4,8)
  uint16_t p1 = ((data >> 0) & 1) ^ ((data >> 1) & 1) ^ ((data >> 3) & 1) ^
                ((data >> 4) & 1) ^ ((data >> 6) & 1) ^ ((data >> 8) & 1) ^
                ((data >> 10) & 1);
  uint16_t p2 = ((data >> 0) & 1) ^ ((data >> 2) & 1) ^ ((data >> 3) & 1) ^
                ((data >> 5) & 1) ^ ((data >> 6) & 1) ^ ((data >> 9) & 1) ^
                ((data >> 10) & 1);
  uint16_t p4 = ((data >> 1) & 1) ^ ((data >> 2) & 1) ^ ((data >> 3) & 1) ^
                ((data >> 7) & 1) ^ ((data >> 8) & 1) ^ ((data >> 9) & 1) ^
                ((data >> 10) & 1);
  uint16_t p8 = ((data >> 4) & 1) ^ ((data >> 5) & 1) ^ ((data >> 6) & 1) ^
                ((data >> 7) & 1) ^ ((data >> 8) & 1) ^ ((data >> 9) & 1) ^
                ((data >> 10) & 1);

  // 15-bit kodlu veriyi oluştur
  uint16_t encoded = 0;
  int dataBitIndex = 0;
  for (int i = 1; i <= 15; i++) {
    if (i == 1)      encoded |= (p1 << (i - 1));
    else if (i == 2) encoded |= (p2 << (i - 1));
    else if (i == 4) encoded |= (p4 << (i - 1));
    else if (i == 8) encoded |= (p8 << (i - 1));
    else {
      encoded |= ((data >> dataBitIndex) & 1) << (i - 1);
      dataBitIndex++;
    }
  }

  return encoded;
}

// Mesafe sensöründen veri al
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
  float distance = duration * 0.034 / 2;
  return (distance > 400 || distance <= 0) ? 0 : distance; // Max 400 cm
}

// ESP-NOW gönderim callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Gönderim durumu: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Başarılı" : "Hata");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW başlatılamadı.");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("Alıcı eklendi.");
  }
}

void loop() {
  float distance = readDistanceCM();
  dataToSend.original_distance = distance;

  // 2 adet 11-bitlik veri üretelim (örneğin 1. tam kısmı, 2. ondalık kısmı)
  uint16_t d1 = (uint16_t)distance;
  uint16_t d2 = (uint16_t)((distance - d1) * 100); // 2 digit precision

  dataToSend.encoded_data[0] = hammingEncode(d1);
  dataToSend.encoded_data[1] = hammingEncode(d2);

  esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

  Serial.print("Mesafe: ");
  Serial.print(distance);
  Serial.println(" cm gönderildi.");

  delay(2000); // 2 saniyede bir gönderim
}