// alıcı kod
#include <esp_now.h>
#include <WiFi.h>

#define INTRODUCE_ERRORS true
#define MAX_FLIP_BITS 2

typedef struct struct_message {
  uint16_t encoded_data[2];  // 2x15-bit veri
  float original_distance;   // Debug için
} struct_message;

struct_message incomingData;

typedef struct {
  uint16_t decodedData;
  bool errorDetected;
  bool errorCorrected;
} HammingResult;

uint16_t flipRandomBits(uint16_t data, int flipCount) {
  for (int i = 0; i < flipCount; i++) {
    int bitPosition = random(0, 15);
    data ^= (1 << bitPosition);
  }
  return data;
}

HammingResult hammingDecode(uint16_t codeword) {
  HammingResult result;
  result.errorDetected = false;
  result.errorCorrected = false;

  uint16_t originalCodeword = codeword;

  // Syndrome hesapla
  uint16_t syndrome = 0;
  uint8_t parity_positions[4] = {1, 2, 4, 8};

  for (int i = 0; i < 4; i++) {
    int parity = 0;
    for (int bit = 1; bit <= 15; bit++) {
      if (bit & parity_positions[i]) {
        parity ^= (codeword >> (bit - 1)) & 1;
      }
    }
    syndrome |= (parity << i);
  }

  if (syndrome != 0) {
    result.errorDetected = true;

    if (syndrome <= 15) {
      // Tek bit hatası olabilir → dene
      codeword ^= (1 << (syndrome - 1));

      // Hâlâ hata varsa → 2+ bit hatası
      uint16_t checkSyndrome = 0;
      for (int i = 0; i < 4; i++) {
        int parity = 0;
        for (int bit = 1; bit <= 15; bit++) {
          if (bit & parity_positions[i]) {
            parity ^= (codeword >> (bit - 1)) & 1;
          }
        }
        checkSyndrome |= (parity << i);
      }

      if (checkSyndrome == 0) {
        result.errorCorrected = true;
      } else {
        // 2+ bit hatası → düzeltme yapma
        codeword = originalCodeword;
        result.errorCorrected = false;
      }
    }
  }

  // Veri bitlerini çıkar
  uint16_t data = 0;
  int dataIndex = 0;
  for (int i = 1; i <= 15; i++) {
    if (i != 1 && i != 2 && i != 4 && i != 8) {
      data |= ((codeword >> (i - 1)) & 1) << dataIndex++;
    }
  }

  result.decodedData = data;
  return result;
}

String toBinaryString(uint16_t value, uint8_t bitLength = 15) {
  String bin = "";
  for (int i = bitLength - 1; i >= 0; i--) {
    bin += ((value >> i) & 1) ? "1" : "0";
  }
  return bin;
}

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  Serial.println("\n--- YENİ PAKET ---");
  Serial.print("Gelen Orijinal Mesafe (verici ölçümü): ");
  Serial.print(incomingData.original_distance);
  Serial.println(" cm");

  uint16_t veri0_raw = incomingData.encoded_data[0];
  uint16_t veri1_raw = incomingData.encoded_data[1];

  uint16_t veri0_corrupt = INTRODUCE_ERRORS ? flipRandomBits(veri0_raw, random(0, MAX_FLIP_BITS + 1)) : veri0_raw;
  uint16_t veri1_corrupt = INTRODUCE_ERRORS ? flipRandomBits(veri1_raw, random(0, MAX_FLIP_BITS + 1)) : veri1_raw;

  HammingResult r0 = hammingDecode(veri0_corrupt);
  HammingResult r1 = hammingDecode(veri1_corrupt);

  float decodedDistance = r0.decodedData + (r1.decodedData / 100.0);

  Serial.println("> Veri [0]");
  Serial.println("  Kodlanmış : " + toBinaryString(veri0_raw));
  Serial.println("  Hatalı    : " + toBinaryString(veri0_corrupt));
  Serial.println("  Çözüldü   : " + toBinaryString(r0.decodedData, 11));
  if (r0.errorDetected)
    Serial.println(r0.errorCorrected ? "  -> 1 bit hata: düzeltildi." : "  -> 2+ bit hata: düzeltilemedi.");

  Serial.println("> Veri [1]");
  Serial.println("  Kodlanmış : " + toBinaryString(veri1_raw));
  Serial.println("  Hatalı    : " + toBinaryString(veri1_corrupt));
  Serial.println("  Çözüldü   : " + toBinaryString(r1.decodedData, 11));
  if (r1.errorDetected)
    Serial.println(r1.errorCorrected ? "  -> 1 bit hata: düzeltildi." : "  -> 2+ bit hata: düzeltilemedi.");

  Serial.print("\n🟢 Final Mesafe (decode sonrası): ");
  Serial.print(decodedDistance, 2);
  Serial.println(" cm");
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW başlatılamadı.");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Beklemede, veri gelince OnDataRecv çalışıyor
}