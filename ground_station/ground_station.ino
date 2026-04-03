#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ─── Constants ────────────────────────────────────────────────────────────────

const int LED_PIN = 2;

const uint8_t SYNC_HEADER[2] = { 0xAA, 0xBB };

// ─── Structs ──────────────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct QuaternionPacket { float q1, q2, q3, q4; };
#pragma pack(pop)

QuaternionPacket dataIn;

// ─── Functions ────────────────────────────────────────────────────────────────

void sendBinaryPacket() {
    Serial.write(SYNC_HEADER, sizeof(SYNC_HEADER));
    Serial.write((uint8_t*)&dataIn, sizeof(dataIn));
}

void OnDataRecv(const esp_now_recv_info* info, const uint8_t* incoming, int len) {
    if (len != sizeof(dataIn)) return;

    memcpy(&dataIn, incoming, sizeof(dataIn));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    sendBinaryPacket();
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);
    delay(100);

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) return;

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() { }
