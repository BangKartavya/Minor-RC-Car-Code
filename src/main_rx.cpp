#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

struct OdomPacket {
        float px;
        float py;
        float vx;
        float vy;
        float yaw;
        uint32_t timestamp;
};

OdomPacket latest;

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    memcpy(&latest, data, sizeof(latest));

    Serial.printf("Recv: X=%.2f Y=%.2f  Yaw=%.2f  Vx=%.2f\n",
                  latest.px, latest.py, latest.yaw, latest.vx);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
}
