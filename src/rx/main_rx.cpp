#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "print.h"

struct OdomPacket {
        float px;
        float py;
        float vx;
        float vy;
        float yaw;
        uint32_t timestamp;
};

OdomPacket latest;

LiveTable* liveTable = LiveTable::Instance();
double vals[] = {0, 0, 0};

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    memcpy(&latest, data, sizeof(latest));

    vals[0] = latest.px;
    vals[1] = latest.py;
    vals[2] = latest.yaw;

    liveTable->Update(vals);
}

LiveColumn cols[] = {
    {"X", "%.2f", 8, [](double v) { return WHITE; }},
    {"Y", "%.2f", 8, [](double v) { return WHITE; }},
    {"Yaw", "%.2f", 8, [](double v) { return WHITE; }},
};

void setup() {
    Serial.begin(115200);
    liveTable->Configure(cols, sizeof(cols) / sizeof(cols[0]), 4);
    liveTable->Init(vals);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("MAC ADD : " + WiFi.macAddress());
}

void loop() {
}
