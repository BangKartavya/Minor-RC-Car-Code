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

        float ax;
        float ay;
        float az;

        uint32_t timestamp;
};

OdomPacket latest;

LiveTable* liveTable = LiveTable::Instance();
double vals[] = {0, 0, 0, 0, 0, 0, 0};

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    memcpy(&latest, data, sizeof(latest));

    vals[0] = latest.px * 100.0f;
    vals[1] = latest.py * 100.0f;
    vals[2] = latest.yaw;
    vals[3] = latest.vx * 100.0f;

    vals[4] = latest.ax;
    vals[5] = latest.ay;
    vals[6] = latest.az;

    liveTable->Update(vals);

    delay(100);
}

LiveColumn cols[] = {
    {"X", "%.2f", 8, [](double v) { return WHITE; }},
    {"Y", "%.2f", 8, [](double v) { return WHITE; }},
    {"Yaw", "%.2f", 8, [](double v) { return WHITE; }},
    {"VelX", "%.2f", 8, [](double v) { return WHITE; }},

    {"ax", "%.2f", 8, [](double v) { return WHITE; }},
    {"ay", "%.2f", 8, [](double v) { return WHITE; }},
    {"az", "%.2f", 8, [](double v) { return WHITE; }},
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
