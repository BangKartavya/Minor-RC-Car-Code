#include "communication.h"
#include "sensor.h"

uint8_t Communication::broadcastAddr[] = {0x6C, 0xC8, 0x40, 0x4D, 0xDD, 0xB0};
bool Communication::ESPNOWReady = false;

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
}

void Communication::Init() {
    ESPNOWReady = false;
    WiFi.mode(WIFI_MODE_STA);
    Serial.println("Wi-Fi Mode set to STATION.");

    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    Serial.println("ESP-NOW initialized successfully.");

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastAddr, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if(esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add Receiver peer!");
        return;
    }

    ESPNOWReady = true;
    Serial.println("Communication Ready. Peer added.");
}

void Communication::SendOdom() {
    if(!ESPNOWReady) return;

    OdomPacket pkt;
    pkt.px = Sensor::PosX;
    pkt.py = Sensor::PosY;
    pkt.vx = Sensor::VelX;
    pkt.vy = Sensor::VelY;
    pkt.yaw = MPU::Gyro.yaw;
    pkt.ax = MPU::Acc.x;
    pkt.ay = MPU::Acc.y;
    pkt.az = MPU::Acc.z;
    pkt.timestamp = millis();

    // Non-blocking async send
    esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}
