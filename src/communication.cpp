#include "communication.h"
#include <esp_now.h>
#include <WiFi.h>
#include "sensor.h" // <-- because we need posX, posY, velX, velY, old.yaw

// Broadcast MAC (send to all ESP-NOW receivers)
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static bool espNowReady = false;

// ------------------ SEND CALLBACK ------------------
void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    // optional: you can print transmission status here
    // (don't print every time — slows loop)
}

// ------------------ INIT ------------------
void commInit() {
    WiFi.mode(WIFI_STA);

    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    // Add broadcast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastAddr, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if(esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer!");
        return;
    }

    espNowReady = true;
    Serial.println("ESP-NOW ready");
}

// ------------------ SEND ODOMETRY ------------------
void commSendOdom() {
    if(!espNowReady) return;

    OdomPacket pkt;
    pkt.px = posX;
    pkt.py = posY;
    pkt.vx = velX;
    pkt.vy = velY;
    pkt.yaw = old.yaw;
    pkt.timestamp = millis();

    // Non-blocking async send
    esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}
