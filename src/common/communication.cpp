#include "communication.h"
#include "sensor.h" // <-- because we need posX, posY, velX, velY, old.yaw

// Broadcast MAC (send to all ESP-NOW receivers)
uint8_t broadcastAddr[] = {0x6C, 0xC8, 0x40, 0x4D, 0xDD, 0xB0};

static bool espNowReady = false;

// ------------------ SEND CALLBACK ------------------
void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    // optional: you can print transmission status here
    // (don't print every time — slows loop)
}

// ------------------ INIT ------------------
void commInit() {
    // 1. Initialize Wi-Fi and set the mode (STA is required for ESP-NOW)
    // This single call handles all underlying NVS, driver, and stack initialization.
    WiFi.mode(WIFI_MODE_STA);
    Serial.println("Wi-Fi Mode set to STATION.");

    // 2. Initialize ESP-NOW
    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    Serial.println("ESP-NOW initialized successfully.");

    // 3. Register the send callback
    esp_now_register_send_cb(onDataSent);

    // 4. ADD THE RECEIVER AS A PEER (Critical for Unicast)
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastAddr, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if(esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add Receiver peer!");
        return;
    }

    espNowReady = true;
    Serial.println("Communication Ready. Peer added.");
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
