#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include "car.h"
#include "sensor.h"

BluetoothSerial SerialBT;
// Motor driver pins

// 5V - YELLOW
// 5V - ORANGE

char lastHorn = 'x';

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_RC_Car"); // Bluetooth name
    Serial.println("Bluetooth RC Car Ready. Connect via app!");
    SPEED = 0;
    lastHorn = 'x';
    shouldBeAutomated = false;

    Wire.begin(MPU_SDA, MPU_SCL, 100000); // SDA=19, SCL=22
    delay(100);

    Serial.println("Initializing MPU6050...");

    mpu.initialize();
    delay(100);

    uint8_t id = mpu.getDeviceID();
    Serial.print("Device ID: 0x");
    Serial.println(id, HEX);

    if(mpu.testConnection()) {
        Serial.println("MPU6050 connection successful ✅");
    } else {
        Serial.println("MPU6050 connection failed ❌");
    }
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    stop();

    pinMode(RIGHT_US_TRIG, OUTPUT);
    pinMode(RIGHT_US_ECHO, INPUT);

    pinMode(LEFT_US_TRIG, OUTPUT);
    pinMode(LEFT_US_ECHO, INPUT);

    pinMode(FRONT_US_TRIG, OUTPUT);
    pinMode(FRONT_US_ECHO, INPUT);

    digitalWrite(FRONT_US_TRIG, LOW);
    digitalWrite(LEFT_US_TRIG, LOW);
    digitalWrite(RIGHT_US_TRIG, LOW);
}

void loop() {
    static unsigned long lastAccRead = 0;
    if(millis() - lastAccRead > 200) {
        getAccValues();
        lastAccRead = millis();
    }

    if(shouldBeAutomated) {
        SPEED = 128;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        double frontDistance = getDistance(FRONT_US_TRIG, FRONT_US_ECHO);

        if(frontDistance < THRESHOLD) {
            stop();
            // obstacle detected in front (go either left or right)
            double leftDistance = getDistance(LEFT_US_TRIG, LEFT_US_ECHO);
            double rightDistance = getDistance(RIGHT_US_TRIG, RIGHT_US_ECHO);

            Serial.println("Left : " + String(leftDistance));
            Serial.println("Right : " + String(rightDistance));

            if(leftDistance >= THRESHOLD) {
                // turn left before
                left();
            } else if(rightDistance >= THRESHOLD) {
                // if left is blocked as well, turn right
                right();
            } else {
                // complete block, reverse the car
                backward();
            }
        } else {
            forward();
        }
    }
    if(SerialBT.hasClient()) {
        if(SerialBT.available()) {
            char command = SerialBT.read();

            if((command == 'X' || command == 'x') && command != lastHorn) {
                shouldBeAutomated = (command == 'X');
                lastHorn = command;
            }

            if(!shouldBeAutomated)
                handleCommand(command);
        }
    }
}
