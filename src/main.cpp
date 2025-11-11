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
volatile bool btDataReady = false;
char btLastChar;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_RC_Car"); // Bluetooth name
    Serial.println("Bluetooth RC Car Ready. Connect via app!");
    SPEED = 0;
    lastHorn = 'x';
    shouldBeAutomated = false;

    sensorInit();
    carInit();
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

            Serial.println("Front : " + String(frontDistance));
            Serial.println("Left : " + String(leftDistance));
            Serial.println("Right : " + String(rightDistance));

            if(leftDistance > CLEAR_MARGIN) {
                Serial.println("Turning LEFT 90°");
                turnAngle(90.0, true);
            } else if(rightDistance > CLEAR_MARGIN) {
                Serial.println("Turning RIGHT 90°");
                turnAngle(90.0, false);
            } else {
                Serial.println("Turning BACK 180°");
                turnAngle(180.0, true); // or false, doesn’t matter
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
