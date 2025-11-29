#include <Arduino.h>
#include <BluetoothSerial.h>
#include "car.h"
#include "sensor.h"
#include "communication.h"
#include "time.h"

BluetoothSerial SerialBT;
char lastHorn = 'x';
volatile bool btDataReady = false;
char btLastChar;

void setup() {
    Serial.begin(115200);
    Serial.begin(115200);
    SerialBT.begin("GroundEye"); // Bluetooth name
    Serial.println("Bluetooth RC Car Ready. Connect via app!");
    lastHorn = 'x';
    Car::ShouldBeAutomated = false;

    Sensor::Init();
    Serial.println("Initialized Sensors");
    Car::Init();
    Serial.println("Initialized Car");
    Communication::Init();
    Serial.println("Initialized Communication");
}

void loop() {
    double dt = Time::ComputeDt();
    Sensor::Update(dt);
    Communication::SendOdom();

    static unsigned long lastAccRead = 0;
    if(millis() - lastAccRead > 200) {
        MPU::GetAccValues();
        lastAccRead = millis();
    }

    if(Car::ShouldBeAutomated) {
        Car::Speed = 128;
        analogWrite(ENA, Car::Speed);
        analogWrite(ENB, Car::Speed);
        double frontDistance = UltraSonic::GetDistance(FRONT_US_TRIG, FRONT_US_ECHO);

        if(frontDistance < CLEAR_MARGIN) {
            Car::Stop();
            Sensor::IMUZUPT();
            // obstacle detected in front (go either left or right)
            double leftDistance = UltraSonic::GetDistance(LEFT_US_TRIG, LEFT_US_ECHO);
            double rightDistance = UltraSonic::GetDistance(RIGHT_US_TRIG, RIGHT_US_ECHO);

            Serial.println("Front : " + String(frontDistance));
            Serial.println("Left : " + String(leftDistance));
            Serial.println("Right : " + String(rightDistance));

            if(leftDistance > CLEAR_MARGIN) {
                Serial.println("Turning LEFT 90°");
                Car::TurnAngle(90, true);
            } else if(rightDistance > CLEAR_MARGIN) {
                Serial.println("Turning RIGHT 90°");
                Car::TurnAngle(90, false);
            } else {
                Serial.println("Turning BACK 180°");
                Car::TurnAngle(180, true); // or false, doesn’t matter
            }
            Sensor::IMUZUPT();
        } else {
            Car::Forward();
        }
    }
    if(SerialBT.hasClient()) {
        if(SerialBT.available()) {
            char command = SerialBT.read();

            if((command == 'X' || command == 'x') && command != lastHorn) {
                Car::ShouldBeAutomated = (command == 'X');
                lastHorn = command;
            }

            if(!Car::ShouldBeAutomated)
                Car::HandleCommand(command);
        }
    }
}
