#include <Arduino.h>
#include "car.h"
#include "sensor.h"

int SPEED = 0;
bool shouldBeAutomated = false;

void left() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void right() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void forward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void backward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void handleCommand(char cmd) {
    if(shouldBeAutomated && cmd != 'x') return;
    switch(cmd) {
    case 'L':
        left();
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case 'R':
        right();
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case 'F':
        forward();
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case 'B':
        backward();
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case 'S':
        stop();
        break;
    case '0':
        SPEED = 0;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '1':
        SPEED = 32;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '2':
        SPEED = 64;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '3':
        SPEED = 96;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '4':
        SPEED = 128;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '5':
        SPEED = 160;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '6':
        SPEED = 192;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '7':
        SPEED = 224;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case '8':
        SPEED = 255;
        analogWrite(ENA, SPEED);
        analogWrite(ENB, SPEED);
        break;
    case 'X':
        SPEED = 128;
        shouldBeAutomated = true;
        break;
    case 'x':
        shouldBeAutomated = false;
        break;
    default:
        break;
    }
}

void resetYaw() {
    yaw = 0;
    lastGyroTime = micros();
}

void turnAngle(float targetAngle, bool leftTurn) {
    stop();
    resetYaw();
    delay(100);

    if(leftTurn) left();
    else right();

    analogWrite(ENA, 128);
    analogWrite(ENB, 128);

    yaw = 0;
    while(fabs(yaw) < targetAngle) {
        updateYaw();
        yield();
        delay(TURN_CHECK_DELAY);
    }

    stop();
    delay(200);
}

void carInit() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    stop();
}