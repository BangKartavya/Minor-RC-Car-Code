#include <Arduino.h>
#include "car.h"
#include "sensor.h"

int SPEED = 0;
bool shouldBeAutomated = false;

TurnState turnState = TURN_IDLE;
float turnTarget = 0;
bool turnLeft = false;
unsigned long turnStartTime = 0;
bool turningFastLoop = false;

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

void updateTurn() {
    if(turnState != TURN_IDLE) return;

    // HIGH-FREQUENCY TURN LOOP
    for(int i = 0; i < 10; i++) {
        getAngle(); // run IMU update 10x per main loop
    }

    if(fabs(old.yaw) >= turnTarget) {
        stop();
        imuZUPT();
        old.yaw = 0;
        turningFastLoop = false; // disable fast loop
    }
}

void turnAngle(float targetAngle, bool leftTurn) {
    stop();
    resetYaw();
    delay(100);

    if(leftTurn) left();
    else right();

    analogWrite(ENA, 128);
    analogWrite(ENB, 128);

    old.yaw = 0;
    while(fabs(old.yaw) < targetAngle) {
        getAngle();
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

void startTurn(float angleDeg, bool left) {
    if(turnState != TURN_IDLE) return;
    turnTarget = angleDeg;
    turnLeft = left;

    // Reset yaw + timestamp BEFORE turning
    old.yaw = 0;
    lastTime = micros();

    turnState = TURN_START;
}

void updateTurnStateMachine() {
    switch(turnState) {
    case TURN_IDLE:
        break;

    case TURN_START:
        stop();
        delay(50);

        // reset yaw again to be safe
        old.yaw = 0;
        lastTime = micros();

        if(turnLeft) left();
        else right();

        analogWrite(ENA, 120); // <-- use enough PWM
        analogWrite(ENB, 120);

        turnStartTime = millis();
        turnState = TURN_EXECUTE;
        break;

    case TURN_EXECUTE:
        getAngle();

        // DEBUG
        Serial.print("Yaw = ");
        Serial.println(old.yaw);

        if(fabs(old.yaw) >= turnTarget) {
            stop();
            turnState = TURN_FINISH;
        }
        break;

    case TURN_FINISH:
        delay(80);
        imuZUPT();
        old.yaw = 0;
        turnState = TURN_IDLE;
        break;
    }
}
