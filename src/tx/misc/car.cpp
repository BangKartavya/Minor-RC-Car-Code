#include "car.h"

int Car::Speed = 128;
int Car::ShouldBeAutomated = false;

void Car::Init() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    Speed = 128;
    Stop();
}

void Car::Left() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void Car::Right() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void Car::Forward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void Car::Backward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void Car::Stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void Car::HandleCommand(char cmd) {
    if(ShouldBeAutomated && cmd != 'x') return;
    switch(cmd) {
    case 'L':
        Left();
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case 'R':
        Right();
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case 'F':
        Forward();
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case 'B':
        Backward();
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case 'S':
        Stop();
        break;
    case '0':
        Speed = 0;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '1':
        Speed = 32;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '2':
        Speed = 64;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '3':
        Speed = 96;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '4':
        Speed = 128;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '5':
        Speed = 160;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '6':
        Speed = 192;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '7':
        Speed = 224;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case '8':
        Speed = 255;
        analogWrite(ENA, Speed);
        analogWrite(ENB, Speed);
        break;
    case 'X':
        Speed = 128;
        ShouldBeAutomated = true;
        break;
    case 'x':
        ShouldBeAutomated = false;
        break;
    default:
        break;
    }
}

void Car::TurnAngle(float targetAngle, bool leftTurn) {
    Stop();
    MPU::ResetYaw();
    delay(100);

    if(leftTurn) Left();
    else Right();

    analogWrite(ENA, 128);
    analogWrite(ENB, 128);

    MPU::Gyro.yaw = 0;
    float cutoff = targetAngle - 10;
    while(fabs(MPU::Gyro.yaw) < cutoff) {
        double dt = Time::ComputeDt();
        MPU::GetAngle(dt);
        Serial.println("Angle : " + String(MPU::Gyro.yaw));
        yield();
    }

    Stop();
    delay(200);
}
