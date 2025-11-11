#include "sensor.h"

MPU6050 mpu(0x68);
float accArr[3] = {0, 0, 0};
float yaw = 0;
unsigned long lastGyroTime = 0;

void sensorInit() {
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

double getDistance(int TRIG, int ECHO) {
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    int pulseDuration = pulseIn(ECHO, HIGH, 50000);
    double distance = (0.0343 * pulseDuration) / 2;

    return distance == 0 ? 1e9 : distance;
}

void getAccValues() {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    accArr[0] = ax_g;
    accArr[1] = ay_g;
    accArr[2] = az_g;
}

float updateYaw() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long now = micros();
    float dt = (now - lastGyroTime) / 1000000.0; // seconds
    lastGyroTime = now;

    yaw += (gz / GYRO_SCALE) * dt; // degrees
    return yaw;
}